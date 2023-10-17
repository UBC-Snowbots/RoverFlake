/*
 * This file is part of libphidget22
 *
 * Copyright (c) 2015-2022 Phidgets Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "phidgetbase.h"
#include "device/hubdevice.h"
#include "stats.h"

//PhidgetDeviceListHandle ActiveSPIDevices = 0;
//PhidgetDeviceListHandle AttachedSPIDevices = 0;

static void
PhidgetSPIConnectionDelete(PhidgetSPIConnectionHandle *conn) {

	mos_mutex_destroy(&(*conn)->rlock);
	mos_cond_destroy(&(*conn)->rcond);

	mos_free(*conn, sizeof(PhidgetSPIConnection));
}

PhidgetReturnCode
PhidgetSPIConnectionCreate(PhidgetSPIConnectionHandle *conn) {

	assert(conn);

	*conn = mos_zalloc(sizeof(PhidgetSPIConnection));
	phidget_init((PhidgetHandle)*conn, PHIDGET_SPI_CONNECTION, (PhidgetDelete_t)PhidgetSPIConnectionDelete);

	mos_mutex_init(&(*conn)->rlock);
	mos_cond_init(&(*conn)->rcond);
	(*conn)->dev = -1;

	return (EPHIDGET_OK);
}

MOS_TASK_RESULT
PhidgetSPIReadThreadFunction(void *param) {
	PhidgetSPIConnectionHandle conn;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;
	int attached;

	res = EPHIDGET_OK;
	conn = NULL;

	device = PhidgetDeviceCast(param);
	if (device == NULL) {
		logerr("SPI ReadThread exiting - Invalid device handle");
		goto exit;
	}

	loginfo("SPI ReadThread running for %"PRIphid"", device);

	conn = PhidgetSPIConnectionCast(device->conn);
	assert(conn != NULL);

	for (;;) {
		mos_mutex_lock(&conn->rlock);
		if (conn->run != 1 || !ISATTACHED(device))
			break;
		mos_mutex_unlock(&conn->rlock);

		res = PhidgetDevice_read(device);
		switch (res) {
		case EPHIDGET_TIMEOUT: // timeout is ok - spi hub only sends data when there is data to send.
		case EPHIDGET_OK:
			break;
		case EPHIDGET_CLOSED: // Don't bother logging, loop will exit next iteration
			break;
		default:
			//logerr("SPI ReadThread exiting - PhidgetDevice_read returned: "PRC_FMT, PRC_ARGS(res));
			//goto exit;

			// XXX - for now, do NOT exit the read thread even in error condition,
			// because this leaves the device open but useless. Eventually, may want to
			// close the device handle in this case? Or critical error and crash?
			logerr("SPI ReadThread continuing - PhidgetDevice_read returned: "PRC_FMT, PRC_ARGS(res));
			break;
		}
	}

	attached = ISATTACHED(device);
	loginfo("SPI ReadThread exiting normally (Phidget %"PRIphid" is %s)", device, attached ? "attached" : "detached");

exit:

	if (conn) {
		conn->run = 0;
		mos_cond_broadcast(&conn->rcond);
		mos_mutex_unlock(&conn->rlock);
	}

	decPhidgetStat("spi.readthreads");
	MOS_TASK_EXIT(res);
}

void
joinSPIReadThread(PhidgetSPIConnectionHandle conn) {

	mos_mutex_lock(&conn->rlock);
	while (conn->run != 0) {
		conn->run = 2;
		mos_cond_broadcast(&conn->rcond);
		mos_cond_wait(&conn->rcond, &conn->rlock);
	}
	mos_mutex_unlock(&conn->rlock);
}

void
stopSPIReadThread(PhidgetSPIConnectionHandle conn) {

	mos_mutex_lock(&conn->rlock);
	if (conn->run != 0)
		conn->run = 2;
	mos_cond_broadcast(&conn->rcond);
	mos_mutex_unlock(&conn->rlock);
}

PhidgetReturnCode
openAttachedSPIDevice(PhidgetDeviceHandle device) {
	PhidgetSPIConnectionHandle conn;
	PhidgetReturnCode res;

	res = PhidgetSPIOpenHandle(device);
	if (res != EPHIDGET_OK)
		return (res);

	conn = PhidgetSPIConnectionCast(device->conn);
	assert(conn);

	mos_mutex_lock(&conn->rlock);
	conn->run = 1;
	res = mos_task_create(&conn->task, PhidgetSPIReadThreadFunction, device);
	if (res != 0) {
		conn->run = 0;
		mos_mutex_unlock(&conn->rlock);
		logwarn("unable to create read thread");
		if (device->_closing)
			device->_closing(device);
		PhidgetSPICloseHandle(conn);
		return (EPHIDGET_UNEXPECTED);
	}
	mos_mutex_unlock(&conn->rlock);
	incPhidgetStat("spi.readthreads_ever");
	incPhidgetStat("spi.readthreads");

	PhidgetSetFlags(device, PHIDGET_ATTACHING_FLAG);
	res = device->initAfterOpen((PhidgetDeviceHandle)device);
	if (res != EPHIDGET_OK) {
		logerr("Device Initialization functions failed: "PRC_FMT, PRC_ARGS(res));
		if (res == EPHIDGET_BADVERSION)
			logwarn("This Phidget requires a newer library - please upgrade.");
		PhidgetCLRFlags(device, PHIDGET_ATTACHING_FLAG);
		if (device->_closing)
			device->_closing(device);
		PhidgetSPICloseHandle(conn);
		return (res);
	}
	PhidgetCLRFlags(device, PHIDGET_ATTACHING_FLAG);

	return (EPHIDGET_OK);
}

#ifndef SPI_SUPPORT

PhidgetReturnCode
PhidgetSPIGetVINTDevicesString(char *str, size_t len) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPIOpenHandle(PhidgetDeviceHandle device) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPICloseHandle(PhidgetSPIConnectionHandle conn) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPIReadPacket(PhidgetSPIConnectionHandle conn, unsigned char *buffer, size_t *len) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPISendPacket(mosiop_t iop, PhidgetSPIConnectionHandle conn, const unsigned char *buffer, size_t len) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPISetLabel(PhidgetDeviceHandle device, char *buffer) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPIRefreshLabelString(PhidgetDeviceHandle device) {
	return (EPHIDGET_UNSUPPORTED);
}

#endif

