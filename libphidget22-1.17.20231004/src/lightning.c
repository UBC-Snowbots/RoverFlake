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

#ifndef LIGHTNING_SUPPORT

PhidgetReturnCode
PhidgetLightningSendPacket(mosiop_t iop, PhidgetLightningConnectionHandle conn, const unsigned char *buffer, size_t len) {

	return (EPHIDGET_UNSUPPORTED);
}

#endif

PhidgetLightningConnectionHandle
mallocPhidgetLightningConnection() {

	return (mos_zalloc(sizeof (PhidgetLightningConnection)));
}

void
freePhidgetLightningConnection(PhidgetLightningConnectionHandle item) {

	mos_free(item, sizeof (*item));
}

PhidgetReturnCode
openAttachedLightningDevice(PhidgetDeviceHandle device) {
	PhidgetReturnCode res;

	PhidgetSetFlags(device, PHIDGET_ATTACHING_FLAG);
	res = device->initAfterOpen(device);
	PhidgetCLRFlags(device, PHIDGET_ATTACHING_FLAG);
	if (res == EPHIDGET_OK)
		return (EPHIDGET_OK);

	logerr("Device Initialization functions failed: "PRC_FMT, PRC_ARGS(res));
	if (res == EPHIDGET_BADVERSION)
		logwarn("This Phidget requires a newer library - please upgrade.");
	// Call device specific close function if it exists
	if (device->_closing)
		device->_closing(device);
	return (res);
}
