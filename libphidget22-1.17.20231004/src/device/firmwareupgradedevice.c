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
#include "gpp.h"
#include "device/firmwareupgradedevice.h"

// === Internal Functions === //

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetFirmwareUpgradeDevice_initAfterOpen(PhidgetDeviceHandle device) {

	PhidgetFirmwareUpgradeDeviceHandle phid = (PhidgetFirmwareUpgradeDeviceHandle)device;
	assert(phid);

	phid->progress[0] = PUNK_DBL;

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetFirmwareUpgradeDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetFirmwareUpgradeDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetFirmwareUpgradeDeviceHandle phid = (PhidgetFirmwareUpgradeDeviceHandle)ch->parent;
	PhidgetReturnCode ret;
	size_t base64Len;
	void *base64;
	int off;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_FIRMWAREUPGRADE);
	assert(ch->class == PHIDCHCLASS_FIRMWAREUPGRADE);

	switch (bp->vpkt) {
	case BP_SENDFIRMWARE:
		off = 0;
		ret = bridgePacketBase64Decode(bp, NULL, &base64Len, &off);
		if (ret != EPHIDGET_OK)
			return (ret);

		base64 = mos_malloc(base64Len);
		ret = bridgePacketBase64Decode(bp, base64, &base64Len, &off);
		if (ret != EPHIDGET_OK)
			return (ret);

		ret = GPP_eraseFirmware(bp->iop, (PhidgetDeviceHandle)phid);
		if (ret != EPHIDGET_OK) {
			mos_free(base64, base64Len);
			return (ret);
		}

		ret = GPP_upgradeFirmware(bp->iop, (PhidgetDeviceHandle)phid, base64, base64Len, ch);
		mos_free(base64, base64Len);
		return (ret);

	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

static void CCONV
PhidgetFirmwareUpgradeDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetFirmwareUpgradeDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetFirmwareUpgradeDevice_create(PhidgetFirmwareUpgradeDeviceHandle *phidp) {
	DEVICECREATE_BODY(FirmwareUpgradeDevice, PHIDCLASS_FIRMWAREUPGRADE);
	return (EPHIDGET_OK);
}
