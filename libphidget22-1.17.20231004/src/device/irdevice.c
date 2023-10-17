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
#include "device/irdevice.h"
#include "usb.h"

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetIRDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetIRDeviceHandle phid = (PhidgetIRDeviceHandle)device;
	assert(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1055:
	case PHIDUID_1055_1_USB:
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetIRDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetIRDeviceHandle phid = (PhidgetIRDeviceHandle)device;
	PhidgetChannelHandle channel;
	PhidgetReturnCode res;

	assert(phid);
	assert(buffer);

	res = EPHIDGET_OK;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1055:
	case PHIDUID_1055_1_USB:
		if ((channel = getChannel(device, 0)) != NULL) {
			res = PhidgetIRSupport_dataInput(channel, buffer, length);
			PhidgetRelease(&channel);
		}
		return (res);

	default:
		MOS_PANIC("Unexpected device");
	}
}

static PhidgetReturnCode CCONV
PhidgetIRDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetIRDeviceHandle phid;
	phid = (PhidgetIRDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_IR);
	assert(ch->class == PHIDCHCLASS_IR);
	assert(ch->index == 0);

	switch (bp->vpkt) {
	case BP_TRANSMIT:
	case BP_TRANSMITRAW:
	case BP_TRANSMITREPEAT:
		return (PhidgetIRSupport_bridgeInput(ch, bp));

	case BP_OPENRESET:
	case BP_CLOSERESET:
		switch (phid->phid.deviceInfo.UDD->uid) {
		case PHIDUID_1055:
		case PHIDUID_1055_1_USB:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected device");
		}
	case BP_ENABLE:
		return (EPHIDGET_OK);

	default:
		MOS_PANIC("Unexpected packet type");
	}
}

static void CCONV
PhidgetIRDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetIRDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetIRDevice_create(PhidgetIRDeviceHandle *phidp) {
	DEVICECREATE_BODY(IRDevice, PHIDCLASS_IR);
	return (EPHIDGET_OK);
}
