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
#include "device/vintdevice.h"
#include "device/hubdevice.h"

/*
 * initAfterOpen
 * sets up the initial state of an object, reading in packets from the device if needed
 * used during attach initialization - on every attach
 */
static PhidgetReturnCode CCONV
PhidgetVINTDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetVINTDeviceHandle phid;

	assert(device);

	phid = (PhidgetVINTDeviceHandle)device;

	// Set board-level props to unknown
	phid->heatingEnabled = PUNK_BOOL;
	phid->powerSupply = PUNK_ENUM;
	phid->inputMode = PUNK_ENUM;
	phid->RTDWireSetup = PUNK_ENUM;

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetVINTDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetChannelHandle vintChannel;
	PhidgetReturnCode res;
	int channelIndex;
	int dataCount;
	int readPtr;

	TESTPTR(device);

	// Data Length and Channel
	if (buffer[0] & VINT_DATA_wCHANNEL) {
		dataCount = (buffer[0] & 0x3F) - 1;
		channelIndex = buffer[1];
		readPtr = 2;
	} else {
		dataCount = buffer[0] & 0x3F;
		channelIndex = 0;
		readPtr = 1;
	}

	// Length needs to be at least 1 (packet type)
	if (dataCount < 1) {
		vintlogerr("Got an invalid data length in a vint message");
		return (EPHIDGET_UNEXPECTED);
	}

	vintChannel = getAttachedChannel(device, channelIndex);
	if (vintChannel == NULL) {
		vintlogverbose("Dropping VINT Packet addressed to closed channel - probably channel was not closed properly previously.");
		return (EPHIDGET_OK);
	}

	if (PhidgetCKFlags(vintChannel, PHIDGET_INITIALIZED_FLAG) == 0) {
		vintlogverbose("Dropping VINT Packet recieved before channel is initialized - probably channel was not closed properly previously.");
		return (EPHIDGET_OK);
	}

	vintlogverbose("Received VINT Packet\nChannel: %"PRIphid"\nPacket Type: %s (0x%02x)\nPacket: "LOGBUFFER_STR,
		vintChannel, Phidget_strVINTPacketType(buffer[readPtr]), buffer[readPtr], LOGBUFFER_ARGS(dataCount - 1, buffer + readPtr + 1));


	res = device->vintIO->recv(vintChannel, buffer + readPtr, dataCount);
	PhidgetRelease(&vintChannel);
	return (res);
}

static PhidgetReturnCode CCONV
PhidgetVINTDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetReturnCode res;

	assert(ch);
	assert(ch->parent);
	assert(ch->parent->parent);

	switch (bp->vpkt) {
	case BP_CLOSERESET:
		// RESET the device channel
		res = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, NULL, 0);
		if (res != EPHIDGET_OK) {
			vintlogerr("Failed to send VINT_PACKET_TYPE_PHIDGET_RESET message: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}

		// Reset the port mode
		res = PhidgetHubDevice_setPortMode(bp->iop, (PhidgetHubDeviceHandle)ch->parent->parent,
		  ch->parent->deviceInfo.hubPort, PORT_MODE_VINT_PORT);
		if (res != EPHIDGET_OK) {
			vintlogerr("Setting Hub Port mode failed: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}

		// Wait for any pending packets.
		waitForPendingPackets(ch->parent->parent, ch->parent->deviceInfo.hubPort);

		return (EPHIDGET_OK);

	case BP_OPENRESET:
		res = PhidgetHubDevice_setPortMode(bp->iop, (PhidgetHubDeviceHandle)ch->parent->parent,
		  ch->parent->deviceInfo.hubPort, ch->openInfo->hubPortMode);
		if (res != EPHIDGET_OK) {
			vintlogerr("Setting Hub Port mode failed: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}

		if (ch->openInfo->openCloseReset) {
			res = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, NULL, 0);
			if (res != EPHIDGET_OK) {
				vintlogerr("Failed to send VINT_PACKET_TYPE_PHIDGET_RESET message: "PRC_FMT, PRC_ARGS(res));
				return (res);
			}
		}

		return (EPHIDGET_OK);

	case BP_ENABLE:
		res = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_ENABLE, NULL, 0);
		if (res != EPHIDGET_OK) {
			vintlogerr("Failed to send VINT_PACKET_TYPE_PHIDGET_ENABLE message: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}
		return (EPHIDGET_OK);

	default:
		MOS_ASSERT(ch->parent->vintIO != NULL);
		return (ch->parent->vintIO->send(ch, bp));
	}
}

PhidgetReturnCode
PhidgetVINTDevice_makePacket(PhidgetVINTDeviceHandle vintDevice, PhidgetChannelHandle vintChannel,
  VINTDeviceCommand deviceCommand, VINTPacketType devicePacketType, const uint8_t *bufferIn,
  size_t bufferInLen, uint8_t *buffer, size_t *bufferLen) {
	size_t bufIndex;

	assert(vintDevice);
	assert(vintChannel);
	assert(!bufferInLen || bufferIn);
	assert(buffer);
	assert(bufferLen);
	assert(*bufferLen >= getMaxOutPacketSize((PhidgetDeviceHandle)vintDevice));

	bufIndex = 0;

	switch (deviceCommand) {
	case VINT_CMD_DATA:
		assert(bufferInLen <= VINT_MAX_OUT_PACKETSIZE);
		if (vintChannel->uniqueIndex) {
			buffer[bufIndex++] = VINT_DATA_wCHANNEL | ((uint8_t)bufferInLen + 2);
			buffer[bufIndex++] = vintChannel->uniqueIndex;
			buffer[bufIndex++] = devicePacketType;
		} else {
			buffer[bufIndex++] = (uint8_t)bufferInLen + 1;
			buffer[bufIndex++] = devicePacketType;
		}
		if (bufferIn) {
			memcpy(buffer + bufIndex, bufferIn, bufferInLen);
			bufIndex += bufferInLen;
		}
		break;

	// Supported data-less commands
	case VINT_CMD_RESET:
	case VINT_CMD_UPGRADE_FIRMWARE:
	case VINT_CMD_FIRMWARE_UPGRADE_DONE:
		buffer[bufIndex++] = deviceCommand;
		break;

	// Supported data-containing commands
	case VINT_CMD_SETSPEED2:
		assert(bufferInLen == 4);
		buffer[bufIndex++] = deviceCommand;
		memcpy(buffer + bufIndex, bufferIn, bufferInLen);
		bufIndex += bufferInLen;
		break;

	default:
		MOS_PANIC("Unexpected packet type");
	}

	*bufferLen = bufIndex;
	assert(*bufferLen <= getMaxOutPacketSize((PhidgetDeviceHandle)vintDevice));

	return (EPHIDGET_OK);
}

static void CCONV
PhidgetVINTDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetVINTDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetVINTDevice_create(PhidgetVINTDeviceHandle *phidp) {

	DEVICECREATE_BODY(VINTDevice, PHIDCLASS_VINT);

	phid->portProtocolVersion = 1;
	phid->deviceProtocolVersion = PUNK_UINT8;

	// NOTE: Default to assuming false
	phid->portSupportsSetSpeed = PFALSE;
	phid->portSupportsAutoSetSpeed = PFALSE;
	phid->portMaxSpeed = PUNK_UINT32;

	// NOTE: Default to assuming false
	phid->deviceSupportsSetSpeed = PFALSE;
	phid->deviceSupportsAutoSetSpeed = PFALSE;
	phid->deviceMaxSpeed = PUNK_UINT32;

	phid->vintCommSpeed = PUNK_UINT32;

	return (EPHIDGET_OK);
}
