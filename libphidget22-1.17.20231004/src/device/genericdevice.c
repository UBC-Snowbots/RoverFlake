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
#include "device/genericdevice.h"

// === Internal Functions === //

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetGenericDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetGenericDeviceHandle phid = (PhidgetGenericDeviceHandle)device;
	PhidgetHIDUSBConnectionHandle hidusbConn;
	PhidgetPHIDUSBConnectionHandle phidusbConn;

	assert(phid);

	phid->OUTPacketLength[0] = (uint32_t)getMaxOutPacketSize((PhidgetDeviceHandle)phid);

	switch (device->connType) {
	case PHIDCONN_SPI:
		phid->INPacketLength[0] = MAX_SPI_PACKET_SIZE;
		break;
	case PHIDCONN_HIDUSB:
		hidusbConn = PhidgetHIDUSBConnectionCast(device->conn);
		assert(hidusbConn);
		phid->INPacketLength[0] = hidusbConn->inputReportByteLength;
		break;
	case PHIDCONN_PHIDUSB:
		phidusbConn = PhidgetPHIDUSBConnectionCast(device->conn);
		assert(phidusbConn);
		// XXX - not really the packet size, but the max size
		phid->INPacketLength[0] = phidusbConn->pusbParams.maxPacketEP1;
		phid->CTRPacketLength[0] = phidusbConn->pusbParams.maxPacketEP0;
		break;
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetGenericDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetGenericDeviceHandle phid = (PhidgetGenericDeviceHandle)device;
	PhidgetChannelHandle channel;

	assert(phid);
	assert(buffer);

	if ((channel = getChannel(phid, 0)) != NULL) {
		bridgeSendToChannel(channel, BP_PACKET, 1, "%*R", (int)length, buffer);
		PhidgetRelease(&channel);
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetGenericDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetGenericDeviceHandle phid = (PhidgetGenericDeviceHandle)ch->parent;
#if (PHIDUID_GENERIC_PHIDUSB_SUPPORTED)
	uint8_t buf[MAX_OUT_PACKET_SIZE];
	PhidgetReturnCode ret;
	size_t len;
#endif

	assert(phid->phid.deviceInfo.class == PHIDCLASS_GENERIC);
	assert(ch->class == PHIDCHCLASS_GENERIC);

	switch (bp->vpkt) {
	case BP_SENDPACKET:
		switch (phid->phid.deviceInfo.UDD->uid) {

#if PHIDUID_GENERIC_HIDUSB_SUPPORTED
		case PHIDUID_GENERIC_HIDUSB:
			if (getBridgePacketArrayLen(bp, 0) != (int)phid->OUTPacketLength[0])
				return (EPHIDGET_INVALIDARG);
			return (PhidgetDevice_sendpacket(bp->iop, (PhidgetDeviceHandle)phid, getBridgePacketUInt8Array(bp, 0), getBridgePacketArrayLen(bp, 0)));
#endif /* PHIDUID_GENERIC_HIDUSB_SUPPORTED */

#if PHIDUID_GENERIC_PHIDUSB_SUPPORTED
		case PHIDUID_GENERIC_PHIDUSB:
			len = getBridgePacketArrayLen(bp, 0);

			if (len > phid->OUTPacketLength[0])
				return (EPHIDGET_INVALIDARG);

			return (PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_BULK_WRITE, 0, 0,
				(uint8_t *)(uintptr_t)getBridgePacketUInt8Array(bp, 0), &len, DEFAULT_TRANSFER_TIMEOUT));
#endif /* PHIDUID_GENERIC_PHIDUSB_SUPPORTED */

		default:
			MOS_PANIC("Unexpected device");
		}

	case BP_SENDCHPACKET:
		switch (phid->phid.deviceInfo.UDD->uid) {
#if PHIDUID_GENERIC_PHIDUSB_SUPPORTED
		case PHIDUID_GENERIC_PHIDUSB:
			len = getBridgePacketArrayLen(bp, 2);

			if (len > phid->CTRPacketLength[0])
				return (EPHIDGET_INVALIDARG);

			return (PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE,
				getBridgePacketInt32(bp, 0), getBridgePacketInt32(bp, 1),
				(uint8_t *)(uintptr_t)getBridgePacketUInt8Array(bp, 2), &len, DEFAULT_TRANSFER_TIMEOUT));
#endif /* PHIDUID_GENERIC_PHIDUSB_SUPPORTED */
		default:
			MOS_PANIC("Unexpected device");
		}

	case BP_SENDDEVPACKET:
		switch (phid->phid.deviceInfo.UDD->uid) {
#if PHIDUID_GENERIC_PHIDUSB_SUPPORTED
		case PHIDUID_GENERIC_PHIDUSB:
			len = getBridgePacketArrayLen(bp, 1);

			if (len > phid->CTRPacketLength[0])
				return (EPHIDGET_INVALIDARG);

			return (PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
				getBridgePacketInt32(bp, 0), 0,
				(uint8_t *)(uintptr_t)getBridgePacketUInt8Array(bp, 1), &len, 2500));
#endif /* PHIDUID_GENERIC_PHIDUSB_SUPPORTED */
		default:
			MOS_PANIC("Unexpected device");
		}

	case BP_READCHPACKET:
		switch (phid->phid.deviceInfo.UDD->uid) {
#if PHIDUID_GENERIC_PHIDUSB_SUPPORTED
		case PHIDUID_GENERIC_PHIDUSB:
			len = getBridgePacketInt32(bp, 2);

			if (len > phid->CTRPacketLength[0])
				len = phid->CTRPacketLength[0];

			ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_READ,
				getBridgePacketInt32(bp, 0), getBridgePacketInt32(bp, 1),
				buf, &len, DEFAULT_TRANSFER_TIMEOUT);

			if (ret == EPHIDGET_OK) {
				bp->reply_bpe = mos_malloc(sizeof(BridgePacketEntry));
				memset(bp->reply_bpe, 0, sizeof(BridgePacketEntry));

				bp->reply_bpe->type = BPE_UI8ARRAY;
				bp->reply_bpe->bpe_len = (uint16_t)len;
				if (len == 0)
					bp->reply_bpe->bpe_ptr = NULL;
				else
					bp->reply_bpe->bpe_ptr = mos_malloc(len);
				bp->reply_bpe->bpe_ui8array = bp->reply_bpe->bpe_ptr;
				bp->reply_bpe->bpe_cnt = (uint16_t)len;

				memcpy(bp->reply_bpe->bpe_ui8array, buf, len);
			}

			return (ret);
#endif /* PHIDUID_GENERIC_PHIDUSB_SUPPORTED */

		default:
			MOS_PANIC("Unexpected device");
		}

	case BP_READDEVPACKET:
		switch (phid->phid.deviceInfo.UDD->uid) {
#if PHIDUID_GENERIC_PHIDUSB_SUPPORTED
		case PHIDUID_GENERIC_PHIDUSB:
			len = getBridgePacketInt32(bp, 1);

			if (len > phid->CTRPacketLength[0])
				len = phid->CTRPacketLength[0];

			ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_READ,
				getBridgePacketInt32(bp, 0), 0,
				buf, &len, DEFAULT_TRANSFER_TIMEOUT);

			if (ret == EPHIDGET_OK) {
				bp->reply_bpe = mos_malloc(sizeof(BridgePacketEntry));
				memset(bp->reply_bpe, 0, sizeof(BridgePacketEntry));

				bp->reply_bpe->type = BPE_UI8ARRAY;
				bp->reply_bpe->bpe_len = (uint16_t)len;
				if (len == 0)
					bp->reply_bpe->bpe_ptr = NULL;
				else
					bp->reply_bpe->bpe_ptr = mos_malloc(len);
				bp->reply_bpe->bpe_ui8array = bp->reply_bpe->bpe_ptr;
				bp->reply_bpe->bpe_cnt = (uint16_t)len;

				memcpy(bp->reply_bpe->bpe_ui8array, buf, len);
			}

			return (ret);
#endif /* PHIDUID_GENERIC_PHIDUSB_SUPPORTED */

		default:
			MOS_PANIC("Unexpected device");
		}

	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

static void CCONV
PhidgetGenericDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetGenericDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetGenericDevice_create(PhidgetGenericDeviceHandle *phidp) {
	DEVICECREATE_BODY(GenericDevice, PHIDCLASS_GENERIC);
	return (EPHIDGET_OK);
}
