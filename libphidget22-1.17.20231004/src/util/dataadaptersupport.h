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

#ifndef __DATAADAPTERSUPPORT
#define __DATAADAPTERSUPPORT



#define DATAADAPTER_MAX_PACKET_LENGTH 8192
#define USB_OUT_PACKET_LENGTH 64

#define USB_OUT_PACKET_OVERHEAD 5
#define USB_IN_PACKET_OVERHEAD 5
#define NEW_PACKET_FLAG 0x8000
#define WAIT_RESP_FLAG 0x4000

#define NEW_RX_READY 0xFFFF
#define NO_ACTIVE_PACKET 0xFFFF
#define ANONYMOUS_PACKET_ID 0x0000

#define DATAADAPTER_MAX_EOL_LENGTH 8

PhidgetReturnCode sendData(PhidgetChannelHandle ch, BridgePacket* bp, int waitResponse);
PhidgetReturnCode sendI2CData(PhidgetChannelHandle ch, BridgePacket* bp, int waitResponse);
PhidgetReturnCode sendDataBuffer(PhidgetChannelHandle ch, size_t len, const uint8_t *buffer, BridgePacket* bp, int waitResponse);


PhidgetReturnCode parseI2CFormat(PhidgetChannelHandle ch, const char *string);

typedef enum DataAdapter_TXDroppedReason {
	TX_DROPPED_UNKNOWN = 0,
	TX_DROPPED_TIMEOUT = 1,
	TX_DROPPED_CORRUPT = 2,
	TX_DROPPED_BUSY = 3,
	TX_DROPPED_NOT_CONFIGURED = 4
} DataAdapter_TXDroppedReason;

typedef struct {

	/* Public Members */
	double baudRate;
	uint8_t lastData[DATAADAPTER_MAX_PACKET_LENGTH];
	size_t lastDataLength;


	/* Private Members */
	uint16_t usbInPacketCount;
	uint32_t packetID;
	uint16_t rxPacketID;

	uint16_t ackID;

	uint16_t rxPacketError;

	mos_mutex_t sendLock;
	mos_mutex_t receiveLock;

	uint16_t droppedPacketID;
	DataAdapter_TXDroppedReason droppedPacketReason;

	uint8_t nakFlag;

	uint8_t storedPacket[8192];
	size_t storedPacketLength;

	PhidgetDataAdapter_Protocol protocol;
	PhidgetDataAdapter_HandshakeMode handshakeMode;
	uint8_t i2cFormatList[128];
	char i2cFormatCount;
	uint32_t address;
	uint16_t txTimeout;

} PhidgetDataAdapterSupport, *PhidgetDataAdapterSupportHandle;

void PhidgetDataAdapterSupport_free(PhidgetDataAdapterSupportHandle *arg);
PhidgetReturnCode PhidgetDataAdapterSupport_create(PhidgetDataAdapterSupportHandle *ir);
void PhidgetDataAdapterSupport_init(PhidgetDataAdapterSupportHandle ir);
PhidgetReturnCode PhidgetDataAdapterSupport_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp);
PhidgetReturnCode PhidgetDataAdapterSupport_dataInput(PhidgetChannelHandle ch, const uint8_t *buf, size_t len);

#endif
