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

#ifndef __CPHIDGETHUBDEVICE
#define __CPHIDGETHUBDEVICE

typedef struct _PhidgetHubDevice *PhidgetHubDeviceHandle;
PhidgetReturnCode PhidgetHubDevice_create(PhidgetHubDeviceHandle *phid);

//USB Out packet types
// 3-bit (Mask: 0x70)
typedef enum {
	VINTHUB_PACKET_HUB = 0x40,
	VINTHUB_PACKET_DEVICE = 0x20
} PhidgetHubDevice_PacketType;

//Hub configuration messages
// 4-bit (Mask: 0x0F)
typedef enum {
	VINTHUB_HUBPACKET_SETPORTMODE = 0x00,
	VINTHUB_HUBPACKET_UPGRADE_FIRMWARE = 0x01,
	VINTHUB_HUBPACKET_GETTXBUFFERSTATUS = 0x02,
	VINTHUB_HUBPACKET_PORTPOWER = 0x03,
	VINTHUB_HUBPACKET_MESHMODE = 0x04,
	VINTHUB_HUBPACKET_CALIBRATION_MODE	= 0x05,
	VINTHUB_HUBPACKET_CALIBRATION_WRITE	= 0x06,
	VINTHUB_HUBPACKET_CALIBRATION_EXIT	= 0x07,
	VINTHUB_HUBPACKET_PORTAUTOSETSPEED = 0x08,
} PhidgetHubDevice_HubPacketType;

PhidgetReturnCode PhidgetHubDevice_setPortMode(mosiop_t iop, PhidgetHubDeviceHandle phid, int index, PhidgetHub_PortMode mode);
PhidgetReturnCode PhidgetHubDevice_sendpacket(mosiop_t iop, PhidgetHubDeviceHandle phid, const uint8_t *buf);
PhidgetReturnCode sendHubPacket(
	mosiop_t iop,
	PhidgetHubDeviceHandle hub,
	PhidgetHubDevice_HubPacketType hubPacketType,
	uint8_t *bufferIn,
	size_t bufferInLen);
PhidgetReturnCode sendHubPortPacket(
	mosiop_t iop,
	PhidgetHubDeviceHandle hub,
	int hubPort,
	PhidgetHubDevice_HubPacketType hubPacketType,
	uint8_t *bufferIn,
	size_t bufferInLen);
PhidgetReturnCode PhidgetHubDevice_makePacket(
	PhidgetHubDeviceHandle		phid,
	PhidgetDeviceHandle			vintDevice,
	int							packetID,
	const uint8_t			*bufferIn,
	size_t						bufferInLen,
	uint8_t				*buffer,
	size_t						*bufferLen);
PhidgetReturnCode PhidgetHubDevice_claimPacketSpace(PhidgetHubDeviceHandle hub, int hubPort, size_t packetSize);
void PhidgetHubDevice_releasePacketSpace(PhidgetHubDeviceHandle phid, int hubPort, size_t packetSize);
PhidgetReturnCode PhidgetHubDevice_updatePortProperties(PhidgetHubDeviceHandle hub, int port);

#define HUB_PORT_ID_MAX				0x0F

#define VINTHUB_MAXPORTS 6
#define VINTHUB_MAXCHANNELS 32

#define VINTHUB_INPACKET_HUBMSG_FLAG 0x80

typedef enum {
	VINTHUB_HUBINPACKET_TXBUFFERSTATUS = 0x00,
	VINTHUB_HUBINPACKET_OVERCURRENT = 0x01,
	VINTHUB_HUBINPACKET_DETACH = 0x02,
	VINTHUB_HUBINPACKET_DISABLE = 0x03,
	VINTHUB_HUBINPACKET_SPEEDCHANGE = 0x04,
	VINTHUB_HUBINPACKET_REENUMERATION = 0x05
} PhidgetHubDevice_HubInPacketType;

#define VINTHUB_PACKETRETURN_notACK	0x80

#define HUB0001_CALIBRATION_UNLOCK_KEY			0x8ef90234
#define HUB0000_HUB0002_CALIBRATION_UNLOCK_KEY	0x8ef95566
#define HUB0007_CALIBRATION_UNLOCK_KEY			0x8ef99873

//Flag for identifying the start of a VINT packet in the USB packet stream
#define VINTHUB_IN_VINTPACKET_START		0x08

#define VINTHUB_PACKETID_MAX		126
#define VINTHUB_PACKETIDS_PER_PORT	(VINTHUB_PACKETID_MAX/6)

//Calibration table defines
#define VINTHUB_ADC_CALIB_TABLE_INDEX	0
#define VINTHUB_ADCCalibTable_ID 		1004
#define VINTHUB_ADCCalibTable_LENGTH 	28

struct _PhidgetHubDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.hub
	PhidgetDevice phid;

	size_t outstandingPacketCnt[VINTHUB_MAXPORTS];
	BOOL outstandingPacketCntValid;

	size_t internalPacketInBufferLen;

	int packetCounter;
	int packetOutCounter[VINTHUB_MAXPORTS];

	uint8_t splitPacketStorage[54];
	int splitPacketStoragePtr;

	//This stuff is for the Mesh Hub
	//6 chars per port, up to 4 portModes, plus ending NULL
	char portDescString[VINTHUB_MAXPORTS * 6 + 4 * 6 + 1];

	// Props that are available at attach - before open, read via descriptors

	// NOTE: These 3 MUST remain as 32-bit because of how they are send to JSON in the hubPortsInfo field - it expects uint32_t arrays
	uint32_t portProtocolVersion[VINTHUB_MAXPORTS];
	uint32_t portSupportsSetSpeed[VINTHUB_MAXPORTS];
	uint32_t portMaxSpeed[VINTHUB_MAXPORTS];

	uint32_t portSupportsAutoSetSpeed[VINTHUB_MAXPORTS];
	uint32_t portSpeed[VINTHUB_MAXPORTS];
	PhidgetHub_PortMode portMode[VINTHUB_MAXPORTS];
	uint8_t portPowered[VINTHUB_MAXPORTS];

} typedef PhidgetHubDeviceInfo;

#endif
