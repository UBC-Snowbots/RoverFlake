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

#ifndef __CPHIDGETMESHDONGLEDEVICE
#define __CPHIDGETMESHDONGLEDEVICE

typedef struct _PhidgetMeshDongleDevice *PhidgetMeshDongleDeviceHandle;
PhidgetReturnCode PhidgetMeshDongleDevice_create(PhidgetMeshDongleDeviceHandle *phid);

PhidgetReturnCode openMeshDongleDeviceDongle(PhidgetMeshDongleDeviceHandle managerMeshDongleDevice, PhidgetMeshDongleDeviceHandle *openedMeshDongleDevice);
PhidgetReturnCode PhidgetMeshDongleDevice_makePacket(
	PhidgetMeshDongleDeviceHandle		phid,
	PhidgetDeviceHandle			meshDevice,
	int							packetID,
	const uint8_t			*bufferIn,
	size_t						bufferInLen,
	uint8_t				*buffer,
	size_t						*bufferLen);

void PhidgetMeshDongleDevice_releasePacketSpace(PhidgetMeshDongleDeviceHandle phid, int hubIndex, size_t packetSize);
PhidgetReturnCode PhidgetMeshDongleDevice_claimPacketSpace(PhidgetMeshDongleDeviceHandle phid, PhidgetDeviceHandle meshDevice, size_t packetSize);

typedef enum {
	MESHDONGLE_PACKET_PACKETSTATUS = 0x00,
	MESHDONGLE_PACKET_GETTXBUFFERSTATUS = 0x01,
	MESHDONGLE_PACKET_TXBUFFERSTATUS = 0x02,
} PhidgetMeshDongle_PacketType;

#define VINTMESH_INPACKET_DONGLE_INDEX 0x7F
#define VINTMESH_OUTPACKET_DONGLE_INDEX 0x7FFFFFFF

#define MESH_PACKETRETURN_notACK		0x80

#define VINTHUB_MAX_IN_PACKET_SIZE 128

#define MESHDONGLE_MAXHUBS	50

struct _PhidgetMeshDongleDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.mesh
	PhidgetDevice phid;

	mos_mutex_t outstandingPacketCntLock[MESHDONGLE_MAXHUBS]; /* protects outstandingPacketCnt */
	mos_cond_t outstandingPacketCntCond[MESHDONGLE_MAXHUBS];
	size_t outstandingPacketCnt[MESHDONGLE_MAXHUBS];
	BOOL outstandingPacketCntValid;

	size_t internalPacketInBufferLen;

	size_t packetCnt;
	uint8_t packetBuf[VINTHUB_MAX_IN_PACKET_SIZE];
	int packetBufWritePtr;
} typedef PhidgetMeshDongleDeviceInfo;

#endif
