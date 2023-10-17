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
#include "device/meshdongledevice.h"

// === Internal Functions === //

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetMeshDongleDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetMeshDongleDeviceHandle phid = (PhidgetMeshDongleDeviceHandle)device;
	uint8_t buffer[64] = { 0 };
	PhidgetReturnCode result;
	int readtries;
	int i;
	assert(phid);

	//set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numHubs; i++)
		phid->outstandingPacketCnt[i] = PUNK_SIZE;

	phid->packetCnt = PUNK_SIZE;
	phid->packetBufWritePtr = 0;
	phid->outstandingPacketCntValid = 0;
	phid->internalPacketInBufferLen = 128;

	pack32(buffer, VINTMESH_OUTPACKET_DONGLE_INDEX);
	buffer[4] = MESHDONGLE_PACKET_GETTXBUFFERSTATUS;
	//The hub sendpacket requires inputlock to be unlocked because it waits for the packet status to come back
	result = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 5);
	if (result != EPHIDGET_OK)
		return result;

	readtries = 16;
	while (readtries-- > 0 && !phid->outstandingPacketCntValid)
		waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	if (!phid->outstandingPacketCntValid)
		logerr("Unable to recover TX buffer free space values. Continuing anyways.");

	return (EPHIDGET_OK);
}

#if PHIDUID_MESHDONGLE_SUPPORTED
static void
readInTXBufferCounts(PhidgetMeshDongleDeviceHandle phid) {
	PhidgetReturnCode ret;
	uint8_t buffer[5] = { 0 };
	int i;

	for (i = 0; i < phid->devChannelCnts.numHubs; i++) {
		mos_mutex_lock(&(phid->outstandingPacketCntLock[i]));
		phid->outstandingPacketCnt[i] = PUNK_SIZE;
		mos_mutex_unlock(&(phid->outstandingPacketCntLock[i]));
	}

	//Request a new count
	pack32(buffer, VINTMESH_OUTPACKET_DONGLE_INDEX);
	buffer[4] = MESHDONGLE_PACKET_GETTXBUFFERSTATUS;


	ret = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 5);
	if (ret != EPHIDGET_OK)
		logerr("Error sending MESHDONGLE_PACKET_GETTXBUFFERSTATUS msg to Hub after a packet loss.");
}
#endif

#if PHIDUID_MESHDONGLE_SUPPORTED
static PhidgetReturnCode
processPacket(PhidgetMeshDongleDeviceHandle phid) {
	PhidgetReturnCode ret = EPHIDGET_OK;
	int index = phid->packetBuf[1];
	PhidgetDeviceHandle device;
	int i;

	if (index == VINTMESH_INPACKET_DONGLE_INDEX) {
		switch (phid->packetBuf[2]) {
		case MESHDONGLE_PACKET_PACKETSTATUS:
			// This means the dongle couldn't deliver a vint device packet to a hub
			break;
		case MESHDONGLE_PACKET_TXBUFFERSTATUS:
			for (i = 0; i < phid->devChannelCnts.numHubs; i++) {
				mos_mutex_lock(&(phid->outstandingPacketCntLock[i]));
				// -1 because the 128 byte buffer can only actually hold 127 bytes or the read/write pointers would overlap
				phid->outstandingPacketCnt[i] = phid->internalPacketInBufferLen - 1 - phid->packetBuf[i + 3];
				mos_cond_broadcast(&phid->outstandingPacketCntCond[i]);
				mos_mutex_unlock(&phid->outstandingPacketCntLock[i]);
			}
			phid->outstandingPacketCntValid = 1;
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}
		//TODO
	} else {
		assert(index < PHIDGET_MAXCHILDREN);
		assert(phid->packetBuf[0] <= MAX_IN_PACKET_SIZE);

		device = getChild((PhidgetDeviceHandle)phid, index);
		if (device != NULL) {
			if (ISOPEN(device))
				/*
				 * Strip the length and device index byte and pass along to the device API
				 * XXX This used to queue the data..
				 */
				device->dataInput(device, phid->packetBuf + 2, phid->packetBuf[0] - 2);
			else
				logverbose("Discarding MESH data packet on not opened device");
			PhidgetRelease(&device);
		} else {
			logwarn("Got data for a null hub index.");
		}
	}

	return (ret);
}
#endif

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetMeshDongleDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetMeshDongleDeviceHandle phid = (PhidgetMeshDongleDeviceHandle)device;
	PhidgetReturnCode ret;
#if PHIDUID_MESHDONGLE_SUPPORTED
	PhidgetPacketTrackerHandle packetTracker;
	VINTPacketStatusCode response;
	int killOutstandingPackets;
	uint32_t packetCounter;
	PhidgetReturnCode res;
	int packetReturnLen;
	int dataEndIndex;
	int packetID;
	int readPtr;
	int i;
#endif

	assert(phid);
	assert(buffer);

	ret = EPHIDGET_OK;

	switch (phid->phid.deviceInfo.UDD->uid) {
#if PHIDUID_MESHDONGLE_SUPPORTED
	case PHIDUID_MESHDONGLE:
		readPtr = 0;
		packetCounter = (buffer[0] >> 4) & 0x07;
		packetReturnLen = buffer[0] & 0x0F;
		dataEndIndex = (((int)length) - packetReturnLen);
		killOutstandingPackets = PFALSE;

		//Check for a missed packet, setup pointers, etc.
		if (phid->packetCnt != PUNK_SIZE) {
			phid->packetCnt = (phid->packetCnt + 1) & 0x07;
			if (phid->packetCnt != packetCounter) {
				logwarn("Lost a mesh dongle USB IN packet.");
				killOutstandingPackets = PTRUE;
				phid->packetCnt = packetCounter;
				phid->packetBufWritePtr = 0;
				readPtr = buffer[1];
			}
		} else {
			phid->packetCnt = packetCounter;
			phid->packetBufWritePtr = 0;
			readPtr = buffer[1]; //next packet start
		}

		//next packet start is 0-indexed but data start as index 2
		readPtr += 2;

		//Copy data into internal buffer
		while (readPtr < dataEndIndex) {
			phid->packetBuf[phid->packetBufWritePtr++] = buffer[readPtr];

			//Detect end of data (not fully used USB packet)
			if (phid->packetBuf[0] == 0) {
				phid->packetBufWritePtr = 0;
				break;
			}

			//1st byte in the buffer is the length
			if (phid->packetBufWritePtr == phid->packetBuf[0]) {
				//If we have a complete packet, pass onto the next system
				ret = processPacket(phid);
				phid->packetBufWritePtr = 0;
			}

			readPtr++;
		}

		readPtr = dataEndIndex;
		while (readPtr < (int)length) {
			//Find this in the list
			packetID = buffer[readPtr] & 0x7F;
			response = VINTPacketStatusCode_ACK;

			if (buffer[readPtr] & MESH_PACKETRETURN_notACK) {
				readPtr++;
				response = (VINTPacketStatusCode)buffer[readPtr];
				loginfo("Got an error back from a mesh dongle: 0x%02x, "PRC_FMT, packetID, response, Phidget_strVINTPacketStatusCode(response));
			}
			readPtr++;

			packetTracker = &phid->phid.packetTracking->packetTracker[packetID];

			res = VINTPacketStatusCode_to_PhidgetReturnCode(response, packetTracker->udd);
			if (setPacketReturnCode(packetTracker, res) != EPHIDGET_OK) {
				//Don't display if it's NOTATTACHED - since this is pretty common/expected.
				if (response != VINTPacketStatusCode_NOTATTACHED)
					loginfo("An unexpected PacketID was returned: %d ("PRC_FMT"). "
						"Probably this packet is from a previous session or detached device.",
						packetID, response, Phidget_strVINTPacketStatusCode(response));

				//Request a new count because our count will now be out
				readInTXBufferCounts(phid);
				continue;
			}

			PhidgetMeshDongleDevice_releasePacketSpace(phid, packetTracker->childIndex, packetTracker->len);
		}

		if (killOutstandingPackets) {
			loginfo("Killing outstanding packets on mesh dongle");
			//Mark the TX buffer as unknown size if there were any outstanding packets (because we may have missed the racket return)
			for (i = 0; i < phid->devChannelCnts.numHubs; i++) {
				mos_mutex_lock(&(phid->outstandingPacketCntLock[i]));
				if (phid->outstandingPacketCnt[i])
					phid->outstandingPacketCnt[i] = PUNK_SIZE;
				mos_mutex_unlock(&(phid->outstandingPacketCntLock[i]));
				setPacketsReturnCode((PhidgetDeviceHandle)phid, i, EPHIDGET_INTERRUPTED);
			}
		}

		break;
#endif /* PHIDUID_MESHDONGLE_SUPPORTED */
	default:
		MOS_PANIC("Unexpected device");
	}

	return (ret);
}

static PhidgetReturnCode CCONV
PhidgetMeshDongleDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {

	assert(((PhidgetMeshDongleDeviceHandle)ch->parent)->phid.deviceInfo.class == PHIDCLASS_MESHDONGLE);
	assert(ch->class == PHIDCHCLASS_MESHDONGLE);
	assert(ch->index == 0);

	switch (bp->vpkt) {
	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

void
PhidgetMeshDongleDevice_releasePacketSpace(PhidgetMeshDongleDeviceHandle phid, int hubIndex, size_t packetSize) {

	mos_mutex_lock(&phid->outstandingPacketCntLock[hubIndex]);
	if (phid->outstandingPacketCnt[hubIndex] != PUNK_SIZE) {
		phid->outstandingPacketCnt[hubIndex] -= packetSize;
		logverbose("MESH Releasing %d bytes, %d remaining, Hub %d", (int)packetSize,
			(int)(phid->internalPacketInBufferLen - phid->outstandingPacketCnt[hubIndex]), hubIndex);
		mos_cond_broadcast(&phid->outstandingPacketCntCond[hubIndex]);
	}
	mos_mutex_unlock(&phid->outstandingPacketCntLock[hubIndex]);
}

PhidgetReturnCode
PhidgetMeshDongleDevice_claimPacketSpace(PhidgetMeshDongleDeviceHandle phid, PhidgetDeviceHandle meshDevice, size_t packetSize) {
	PhidgetReturnCode res;
	int hubIndex;

	TESTPTR(phid);
	TESTATTACHED(phid);

	hubIndex = meshDevice->deviceInfo.uniqueIndex;

	//make sure there is room for this on the dongle, so we don't just get a NO_SPACE response

	mos_mutex_lock(&phid->outstandingPacketCntLock[hubIndex]);
	for (;;) {

		if (phid->outstandingPacketCnt[hubIndex] != PUNK_SIZE &&
		  (phid->outstandingPacketCnt[hubIndex] + packetSize) < phid->internalPacketInBufferLen)
			break;

		res = mos_cond_timedwait(&phid->outstandingPacketCntCond[hubIndex],
		  &phid->outstandingPacketCntLock[hubIndex], 2 * MOS_SEC);
		if (res != 0) {
			mos_mutex_unlock(&phid->outstandingPacketCntLock[hubIndex]);
			return (res);
		}
	}

	phid->outstandingPacketCnt[hubIndex] += packetSize;
	logverbose("Claiming %d bytes, %d remaining, Port %d", (int)packetSize,
		(int)(phid->internalPacketInBufferLen - phid->outstandingPacketCnt[hubIndex]),
		hubIndex);
	mos_mutex_unlock(&(phid->outstandingPacketCntLock[hubIndex]));

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetMeshDongleDevice_makePacket(
	PhidgetMeshDongleDeviceHandle		phid,
	PhidgetDeviceHandle			meshDevice,
	int							packetID,
	const uint8_t			*bufferIn,
	size_t						bufferInLen,
	uint8_t				*buffer,
	size_t						*bufferLen) {

	assert(phid);
	assert(meshDevice);
	assert(bufferIn);
	assert(buffer);
	assert(bufferLen);
	assert(*bufferLen >= getMaxOutPacketSize((PhidgetDeviceHandle)phid));
	assert(getMaxOutPacketSize((PhidgetDeviceHandle)phid) >= bufferInLen + 6);

	pack32(buffer, meshDevice->deviceInfo.serialNumber);
	buffer[4] = (uint8_t)bufferInLen + 6;
	buffer[5] = packetID;
	memcpy(buffer + 6, bufferIn, bufferInLen);
	*bufferLen = bufferInLen + 6;

	return (EPHIDGET_OK);
}

static void CCONV
PhidgetMeshDongleDevice_free(PhidgetDeviceHandle *phidG) {
	PhidgetMeshDongleDeviceHandle phid;
	int i;

	phid = (PhidgetMeshDongleDeviceHandle)*phidG;

	for (i = 0; i < MESHDONGLE_MAXHUBS; i++) {
		mos_cond_destroy(&phid->outstandingPacketCntCond[i]);
		mos_mutex_destroy(&phid->outstandingPacketCntLock[i]);
	}

	mos_free(phid, sizeof(*phid));
	*phidG = NULL;
}

// === Exported Functions === //

//create and initialize a device structure
PhidgetReturnCode
PhidgetMeshDongleDevice_create(PhidgetMeshDongleDeviceHandle *phidp) {
	int i;
	DEVICECREATE_BODY(MeshDongleDevice, PHIDCLASS_MESHDONGLE);

	for (i = 0; i < MESHDONGLE_MAXHUBS; i++) {
		mos_mutex_init(&phid->outstandingPacketCntLock[i]);
		mos_cond_init(&phid->outstandingPacketCntCond[i]);
	}
	phid->phid.packetTracking = mallocPhidgetPacketTrackers();
	return (EPHIDGET_OK);
}
