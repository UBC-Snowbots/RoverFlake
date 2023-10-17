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

#ifndef __PACKETTRACKER
#define __PACKETTRACKER

#include "phidget.h"

#define MAX_PACKET_IDS	128

#define PACKETTRACKER_INUSE		0x01
#define PACKETTRACKER_SIGNALLED	0x02
#define PACKETTRACKER_SENT		0x04
#define PACKETTRACKER_ABANDONED	0x08

typedef struct _PhidgetPacketTracker {
	int									_state;
	PhidgetReturnCode					_returnCode;
	int									childIndex;
	size_t								len;
	mos_cond_t							_cond;
	mos_mutex_t							_lock;
	const PhidgetUniqueDeviceDef		*udd;
	MTAILQ_ENTRY(_PhidgetPacketTracker)	link;
} PhidgetPacketTracker, *PhidgetPacketTrackerHandle;

typedef struct {
	PhidgetPacketTracker packetTracker[MAX_PACKET_IDS];
	int counter[PHIDGET_MAXCHILDREN];
} PhidgetPacketTrackers, *PhidgetPacketTrackersHandle;

typedef MTAILQ_HEAD(PhidgetPacketTrackerlist, _PhidgetPacketTracker) PhidgetPacketTrackerlist_t;

typedef struct {
	PhidgetReturnCode			error;
	int							errorCnt;
	PhidgetPacketTrackerlist_t	list;
} PhidgetTransaction, *PhidgetTransactionHandle;

PhidgetPacketTrackersHandle mallocPhidgetPacketTrackers(void);
void freePhidgetPacketTrackers(PhidgetPacketTrackersHandle);
void setPacketLength(PhidgetPacketTrackerHandle, size_t len);
void setPacketDevice(PhidgetPacketTrackerHandle packetTracker, const PhidgetUniqueDeviceDef *udd);
PhidgetReturnCode setPacketReturnCode(PhidgetPacketTrackerHandle, PhidgetReturnCode);
void setPacketsReturnCode(PhidgetDeviceHandle, int child, PhidgetReturnCode status);
void _setPacketsReturnCode(PhidgetDeviceHandle, int child, PhidgetReturnCode status);
PhidgetReturnCode getPacketReturnCode(PhidgetPacketTrackerHandle, PhidgetReturnCode *);
PhidgetReturnCode waitForPendingPacket(PhidgetPacketTrackerHandle, uint32_t ms);
void waitForPendingPackets(PhidgetDeviceHandle, int child);
void waitForAllPendingPackets(PhidgetDeviceHandle);
PhidgetReturnCode getPacketTracker(PhidgetDeviceHandle, int *packetID, PhidgetPacketTrackerHandle *,
   int min, int max, int childIndex);
PhidgetReturnCode getPacketTrackerWait(PhidgetDeviceHandle device, int *packetID, PhidgetPacketTrackerHandle *packetTracker,
	int min, int max, int childIndex, uint32_t timeout);
void releasePacketTracker(PhidgetDeviceHandle, PhidgetPacketTrackerHandle, int force);

#endif
