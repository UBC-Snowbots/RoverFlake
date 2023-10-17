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

#ifndef __CPHIDGETRFIDDEVICE
#define __CPHIDGETRFIDDEVICE
//#include "class/rfid.h"

typedef struct _PhidgetRFIDDevice *PhidgetRFIDDeviceHandle;
PhidgetReturnCode PhidgetRFIDDevice_create(PhidgetRFIDDeviceHandle *phid);

#include "util/rfidsupport.h"

#define RFIDDevice_PACKET_TAG 0
#define RFIDDevice_PACKET_OUTPUT_ECHO 1

#define RFIDDevice_LED_FLAG 0x04
#define RFIDDevice_ANTENNA_FLAG 0x08
#define RFIDDevice_LISTEN_DURING_EOF_FLAG 0x10

#define RFIDDevice_WRITE_DATA_OUT_PACKET	0x00
#define RFIDDevice_CONTROL_OUT_PACKET		0x40

#define RFIDDevice_READ_DATA_IN_PACKET	0x00
#define RFIDDevice_ECHO_IN_PACKET			0x40

#define RFIDDevice_MAX_DATA_PER_PACKET	63

#define RFIDDevice_DATA_ARRAY_SIZE		1024
#define RFIDDevice_DATA_ARRAY_MASK		0x3ff

#define RFIDDevice_MAXOUTPUTS 3
#define RFIDDevice_MAXRFIDS 1


/* 4097 constants */
#define RFIDDevice_4097_AmpDemod		0x00	//Amplitude demodulation
#define RFIDDevice_4097_PhaseDemod	0x01	//Phase demodulation

#define RFIDDevice_4097_PowerDown		0x00
#define RFIDDevice_4097_Active		0x02

#define RFIDDevice_4097_DataOut		0x00	//DATA_OUT is data from the rfid card
#define RFIDDevice_4097_ClkOut		0x04	//DATA_OUT is the internal clock/32

#define	RFIDDevice_4097_IntPLL		0x00
#define RFIDDevice_4097_ExtClk		0x08

#define RFIDDevice_4097_FastStart		0x10

#define RFIDDevice_4097_Gain960		0x40
#define RFIDDevice_4097_Gain480		0x00
#define RFIDDevice_4097_Gain240		0x60
#define RFIDDevice_4097_Gain120		0x20

#define RFIDDevice_4097_TestMode		0x80

#define RFIDDevice_4097_DefaultON		(RFIDDevice_4097_AmpDemod | RFIDDevice_4097_Active | RFIDDevice_4097_DataOut | RFIDDevice_4097_IntPLL | RFIDDevice_4097_FastStart | RFIDDevice_4097_Gain960)


/* T5577 Write Timing Constants */
#define RFIDDevice_T5577_StartGap 30
#define RFIDDevice_T5577_WriteGap 15
#define RFIDDevice_T5577_EndGap 15
#define RFIDDevice_T5577_Zero 24
#define RFIDDevice_T5577_One 56
#define RFIDDevice_T5577_EOF 100
#define RFIDDevice_T5577_PrePulse (136 + RFIDDevice_T5577_Zero)

#define RFIDDevice_MAX_TAG_STRING_LEN 25
typedef struct _PhidgetRFIDDevice_Tag {
	PhidgetRFID_Protocol protocol;
	char tagString[RFIDDevice_MAX_TAG_STRING_LEN];
#ifdef RFIDDevice_HITAGS_SUPPORT
	PhidgetRFIDDevice_TagType tagType;
#endif
} PhidgetRFIDDevice_Tag, *PhidgetRFIDDevice_TagHandle;

struct _PhidgetRFIDDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.rfid
	PhidgetDevice phid;

	/* Public Members */

	uint8_t outputState[RFIDDevice_MAXOUTPUTS];
	uint8_t antennaEnabled[RFIDDevice_MAXRFIDS];
	uint8_t tagPresent[RFIDDevice_MAXRFIDS];

	/* Private Members */

	/* State */

	int32_t spaceClocks, pregapClocks, postgapClocks, oneClocks, zeroClocks, prepulseClocks, eofpulseClocks;
	uint8_t listenDuringEOF;
	int32_t _4097Conf;

	uint8_t outputStateSet[RFIDDevice_MAXOUTPUTS];
	uint8_t antennaState;
	int32_t spaceClocksEcho, pregapClocksEcho, postgapClocksEcho, oneClocksEcho, zeroClocksEcho, prepulseClocksEcho, eofpulseClocksEcho;
	uint8_t listenDuringEOFEcho;
	int32_t _4097ConfEcho;

	uint8_t fullStateEcho;

	mos_task_t tagTimerThread;
	mos_mutex_t tagLock; /* protects tag thread access to things */
	mos_cond_t tagCond;
	int tagThreadRun;	/* 0 stopped; 1 running; 2 stop pending */

	/* Tag event */
	PhidgetRFIDDevice_Tag lastTag;
	uint8_t lastTagValid;
	mostime_t lastTagTime;
	PhidgetRFIDDevice_Tag pendingTag;
	uint8_t tagEventPending;

	uint8_t tagWriting;
	uint8_t prevTagPresent;

	/* Raw data buffer */
	int32_t dataBuffer[RFIDDevice_DATA_ARRAY_SIZE];
	uint32_t dataReadPtr, dataWritePtr;

	int shortClocks, longClocks;

	/* Manchester decoder */
	uint8_t manBuffer[RFIDDevice_DATA_ARRAY_SIZE];
	uint32_t manReadPtr, manWritePtr;
	uint8_t manLockedIn;
	uint8_t manShortChange;

	/* BiPhase Decoder */
	uint8_t biphaseBuffer[RFIDDevice_DATA_ARRAY_SIZE];
	uint32_t biphaseReadPtr, biphaseWritePtr;
	uint8_t biphaseLockedIn;
	uint8_t biphaseShortChange;

#ifdef RFIDDevice_RAWDATA_API_SUPPORT
	uint32_t userReadPtr;
	uint32_t manEventReadPtr;
	uint8_t lastManEventLong;
#endif

} typedef PhidgetRFIDDeviceInfo;

#endif
