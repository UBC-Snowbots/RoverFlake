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

#ifndef __CPHIDGETGPSDEVICE
#define __CPHIDGETGPSDEVICE

typedef struct _PhidgetGPSDevice *PhidgetGPSDeviceHandle;
PhidgetReturnCode PhidgetGPSDevice_create(PhidgetGPSDeviceHandle *phid);

#define GPS_MAXGPSES 1

// SkyTraq 6 binary messages
// Input system messages (commands)
#define GPS_SKYTRAQ_IN_SYSTEM_RESTART				0x01
#define GPS_SKYTRAQ_IN_QUERY_SOFTWARE_VERSION		0x02
#define GPS_SKYTRAQ_IN_QUERY_POSITION_UPDATE_RATE	0x10
// Input GPS messages (commands)
#define GPS_SKYTRAQ_IN_CONFIGURE_WAAS				0x37
#define GPS_SKYTRAQ_IN_QUERY_WAAS					0x38
// Output system messages (responses)
#define GPS_SKYTRAQ_OUT_SOFTWARE_VERSION			0x80
#define GPS_SKYTRAQ_OUT_SOFTWARE_CRC				0x81
#define GPS_SKYTRAQ_OUT_ACK							0x83
#define GPS_SKYTRAQ_OUT_NACK						0x84
#define GPS_SKYTRAQ_OUT_POSITION_UPDATE_RATE		0x86
// Output GPS messages

struct _PhidgetGPSDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.gps
	PhidgetDevice phid;

	/* Public Members */

	PhidgetGPS_NMEAData NMEAData[GPS_MAXGPSES];
	uint8_t NMEADataValid[GPS_MAXGPSES];
	double heading[GPS_MAXGPSES];
	double velocity[GPS_MAXGPSES];
	double altitude[GPS_MAXGPSES];
	double latitude[GPS_MAXGPSES];
	double longitude[GPS_MAXGPSES];
	uint8_t positionFixState[GPS_MAXGPSES];
	uint8_t timeValid[GPS_MAXGPSES];
	PhidgetGPS_Time time[GPS_MAXGPSES];
	uint8_t dateValid[GPS_MAXGPSES];
	PhidgetGPS_Date date[GPS_MAXGPSES];

	/* Private Members */

	double lastLongitude, lastLatitude, lastAltitude;
	uint8_t lastFix, lastDateValid, lastTimeValid;

	uint8_t sckbuf[256];
	uint8_t sckbuf_write, sckbuf_read;
} typedef PhidgetGPSDeviceInfo;
#endif
