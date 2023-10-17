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

#ifndef __CPHIDGETLEDDEVICE
#define __CPHIDGETLEDDEVICE

typedef struct _PhidgetLEDDevice *PhidgetLEDDeviceHandle;
PhidgetReturnCode PhidgetLEDDevice_create(PhidgetLEDDeviceHandle *phid);

#define LED_MAXLEDS 64

//OUT Packet Types
#define LED64_NORMAL_PACKET 0x00
#define LED64_CONTROL_PACKET 0x40
#define LED64_OUTLOW_PACKET 0x80
#define LED64_OUTHIGH_PACKET 0xc0

#define LED64_M3_OUT_LOW_PACKET 0x00
#define LED64_M3_OUT_HIGH_PACKET 0x20
#define LED64_M3_CONTROL_PACKET 0x40

//IN Packet Types
#define LED64_IN_LOW_PACKET 0x00
#define LED64_IN_HIGH_PACKET 0x80

#define LED64_M3_IN_LOW_PACKET 0x00
#define LED64_M3_IN_HIGH_PACKET 0x20
#define LED64_M3_IN_MISC_PACKET 0x40

//Flags
#define LED64_PGOOD_FLAG 0x01
#define LED64_CURSELA_FLAG 0x02
#define LED64_CURSELB_FLAG 0x04
#define LED64_PWRSELA_FLAG 0x08
#define LED64_PWRSELB_FLAG 0x10
#define LED64_FAULT_FLAG 0x20
#define LED64_OE_FLAG 0x40

#define LED64_CURRENTLIMIT	0.08 //80 mA max

struct _PhidgetLEDDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.led
	PhidgetDevice phid;

	/* Public Members */

	/* Private Members */

	double currentLimit[LED_MAXLEDS];
	double currentLimitBoard;
	PhidgetDigitalOutput_LEDForwardVoltage voltage[LED_MAXLEDS];
	PhidgetDigitalOutput_LEDForwardVoltage voltageBoard;

	double LED_Power[LED_MAXLEDS];
	double nextLED_Power[LED_MAXLEDS];
	double lastLED_Power[LED_MAXLEDS];
	uint8_t changedLED_Power[LED_MAXLEDS];

	uint8_t TSDCount[4], TSDClearCount[4], TWarnCount[4], TWarnClearCount[4];

	uint8_t lastOutputPacket;

	unsigned char lastControlPacket[MAX_OUT_PACKET_SIZE];
	int lastControlPacketValid;

} typedef PhidgetLEDDeviceInfo;

#endif
