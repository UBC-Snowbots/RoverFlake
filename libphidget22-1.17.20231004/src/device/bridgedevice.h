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

#ifndef __CPHIDGETBRIDGEDEVICE
#define __CPHIDGETBRIDGEDEVICE

typedef struct _PhidgetBridgeDevice *PhidgetBridgeDeviceHandle;
PhidgetReturnCode PhidgetBridgeDevice_create(PhidgetBridgeDeviceHandle *phid);

#define BRIDGE_MAXINPUTS 4
struct _PhidgetBridgeDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.bridge
	PhidgetDevice phid;

	/* Public Members */

	double voltageRatio[BRIDGE_MAXINPUTS];

	/* Private Members */

	double dataInterval[BRIDGE_MAXINPUTS];
	double voltageRatioChangeTrigger[BRIDGE_MAXINPUTS];

	uint8_t enabled[BRIDGE_MAXINPUTS];
	PhidgetVoltageRatioInput_BridgeGain gain[BRIDGE_MAXINPUTS];
	uint32_t dataRate;

	uint8_t enabledEcho[BRIDGE_MAXINPUTS];
	PhidgetVoltageRatioInput_BridgeGain gainEcho[BRIDGE_MAXINPUTS];
	double bridgeLastTrigger[BRIDGE_MAXINPUTS];

	uint32_t dataRateMin, dataRateMax;
	double bridgeMin[BRIDGE_MAXINPUTS], bridgeMax[BRIDGE_MAXINPUTS];

	uint8_t outOfRange[BRIDGE_MAXINPUTS];

	//Firmware bug handling
	uint8_t chEnabledBugNotValid[BRIDGE_MAXINPUTS];
	uint8_t ch0EnableOverride;
} typedef PhidgetBridgeDeviceInfo;

#endif
