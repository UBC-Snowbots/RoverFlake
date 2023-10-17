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

#ifndef __CPHIDGETADVANCEDSERVODEVICE
#define __CPHIDGETADVANCEDSERVODEVICE

typedef struct _PhidgetAdvancedServoDevice *PhidgetAdvancedServoDeviceHandle;
PhidgetReturnCode PhidgetAdvancedServoDevice_create(PhidgetAdvancedServoDeviceHandle *phid);

#define ADVSERVO_MAXSERVOS 8

#define ADVSERVO_NORMAL_PACKET	0x00
#define ADVSERVO_MINMAX_PACKET	0x10

//Flags - need to match those defined in firmware
#define MOTOR_DONE_ADVSERVO	 		0x20
#define MOTOR_DISABLED_ADVSERVO		0x40
#define NO_RAMPING_FLAG_ADVSERVO	0x80

#define ADVSERVO_PGOOD_FLAG		0x01
#define ADVSERVO_PRWSELA_FLAG	0x02
#define ADVSERVO_PRWSELB_FLAG	0x04

struct _PhidgetAdvancedServoDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.advancedservo
	PhidgetDevice phid;

	/* Public Members */

	double position[ADVSERVO_MAXSERVOS];
	double velocity[ADVSERVO_MAXSERVOS];
	int isMoving[ADVSERVO_MAXSERVOS];
	double current[ADVSERVO_MAXSERVOS];

	/* Private Members */

	/* Sets */
	double currentChangeTrigger[ADVSERVO_MAXSERVOS];
	double maxPulseWidth[ADVSERVO_MAXSERVOS];
	double minPulseWidth[ADVSERVO_MAXSERVOS];
	double targetPosition[ADVSERVO_MAXSERVOS];
	double acceleration[ADVSERVO_MAXSERVOS];
	double velocityLimit[ADVSERVO_MAXSERVOS];
	int motorSpeedRampingState[ADVSERVO_MAXSERVOS];
	int motorEngagedState[ADVSERVO_MAXSERVOS];

	/* Internal use */
	int32_t packetCounter[ADVSERVO_MAXSERVOS];
	int32_t packetCounterEcho[ADVSERVO_MAXSERVOS];
	double currentLastTrigger[ADVSERVO_MAXSERVOS];
	double velocityLastTrigger[ADVSERVO_MAXSERVOS];
	double positionLastTrigger[ADVSERVO_MAXSERVOS];
	double velocityMax;
	double accelerationMax;
	double motorPositionMaxLimit;
	double motorPositionMinLimit;
	PhidgetRCServo_Voltage voltage;
	PhidgetRCServo_Voltage voltageEcho;

	uint32_t _interruptRate;
	uint32_t _callcnt;
	uint32_t _rcservoDataInterval[ADVSERVO_MAXSERVOS];
	mostime_t _rcservoDeadline[ADVSERVO_MAXSERVOS];
	uint32_t _currentinputDataInterval[ADVSERVO_MAXSERVOS];
	mostime_t _currentinputDeadline[ADVSERVO_MAXSERVOS];
} typedef PhidgetAdvancedServoDeviceInfo;

#endif
