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

#ifndef __CPHIDGETMOTORCONTROLDEVICE
#define __CPHIDGETMOTORCONTROLDEVICE

typedef struct _PhidgetMotorControlDevice *PhidgetMotorControlDeviceHandle;
PhidgetReturnCode PhidgetMotorControlDevice_create(PhidgetMotorControlDeviceHandle *phid);

#define MOTORCONTROL_MAXMOTORS			2
#define MOTORCONTROL_MAXINPUTS			4
#define MOTORCONTROL_MAXENCODERS		1
#define MOTORCONTROL_MAXVOLTAGEINPUTS	3 // 2 analog inputs and a supply voltage
#define MOTORCONTROL_MAXSENSORS			2

//#define MOTORCONTROL_CHANNELS (MOTORCONTROL_MAXMOTORS + MOTORCONTROL_MAXINPUTS + MOTORCONTROL_MAXENCODERS + MOTORCONTROL_MAXVOLTAGEINPUTS + MOTORCONTROL_MAXSENSORS)

struct _PhidgetMotorControlDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.motorcontrol
	PhidgetDevice phid;

	/* Public Members */
	double current[MOTORCONTROL_MAXMOTORS];
	double currentChangeTrigger[MOTORCONTROL_MAXMOTORS];

	double dutyCycle[MOTORCONTROL_MAXMOTORS];
	double backEMF[MOTORCONTROL_MAXMOTORS];
	double acceleration[MOTORCONTROL_MAXMOTORS];
	double brakingDutyCycle[MOTORCONTROL_MAXMOTORS];

	double voltage[MOTORCONTROL_MAXVOLTAGEINPUTS];
	double voltageChangeTrigger[MOTORCONTROL_MAXVOLTAGEINPUTS];

	double voltageRatio[MOTORCONTROL_MAXSENSORS];
	double voltageRatioChangeTrigger[MOTORCONTROL_MAXSENSORS];

	/* Private Members */
	// Returned from the device
	uint8_t inputState[MOTORCONTROL_MAXINPUTS];
	uint8_t backEMFSensingStateEcho[MOTORCONTROL_MAXMOTORS];
	double motorSetSpeedEcho[MOTORCONTROL_MAXMOTORS];

	int encoderPositionEcho[MOTORCONTROL_MAXENCODERS];
	int32_t encoderTimeStamp[MOTORCONTROL_MAXENCODERS];
	uint32_t encoderChangeTrigger[MOTORCONTROL_MAXENCODERS];
	int positionChangeAccumulator[MOTORCONTROL_MAXENCODERS];
	uint32_t timeChangeAccumulator[MOTORCONTROL_MAXENCODERS];

	uint8_t ratiometricEcho;

	double sensorVoltageLastValue[MOTORCONTROL_MAXVOLTAGEINPUTS];
	double sensorRatioLastValue[MOTORCONTROL_MAXSENSORS];

	// Local set data
	double motorSpeed[MOTORCONTROL_MAXMOTORS];
	double motorAcceleration[MOTORCONTROL_MAXMOTORS];
	uint8_t backEMFSensingState[MOTORCONTROL_MAXMOTORS];
	double motorBraking[MOTORCONTROL_MAXMOTORS];
	uint8_t ratiometric;
	uint8_t ratiometricSwitching;

	int encoderPositionDelta[MOTORCONTROL_MAXENCODERS];

	double currentLastTrigger[MOTORCONTROL_MAXMOTORS];

	uint32_t _dcmotorDataInterval[MOTORCONTROL_MAXMOTORS];
	mostime_t _dcmotorDeadline[MOTORCONTROL_MAXMOTORS];
	uint32_t _currentinputDataInterval[MOTORCONTROL_MAXMOTORS];
	mostime_t _currentinputDeadline[MOTORCONTROL_MAXMOTORS];
	uint32_t _encoderDataInterval[MOTORCONTROL_MAXENCODERS];
	mostime_t _encoderDeadline[MOTORCONTROL_MAXENCODERS];
	uint32_t _voltageinputDataInterval[MOTORCONTROL_MAXVOLTAGEINPUTS];
	mostime_t _voltageinputDeadline[MOTORCONTROL_MAXVOLTAGEINPUTS];
	uint32_t _voltageratioinputDataInterval[MOTORCONTROL_MAXSENSORS];
	mostime_t _voltageratioinputDeadline[MOTORCONTROL_MAXSENSORS];
	uint32_t _callcnt;

	// Constants
	// Make sure values are within defined range, and store to structure
	double accelerationMax, accelerationMin, currentMax;
	uint32_t _interruptRate;

} typedef PhidgetMotorControlDeviceInfo;

#endif
