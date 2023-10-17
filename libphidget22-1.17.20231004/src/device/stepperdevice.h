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

#ifndef __CPHIDGETSTEPPERDEVICE
#define __CPHIDGETSTEPPERDEVICE

typedef struct _PhidgetStepperDevice *PhidgetStepperDeviceHandle;
PhidgetReturnCode PhidgetStepperDevice_create(PhidgetStepperDeviceHandle *phid);

#define STEPPER_MAXSTEPPERS 8
#define STEPPER_MAXINPUTS 8
#define STEPPER_CHANNELS ((STEPPER_MAXSTEPPERS * 2) + STEPPER_MAXINPUTS)

#define BIPOLAR_STEPPER_CURRENT_SENSE_GAIN 8.5
#define BIPOLAR_STEPPER_CURRENT_LIMIT_Rs 0.150

//flags - make sure these are in the upper 4 bits
#define MOTOR_DONE_STEPPER	 	0x10
#define MOTOR_DISABLED_STEPPER	0x20

//packet types - room for one more
#define STEPPER_POSITION_PACKET		0x00
#define STEPPER_VEL_ACCEL_PACKET	0x10
#define STEPPER_RESET_PACKET		0x20

struct _PhidgetStepperDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.stepper
	PhidgetDevice phid;

	/* Public Members */

	double current[STEPPER_MAXSTEPPERS];
	uint8_t inputState[STEPPER_MAXINPUTS];
	int64_t position[STEPPER_MAXSTEPPERS];
	double velocity[STEPPER_MAXSTEPPERS];
	uint8_t isMoving[STEPPER_MAXSTEPPERS];

	/* Private Members */

	double currentChangeTrigger[STEPPER_MAXSTEPPERS];
	uint8_t engaged[STEPPER_MAXSTEPPERS];
	int64_t targetPosition[STEPPER_MAXSTEPPERS];
	double velocityLimit[STEPPER_MAXSTEPPERS];
	double acceleration[STEPPER_MAXSTEPPERS];
	double currentLimit[STEPPER_MAXSTEPPERS];

	//data from the device
	int32_t packetCounterEcho[STEPPER_MAXSTEPPERS];

	//data from the user
	int64_t motorPositionReset[STEPPER_MAXSTEPPERS];
	uint8_t motorEngagedState[STEPPER_MAXSTEPPERS];
	int32_t packetCounter[STEPPER_MAXSTEPPERS];
	double motorSensedCurrentLastTrigger[STEPPER_MAXSTEPPERS];
	PhidgetStepper_ControlMode controlMode[STEPPER_MAXSTEPPERS];
	int64_t lastPosition[STEPPER_MAXSTEPPERS];
	double lastVelocity[STEPPER_MAXSTEPPERS];

	int stepper1stepBugActive[STEPPER_MAXSTEPPERS];

	double motorSpeedMax, motorSpeedMin;
	double accelerationMax, accelerationMin;
	int64_t motorPositionMax, motorPositionMin;
	double currentMax, currentMin;
	int microSteps;

	int interruptRate;
	uint32_t _stepperDataInterval[STEPPER_MAXSTEPPERS];
	mostime_t _stepperDeadline[STEPPER_MAXSTEPPERS];
	uint32_t _currentinputDataInterval[STEPPER_MAXSTEPPERS];
	mostime_t _currentinputDeadline[STEPPER_MAXSTEPPERS];
	uint32_t _callcnt;
} typedef PhidgetStepperDeviceInfo;

#endif
