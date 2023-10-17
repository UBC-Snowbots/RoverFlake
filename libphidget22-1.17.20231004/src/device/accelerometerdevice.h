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

#ifndef __CPHIDGETACCELEROMETERDEVICE
#define __CPHIDGETACCELEROMETERDEVICE

typedef struct _PhidgetAccelerometerDevice *PhidgetAccelerometerDeviceHandle;
PhidgetReturnCode PhidgetAccelerometerDevice_create(PhidgetAccelerometerDeviceHandle *phid);

#define ACCEL_MAXACCELEROMETERS 1
#define ACCEL_MAXAXES 3
struct _PhidgetAccelerometerDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.accelerometer
	PhidgetDevice phid;

	/* Public Members */
	double acceleration[ACCEL_MAXACCELEROMETERS][ACCEL_MAXAXES];
	double accelerationChangeTrigger[ACCEL_MAXACCELEROMETERS];
	double timestamp[ACCEL_MAXACCELEROMETERS];

	/* Private Members */
	double _maxAcceleration;
	double _minAcceleration;
	uint32_t _interruptRate;
	uint32_t _accelDataInterval[ACCEL_MAXACCELEROMETERS];
	mostime_t _accelDeadline[ACCEL_MAXACCELEROMETERS];
	uint32_t _callcnt;

	double _accelerationLastTrigger[ACCEL_MAXAXES];
} typedef PhidgetAccelerometerDeviceInfo;

#endif
