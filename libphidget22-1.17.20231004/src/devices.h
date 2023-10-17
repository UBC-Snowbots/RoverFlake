#ifndef EXTERNALPROTO
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
#endif

#ifndef __PHIDGETDEVICESH
#define __PHIDGETDEVICESH

#define USBVID_PHIDGETS			0x06C2
#define USBVID_OLD				0x0925
#define USBPID_PHIDGETS_MIN		0x0030
#define USBPID_PHIDGETS_MAX		0x00AF

#include "constantsinternal.h"
#include "devices.gen.h"
#include "bridge.h"

typedef enum {
	PHIDTYPE_USB=1,
	PHIDTYPE_VINT=2,
	PHIDTYPE_MESH=3,
	PHIDTYPE_SPI=4,
	PHIDTYPE_LIGHTNING=5,
	PHIDTYPE_VIRTUAL=6
} PhidgetUniqueDeviceType;

typedef struct {
	int dummy1;				int dummy2;				int dummy3;				int dummy4;				int dummy5; int dummy6;
}	PhidgetDummyAttr;
typedef struct {
	int numAxis;
}	PhidgetAccelerometerAttr;
typedef struct {
	int numMotors;			int numCurrentInputs;
}	PhidgetAdvancedServoAttr;
typedef struct {
	int numAnalogOutputs;
}	PhidgetAnalogAttr;
typedef struct {
	int numBridgeInputs;
}	PhidgetBridgeAttr;
typedef struct {
	int numDataAdapters;		int numInputs;		int numOutputs;
}	PhidgetDataAdapterAttr;
typedef struct {
	int numEncoders;		int numInputs;
}	PhidgetEncoderAttr;
typedef struct {
	int numFreqInputs;
}	PhidgetFrequencyCounterAttr;
typedef struct {
	int numGPSes;
}	PhidgetGPSAttr;
typedef struct {
	int numVintPorts;		int numVintPortModes;
}	PhidgetHubAttr;
typedef struct {
	int numVoltageInputs;	int numSensors;			int numInputs;			int numOutputs;			int numCapTouches;
}	PhidgetInterfaceKitAttr;
typedef struct {
	int numIRs;
}	PhidgetIRAttr;
typedef struct {
	int numLEDs;
}	PhidgetLEDAttr;
typedef struct {
	int numHubs;
}	PhidgetMeshDongleAttr;
typedef struct {
	int numMotors;			int numInputs;			int numEncoders;		int numVoltageInputs;	int numSensors; int numCurrentInputs;
}	PhidgetMotorControlAttr;
typedef struct {
	int numVoltageInputs;	int numPHInputs;
}	PhidgetPHSensorAttr;
typedef struct {
	int numOutputs;
}	PhidgetRFIDAttr;
typedef struct {
	int numMotors;
}	PhidgetServoAttr;
typedef struct {
	int numAccelAxes;		int numGyroAxes;		int numCompassAxes;		int spatial;	int temperature;
}	PhidgetSpatialAttr;
typedef struct {
	int numMotors;			int numInputs;			int numCurrentInputs;
}	PhidgetStepperAttr;
typedef struct {
	int numTempInputs;		int numVoltageInputs;
}	PhidgetTemperatureSensorAttr;
typedef struct {
	int numScreens;			int numInputs;			int numOutputs;
}	PhidgetTextLCDAttr;
typedef union {
	PhidgetDummyAttr				dummy; /* field for unnamed initializers */
	PhidgetAccelerometerAttr		accelerometer;
	PhidgetAdvancedServoAttr		advancedservo;
	PhidgetAnalogAttr				analog;
	PhidgetBridgeAttr				bridge;
	PhidgetDataAdapterAttr			dataadapter;
	PhidgetEncoderAttr				encoder;
	PhidgetFrequencyCounterAttr		frequencycounter;
	PhidgetGPSAttr					gps;
	PhidgetHubAttr					hub;
	PhidgetInterfaceKitAttr			ifkit;
	PhidgetIRAttr					ir;
	PhidgetLEDAttr					led;
	PhidgetMeshDongleAttr			mesh;
	PhidgetMotorControlAttr			motorcontrol;
	PhidgetPHSensorAttr				phsensor;
	PhidgetRFIDAttr					rfid;
	PhidgetServoAttr				servo;
	PhidgetSpatialAttr				spatial;
	PhidgetStepperAttr				stepper;
	PhidgetTemperatureSensorAttr	temperaturesensor;
	PhidgetTextLCDAttr				textlcd;
} Phidget_DeviceAttr;

typedef struct {
	Phidget_DeviceChannelUID uid;
	Phidget_ChannelClass class;
	Phidget_ChannelSubclass subclass;
	int index;
	int count;
	int exclusive;
	const char *name;
} PhidgetUniqueChannelDef;

typedef struct {
	PhidgetUniqueDeviceType type;
	Phidget_DeviceID id;

	Phidget_DeviceUID uid;
	Phidget_DeviceClass class;
	Phidget_DeviceAttr channelCnts;
	int productID;
	int vendorID;
	int interfaceNum;
	int canNak;
	unsigned short vintID;

	int versionLow;
	int versionHigh;
	const char *SKU;
	const char *name;
	PhidgetUniqueChannelDef channels[9];
} PhidgetUniqueDeviceDef;

typedef struct {
	Phidget_ChannelClass chclass;
	int version;
	int remoteFlags;
} PhidgetChannelAttributeDef;

PhidgetReturnCode getUniqueChannelDef(const PhidgetUniqueDeviceDef *, Phidget_ChannelClass, int uniqueIndex,
  int *cindex, const PhidgetUniqueChannelDef **);
int allowNetworkAccess(PhidgetChannelHandle channel, int multiple);

PhidgetReturnCode createTypedPhidgetDeviceHandle(PhidgetDeviceHandle *phid, Phidget_DeviceClass deviceClass);
PhidgetReturnCode deviceBridgeInput(PhidgetChannelHandle, BridgePacket *);

#endif /* __PHIDGETDEVICESH */
