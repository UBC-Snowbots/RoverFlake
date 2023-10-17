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

#ifndef __CPHIDGETVINTDEVICE
#define __CPHIDGETVINTDEVICE

typedef struct _PhidgetVINTDevice *PhidgetVINTDeviceHandle;
PhidgetReturnCode PhidgetVINTDevice_create(PhidgetVINTDeviceHandle *phid);

PhidgetReturnCode PhidgetVINTDevice_makePacket(
	PhidgetVINTDeviceHandle		vintDevice,
	PhidgetChannelHandle		vintChannel,
	VINTDeviceCommand			deviceCommand,
	VINTPacketType				devicePacketType,
	const uint8_t				*bufferIn,
	size_t						bufferInLen,
	uint8_t						*buffer,
	size_t						*bufferLen);

struct _PhidgetVINTDevice {
	PhidgetDevice phid;

	// VINT Protocol version
	uint8_t deviceProtocolVersion;
	uint8_t deviceSupportsSetSpeed;
	uint8_t deviceSupportsAutoSetSpeed;
	uint32_t deviceMaxSpeed;

	// XXX - these are redundant as they could be read out of the hub device structure
	uint8_t portProtocolVersion;
	uint8_t portSupportsSetSpeed;
	uint8_t portSupportsAutoSetSpeed;
	uint32_t portMaxSpeed;

	// The communication speed in Hz
	uint32_t vintCommSpeed;

	// Device-level properties - NOTE we are just recording ALL device-level props for ALL vint device
	//  In the future, this could be turned into a union or something to save space..
	uint8_t heatingEnabled;
	Phidget_InputMode inputMode;
	Phidget_PowerSupply powerSupply;
	Phidget_RTDWireSetup RTDWireSetup;

} typedef PhidgetVINTDeviceInfo;

#endif
