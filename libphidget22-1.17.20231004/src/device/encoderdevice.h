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

#ifndef __CPHIDGETENCODERDEVICE
#define __CPHIDGETENCODERDEVICE

 //packet types
 //IN
 //OUT
#define ENCODER_GENERIC_PACKET			0x01
#define ENCODER_IO_MODE_PACKET			0x02

typedef struct _PhidgetEncoderDevice *PhidgetEncoderDeviceHandle;
PhidgetReturnCode PhidgetEncoderDevice_create(PhidgetEncoderDeviceHandle *phid);

#define ENCODER_MAXENCODERS 4
#define ENCODER_MAXINPUTS 4
struct _PhidgetEncoderDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.encoder
	PhidgetDevice phid;

	/* Public Members */

	uint8_t enabled[ENCODER_MAXENCODERS];
	uint8_t inputState[ENCODER_MAXINPUTS];

	/* Private Members */

	uint8_t enableState[ENCODER_MAXENCODERS];

	int32_t encoderTimeStamp[ENCODER_MAXENCODERS];
#if PHIDUID_1047_2_300_SUPPORTED
	int32_t deadTime[ENCODER_MAXENCODERS];
#endif

	uint32_t encoderChangeTrigger[ENCODER_MAXENCODERS];
	int positionChangeAccumulator[ENCODER_MAXENCODERS];
	uint64_t timeChangeAccumulator[ENCODER_MAXENCODERS]; //ns
	int indexTrueAccumulator[ENCODER_MAXENCODERS];
	int indexOffset[ENCODER_MAXENCODERS];

	/* we only need enough for encoders as we do not limit inputs */
	uint32_t _encoderDataInterval[ENCODER_MAXENCODERS];
	mostime_t _encoderDeadline[ENCODER_MAXENCODERS];
	uint32_t _callcnt;
	uint32_t _interruptRate;

} typedef PhidgetEncoderDeviceInfo;

#endif
