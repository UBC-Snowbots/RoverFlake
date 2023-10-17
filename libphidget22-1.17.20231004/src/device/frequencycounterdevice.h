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

#ifndef __CPHIDGETFREQUENCYCOUNTERDEVICE
#define __CPHIDGETFREQUENCYCOUNTERDEVICE

typedef struct _PhidgetFrequencyCounterDevice *PhidgetFrequencyCounterDeviceHandle;
PhidgetReturnCode PhidgetFrequencyCounterDevice_create(PhidgetFrequencyCounterDeviceHandle *phid);

#define FREQCOUNTER_MAXINPUTS 2

#define FREQCOUNTER_TICKS_PER_SEC	100000
#define FREQCOUNTER_MS_PER_TICK	(1000 / (double)FREQCOUNTER_TICKS_PER_SEC)

//OUT packet flags
#define FREQCOUNTER_FLAG_CH1_LOGIC 0x01
#define FREQCOUNTER_FLAG_CH0_LOGIC 0x02
#define FREQCOUNTER_FLAG_CH1_ENABLE 0x04
#define FREQCOUNTER_FLAG_CH0_ENABLE 0x08

struct _PhidgetFrequencyCounterDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.frequencycounter
	PhidgetDevice phid;

	/* Public Members */

	/* Private Members */

	PhidgetFrequencyCounter_FilterType filter[FREQCOUNTER_MAXINPUTS];
	uint8_t enabled[FREQCOUNTER_MAXINPUTS];

	int flip[FREQCOUNTER_MAXINPUTS];
	int lastPacketCount;

	uint32_t ticksAcc[FREQCOUNTER_MAXINPUTS];
	uint32_t countsAcc[FREQCOUNTER_MAXINPUTS];
	uint32_t ticksAtLastCountAcc[FREQCOUNTER_MAXINPUTS];

	uint32_t _interruptRate;
	uint32_t _frequencycounterDataInterval[FREQCOUNTER_MAXINPUTS];
	mostime_t _frequencycounterDeadline[FREQCOUNTER_MAXINPUTS];
	uint32_t _callcnt;
} typedef PhidgetFrequencyCounterDeviceInfo;

#endif
