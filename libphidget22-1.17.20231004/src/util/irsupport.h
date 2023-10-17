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

#ifndef __IRSUPPORT
#define __IRSUPPORT

// Maximum array size for a repeat code
#define IR_MAX_REPEAT_LENGTH	26

#define IR_DATA_ARRAY_SIZE		2048
// Maximum array size needed to hold the longest code
#define IR_MAX_CODE_DATA_LENGTH		(IR_MAX_CODE_BIT_COUNT / 8)

#define IR_DATASIZE(bitCount) ((bitCount / 8) + ((bitCount % 8) ? 1 : 0))

#define IR_MAX_DATA_PER_PACKET	31

#define IR_DATA_ARRAY_MASK		0x7ff

// for transmitting / receiving raw data
#define IR_MAX_DATA_us			327670

// this is just actual gap, not the gap that includes data
#define IR_MAX_GAP_LENGTH		100000 //us
#define IR_MIN_GAP_LENGTH		20000 //us

#define IR_DEFINEDATA_PACKET	0

#define IR_STOP_RX_WHILE_TX_FLAG	0x01

#define IR_RAW_DATA_WS_KEYS_MAX		100

typedef struct {
	PhidgetIR_CodeInfo lastCodeInfo;
	PhidgetIR_CodeInfo lastLearnedCodeInfo;
	char lastCodeStr[IR_MAX_CODE_STR_LENGTH];
	char lastLearnedCodeStr[IR_MAX_CODE_STR_LENGTH];
	int lastCodeKnown;
	int lastLearnedCodeKnown;
	uint32_t dataReadPtr;
	uint32_t dataWritePtr;
	uint32_t dataBuffer[IR_DATA_ARRAY_SIZE];
	uint32_t dataBufferNormalized[IR_DATA_ARRAY_SIZE];
	uint32_t learnReadPtr;
	uint8_t lastRepeat;
	uint32_t lastGap;
	uint8_t lastSentCode[16];
	PhidgetIR_CodeInfo lastSentCodeInfo;
	uint8_t polarity;
	int64_t lastDataTime;
	uint8_t delayCode;
	uint8_t nakFlag;
} PhidgetIRSupport, *PhidgetIRSupportHandle;

void PhidgetIRSupport_free(PhidgetIRSupportHandle *arg);
PhidgetReturnCode PhidgetIRSupport_create(PhidgetIRSupportHandle *ir);
void PhidgetIRSupport_init(PhidgetIRSupportHandle ir);
PhidgetReturnCode PhidgetIRSupport_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp);
PhidgetReturnCode PhidgetIRSupport_dataInput(PhidgetChannelHandle ch, const uint8_t *buf, size_t len);

#endif
