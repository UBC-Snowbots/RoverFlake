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

#ifndef __CPHIDGETVINT
#define __CPHIDGETVINT

#include "phidget.h"
#include "macros.h"
#include "vintpackets.h"

//Max packet sizes
#define VINT_MAX_OUT_PACKETSIZE	48
#define VINT_MAX_IN_PACKETSIZE	48

//Messages
#define VINT_CMD 0x80
#define VINT_DATA 0x00

#define VINT_DATA_wCHANNEL 0x40

typedef PhidgetReturnCode (*vintsend_t)(PhidgetChannelHandle, BridgePacket *);
typedef PhidgetReturnCode (*vintrecv_t)(PhidgetChannelHandle, const uint8_t *, size_t);

typedef struct {
	const vintsend_t send;
	const vintrecv_t recv;
} VINTIO_t;

const VINTIO_t * const getVINTIO(unsigned int uid);

//PC -> VINT device Commands / Data
typedef enum {
	// VINT1
	VINT_CMD_DATA					= VINT_DATA,
	VINT_CMD_RESET 					= (VINT_CMD | 0x03),
	VINT_CMD_UPGRADE_FIRMWARE		= (VINT_CMD | 0x0B),
	VINT_CMD_FIRMWARE_UPGRADE_DONE	= (VINT_CMD | 0x0C),
	// VINT2
	VINT_CMD_SETSPEED2				= (VINT_CMD | 0x0F)
} VINTDeviceCommand;

typedef struct _PhidgetVINTConnection {
	PHIDGET_STRUCT_START
} PhidgetVINTConnection, *PhidgetVINTConnectionHandle;

//Messages going to PC in the data stream
#define VINT_MSG_ATTACH	0x80
#define VINT_MSG_DETACH	0x81

//VINT Properties
// [7-5: length],[4-0: ID]
// Property IDs are 0x00 - 0x1F
// Top 3 bits are number of data bytes following property

// 1 data byte - 0 means vint powered, 1 means self powered
#define VINT_PROP_POWERSOURCE			0x00
// 1 data byte - bit0 is means isolation, bit1 is means high-speed isolation
#define VINT_PROP_ISOLATION				0x01
// 0 data bytes - means that the communication speed can be set
#define VINT_PROP_SETSPEEDSUPPORT		0x02
// 4 data bytes (uint32_t) - max supported VINT speed in Hz
#define VINT_PROP_SETSPEEDLIMIT			0x03
// 0 data bytes - means that comm speed can be set automatically
#define VINT_PROP_AUTOSETSPEEDSUPPORT	0x04


PhidgetReturnCode sendVINTPacket(mosiop_t iop, PhidgetChannelHandle channel, VINTDeviceCommand command,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen);
PhidgetReturnCode sendVINTDataPacket(mosiop_t iop, PhidgetChannelHandle channel,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen);
PhidgetReturnCode sendVINTDataPacketTransaction(mosiop_t iop, PhidgetChannelHandle channel,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen, PhidgetTransactionHandle trans);

PhidgetReturnCode scanVintDevices(PhidgetDeviceHandle);
PhidgetReturnCode scanVintDevice(PhidgetDeviceHandle, int, int, int, int);

PhidgetReturnCode PhidgetVINTConnectionCreate(PhidgetVINTConnectionHandle *phid);
PhidgetVINTConnectionHandle PhidgetVINTConnectionCast(void *);

const char * Phidget_strVINTPacketStatusCode(VINTPacketStatusCode code);
const char * Phidget_strPhidgetHub_PortMode(PhidgetHub_PortMode mode);
const char * Phidget_strVINTPacketType(VINTPacketType packetType);
const char * Phidget_strVINTDeviceCommand(VINTDeviceCommand command);

PhidgetReturnCode Phidget_setHubPortSpeed_internal(mosiop_t iop, PhidgetChannelHandle channel, uint32_t speed);

#ifdef NDEBUG
#define vintlogerr(...) PhidgetLog_loge(NULL, 0, __func__, "phidget22vint", PHIDGET_LOG_ERROR, __VA_ARGS__)
#define vintloginfo(...) PhidgetLog_loge(NULL, 0, __func__, "phidget22vint", PHIDGET_LOG_INFO, __VA_ARGS__)
#define vintlogwarn(...) PhidgetLog_loge(NULL, 0, __func__, "phidget22vint", PHIDGET_LOG_WARNING, __VA_ARGS__)
#define vintlogdebug(...)
#define vintlogverbose(...)
#define vintlogbufferverbose(...)
#else
#define vintlogerr(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22vint", PHIDGET_LOG_ERROR, __VA_ARGS__)
#define vintloginfo(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22vint", PHIDGET_LOG_INFO, __VA_ARGS__)
#define vintlogwarn(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22vint", PHIDGET_LOG_WARNING, __VA_ARGS__)
#define vintlogdebug(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22vint", PHIDGET_LOG_DEBUG, __VA_ARGS__)
#define vintlogverbose(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22vint", PHIDGET_LOG_VERBOSE, __VA_ARGS__)
#define vintlogbufferverbose(msg, datalen, databuf) vintlogverbose("%s"LOGBUFFER_STR, msg, LOGBUFFER_ARGS(datalen, databuf))
#endif

#endif
