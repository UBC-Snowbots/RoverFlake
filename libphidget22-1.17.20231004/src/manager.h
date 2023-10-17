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

#ifndef _PHIDGET_MANAGER_H_
#define _PHIDGET_MANAGER_H_

#include "phidget.h"

typedef struct _PhidgetManager *PhidgetManagerHandle;

/* Methods */
API_PRETURN_HDR PhidgetManager_create				(PhidgetManagerHandle *phidm);
API_PRETURN_HDR PhidgetManager_delete				(PhidgetManagerHandle *phidm);

API_PRETURN_HDR PhidgetManager_open					(PhidgetManagerHandle phidm);
API_PRETURN_HDR PhidgetManager_close				(PhidgetManagerHandle phidm);

/* Events */
typedef void (CCONV *PhidgetManager_OnAttachCallback)	(PhidgetManagerHandle phidm, void *ctx, PhidgetHandle phid);
typedef void (CCONV *PhidgetManager_OnDetachCallback)	(PhidgetManagerHandle phidm, void *ctx, PhidgetHandle phid);
typedef void (CCONV *PhidgetManager_OnErrorCallback)	(PhidgetManagerHandle phidm, void *ctx, Phidget_ErrorEventCode errorCode, const char *errorString);

API_PRETURN_HDR PhidgetManager_setOnAttachHandler	(PhidgetManagerHandle phidm, PhidgetManager_OnAttachCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetManager_setOnDetachHandler	(PhidgetManagerHandle phidm, PhidgetManager_OnDetachCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetManager_setOnErrorHandler	(PhidgetManagerHandle phidm, PhidgetManager_OnErrorCallback fptr, void *ctx);

#ifndef EXTERNALPROTO

typedef struct _PhidgetManager {
	PHIDGET_STRUCT_START
	PhidgetManager_OnAttachCallback onAttach;
	void *onAttachCtx;
	PhidgetManager_OnDetachCallback onDetach;
	void *onDetachCtx;
	PhidgetManager_OnErrorCallback onError;
	void *onErrorCtx;

	MTAILQ_ENTRY(_PhidgetManager) link;
} PhidgetManager;

MTAILQ_HEAD(phidgetmanager_list, _PhidgetManager);
extern struct phidgetmanager_list phidgetManagerList;

PhidgetReturnCode sendAttachVisitor(PhidgetDeviceHandle, const PhidgetUniqueChannelDef *, int index,
  int uniqueIndex, void *ctx);
PhidgetReturnCode sendDetachVisitor(PhidgetDeviceHandle, const PhidgetUniqueChannelDef *, int index,
  int uniqueIndex, void *ctx);

#define FOREACH_MANAGER(phidm)				MTAILQ_FOREACH((phidm), &phidgetManagerList, link)
#define FOREACH_MANAGER_SAFE(phidm, tmp)	\
	MTAILQ_FOREACH_SAFE((phidm), &phidgetManagerList, link, (PhidgetManager *)(tmp))

void PhidgetLockManagers(void);
void PhidgetUnlockManagers(void);
PhidgetManagerHandle PhidgetManagerCast(void *);

void handleDetaches(void);
PhidgetReturnCode PhidgetManager_poll(void);
void channelAttach(PhidgetChannelHandle channel);
void channelDetach(PhidgetChannelHandle channel);
void channelClose(PhidgetChannelHandle channel);
PhidgetReturnCode deviceAttach(PhidgetDeviceHandle device, int needDevicesLock);
void deviceDetach(PhidgetDeviceHandle device);
PhidgetReturnCode walkDeviceChannels(PhidgetDeviceHandle, deviceChannelVisitor_t, void *ctx);

void queueDeviceAttach(PhidgetDeviceHandle device);
void queueDeviceDetach(PhidgetDeviceHandle device);
void queueVintDeviceDetach(PhidgetDeviceHandle hub, int index);
void queueDeviceUSBErr(PhidgetDeviceHandle device);
void runAttachDetachQueue(void);

void PhidgetManagerInit(void);
void PhidgetManagerFini(void);

#endif //#ifndef EXTERNALPROTO

#endif /* _PHIDGET_MANAGER_H_ */
