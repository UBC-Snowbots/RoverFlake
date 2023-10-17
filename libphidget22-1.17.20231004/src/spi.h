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

#ifndef __PHIDGET_SPI_H
#define __PHIDGET_SPI_H

// NOTE: SPI transfers are 128 bytes with 3 byte overhead
#define MAX_SPI_PACKET_SIZE	125

typedef struct {
	PHIDGET_STRUCT_START
	mos_task_t task;
	mos_mutex_t rlock;
	mos_cond_t rcond;
	int run;
	int dev;
} PhidgetSPIConnection, *PhidgetSPIConnectionHandle;

PhidgetReturnCode PhidgetSPIConnectionCreate(PhidgetSPIConnectionHandle *);
PhidgetSPIConnectionHandle PhidgetSPIConnectionCast(void *);

MOS_TASK_RESULT PhidgetSPIReadThreadFunction(void *);
void stopSPIReadThread(PhidgetSPIConnectionHandle);
void joinSPIReadThread(PhidgetSPIConnectionHandle);

PhidgetReturnCode PhidgetSPIGetVINTDevicesString(char *, size_t);
PhidgetReturnCode PhidgetSPIOpenHandle(PhidgetDeviceHandle);
PhidgetReturnCode PhidgetSPICloseHandle(PhidgetSPIConnectionHandle);
PhidgetReturnCode PhidgetSPIReadPacket(PhidgetSPIConnectionHandle, unsigned char *, size_t *);
PhidgetReturnCode PhidgetSPISendPacket(mosiop_t iop, PhidgetSPIConnectionHandle, const unsigned char *, size_t);
PhidgetReturnCode PhidgetSPISetLabel(PhidgetDeviceHandle, char *);
PhidgetReturnCode PhidgetSPIRefreshLabelString(PhidgetDeviceHandle);

PhidgetReturnCode openAttachedSPIDevice(PhidgetDeviceHandle);

#ifdef SPI_SUPPORT
PhidgetReturnCode populateAttachedSPIDevices(void);
PhidgetReturnCode clearAttachedSPIDevices(void);
#endif /* SPI_SUPPORT */

#endif /* __PHIDGET_SPI_H */
