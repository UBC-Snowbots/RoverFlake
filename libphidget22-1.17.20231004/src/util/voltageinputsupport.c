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

#include "phidgetbase.h"
#include "util/voltageinputsupport.h"

// Access the PhidgetVoltageInputSupport struct via the channel private pointer
#define VOLTAGEINPUT_SUPPORT(ch) ((PhidgetVoltageInputSupportHandle)(((PhidgetChannelHandle)(ch))->private))

void
PhidgetVoltageInputSupport_free(PhidgetVoltageInputSupportHandle *arg) {

	if (arg == NULL || *arg == NULL)
		return;

	mos_free(*arg, sizeof(PhidgetVoltageInputSupport));
	*arg = NULL;
}

PhidgetReturnCode
PhidgetVoltageInputSupport_create(PhidgetVoltageInputSupportHandle *arg) {

	TESTPTR_PR(arg);
	*arg = mos_zalloc(sizeof(PhidgetVoltageInputSupport));

	return (EPHIDGET_OK);
}

void
PhidgetVoltageInputSupport_init(PhidgetVoltageInputSupportHandle arg) {

	assert(arg);

	arg->voltageBufferIndex = 0;
	arg->voltageBufferReady = 0;
	arg->motionSensorCountdown = 0;
	arg->motionSensorBaseline = PUNK_DBL;
}

// Access the PhidgetIRSupport struct via the channel private pointer
#define VOLTAGEINPUT_SUPPORT(ch) ((PhidgetVoltageInputSupportHandle)(((PhidgetChannelHandle)(ch))->private))

void PhidgetVoltageInputSupport_updateVoltageBuffer(PhidgetVoltageInputSupportHandle voltageInputSupport, double voltage) {

	voltageInputSupport->voltageBuffer[voltageInputSupport->voltageBufferIndex] = voltage;

	voltageInputSupport->voltageBufferIndex++;
	voltageInputSupport->voltageBufferIndex %= VOLTAGE_BUFFER_LEN;
	if (voltageInputSupport->voltageBufferIndex == 0)
		voltageInputSupport->voltageBufferReady = 1;
}