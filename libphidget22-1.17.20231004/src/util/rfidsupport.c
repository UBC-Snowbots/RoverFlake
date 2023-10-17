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
#include "util/rfidsupport.h"

// Access the PhidgetRFIDSupport struct via the channel private pointer
#define RFID_SUPPORT(ch) ((PhidgetRFIDSupportHandle)(((PhidgetChannelHandle)(ch))->private))


/*
 * Public API
 */

PhidgetReturnCode
RFIDSupport_setLatestTagString(mosiop_t iop, PhidgetChannelHandle ch, const char *tagString) {

	PhidgetRFIDSupportHandle rfidSupport = RFID_SUPPORT(ch);

	mos_strncpy(rfidSupport->latestTagString, tagString, sizeof(rfidSupport->latestTagString) - 1);

	return EPHIDGET_OK;
}

PhidgetReturnCode
RFIDSupport_waitForTag(mosiop_t iop, PhidgetChannelHandle ch, const char *tagString, int timeout, mos_mutex_t* tagLock) {

	PhidgetRFIDSupportHandle rfidSupport = RFID_SUPPORT(ch);

	//Wait for this tag to show up
	while (timeout > 0) {
		if (tagLock == NULL)
			PhidgetLock(ch);
		else
			mos_mutex_lock(tagLock);

		if (!strncmp(rfidSupport->latestTagString, tagString, RFIDDevice_MAX_TAG_STRING_LEN)) {
			if (tagLock == NULL)
				PhidgetUnlock(ch);
			else
				mos_mutex_unlock(tagLock);
			return (EPHIDGET_OK);
		}
		if (tagLock == NULL)
			PhidgetUnlock(ch);
		else
			mos_mutex_unlock(tagLock);
		mos_usleep(50000);
		timeout -= 50;
	}

	return (MOS_ERROR(iop, EPHIDGET_TIMEOUT, "Timed out waiting for tag to appear after writing. Try again."));
}

void
PhidgetRFIDSupport_free(PhidgetRFIDSupportHandle *arg) {

	if (arg == NULL || *arg == NULL)
		return;

	mos_free(*arg, sizeof(PhidgetRFIDSupport));
	*arg = NULL;
}

PhidgetReturnCode
PhidgetRFIDSupport_create(PhidgetRFIDSupportHandle *rfid) {

	TESTPTR_PR(rfid);
	*rfid = mos_zalloc(sizeof(PhidgetRFIDSupport));

	return (EPHIDGET_OK);
}

void
PhidgetRFIDSupport_init(PhidgetRFIDSupportHandle rfid) {

	assert(rfid);
}
