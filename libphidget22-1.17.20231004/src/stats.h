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

#ifndef _STATS_H_
#define _STATS_H_

#include "phidget.h"

typedef struct _phidstat {
	const char			*name;
	uint32_t			count;
	RB_ENTRY(_phidstat)	link;
} phidstat_t;

int phidstat_compare(phidstat_t *a, phidstat_t *b);

typedef RB_HEAD(phidstats, _phidstat) phidstats_t;
RB_PROTOTYPE(phidstats, _phidstat, link, phidstat_compare);

void PhidgetStatsInit(void);
void PhidgetStatsFini(void);

PhidgetReturnCode incPhidgetStat(const char *key);
PhidgetReturnCode decPhidgetStat(const char *key);
PhidgetReturnCode getPhidgetStat(const char *key, uint32_t *cnt);
PhidgetReturnCode setPhidgetStat(const char *key, uint32_t cnt);
PhidgetReturnCode getPhidgetStatKeys(const char *startkey, char *keys, size_t keyssz);

#endif /* _STATS_H_ */
