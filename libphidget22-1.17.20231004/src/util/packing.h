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

#ifndef __CPACKING
#define __CPACKING

/*
 * This file is used by firmware, please be careful with includes etc.
 */

#ifdef _WINDOWS
#include "phidgetbase.h"
#else
#include <stddef.h>
#endif

#include <stdint.h>

void pack(size_t, uint8_t *, uint64_t);

void pack16(uint8_t *, uint16_t);
uint16_t unpack16(const uint8_t *);

void pack32(uint8_t *, uint32_t);
uint32_t unpack32(const uint8_t *);

void pack64(uint8_t *, uint64_t);
uint64_t unpack64(const uint8_t *);

void packfloat(uint8_t *, float);
float unpackfloat(const uint8_t *);

void packdouble(uint8_t *, double);
double unpackdouble(const uint8_t *);

void pack16to16xS(uint8_t *, uint16_t, unsigned int, int);
void packfltto16xS(uint8_t *, float, unsigned int);

double unpacku16xS(const uint8_t *, unsigned int);
double unpack16xS(const uint8_t *, unsigned int);

double unpack32xS(const uint8_t *buf, unsigned int shift);
double unpacku32xS(const uint8_t *buf, unsigned int shift);

void doubleToUnsignedFixedPoint(double, unsigned char *, int, int, int);

#endif /* __CPACKING */
