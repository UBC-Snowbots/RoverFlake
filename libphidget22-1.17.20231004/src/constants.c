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
#include "constantsinternal.h"
#include "constants.h"

#define LIBRARY_VERSION "1.17"

#ifdef DEBUG
const char *LibraryVersion = "Phidget22 Debug - Version " LIBRARY_VERSION " - Built " __DATE__ " " __TIME__;
#else
const char *LibraryVersion = "Phidget22 - Version " LIBRARY_VERSION " - Built " __DATE__ " " __TIME__;
#endif

#ifdef P22_LIB_VERSION
const char *LibraryVersionNumber = "" P22_LIB_VERSION;
#else
const char *LibraryVersionNumber = "" LIBRARY_VERSION;
#endif

#if UINTPTR_MAX == 0xffffffff
#define P22_WIDTH "32-bit"
#elif UINTPTR_MAX == 0xffffffffffffffff
#define P22_WIDTH "64-bit"
#else
#define P22_WIDTH "?""?-bit"
#endif

#if defined(_WINDOWS)

#if defined(_WINDOWSUAP)
#define P22_OS "Windows (UWP)"
#else
#define P22_OS "Windows"
#endif

#elif defined(_MACOSX)

#if defined(_IPHONE)
#define P22_OS "iOS"
#else
#define P22_OS "macOS"
#endif

#elif defined(_ANDROID)
#define P22_OS "Android"
#elif defined(_FREEBSD)
#define P22_OS "FreeBSD"
#elif defined(_LINUX)
#define P22_OS "Linux"
#else
#define P22_OS "Unknown"
#endif

const char *LibrarySystem = "" P22_OS " (" P22_WIDTH ")";
