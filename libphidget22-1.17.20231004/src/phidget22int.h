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

#ifndef PHIDGET22_H
#define PHIDGET22_H

#ifdef _WINDOWS
#ifdef PHIDGET22_EXPORTS
#ifndef PHIDGET22_API
#define PHIDGET22_API
#endif
#else
#ifndef PHIDGET22_API
#define PHIDGET22_API __declspec(dllimport)
#endif
#endif
#ifndef CCONV
#define CCONV __stdcall
#endif
#ifndef PRINTF_LIKE
#define PRINTF_LIKE(a, b)
#endif

#else
#ifndef PHIDGET22_API
#define PHIDGET22_API
#endif
#ifndef CCONV
#define CCONV
#endif
#ifndef PRINTF_LIKE
#ifdef CHK_FORMAT_STRINGS
#define PRINTF_LIKE(a, b) __attribute__((format(printf, a, b)))
#else
#define PRINTF_LIKE(a, b)
#endif
#endif

#endif

#ifndef API_PRETURN_HDR
#define API_PRETURN_HDR PHIDGET22_API PhidgetReturnCode CCONV
#endif
#ifndef API_VRETURN_HDR
#define API_VRETURN_HDR PHIDGET22_API void CCONV
#endif
#ifndef API_CRETURN_HDR
#define API_CRETURN_HDR PHIDGET22_API const char * CCONV
#endif
#ifndef API_IRETURN_HDR
#define API_IRETURN_HDR PHIDGET22_API int CCONV
#endif

#if !defined(EXTERNALPROTO) || defined(DEBUG) || defined(INTERNAL)
#define INCLUDE_PRIVATE
#else
#undef INCLUDE_PRIVATE
#endif
#if defined(DEBUG) || defined(INTERNAL)
#define INCLUDE_UNRELEASED
#else
#undef INCLUDE_UNRELEASED
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "macros.h"
#include "phidget.h"
#include "manager.h"
#include "util/phidgetlog.h"
#include "network/network.h"
#include "enumutil.gen.h"
#include "phidget22int.gen.h"

#ifdef __cplusplus
}
#endif

#endif
