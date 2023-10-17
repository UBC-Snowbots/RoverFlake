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

#ifndef _PHIDGET_CONSTANTS_H_
#define _PHIDGET_CONSTANTS_H_

#define PUNK_BOOL					0x02			/* Unknown Boolean */
#define PUNK_INT8					INT8_MAX		/* Unknown Short   (8-bit) */
#define PUNK_UINT8					UINT8_MAX		/* Unknown Short   (8-bit unsigned) */
#define PUNK_INT16					INT16_MAX		/* Unknown Short   (16-bit) */
#define PUNK_UINT16					UINT16_MAX		/* Unknown Short   (16-bit unsigned) */
#define PUNK_INT32					INT32_MAX		/* Unknown Integer (32-bit) */
#define PUNK_UINT32					UINT32_MAX		/* Unknown Integer (32-bit unsigned) */
#define PUNK_INT64					INT64_MAX		/* Unknown Integer (64-bit) */
#define PUNK_UINT64					UINT64_MAX		/* Unknown Integer (64-bit unsigned) */
#define PUNK_DBL					1e300			/* Unknown Double */
#define PUNK_FLT					1e30			/* Unknown Float */
#define PUNK_ENUM					INT32_MAX		/* Unknown Enum */
#define PUNK_SIZE					SIZE_MAX		/* Unknown size_t */

#define PFALSE		0x00	/* False. Used for boolean values. */
#define PTRUE		0x01	/* True. Used for boolean values. */

#define PRIphid "P"			/* mos_printf format string for printing a PhidgetHandle */

#endif /* _PHIDGET_CONSTANTS_H_ */
