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

#ifndef _PHIDGET_CONFIG_H_
#define _PHIDGET_CONFIG_H_

#ifndef EXTERNALPROTO
#include "phidgetbase.h"
#include "constants.h"
#endif

#include "mos/bsdtree.h"

#define PHIDGETCONFIG_TOKEN_MAX		4096
#define PHIDGETCONFIG_PATH_MAX		512
#define PHIDGETCONFIG_COMP_MAX		64
#define PHIDGETCONFIG_STR_MAX		8192

#define PHIDGETCONFIG_CREATEMISSING	0x01
#define PHIDGETCONFIG_INARRAY		0x80

typedef enum pconftype {
	PHIDGETCONFIG_BLOCK = 1,
	PHIDGETCONFIG_ARRAY,
	PHIDGETCONFIG_STRING,
	PHIDGETCONFIG_NUMBER,
	PHIDGETCONFIG_U64,
	PHIDGETCONFIG_I64,
	PHIDGETCONFIG_BOOLEAN,
	PHIDGETCONFIG_NULL
} pconftype_t;

typedef union pconfvalue	{
	double					num;
	int64_t					i64;
	uint64_t				u64;
	char					*str;
	const char				*cstr;
	int						bl;
} pconfvalue_t;

struct pconfentry;
typedef RB_HEAD(pconfentries, pconfentry) pconfentries_t;

typedef union pconfentryvalue	{
	pconfvalue_t			val;
	pconfentries_t			entries;
} pconfentryvalue_t;

typedef struct pconfentry {
	pconftype_t				type;
	int						flags;
	char					*key;
	int32_t					cnt;
	pconfentryvalue_t		value;
	RB_ENTRY(pconfentry)	link;
} pconfentry_t;

#define e_i64	value.val.i64
#define e_u64	value.val.u64
#define e_num	value.val.num
#define e_bool	value.val.bl
#define e_str	value.val.str
#define e_cstr	value.val.cstr

int phidgetentrycompare(pconfentry_t *, pconfentry_t *);

RB_PROTOTYPE(pconfentries, pconfentry, link, phidgetentrycompare)

typedef struct _pconf {
	void			*ctx;		/* external context pointer */
#define pc_root root->value.entries
	int				flags;
	pconfentry_t	*root;
} pconf_t;

API_PRETURN_HDR pconf_create(pconf_t **);
API_PRETURN_HDR pconf_parsejson(pconf_t **, const char *, size_t);
API_PRETURN_HDR pconf_renderjson(pconf_t *, char *, size_t);
API_PRETURN_HDR pconf_renderpc(pconf_t *, char *, size_t);
API_PRETURN_HDR pconf_release(pconf_t **);
API_PRETURN_HDR pconf_getentryv(pconf_t *, int, pconfentry_t **, const char *, va_list);
API_PRETURN_HDR pconf_getentry(pconf_t *, int, pconfentry_t **, const char *, ...) PRINTF_LIKE(4, 5);
API_PRETURN_HDR pconf_merge(pconf_t *, pconf_t **, const char *, const char *, ...) PRINTF_LIKE(4, 5);
API_PRETURN_HDR pconf_setcreatemissing(pconf_t *, int);
API_PRETURN_HDR pconf_remove(pconf_t *, const char *, ...) PRINTF_LIKE(2, 3);
API_CRETURN_HDR pconf_getentryname(pconf_t *, int, const char *, ...) PRINTF_LIKE(3, 4);
API_I32RETURN_HDR pconf_getcount(pconf_t *, const char *, ...) PRINTF_LIKE(2, 3);
API_PRETURN_HDR pconf_detecttype(const char *, pconftype_t *, pconfvalue_t *);
API_PRETURN_HDR pconf_cast(const char *, pconftype_t, pconfvalue_t *);

API_PRETURN_HDR pconf_addblockv(pconf_t *, const char *, va_list);
API_PRETURN_HDR pconf_addblock(pconf_t *, const char *, ...) PRINTF_LIKE(2, 3);
API_PRETURN_HDR pconf_addarrayv(pconf_t *, const char *, va_list);
API_PRETURN_HDR pconf_addarray(pconf_t *, const char *, ...) PRINTF_LIKE(2, 3);
API_PRETURN_HDR pconf_addstrv(pconf_t *, const char *, const char *, va_list);
API_PRETURN_HDR pconf_addstr(pconf_t *, const char *, const char *, ...) PRINTF_LIKE(3, 4);
API_PRETURN_HDR pconf_addnumv(pconf_t *, double, const char *, va_list);
API_PRETURN_HDR pconf_addnum(pconf_t *, double, const char *, ...) PRINTF_LIKE(3, 4);
API_PRETURN_HDR pconf_addiv(pconf_t *, int64_t, const char *, va_list);
API_PRETURN_HDR pconf_addi(pconf_t *, int64_t, const char *, ...) PRINTF_LIKE(3, 4);
API_PRETURN_HDR pconf_adduv(pconf_t *, uint64_t, const char *, va_list);
API_PRETURN_HDR pconf_addu(pconf_t *, uint64_t, const char *, ...) PRINTF_LIKE(3, 4);
API_PRETURN_HDR pconf_addboolv(pconf_t *, int, const char *, va_list);
API_PRETURN_HDR pconf_addbool(pconf_t *, int, const char *, ...) PRINTF_LIKE(3, 4);
API_PRETURN_HDR pconf_addv(pconf_t *, const char *, const char *, va_list);
API_PRETURN_HDR pconf_add(pconf_t *, const char *, const char *, ...) PRINTF_LIKE(3, 4);
API_PRETURN_HDR pconf_updatev(pconf_t *, const char *, const char *, va_list);
API_PRETURN_HDR pconf_update(pconf_t *, const char *, const char *, ...) PRINTF_LIKE(3, 4);
API_PRETURN_HDR pconf_setv(pconf_t *, const char *, const char *, va_list);
API_PRETURN_HDR pconf_set(pconf_t *, const char *, const char *, ...) PRINTF_LIKE(3, 4);
API_PRETURN_HDR pconf_removev(pconf_t *, const char *, va_list);

API_PRETURN_HDR pconf_tostring(pconf_t *, char *, size_t, const char *, ...) PRINTF_LIKE(4, 5);
API_I32RETURN_HDR pconf_get32v(pconf_t *, int32_t, const char *, va_list);
API_I32RETURN_HDR pconf_get32(pconf_t *, int32_t, const char *, ...) PRINTF_LIKE(3, 4);
API_U32RETURN_HDR pconf_getu32v(pconf_t *, uint32_t, const char *, va_list);
API_U32RETURN_HDR pconf_getu32(pconf_t *, uint32_t, const char *, ...) PRINTF_LIKE(3, 4);
API_I64RETURN_HDR pconf_get64v(pconf_t *, int64_t, const char *, va_list);
API_I64RETURN_HDR pconf_get64(pconf_t *, int64_t, const char *, ...) PRINTF_LIKE(3, 4);
API_U64RETURN_HDR pconf_getu64v(pconf_t *, uint64_t, const char *, va_list);
API_U64RETURN_HDR pconf_getu64(pconf_t *, uint64_t, const char *, ...) PRINTF_LIKE(3, 4);
API_DRETURN_HDR pconf_getdblv(pconf_t *, double, const char *, va_list);
API_DRETURN_HDR pconf_getdbl(pconf_t *, double, const char *, ...) PRINTF_LIKE(3, 4);
API_CRETURN_HDR pconf_getstrv(pconf_t *, const char *, const char *, va_list);
API_CRETURN_HDR pconf_getstr(pconf_t *, const char *, const char *, ...) PRINTF_LIKE(3, 4);
API_IRETURN_HDR pconf_getboolv(pconf_t *, int, const char *, va_list);
API_IRETURN_HDR pconf_getbool(pconf_t *, int, const char *, ...) PRINTF_LIKE(3, 4);

API_IRETURN_HDR pconf_existsv(pconf_t *, const char *, va_list);
API_IRETURN_HDR pconf_exists(pconf_t *, const char *, ...) PRINTF_LIKE(2, 3);
API_IRETURN_HDR pconf_isblockv(pconf_t *, const char *, va_list);
API_IRETURN_HDR pconf_isblock(pconf_t *, const char *, ...) PRINTF_LIKE(2, 3);
API_IRETURN_HDR pconf_isarrayv(pconf_t *, const char *, va_list);
API_IRETURN_HDR pconf_isarray(pconf_t *, const char *, ...) PRINTF_LIKE(2, 3);

#endif /* _PHIDGET_CONFIG_H_ */
