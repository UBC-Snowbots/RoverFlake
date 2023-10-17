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

#ifndef __CZEROCONF
#define __CZEROCONF

#include "mos/mos_net.h"
#include "mos/kv/kv.h"

#if ZEROCONF_SUPPORT

typedef struct _ZeroconfListener *ZeroconfListenerHandle;

#define ZEROCONF_MAX_TXTRECORD	1024

#if defined(_LINUX) || defined(_FREEBSD)
#define _AVAHI	1
#include "avahi-common/address.h"
#include "avahi-client/lookup.h"
#include "avahi-client/publish.h"
#else /* LINUX */
#include "ext/include/dns_sd.h"
#endif

#ifdef _AVAHI
typedef enum {
	ZCP_IPv4 = AVAHI_PROTO_INET,
	ZCP_IPv6 = AVAHI_PROTO_INET6,
	ZCP_UNSPEC = AVAHI_PROTO_UNSPEC
} Zeroconf_Protocol;
#else
typedef enum {
	ZCP_IPv4 = kDNSServiceProtocol_IPv4,
	ZCP_IPv6 = kDNSServiceProtocol_IPv6,
	ZCP_UNSPEC = 0
} Zeroconf_Protocol;
#endif

typedef struct _ZeroconfPublish {
#ifdef _AVAHI
	AvahiEntryGroup		*ref;
	AvahiEntryGroupState state;
	mos_mutex_t			lock;
	mos_cond_t			cond;
#else
	DNSServiceRef		ref;
#endif
	char				*host;
	char				*type;
} ZeroconfPublish, *ZeroconfPublishHandle;

#define ZCL_RUN		0x01
#define ZCL_RUNNING	0x02
#define ZCL_STOPPED	0x04
#define ZCL_ERROR	0x08

/* handle, context, added, interface, protocol, name, type, domain */
typedef void(*ZeroconfListener_t)(ZeroconfListenerHandle, void *, int, int, Zeroconf_Protocol, const char *,
  const char *, const char *, const char *);

typedef struct _ZeroconfListener {
	int 				flags;
#ifdef _AVAHI
	AvahiServiceBrowser	*sb;
#else
	DNSServiceRef		ref;		/* browser */
	mos_mutex_t			lock;
	mos_cond_t			cond;
#endif
	char				*type;
	mos_task_t			task;
	ZeroconfListener_t	listener;
	void				*listenerctx;
} ZeroconfListener;

char *Zeroconf_unescape_label(const char **name, char *dest, size_t size);

/*
 * We take the name, hostname, type and domain in lookup because bonjour and avahi differ in the browse
 * and resolve provide, and it hasn't been worth the effort to fix after the initial implementation.
 *
 * Really, the hostname should be resolved and return by this call, not taken as a parameter.
 */
PhidgetReturnCode Zeroconf_lookup(ZeroconfListenerHandle, int, Zeroconf_Protocol, const char *, const char *,
  const char *, const char *, Zeroconf_Protocol, mos_sockaddr_list_t **, uint16_t *, kv_t **);
PhidgetReturnCode Zeroconf_addr_lookup(const char *, Zeroconf_Protocol, mos_sockaddr_list_t **);
PhidgetReturnCode Zeroconf_listen(ZeroconfListenerHandle *, const char *, ZeroconfListener_t, void *);
void Zeroconf_listenclose(ZeroconfListenerHandle *);
PhidgetReturnCode Zeroconf_publish(ZeroconfPublishHandle *, const char *, const char *, const char *, int, kv_t *);
PhidgetReturnCode Zeroconf_unpublish(ZeroconfPublishHandle *handle);
#endif
#endif
