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

#ifndef _CPHIDGET_MACROS
#define _CPHIDGET_MACROS

#define PRC_FMT "0x%02x - %s"
#define PRC_ARGS(res) res, Phidget_strerror(res)

#define PHID_RETURN_IOP(code, iop) 									\
  ((((iop) == NULL) ? Phidget_setLastError((code), NULL) : Phidget_setLastError((code), "%#N", (iop))), (mos_iop_release(&(iop))), (code))
#define PHID_RETURN_ERRSTR(code, ...) 								\
  ((Phidget_setLastError((code), __VA_ARGS__)), (code))
#define PHID_RETURN(code) 											\
  ((Phidget_setLastError((code), NULL)), (code))

#define TESTPTR(arg)		do { if (arg == NULL) return (EPHIDGET_INVALIDARG); } while (0)
#define TESTPTR_PR(arg)		do { if (arg == NULL) return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "'" #arg "' argument cannot be NULL.")); } while (0)

#define TESTRANGE_PR(newVal, fmt, min, max)	do {											\
	if (newVal < (min) || newVal > (max)) return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Value must be in range: " fmt " - " fmt ".", min, max));	\
} while (0)
#define TESTRANGE_IOP(iop, fmt, newVal, min, max)	do {										\
	if (newVal < (min) || newVal > (max)) return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Value must be in range: " fmt " - " fmt ".", min, max));	\
} while (0)

#define TESTBOOL_IOP(iop, newVal)	do {															\
	if (newVal != (PFALSE) && newVal != (PTRUE)) return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Value must be a boolean."));	\
} while (0)

#define TESTCHANNELCLASS_PR(ch, def)	\
	do { if (((PhidgetChannelHandle)(ch))->class != def) return (PHID_RETURN(EPHIDGET_WRONGDEVICE)); } while (0)

#define TESTATTACHED(phid) do {										\
	if (!ISATTACHED(phid))												\
		return (EPHIDGET_NOTATTACHED);						\
} while (0)
#define TESTATTACHED_PR(phid) do {										\
	if (!ISATTACHED(phid))												\
		return (PHID_RETURN(EPHIDGET_NOTATTACHED));						\
} while (0)

#define TESTATTACHEDORDETACHING_PR(phid) do {							\
	if (!ISATTACHEDORDETACHING(phid))									\
		return (PHID_RETURN(EPHIDGET_NOTATTACHED));						\
} while (0)

#define TESTCHANNEL_PR(ch) do {											\
	if (!ISCHANNEL(ch))													\
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "'" #ch "' must be a valid channel object."));				\
} while (0)

#define FIRECH(ch, ename, ...) do {																	\
	if ((ch)->ename)																				\
		(ch)->ename((ch), (ch)->ename##Ctx, __VA_ARGS__);											\
} while (0)

#define FIRECH0(ch, ename) do {																		\
	if ((ch)->ename)																				\
		(ch)->ename((ch), (ch)->ename##Ctx);														\
} while (0)

#define FIRE_PROPERTYCHANGE(phid, prop) do {														\
	PhidgetChannelHandle _ch_;																		\
	_ch_ = PhidgetChannelCast(phid);																\
	if (_ch_ && _ch_->PropertyChange)																\
		_ch_->PropertyChange((PhidgetHandle)_ch_, _ch_->PropertyChangeCtx, (prop));					\
} while (0)

#define DISPATCH_PROPERTYCHANGE(phid, prop) \
	bridgeSendToChannel((PhidgetChannelHandle)ch, BP_PROPERTYCHANGE, 1, "%s", (prop))

#define FIRE_ERROR(phid, ecode, ...)	do {														\
	PhidgetChannelHandle _ch_;																		\
	char _errbuf_[1024];																			\
	_ch_ = PhidgetChannelCast(phid);																\
	if (_ch_ && _ch_->Error) {																		\
		mos_snprintf(_errbuf_, sizeof(_errbuf_), __VA_ARGS__);										\
		_ch_->Error((PhidgetHandle)_ch_, _ch_->ErrorCtx, ecode, _errbuf_);							\
	}																								\
} while (0)

#define SEND_ERROR_EVENT_THROTTLED(phid, ecode, ...)	do {										\
	PhidgetChannelHandle _ch_;																		\
	char _errbuf_[1024];																			\
	_ch_ = PhidgetChannelCast(phid);																\
	if (_ch_) {																						\
		mos_snprintf(_errbuf_, sizeof(_errbuf_), __VA_ARGS__);										\
		PhidgetChannel_sendErrorEventThrottled(_ch_, ecode, _errbuf_);								\
	}																								\
} while (0)

#define SEND_ERROR_EVENT(phid, ecode, ...)	do {													\
	PhidgetChannelHandle _ch_;																		\
	char _errbuf_[1024];																			\
	_ch_ = PhidgetChannelCast(phid);																\
	if (_ch_) {																						\
		mos_snprintf(_errbuf_, sizeof(_errbuf_), __VA_ARGS__);										\
		PhidgetChannel_sendErrorEvent(_ch_, ecode, _errbuf_);										\
	}																								\
} while (0)

#define DEVICECREATE_BODY(pname, pdef)											\
	Phidget##pname##Handle phid;												\
	TESTPTR(phidp);																\
	phid = mos_zalloc(sizeof (*phid));											\
	phidget_init((PhidgetHandle)phid, PHIDGET_DEVICE, 							\
		(PhidgetDelete_t)Phidget##pname##_free);								\
	phid->phid.deviceInfo.class = pdef;											\
	phid->phid.bridgeInput = Phidget##pname##_bridgeInput;						\
	phid->phid.initAfterOpen = Phidget##pname##_initAfterOpen;					\
	phid->phid.dataInput = Phidget##pname##_dataInput;							\
	*phidp = phid

#define CHANNELCREATE_BODY(pname, pdef)											\
	Phidget##pname##Handle phid;												\
	TESTPTR_PR(phidp);															\
	phid = mos_zalloc(sizeof (*phid));											\
	phidget_init((PhidgetHandle)phid, PHIDGET_CHANNEL, 							\
		(PhidgetDelete_t)Phidget##pname##_free);								\
	phid->phid.class = pdef;													\
	phid->phid.initAfterOpen = Phidget##pname##_initAfterOpen;					\
	phid->phid.setDefaults = Phidget##pname##_setDefaults;						\
	phid->phid.fireInitialEvents = Phidget##pname##_fireInitialEvents;			\
	phid->phid.hasInitialState = Phidget##pname##_hasInitialState;				\
	phid->phid.bridgeInput = Phidget##pname##_bridgeInput;						\
	phid->phid.errorHandler = Phidget##pname##_errorHandler;					\
	phid->phid.getStatus = Phidget##pname##_getStatus;							\
	phid->phid.setStatus = Phidget##pname##_setStatus;							\
	MTAILQ_INIT(&phid->phid.netconns);											\
	mos_mutex_init(&phid->phid.netconnslk);										\
	phid->phid.openInfo = mallocPhidgetOpenInfo();								\
	*phidp = phid

#define DEVBRIDGEINPUT(ch, bp)	deviceBridgeInput((ch), (bp))

#define ASYNC_CALLBACK_ON_ERROR(stat)	do {									\
	PhidgetReturnCode res1 = (stat);											\
	if (res1 != EPHIDGET_OK && fptr)											\
		fptr((PhidgetHandle)phid, ctx, res1);									\
} while (0)

#define LOCALCALL(x)	do {													\
	if (!isNetworkPhidget(phid)) res = (x);										\
	else res = EPHIDGET_OK;														\
} while (0)

#define ISCHANNEL(phid)				(phid->type == PHIDGET_CHANNEL)
#define ISATTACHED(phid)			(PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG) == PHIDGET_ATTACHED_FLAG)
#define ISATTACHEDORATTACHING(phid)	(PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG) ? 1 : 0)
#define ISATTACHEDORDETACHING(phid)	(PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG | PHIDGET_DETACHING_FLAG) ? 1 : 0)
#define ISATTACHEDDONE(phid)		(PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG) == PHIDGET_ATTACHED_FLAG)
#define ISOPEN(phid)				(PhidgetCKFlags(phid, PHIDGET_OPEN_FLAG) == PHIDGET_OPEN_FLAG)

#define _ISATTACHEDDONE(phid)			(((phid)->__flags & (PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG)) == PHIDGET_ATTACHED_FLAG)
#define _ISATTACHED(phid)				(((phid)->__flags & PHIDGET_ATTACHED_FLAG) == PHIDGET_ATTACHED_FLAG)
#define _ISATTACHEDORATTACHING(phid)	(((phid)->__flags & (PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG)) ? 1 : 0)
#define _ISATTACHEDORDETACHING(phid)	(((phid)->__flags & (PHIDGET_ATTACHED_FLAG | PHIDGET_DETACHING_FLAG)) ? 1 : 0)
#define _ISOPEN(phid)					(((phid)->__flags & PHIDGET_OPEN_FLAG) == PHIDGET_OPEN_FLAG)
#define _ISDISPATCH(phid)				(((phid)->__flags & PHIDGET_DISPATCH_FLAG) == PHIDGET_DISPATCH_FLAG)
#define _ISDETACHING(phid)				(((phid)->__flags & PHIDGET_DETACHING_FLAG) == PHIDGET_DETACHING_FLAG)
#define _ISATTACHING(phid)				(((phid)->__flags & PHIDGET_ATTACHING_FLAG) == PHIDGET_ATTACHING_FLAG)
#define _HASINITIALSTATE(phid)			(((phid)->__flags & PHIDGET_HASINITIALSTATE_FLAG) == PHIDGET_HASINITIALSTATE_FLAG)

#define GETDEVICE(device, phid) do {											\
	device = PhidgetDeviceCast(phid);											\
	if (device == NULL)															\
		device = getParent(phid);												\
	else																		\
		PhidgetRetain(device);													\
} while (0);

#define CHANNELNOTDEVICE_PR(ch, phid) do {						\
	PhidgetDeviceHandle __devh__;							\
	if (phid == NULL)										\
		return (PHID_RETURN(EPHIDGET_INVALIDARG));			\
	(ch) = PhidgetChannelCast((phid));						\
	if ((ch) == NULL) {										\
		__devh__ = PhidgetDeviceCast(phid);					\
		if (__devh__ != NULL)								\
			return (PHID_RETURN(EPHIDGET_UNSUPPORTED));		\
		return (PHID_RETURN(EPHIDGET_INVALIDARG));			\
	}														\
} while (0)

#endif
