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

#define _PHIDGET_NETWORKCODE

#include "phidgetbase.h"
#include "device/hubdevice.h"
#include "device/vintdevice.h"
#include "network/network.h"
#include "util/json.h"
#include "util/phidgetconfig.h"
#include "manager.h"
#include "mos/mos_byteorder.h"

static phidgetnetconnlist_t openServers;
static int openServersCnt;

static PhidgetReturnCode sendNetChannelAttachedVisitor(PhidgetDeviceHandle, const PhidgetUniqueChannelDef *,
  int, int, void *);
static PhidgetReturnCode sendDevicesToNewServer(PhidgetNetConnHandle);
static mos_mutex_t srvlock;
static int initialized;
static int srvstarted;

extern int _allowDataGram;

static int
isInitialized() {
	int init;

	mos_glock((void *)1);
	init = initialized;
	mos_gunlock((void *)1);

	return (init);
}
#define CHECKINITIALIZED do {			\
	if (!isInitialized())				\
		return (EPHIDGET_CLOSED);		\
} while (0)

void
ServerInit() {

	mos_glock((void *)1);
	if (initialized) {
		mos_gunlock((void *)1);
		return;
	}

	MTAILQ_INIT(&openServers);
	openServersCnt = 0;
	mos_mutex_init(&srvlock);
	initialized = 1;
	mos_gunlock((void *)1);
}

void
ServerFini() {

	mos_glock((void *)1);
	if (!initialized) {
		mos_gunlock((void *)1);
		return;
	}

	mos_mutex_destroy(&srvlock);
	initialized = 0;
	mos_gunlock((void *)1);
}

void
ServerStart() {

	mos_glock((void *)1);
	if (!initialized || srvstarted) {
		mos_gunlock((void *)1);
		return;
	}

	srvstarted = 1;
	mos_gunlock((void *)1);
}

void
ServerStop() {
	PhidgetNetConnHandle nc;

	mos_glock((void *)1);
	if (!initialized || !srvstarted) {
		mos_gunlock((void *)1);
		return;
	}
	mos_gunlock((void *)1);

	mos_mutex_lock(&srvlock);
	MTAILQ_FOREACH(nc, &openServers, openlink) {
		stopPhidgetNetConn(nc);
		/* will be removed by the task */
	}
	mos_mutex_unlock(&srvlock);

	while (!MTAILQ_EMPTY(&openServers))
		mos_usleep(10000);

	mos_glock((void *)1);
	srvstarted = 0;
	mos_gunlock((void *)1);
}

/*
 * Opens the channel on the server, and sends back the current state of that channel.
 * If an error occurs, we send back a simple reply, which is of type COMMAND/REPLY.
 * If successful, we send back CHANNEL/BRIDGEPKT.
 */
static PhidgetReturnCode
handleOpenChannel(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	const PhidgetChannelAttributeDef *def;
	PhidgetChannelHandle channel;
	PhidgetReturnCode res, res1;
	mosiop_t siop;
	int err;

	Phidget_ChannelClass cclass;
	channelid_t chid;
	uint64_t pid; /* parent device id */
	uint32_t cnt;
	int version;  /* channel version */
	int index;
	int chreset;

	if ((nc->ppmajor > 2) || (nc->ppmajor == 2 && nc->ppminor >= 4)) {
		// Starts in 2.4
		err = parseJSON((char *)req->nr_data, req->nr_len, NULL, 0,
			"%O,phid:%lu,channel:%lu,class=%d,index:%d,version:%d,chreset:%d",
			&cnt, &pid, &chid.c_id, &cclass, &index, &version, &chreset);
	} else {
		err = parseJSON((char *)req->nr_data, req->nr_len, NULL, 0,
		  "%O,phid:%lu,channel:%lu,class=%d,index:%d,version:%d",
		  &cnt, &pid, &chid.c_id, &cclass, &index, &version);
		chreset = PTRUE;
	}

	chid.c_id = mos_le64toh(chid.c_id);
	chid.c_serial = mos_le32toh(chid.c_serial);

	if (err <= 0)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "invalid json in channel open"));

	def = getPhidgetChannelAttributesByClass(cclass);
	if (def == NULL) {
		res1 = sendSimpleReply(nc, req->nr_reqseq, EPHIDGET_INVALIDARG, "Invalid Channel Class.");
		if (res1 != EPHIDGET_OK)
			return (MOS_ERROR(iop, res1, "failed to send simple reply"));
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Invalid Channel Class."));
	}

	siop = mos_iop_alloc();
	res = openServerChannel(siop, pid, cclass, index, chreset, nc, &channel, req->nr_reqseq);
	if (res != EPHIDGET_OK) {
		char *errbuf;
		errbuf = mos_malloc(4096);
		mos_snprintf(errbuf, 4096, "%#N", siop);
		// Trim trailing newline
		if (errbuf[mos_strlen(errbuf) - 1] == '\n')
			errbuf[mos_strlen(errbuf) - 1] = '\0';

		res1 = sendSimpleReply(nc, req->nr_reqseq, res, strlen(errbuf) > 0 ? errbuf : NULL);

		mos_free(errbuf, 4096);
		mos_iop_release(&siop);

		if (res1 != EPHIDGET_OK)
			return (MOS_ERROR(iop, res1, "failed to send simple reply"));

		return (res);
	}
	mos_iop_release(&siop);

	PhidgetRelease(&channel); // now the only reference is the systems reference
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
handleCloseChannel(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	PhidgetReturnCode res, res1;
	uint32_t cnt;
	uint64_t pid;
	int index;
	int err;
	int chreset;

	if ((nc->ppmajor > 2) || (nc->ppmajor == 2 && nc->ppminor >= 4)) {
		// Starts in 2.4
		err = parseJSON((char *)req->nr_data, req->nr_len, NULL, 0, "%O,phid:%lu,index:%d,chreset:%d", &cnt, &pid, &index, &chreset);
		if (err <= 0)
			return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid json in channel close"));
	} else {
		err = parseJSON((char *)req->nr_data, req->nr_len, NULL, 0, "%O,phid:%lu,index:%d", &cnt, &pid, &index);
		if (err <= 0)
			return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid json in channel close"));
		chreset = PTRUE;
	}

	res = closeServerChannel(pid, index, chreset, nc);
	res1 = sendSimpleReply(nc, req->nr_reqseq, res, NULL);
	if (res1 != EPHIDGET_OK)
		return (MOS_ERROR(iop, res1, "failed to send simple reply"));

	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to close server channel"));

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
handleBridgePacket(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	PhidgetReturnCode res, res1;
	BridgePacket *bp;

	res = parseBridgePacketJSON(nc->tokens, &bp, (char *)req->nr_data, req->nr_len);
	if (res != EPHIDGET_OK) {
		res1 = sendSimpleReply(nc, req->nr_reqseq, EPHIDGET_UNEXPECTED, "failed to parse bridge packet JSON");
		if (res1 != EPHIDGET_OK)
			return (MOS_ERROR(iop, res1, "failed to send simple reply"));
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "failed to parse bridge packet JSON"));
	}

	bridgePacketSetIsFromNet(bp, nc);

	/*
	 * Flags as an event so we do not dispatch as a command.
	 * This is never expected to happen (client do not send events to the server), but handle it anyway
	 * just incase that changes someday.
	 */
	if (req->nr_flags & NRF_EVENT)
		bridgePacketSetIsEvent(bp);

	res = dispatchServerBridgePacket(iop, nc, bp, 1, req->nr_reqseq);
	if (res != EPHIDGET_OK) {
		char *errbuf;
		errbuf = mos_malloc(4096);
		mos_snprintf(errbuf, 4096, "%#N", iop);
		// Trim trailing newline
		if (errbuf[mos_strlen(errbuf) - 1] == '\n')
			errbuf[mos_strlen(errbuf) - 1] = '\0';

		// NOTE: if dispatchServerBridgePacket succeeds, sendSimpleReply is called by the dispatcher for this msg
		res1 = sendSimpleReply(nc, req->nr_reqseq, res, strlen(errbuf) > 0 ? errbuf : NULL);

		mos_free(errbuf, 4096);

		if (res1 != EPHIDGET_OK)
			return (MOS_ERROR(iop, res1, "failed to send simple reply"));
		return (MOS_ERROR(iop, res, "failed to dispatch server bridge packet"));
	}
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
handleDevice(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {

	switch (req->nr_stype) {
	case SMSG_DEVOPEN:
		return (handleOpenChannel(iop, nc, req));
	case SMSG_DEVCLOSE:
		return (handleCloseChannel(iop, nc, req));
	case SMSG_DEVBRIDGEPKT:
		return (handleBridgePacket(iop, nc, req));
	default:
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "unexpected channel submsg type:%d", req->nr_stype));
	}
}

/*
 * nc lock must be held by caller.
 */
static PhidgetReturnCode
sendEventToClient(PhidgetNetConnHandle nc, mosiop_t iop, msgtype_t type, msgsubtype_t subtype, void *buf,
  uint32_t len, int dgram) {

	if (type == MSG_DEVICE && subtype == SMSG_DEVCHANNEL && !PhidgetCKFlags(nc, PNCF_SENDCHANNELS))
		return (EPHIDGET_OK);

	return (writeEvent(iop, nc, type, subtype, buf, len, dgram));
}

static PhidgetReturnCode
sendEventToEachClient(mosiop_t iop, msgtype_t type, msgsubtype_t subtype, void *buf, uint32_t len, int dgram) {
	PhidgetNetConnHandle nc;
	PhidgetReturnCode res;

	mos_mutex_lock(&srvlock);
	MTAILQ_FOREACH(nc, &openServers, openlink) {
		NetConnWriteLock(nc);
		res = sendEventToClient(nc, iop, type, subtype, buf, len, dgram);
		NetConnWriteUnlock(nc);
		if (res != EPHIDGET_OK)
			netlogerr("sendEventToClient() failed for %s: "PRC_FMT, nc->peername, PRC_ARGS(res));
	}
	mos_mutex_unlock(&srvlock);

	return (EPHIDGET_OK);
}

/*
 * Called internally, and from phidgetvint.c and phidgetmanager.c
 */
PhidgetReturnCode
sendNetDeviceAttached(PhidgetDeviceHandle device, PhidgetNetConnHandle nc) {
	PhidgetVINTDeviceHandle vint;
	PhidgetHubDeviceHandle hub;
	PhidgetReturnCode res;
	const char *type;
	size_t labelLen;
	char desc[128];
	char buf[1024];
	uint32_t len;
	char *bufp;

	CHECKINITIALIZED;

	if (isNetworkPhidget(device))
		return (EPHIDGET_OK);

	//netlogverbose("%"PRIphid"", device);
	switch (device->deviceInfo.UDD->type) {
	case PHIDTYPE_USB:
		type = "USB";
		break;
	case PHIDTYPE_VINT:
		type = "VINT";
		break;
	case PHIDTYPE_MESH:
		type = "MESH";
		break;
	case PHIDTYPE_SPI:
		type = "SPI";
		break;
	case PHIDTYPE_VIRTUAL:
		type = "VIRTUAL";
		break;
	default:
		return (EPHIDGET_UNSUPPORTED);
	}

	labelLen = mos_strlen(device->deviceInfo.label);
	deviceInfo(device, desc, sizeof(desc));

	if (nc) {
		NetConnWriteLock(nc);
		bufp = nc->databuf;
		len = nc->databufsz;
	} else {
		bufp = buf;
		len = sizeof (buf);
	}

	if (device->deviceInfo.UDD->class == PHIDCLASS_HUB) {
		hub = (PhidgetHubDeviceHandle)device;
		len = mkJSON(bufp, len, "{type:%s,"
			"phid:%lu,parent:%lu,vendorID:%d,productID:%d,interfaceNum:%d,version:%d,serialNumber:%d,label:%s,"
			"index:%d,deviceID:%d,vintID:%d,hubPort:%d,isHubPort:%d,name:%s,desc:%s,fwstr:%s,"
			"hubPortsInfo:{portProto:%Au,portSuppSetSpeed:%Au,portMaxSpeed:%Au}}",
			type, PHIDID(device), PHIDID(device->parent),
			device->deviceInfo.UDD->vendorID, device->deviceInfo.UDD->productID,
			device->deviceInfo.UDD->interfaceNum, device->deviceInfo.version, device->deviceInfo.serialNumber,
			labelLen ? device->deviceInfo.label : "", device->deviceInfo.uniqueIndex, device->deviceInfo.UDD->id,
			device->deviceInfo.UDD->vintID, device->deviceInfo.hubPort, device->deviceInfo.isHubPort,
			device->deviceInfo.UDD->name, desc, device->fwstr,
			device->dev_hub.numVintPorts, hub->portProtocolVersion,
			device->dev_hub.numVintPorts, hub->portSupportsSetSpeed,
			device->dev_hub.numVintPorts, hub->portMaxSpeed);
	} else if (device->deviceInfo.UDD->type == PHIDTYPE_VINT) {
		vint = (PhidgetVINTDeviceHandle)device;
		len = mkJSON(bufp, len, "{type:%s,"
		  "phid:%lu,parent:%lu,vendorID:%d,productID:%d,interfaceNum:%d,version:%d,serialNumber:%d,label:%s,"
		  "index:%d,deviceID:%d,vintID:%d,hubPort:%d,isHubPort:%d,name:%s,desc:%s,fwstr:%s,vintProto:%d,suppSetSpeed:%d,maxSpeed:%u,commSpeed:%u}",
		  type, PHIDID(device), PHIDID(device->parent),
		  device->deviceInfo.UDD->vendorID, device->deviceInfo.UDD->productID,
		  device->deviceInfo.UDD->interfaceNum, device->deviceInfo.version, device->deviceInfo.serialNumber,
		  labelLen ? device->deviceInfo.label : "", device->deviceInfo.uniqueIndex, device->deviceInfo.UDD->id,
		  device->deviceInfo.UDD->vintID, device->deviceInfo.hubPort, device->deviceInfo.isHubPort,
		  device->deviceInfo.UDD->name, desc, device->fwstr,
		  (int)vint->deviceProtocolVersion, (int)vint->deviceSupportsSetSpeed, vint->deviceMaxSpeed, vint->vintCommSpeed);
	} else {
		len = mkJSON(bufp, len, "{type:%s,"
			"phid:%lu,parent:%lu,vendorID:%d,productID:%d,interfaceNum:%d,version:%d,serialNumber:%d,label:%s,"
			"index:%d,deviceID:%d,vintID:%d,hubPort:%d,isHubPort:%d,name:%s,desc:%s,fwstr:%s}",
			type, PHIDID(device), PHIDID(device->parent),
			device->deviceInfo.UDD->vendorID, device->deviceInfo.UDD->productID,
			device->deviceInfo.UDD->interfaceNum, device->deviceInfo.version, device->deviceInfo.serialNumber,
			labelLen ? device->deviceInfo.label : "", device->deviceInfo.uniqueIndex, device->deviceInfo.UDD->id,
			device->deviceInfo.UDD->vintID, device->deviceInfo.hubPort, device->deviceInfo.isHubPort,
			device->deviceInfo.UDD->name, desc, device->fwstr);
	}

	if (nc == NULL) {
		res = sendEventToEachClient(MOS_IOP_IGNORE, MSG_DEVICE, SMSG_DEVATTACH, bufp, len, 0);
	} else {
		res = sendEventToClient(nc, MOS_IOP_IGNORE, MSG_DEVICE, SMSG_DEVATTACH, NULL, len, 0);
		NetConnWriteUnlock(nc);
	}
	if (res != EPHIDGET_OK)
		return (res);

	return (walkDeviceChannels(device, sendNetChannelAttachedVisitor, nc));
}

PhidgetReturnCode
sendNetDeviceDetached(PhidgetDeviceHandle phid) {
	char buf[128];
	uint32_t len;

	MOS_ASSERT(phid != NULL);

	/*
	 * Network devices cannot detach..
	 */
	if (isNetworkPhidget(phid))
		return (EPHIDGET_OK);

	netlogdebug("%"PRIu64" %"PRIphid"", PHIDID(phid), phid);
	len = mkJSON(buf, sizeof(buf), "{phid:%lu,parent:%lu}", PHIDID(phid), PHIDID(phid->parent));
	return (sendEventToEachClient(MOS_IOP_IGNORE, MSG_DEVICE, SMSG_DEVDETACH, buf, len, 0));
}

static PhidgetReturnCode
sendNetChannelAttachedVisitor(PhidgetDeviceHandle device, const PhidgetUniqueChannelDef *UCD, int index,
  int uniqueIndex, void *ctx) {
	const PhidgetChannelAttributeDef *def;
	PhidgetNetConnHandle nc;
	PhidgetReturnCode res;
	int32_t serial;
	uint64_t chid;
	int isvint;

	char buf[256];
	uint32_t len;
	char *bufp;

	CHECKINITIALIZED;

	nc = ctx;

	if (nc != NULL && !PhidgetCKFlags(nc, PNCF_SENDCHANNELS))
		return (EPHIDGET_OK);

	if (isNetworkPhidget(device))
		return (EPHIDGET_OK);

	Phidget_getDeviceSerialNumber((PhidgetHandle)device, &serial);
	isvint = (device->deviceInfo.UDD->class == PHIDCLASS_VINT);
	chid = mkChannelId(uniqueIndex, UCD->class, serial, isvint,
	  isvint ? device->deviceInfo.hubPort : 0, isvint ? device->deviceInfo.isHubPort : 0);

	def = getPhidgetChannelAttributesByClass(UCD->class);

	if (nc) {
		NetConnWriteLock(nc);
		bufp = nc->databuf;
		len = nc->databufsz;
	} else {
		bufp = buf;
		len = sizeof (buf);
	}


	len = mkJSON(bufp, len,
	  "{parent:%lu,chid:%lu,class:%d,uniqueIndex:%d,index:%d,version:%d,name:%s,channelname:%s}",
	  PHIDID(device), chid, UCD->class, uniqueIndex, index, def->version, UCD->name,
	  Phid_ChannelClassName[UCD->class]);

	if (nc != NULL) {
		res = sendEventToClient(nc, MOS_IOP_IGNORE, MSG_DEVICE, SMSG_DEVCHANNEL, NULL, len, 0);
		NetConnWriteUnlock(nc);
	} else {
		res = sendEventToEachClient(MOS_IOP_IGNORE, MSG_DEVICE, SMSG_DEVCHANNEL, bufp, len, 0);
	}

	return (res);
}

/*
 * Sends devices to new clients.
 * The order is USB/SPI -> MESH -> VINT.
 */
PhidgetReturnCode
sendDevicesToNewServer(PhidgetNetConnHandle nc) {
	PhidgetDeviceHandle device;

	PhidgetReadLockDevices();

	FOREACH_DEVICE(device)
		if (device->connType == PHIDCONN_PHIDUSB || device->connType == PHIDCONN_HIDUSB || device->connType == PHIDCONN_SPI || device->connType == PHIDCONN_LIGHTNING)
			sendNetDeviceAttached(device, nc);

	FOREACH_DEVICE(device)
		if (device->connType == PHIDCONN_MESH)
			sendNetDeviceAttached(device, nc);

	FOREACH_DEVICE(device)
		if (device->connType == PHIDCONN_VINT)
			sendNetDeviceAttached(device, nc);

	FOREACH_DEVICE(device)
		if (device->connType == PHIDCONN_VIRTUAL)
			sendNetDeviceAttached(device, nc);

	PhidgetUnlockDevices();

	return (EPHIDGET_OK);
}

API_PRETURN
handleDeviceRequest(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req, int *stop) {

	*stop = 0;

	switch (req->nr_type) {
	case MSG_CONNECT:
		if (req->nr_stype == SMSG_DGRAMSTARTOK) {
			netloginfo("%"PRIphid" DATAGRAM handshake completed", nc);
			if (_allowDataGram)
				PhidgetSetFlags(nc, PNCF_DGRAMENABLED);
			return (EPHIDGET_OK);
		}
		break;
	case MSG_DEVICE:
		return (handleDevice(iop, nc, req));
	}
	return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "unexpected msg type: %s/%s", strmsgtype(req->nr_type), strmsgsubtype(req->nr_stype)));
}

static PhidgetReturnCode
sendKeepAlive(PhidgetNetConnHandle nc) {
	PhidgetReturnCode res;

	netlogdebug("sending keepalive (%"PRIphid")", nc);

	NetConnWriteLock(nc);
	nc->keepalive_dl = mos_gettime_usec() + nc->keepalive;
	res = writeEvent(MOS_IOP_IGNORE, nc, MSG_COMMAND, SMSG_KEEPALIVE, NULL, 0, DATAGRAM_DENY);
	NetConnWriteUnlock(nc);

	return (res);
}

static PhidgetReturnCode
keepAlive(IPhidgetServerHandle server) {

	/*
	 * 1.0 does not support keepAlive.
	 */
	if (server->nc->ppminor == 0) {
		if (server->nc->ppmajor == 1)
			return (EPHIDGET_OK);
	}

	/*
	 * keepalive disabled.
	 */
	if (server->nc->keepalive == 0)
		return (EPHIDGET_OK);

	/*
	 * Do not send a keepalive if one is pending.
	 */
	if (server->nc->keepalive_dl != 0)
		return (EPHIDGET_OK);

	/*
	 * Send a keepalive request
	 */
	if (server->nc->keepalive_last == 0 ||
	  (mos_gettime_usec() > server->nc->keepalive_last + server->nc->keepalive))
		return (sendKeepAlive(server->nc));

	return (EPHIDGET_OK);
}

static void
openDataGramSocket(PhidgetNetConnHandle nc) {
	PhidgetReturnCode res;
	int err;

	if (_allowDataGram == 0) {
		netloginfo("%"PRIphid" requested DATAGRAM events: administratively disabled", nc);
		return;
	}

	netloginfo("%"PRIphid" requested DATAGRAM events: starting", nc);

	err = mos_netop_udp_opensocket(MOS_IOP_IGNORE, &nc->dgsock, &nc->dgaddr);
	if (err != 0)
		netlogwarn("Failed to open datagram socket:%d", err);
	res = writeEvent(MOS_IOP_IGNORE, nc, MSG_CONNECT, SMSG_DGRAMSTART, NULL, 0, DATAGRAM_FORCE);
	if (res != EPHIDGET_OK)
		netlogwarn("Failed to send datagram start to client %"PRIphid"", nc);
}

API_PRETURN
handleDeviceClient(mosiop_t iop, IPhidgetServerHandle server) {
	PhidgetReturnCode res;
	mosiop_t siop;
	int stop;

	CHECKINITIALIZED;

	server->nc->keepalive = network_keepalive;

	res = mos_netop_tcp_setnonblocking(iop, &server->nc->sock, 1);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to set connection socket non-blocking"));

	/*
	 * Read the initial packets from the client.
	 */
	NetConnWriteLock(server->nc);
	res = startServerConnection(iop, server);
	NetConnWriteUnlock(server->nc);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to start server connection"));

	/*
	 * If the client supports datagram, then try to establish the connection.
	 */
	if (PhidgetCKFlags(server->nc, PNCF_DGRAM)) {
		NetConnWriteLock(server->nc);
		openDataGramSocket(server->nc);
		NetConnWriteUnlock(server->nc);
	}

	mos_mutex_lock(&srvlock);
	MTAILQ_INSERT_HEAD(&openServers, server->nc, openlink);
	openServersCnt++;
	mos_mutex_unlock(&srvlock);

	res = sendDevicesToNewServer(server->nc);
	if (res != EPHIDGET_OK) {
		mos_mutex_lock(&srvlock);
		MTAILQ_REMOVE(&openServers, server->nc, openlink);
		openServersCnt--;
		MOS_ASSERT(openServersCnt >= 0);
		mos_mutex_unlock(&srvlock);
		return (MOS_ERROR(iop, res, "failed to send devices to new client"));
	}

	startKeepAliveTask(server->nc);

	for (stop = 0; stop == 0 && server->nc->errcondition == 0;) {
		res = keepAlive(server);
		if (res != EPHIDGET_OK)
			break;

		siop = mos_iop_alloc();
		res = handleNetworkRequest(iop, server->nc, &stop);
		if (res == EPHIDGET_TIMEOUT) {
			mos_iop_release(&siop);
			continue;
		}
		if (res != EPHIDGET_OK) {
			MOS_ERROR_CHAIN(iop, siop, res, "handleNetworkRequest() failed");
			server->nc->errcondition = res;	/* early flag for the keep alive task to exit */
			mos_iop_release(&siop);
			break;
		}

		mos_iop_release(&siop);

		/*
		 * We have been flagged to stop.
		 */

		if (PhidgetCKFlags(server->nc, PNCF_STOP)) {
			netlogdebug("connection flagged as closing");
			break;
		}
	}
	PhidgetBroadcast(server->nc);	/* wake anybody waiting for us: namely the keep alive task */

	netlogdebug("request looped exited:%s", server->nc->peername);
	if (stop)
		netlogdebug("stopped by network request");

	mos_mutex_lock(&srvlock);
	MTAILQ_REMOVE(&openServers, server->nc, openlink);
	openServersCnt--;
	MOS_ASSERT(openServersCnt >= 0);
	mos_mutex_unlock(&srvlock);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to handle client request"));
	return (0);
}

#define CK(stmt)	do {			\
	res = (stmt);					\
	if (res != EPHIDGET_OK)			\
		return (res);				\
} while (0)

#define CKBAD(stmt)	do {			\
	res = (stmt);					\
	if (res != EPHIDGET_OK)			\
		goto bad;					\
} while (0)

PhidgetReturnCode
netConnToPConf(PhidgetNetConnHandle nc, pconf_t **upc) {
	PhidgetReturnCode res;
	char ctime[32];
	pconf_t *pc;

	res = pconf_create(&pc);
	if (res != EPHIDGET_OK)
		return (res);

	mostimestamp_string(&nc->ctime, ctime, sizeof (ctime));

	CKBAD(pconf_addstr(pc, nc->conntypestr, "conntype"));
	CKBAD(pconf_addstr(pc, nc->protocol, "proto"));
	CKBAD(pconf_addi(pc, nc->pmajor, "pmajor"));
	CKBAD(pconf_addi(pc, nc->pminor, "pminor"));
	CKBAD(pconf_addi(pc, nc->ppmajor, "ppmajor"));
	CKBAD(pconf_addi(pc, nc->ppminor, "ppminor"));
	CKBAD(pconf_addstr(pc, nc->peername, "peer"));
	CKBAD(pconf_addi(pc, nc->keepalive, "keepalive"));
	CKBAD(pconf_addu(pc, nc->io_in, "ioin"));
	CKBAD(pconf_addu(pc, nc->io_out, "ioout"));
	CKBAD(pconf_addu(pc, nc->io_ev, "ioev"));
	CKBAD(pconf_addstr(pc, ctime, "ctime"));
	CKBAD(pconf_addi(pc, nc->openchannels, "openchannels"));

	*upc = pc;
	return (EPHIDGET_OK);

bad:
	pconf_release(&pc);
	return (res);
}

PhidgetReturnCode
openServersToPConf(pconf_t **upc) {
	PhidgetNetConnHandle nc;
	PhidgetReturnCode res;
	pconf_t *pc, *ncpc;
	int i;

	res = pconf_create(&pc);
	if (res != EPHIDGET_OK)
		return (res);

	CKBAD(pconf_addi(pc, 1, "ver"));
	CKBAD(pconf_addi(pc, openServersCnt, "cnt"));
	CKBAD(pconf_addarray(pc, "connections"));

	mos_mutex_lock(&srvlock);
	i = 0;
	MTAILQ_FOREACH(nc, &openServers, openlink) {
		res = netConnToPConf(nc, &ncpc);
		if (res != EPHIDGET_OK)
			goto badlk;
		res = pconf_merge(pc, &ncpc, NULL, "connections");
		if (res != EPHIDGET_OK) {
			pconf_release(&ncpc);
			goto badlk;
		}
		i++;
	}

	*upc = pc;

badlk:
	mos_mutex_unlock(&srvlock);

bad:
	if (res != EPHIDGET_OK)
		netlogerr("failed to generate open servers pconf");

	return (res);
}
