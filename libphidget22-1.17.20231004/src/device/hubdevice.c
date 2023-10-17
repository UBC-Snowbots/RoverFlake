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
#include "device/hubdevice.h"
#include "gpp.h"
#include "manager.h"
#include "vintpackets.h"

#include "device/vintdevice.h"
#include "mos/mos_time.h"

static PhidgetReturnCode _setADCCalibrationValues(mosiop_t iop, PhidgetHubDeviceHandle phid,
  const double voltageInputGain[6], const double voltageRatioGain[6]);
static PhidgetReturnCode _setADCCalibrationValuesGainOffset(mosiop_t iop, PhidgetHubDeviceHandle phid, int portCount,
	const double *voltageInputOffset, const double *voltageInputGain, const double *voltageRatioOffset, const double *voltageRatioGain);
static void logTXBufferStatus(const char *file, int line, const char *func, Phidget_LogLevel level,
	const char *message, PhidgetHubDeviceHandle hub, uint8_t *buf);

#define logtxbufferstatus(lvl, msg, hub, buf) logTXBufferStatus(__FILE__, __LINE__, __func__, lvl, msg, hub, buf)
#define TXBUF_USED(hub, i) ((int)(hub->outstandingPacketCnt[i]))
#define TXBUF_FREE(hub, i) ((int)(hub->internalPacketInBufferLen - hub->outstandingPacketCnt[i] - 1))

/*
 * initAfterOpen
 * sets up the initial state of an object, reading in packets from the device if needed
 * used during attach initialization - on every attach
 */
static PhidgetReturnCode CCONV
PhidgetHubDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetHubDeviceHandle phid = (PhidgetHubDeviceHandle)device;
	PhidgetReturnCode result;
	uint8_t buffer[VINTHUB_MAXPORTS + 1];
	int readtries;
	size_t len;
	int i;

	assert(phid);

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_HUB0000:
#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDUID_MESHHUB:
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
	case PHIDUID_HUB5000:
		phid->internalPacketInBufferLen = 128;
		break;

	case PHIDUID_HUB0000_PHIDUSB:
#if PHIDUID_HUB0000_1_SUPPORTED
	case PHIDUID_HUB0000_1:
#endif
	case PHIDUID_HUB5000_PHIDUSB:
	case PHIDUID_HUB0001:
#if PHIDUID_HUB0001_AUTOSETSPEED_SUPPORTED
	case PHIDUID_HUB0001_AUTOSETSPEED:
#endif
#if PHIDUID_HUB0002_SUPPORTED
	case PHIDUID_HUB0002:
#endif
#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
#endif
		phid->internalPacketInBufferLen = PUNK_SIZE;
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	//set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numVintPorts; i++)
		phid->outstandingPacketCnt[i] = PUNK_SIZE;

	phid->packetCounter = -1;
	phid->splitPacketStoragePtr = 0;
	phid->outstandingPacketCntValid = 0;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_HUB0000:
#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDUID_MESHHUB:
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
	case PHIDUID_HUB5000:

		buffer[0] = VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_GETTXBUFFERSTATUS;

		/*
		 * The hub sendpacket requires inputlock to be unlocked because it waits for
		 * the packet status to come back
		 */
		result = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 1);
		if (result != EPHIDGET_OK)
			return result;

		readtries = 16;
		while (readtries-- > 0 && !phid->outstandingPacketCntValid)
			waitForReads((PhidgetDeviceHandle)phid, 1, 100);

		if (!phid->outstandingPacketCntValid)
			vintlogerr("Unable to recover TX buffer free space values. Continuing anyways.");

		break;

	case PHIDUID_HUB0000_PHIDUSB:
#if PHIDUID_HUB0000_1_SUPPORTED
	case PHIDUID_HUB0000_1:
#endif
	case PHIDUID_HUB5000_PHIDUSB:
	case PHIDUID_HUB0001:
#if PHIDUID_HUB0001_AUTOSETSPEED_SUPPORTED
	case PHIDUID_HUB0001_AUTOSETSPEED:
#endif
#if PHIDUID_HUB0002_SUPPORTED
	case PHIDUID_HUB0002:
#endif
#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
#endif

		len = sizeof(buffer);
		result = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_READ,
			(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_GETTXBUFFERSTATUS), 0, buffer, &len, 100);
		if (result != EPHIDGET_OK)
			return result;

		assert((int)len == (phid->devChannelCnts.numVintPorts + 1));

		phid->internalPacketInBufferLen = buffer[0];
		for (i = 0; i < phid->devChannelCnts.numVintPorts; i++) {
			/*
			* -1 because the internalPacketInBufferLen byte buffer can only actually
			* hold internalPacketInBufferLen-1 bytes or the read/write pointers would overlap
			*  Firmware in sending buffer free-space, which I convert to used space
			*/
			phid->outstandingPacketCnt[i] = phid->internalPacketInBufferLen - 1 - buffer[i + 1];
		}
		logtxbufferstatus(PHIDGET_LOG_DEBUG, "TXBUFFERSTATUS: ", phid, NULL);
		phid->outstandingPacketCntValid = 1;

		break;
	}

	return (EPHIDGET_OK);
}

// Called after we have a conn object, but before attach events, open, etc.
static PhidgetReturnCode CCONV
PhidgetHubDevice_initAfterCreate(PhidgetDeviceHandle device) {
	PhidgetHubDeviceHandle hub;
	PhidgetReturnCode res;
	int i;

	if (isNetworkPhidget(device))
		return (EPHIDGET_OK);

	hub = (PhidgetHubDeviceHandle)device;

	// Populate port properties
	for (i = 0; i < device->dev_hub.numVintPorts; i++) {
		switch (device->deviceInfo.UDD->uid) {

#if PHIDUID_MESHHUB_SUPPORTED
		case PHIDUID_MESHHUB:
			hub->portProtocolVersion[i] = 1;
			hub->portSupportsSetSpeed[i] = PFALSE;
			hub->portSupportsAutoSetSpeed[i] = PFALSE;
			hub->portMaxSpeed[i] = 10000;
			hub->portSpeed[i] = 10000;
			break;
#endif /* PHIDUID_MESHHUB_SUPPORTED */

		case PHIDUID_HUB0000:
		case PHIDUID_HUB0004:
		case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
		case PHIDUID_LIGHTNINGHUB:
#endif
		case PHIDUID_HUB5000:
			hub->portProtocolVersion[i] = 1;
			hub->portSupportsSetSpeed[i] = PFALSE;
			hub->portSupportsAutoSetSpeed[i] = PFALSE;
			hub->portMaxSpeed[i] = 100000;
			break;

		case PHIDUID_HUB0000_PHIDUSB:
#if PHIDUID_HUB0000_1_SUPPORTED
		case PHIDUID_HUB0000_1:
#endif
		case PHIDUID_HUB5000_PHIDUSB:
		case PHIDUID_HUB0001:
#if PHIDUID_HUB0001_AUTOSETSPEED_SUPPORTED
		case PHIDUID_HUB0001_AUTOSETSPEED:
#endif
#if PHIDUID_HUB0002_SUPPORTED
		case PHIDUID_HUB0002:
#endif
#if PHIDUID_HUB0007_SUPPORTED
		case PHIDUID_HUB0007:
#endif
			res = PhidgetHubDevice_updatePortProperties(hub, i);
			if (res != EPHIDGET_OK)
				return (res);
			break;

		default:
			MOS_PANIC("Unexpected device.");
		}
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetHubDevice_updatePortProperties(PhidgetHubDeviceHandle hub, int port) {
	PhidgetUSBConnectionHandle usbConn;
	USBD_VINTPortDescStruct vintPortDesc;
	uint8_t *props, *propsEnd;
	uint8_t prop, propLen;
	size_t bufLen;

	if (isNetworkPhidget((PhidgetDeviceHandle)hub))
		return (EPHIDGET_OK);

	// Only PHIDUSB hubs support the reading out port props
	if (hub->phid.connType != PHIDCONN_PHIDUSB)
		return (EPHIDGET_OK);

	usbConn = PhidgetUSBConnectionCast(hub->phid.conn);
	assert(usbConn);
	bufLen = sizeof(USBD_VINTPortDescStruct);
	if (PhidgetUSBGetDeviceDescriptor(usbConn, USB_DESC_TYPE_VINT_PORT_DESC, port, (uint8_t *)&vintPortDesc, &bufLen) != EPHIDGET_OK) {
		vintlogerr("Couldn't read VINT Port descriptor from a Hub.");
		return (EPHIDGET_UNEXPECTED);
	}
	hub->portProtocolVersion[port] = vintPortDesc.bVINTProtocolVersion;
	hub->portSpeed[port] = vintPortDesc.dwSpeedHz;
	hub->portMode[port] = (PhidgetHub_PortMode)vintPortDesc.bPortMode;
	hub->portPowered[port] = vintPortDesc.bPowered;
	hub->portSupportsSetSpeed[port] = PFALSE;
	hub->portSupportsAutoSetSpeed[port] = PFALSE;
	hub->portMaxSpeed[port] = 100000; //default
	props = vintPortDesc.VINTProperties;
	propsEnd = (uint8_t *)&vintPortDesc + vintPortDesc.bLength;
	while (props < propsEnd) {
		prop = (props[0] & 0x1F);
		propLen = ((props[0] & 0xE0) >> 5) + 1;
		switch (prop) {
		case VINT_PROP_SETSPEEDSUPPORT:
			hub->portSupportsSetSpeed[port] = PTRUE;
			break;
		case VINT_PROP_SETSPEEDLIMIT:
			hub->portMaxSpeed[port] = unpack32(props + 1);
			break;
		case VINT_PROP_AUTOSETSPEEDSUPPORT:
			hub->portSupportsAutoSetSpeed[port] = PTRUE;
			break;
		default:
			vintloginfo("Unknown VINT Port property: 0x%0x", prop);
			break;
		}
		props += propLen;
	}

	return (EPHIDGET_OK);
}

#if !defined(NDEBUG)
static const char *
Phidget_strPhidgetHubDevice_HubPacketType(PhidgetHubDevice_HubPacketType packetType) {

	switch (packetType) {
	case VINTHUB_HUBPACKET_SETPORTMODE:
		return ("Set Port Mode");

		// XXX add more

	default:
		return ("");
	}
}
#endif

static void
logTXBufferStatus(const char *file, int line, const char *func, Phidget_LogLevel level, const char *message,
  PhidgetHubDeviceHandle hub, uint8_t *buf) {
	Phidget_LogLevel ll;
	char str[200];
	char *strptr;
	size_t strlen = sizeof(str);
	int i, n;

	PhidgetLog_getLevel(&ll);
	if (ll < level)
		return;

	str[0] = '\0';
	strptr = str;
	for (i = 0; i < hub->devChannelCnts.numVintPorts; i++) {
		if (buf != NULL) {
			n = mos_snprintf(strptr, strlen, "%d|%d(%d|%d), ",
				TXBUF_USED(hub, i), TXBUF_FREE(hub, i), (int)(hub->internalPacketInBufferLen - 1 - buf[i]), (int)buf[i]);
		} else {
			n = mos_snprintf(strptr, strlen, "%d|%d, ", TXBUF_USED(hub, i), TXBUF_FREE(hub, i));
		}
		strptr += n;
		strlen -= n;
	}
	strptr[-2] = '\0'; //delete last ','

	PhidgetLog_loge(file, line, func, "phidget22vint", level, "%s%s", message, str);
}

static PhidgetReturnCode
processVintPacket(PhidgetHubDeviceHandle phid, uint8_t *buffer) {
	PhidgetDeviceHandle vintDevice;
	int vintID;
	int childIndex;
	int dataCount;
	int vintPort;

	vintPort = buffer[0] & 0x07;
	vintID = (int)(((uint16_t)(buffer[0] & 0xF0) << 4) + buffer[1]);
	dataCount = (buffer[2] & 0x3F);

	if (vintPort > phid->devChannelCnts.numVintPorts) {
		vintlogerr("Got an invalid port from a VINT message: %d", vintPort);
		return (EPHIDGET_UNEXPECTED);
	}

	childIndex = vintPort;

	// VINT port devices live higher in the child array.
	if (vintID <= HUB_PORT_ID_MAX)
		childIndex += vintID * phid->devChannelCnts.numVintPorts;

	assert(childIndex < PHIDGET_MAXCHILDREN);

	vintDevice = getChild((PhidgetDeviceHandle)phid, childIndex);
	if (vintDevice == NULL)
		return (EPHIDGET_OK);

	assert(childIndex == vintDevice->deviceInfo.uniqueIndex);

	if (vintDevice->deviceInfo.UDD->vintID != vintID) {
		vintloginfo("Seeing VINT Data on Port: %d for VINT Device: 0x%03x, "
			"but device in structure is: 0x%03x",
			vintPort, vintID, vintDevice->deviceInfo.UDD->vintID);
		  /*
		   * Notify main thread that something interesting is happening,
		   * so it notices the detach quickly
		   */
		NotifyCentralThread();
		return (EPHIDGET_OK);
	}

	// Sending the data count portion, so the packet length is +1
	// NOTE: We ignore the return value on purpose
	vintDevice->dataInput(vintDevice, buffer + 2, dataCount + 1);
	PhidgetRelease(&vintDevice);

	return (EPHIDGET_OK);
}

static void
readInTXBufferCounts(PhidgetHubDeviceHandle phid, int port) {
	PhidgetReturnCode ret;

	PhidgetLock(phid);
	vintlogdebug("outstandingPacketCnt changed from %d|%d (used|free) to UNK, Port %d", TXBUF_USED(phid, port), TXBUF_FREE(phid, port), port);
	phid->outstandingPacketCnt[port] = PUNK_SIZE;
	PhidgetUnlock(phid);

	// Request a new count
	vintlogdebug("VINTHUB_HUBINPACKET_TXBUFFERSTATUS request, Port %d", port);
	if ((ret = sendHubPacket(NULL, phid, VINTHUB_HUBPACKET_GETTXBUFFERSTATUS, NULL, 0)) != EPHIDGET_OK)
		vintlogerr("Error sending VINTHUB_HUBPACKET_GETTXBUFFERSTATUS msg to Hub.");
}

static PhidgetReturnCode
processPacketReturnCodes(PhidgetHubDeviceHandle phid, uint8_t *buffer, size_t length) {
	PhidgetPacketTrackerHandle packetTracker;
	VINTPacketStatusCode response;
	PhidgetReturnCode res;
	size_t packetSpace;
	int packetID;
	int readPtr;
	int port;

	readPtr = 0;
	while (readPtr < (int)length) {
		packetID = buffer[readPtr] & 0x7F;
		port = (packetID - 1) / VINTHUB_PACKETIDS_PER_PORT;
		response = VINTPacketStatusCode_ACK;
		if (buffer[readPtr] & VINTHUB_PACKETRETURN_notACK) {
			readPtr++;
			response = (VINTPacketStatusCode)buffer[readPtr];
		}
		readPtr++;

		packetTracker = &phid->phid.packetTracking->packetTracker[packetID];
		packetSpace = packetTracker->len;

		vintlogverbose("Packet 0x%02x response: "PRC_FMT", Port %d", packetID, response, Phidget_strVINTPacketStatusCode(response), port);

		//Don't bother locking if the list is empty
		res = VINTPacketStatusCode_to_PhidgetReturnCode(response, packetTracker->udd);
		// NOTE: setting the return code marks this tracker as signalled.
		// We can no longer use it here because it can be claimed immediately by another thread.
		if (setPacketReturnCode(packetTracker, res) != EPHIDGET_OK) {
			//Don't display if it's NOTATTACHED - since this is pretty common/expected.
			if (response != VINTPacketStatusCode_NOTATTACHED)
				vintloginfo("An unexpected PacketID was returned: 0x%02x ("PRC_FMT"). "
				  "Probably this packet is from a previous session or detached device.",
				  packetID, response, Phidget_strVINTPacketStatusCode(response));
			//Request a new count because our count will now be out
			readInTXBufferCounts(phid, port);
			continue;
		}

		PhidgetHubDevice_releasePacketSpace(phid, port, packetSpace);

		switch (res) {

			// Don't log these cases, just return result to user
		case EPHIDGET_OK:
			break;

		case EPHIDGET_NOTCONFIGURED:
		case EPHIDGET_INVALIDARG:
		case EPHIDGET_INVALID:
		case EPHIDGET_INVALIDPACKET:
			break;

		case EPHIDGET_FAILSAFE:
			break;

		case EPHIDGET_NOSPC:
			vintlogerr("Got a NOSPACE response from a VINT device, Port %d. "
			  "This usually indicates firmware problems.", port);
			//Request a new count because our count will now be out
			readInTXBufferCounts(phid, port);
			break;
		case EPHIDGET_NOTATTACHED:
			vintloginfo("Got a NOTATTACHED response from a VINT device, Port %d", port);
			/*
			 * Notify main thread that something interesting is happening,
			 * so it notices the detach quickly.
			 */
			NotifyCentralThread();
			break;
		case EPHIDGET_BUSY:
			vintlogerr("Got a NAK response from a VINT device, Port %d. "
				"This means the device decided to not deal with this data; try again.", port);
			break;
		case EPHIDGET_FBIG:
			vintlogerr("Got a TOOBIG response from a VINT device, Port %d. "
				"This means the packet was too big.", port);
			break;
		case EPHIDGET_UNEXPECTED:
		default:
			vintlogerr("Got an unexpected response from a VINT device: "PRC_FMT". "
				"This usually indicates firmware problems.", response, Phidget_strVINTPacketStatusCode(response));
			break;
		}
	}
	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetHubDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle childdev;
	PhidgetHubDeviceHandle hub;
	int killOutstandingPackets;
	PhidgetReturnCode ret;
	uint8_t buf[1];
	int packetReturnLen;
	int nextPacketStart;
	int packetCounter;
	int dataEndIndex;
	int dataCount;
	int readPtr;
	int vintdevice;
	int ch;
	int i;

	hub = (PhidgetHubDeviceHandle)device;
	assert(hub);
	assert(buffer);

	//Parse device packets - store data locally
	switch (hub->phid.deviceInfo.UDD->uid) {
	case PHIDUID_HUB0000:
	case PHIDUID_HUB0000_PHIDUSB:
#if PHIDUID_HUB0000_1_SUPPORTED
	case PHIDUID_HUB0000_1:
#endif
	case PHIDUID_HUB5000_PHIDUSB:
	case PHIDUID_HUB0001:
#if PHIDUID_HUB0001_AUTOSETSPEED_SUPPORTED
	case PHIDUID_HUB0001_AUTOSETSPEED:
#endif
#if PHIDUID_HUB0002_SUPPORTED
	case PHIDUID_HUB0002:
#endif
#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
#endif
#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDUID_MESHHUB:
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
	case PHIDUID_HUB5000:
		packetCounter = (buffer[0] >> 4) & 0x07;
		packetReturnLen = buffer[0] & 0x0F;
		dataEndIndex = (((int)length) - packetReturnLen);
		nextPacketStart = buffer[1] & 0x3F;
		readPtr = 2;
		killOutstandingPackets = PFALSE;

		//OUT Packet status return (0-31 bytes)
		// We process these 1st in case a detach message comes in
		processPacketReturnCodes(hub, buffer + dataEndIndex, length - dataEndIndex);

		if (buffer[1] & VINTHUB_INPACKET_HUBMSG_FLAG) {
			switch (buffer[readPtr]) {
			case VINTHUB_HUBINPACKET_TXBUFFERSTATUS:
				readPtr++;
				PhidgetLock(hub);
				vintlogdebug("VINTHUB_HUBINPACKET_TXBUFFERSTATUS packet filling outstandingPacketCnt");
				for (i = 0; i < hub->devChannelCnts.numVintPorts; i++) {
					// Only fill in the buffer status for ports that we have requested this for
					if (hub->outstandingPacketCnt[i] != PUNK_SIZE) {
						readPtr++;
						continue;
					}
					/*
					 * -1 because the 128 byte buffer can only actually hold 127 bytes or
					 * the read/write pointers would overlap
					 */
					hub->outstandingPacketCnt[i] = hub->internalPacketInBufferLen - 1 - buffer[readPtr++];

					vintlogverbose("outstandingPacketCnt updated from firmware: %d|%d (used|free), Port %d", TXBUF_USED(hub, i), TXBUF_FREE(hub, i), i);
				}
				logtxbufferstatus(PHIDGET_LOG_VERBOSE, "TXBUFFERSTATUS: ", hub, buffer + 3);
				PhidgetBroadcast(hub);
				PhidgetUnlock(hub);
				hub->outstandingPacketCntValid = 1;
				break;
			case VINTHUB_HUBINPACKET_OVERCURRENT:
				readPtr++;
				for (i = 0; i < hub->devChannelCnts.numVintPorts; i++) {
					if (buffer[readPtr] & (0x01 << i)) {
						vintlogwarn("Hub overcurrent detected on Port: %d", i);

						// Send to the Hub device channel
						if ((channel = getChannel(device, 0)) != NULL) {
							SEND_ERROR_EVENT_THROTTLED(channel, EEPHIDGET_OVERCURRENT,
								"Hub overcurrent detected on Port %d. Check for short.", i);
							PhidgetRelease(&channel);
						}

						//Iterate over all devices attached to the hub, sending the message to the ones on this port
						for (vintdevice = 0; vintdevice < PHIDGET_MAXCHILDREN; vintdevice++) {
							childdev = getChild(device, vintdevice);
							if (childdev == NULL)
								continue;
							if (childdev->deviceInfo.hubPort != i) {
								PhidgetRelease(&childdev);
								continue;
							}

							for (ch = 0; ch < VINTHUB_MAXCHANNELS; ch++) {
								if (childdev->channel[ch] == NULL)
									continue;
								if ((channel = getChannel(childdev, ch)) != NULL) {
									SEND_ERROR_EVENT_THROTTLED(channel, EEPHIDGET_OVERCURRENT,
									  "Hub overcurrent detected on Port %d. Check for short.", i);
									PhidgetRelease(&channel);
								}
							}
							PhidgetRelease(&childdev);
						}
					}
				}
				readPtr++;
				break;
			case VINTHUB_HUBINPACKET_DETACH:
				readPtr++;
				for (i = 0; i < hub->devChannelCnts.numVintPorts; i++) {
					if (buffer[readPtr] & (0x01 << i))
						queueVintDeviceDetach(device, i);
				}
				readPtr++;
				break;
			case VINTHUB_HUBINPACKET_DISABLE:
				readPtr++;
				for (i = 0; i < hub->devChannelCnts.numVintPorts; i++) {
					if (buffer[readPtr] & (0x01 << i))
						vintlogwarn("Invalid VINT activity detected on Port: %d. This can be safely ignored if something other than a VINT device is attached.", i);
				}
				readPtr++;
				break;
			case VINTHUB_HUBINPACKET_SPEEDCHANGE:
				readPtr++;
				// NOTE: We only expect to get this when the hub changes a port speed because of AutoSetSpeed
				for (i = 0; i < hub->devChannelCnts.numVintPorts; i++) {
					if (buffer[readPtr] & (0x01 << i)) {
						PhidgetHubDevice_updatePortProperties(hub, i);

						vintloginfo("Hub Port Speed changed to: %d on Port: %d.", hub->portSpeed[i], i);

						//Iterate over all children, sending message to the ones on this port
						for (vintdevice = 0; vintdevice < PHIDGET_MAXCHILDREN; vintdevice++) {
							childdev = getChild(device, vintdevice);
							if (childdev == NULL)
								continue;
							if (childdev->deviceInfo.hubPort != i || childdev->deviceInfo.isHubPort) {
								PhidgetRelease(&childdev);
								continue;
							}

							for (ch = 0; ch < VINTHUB_MAXCHANNELS; ch++) {
								if (childdev->channel[ch] == NULL)
									continue;
								if ((channel = getChannel(childdev, ch)) != NULL) {
									bridgeSendToChannel(channel, BP_VINTSPEEDCHANGE, 1, "%u", hub->portSpeed[i]);
									PhidgetRelease(&channel);
								}
							}
							PhidgetRelease(&childdev);
						}
					}
				}
				readPtr++;
				break;
			case VINTHUB_HUBINPACKET_REENUMERATION:
				readPtr++;
				for (i = 0; i < hub->devChannelCnts.numVintPorts; i++) {
					if (buffer[readPtr] & (0x01 << i)) {
						for (vintdevice = 0; vintdevice < PHIDGET_MAXCHILDREN; vintdevice++) {
							childdev = getChild(device, vintdevice);
							if (childdev == NULL)
								continue;
							if (childdev->deviceInfo.hubPort != i || childdev->deviceInfo.isHubPort) {
								PhidgetRelease(&childdev);
								continue;
							}

							vintlogwarn("Re-Enumeration of: %"PRIphid". This could indicate EMI issues, an unstable Hub Port Speed, or firmware issues.", childdev);

							// XXX We could iterate channels and send error events

							PhidgetRelease(&childdev);

							// We know only one vint device per port
							break;
						}
					}
				}
				readPtr++;
				break;
			default:
				// NOTE: Wince we don't know the size of this unknown packet type, we have to error
				vintlogerr("Got unexpected HubInPacketType: 0x%02x", buffer[readPtr]);
				return (EPHIDGET_UNEXPECTED);
			}
		}

		//We can only run these checks if we have previously recieved a packet
		if (hub->packetCounter == -1) {
			//1st packet: start at the 1st whole packet
			readPtr += nextPacketStart;
		} else {
			//Try to detect if we missed a packet
			if (hub->packetCounter != packetCounter) {
				vintlogwarn("One or more data packets were lost on the Hub.");
				//throw away any partial device packet data from previous packets
				hub->splitPacketStoragePtr = 0;
				readPtr += nextPacketStart;
				killOutstandingPackets = PTRUE;

				//Request a new count
				ret = sendHubPacket(NULL, hub, VINTHUB_HUBPACKET_GETTXBUFFERSTATUS, buf, 0);
				if (ret != EPHIDGET_OK)
					vintlogerr("Error sending VINTHUB_HUBPACKET_GETTXBUFFERSTATUS msg to Hub "
					  "after a packet loss.");
			} else {
				//these need to both be 0 or both non-zero
				if (nextPacketStart && !hub->splitPacketStoragePtr) {
					vintlogerr("Problem with split data in vint packet. "
					  "This should be a firmware/library bug.");
					return (EPHIDGET_UNEXPECTED);
				}
			}
		}

		hub->packetCounter = ((packetCounter + 1) & 0x07);

		//If we're starting with a split packet, read in the remainder.
		if (hub->splitPacketStoragePtr) {
			while (nextPacketStart--)
				hub->splitPacketStorage[hub->splitPacketStoragePtr++] = buffer[readPtr++];
			//Process packet
			if ((ret = processVintPacket(hub, hub->splitPacketStorage)) != EPHIDGET_OK)
				return (ret);
			//reset pointer
			hub->splitPacketStoragePtr = 0;
		}

		// Then, read any full packets in
		while (readPtr < dataEndIndex) {
			//See if we have another packet - the end will be marked by a NULL
			if ((buffer[readPtr] & VINTHUB_IN_VINTPACKET_START) == 0x00)
				break;

			/*
			 * Determine if we have a full or partial packet -
			 *  require at least 3 bytes to get the length byte
			 */
			if (readPtr < (dataEndIndex - 2)) {
				if (buffer[readPtr + 2] & 0x80) {
					/*
					 * NOTE: We don't actually have any messages anymore...
					 * but maybe we'll need some in the future.
					 */
					vintlogerr("Got an unexpected MSG in vint data: 0x%02x", buffer[readPtr + 2]);
					return (EPHIDGET_UNEXPECTED);
				}

				dataCount = buffer[readPtr + 2] & 0x3F;

				if ((readPtr + 3 + dataCount) <= dataEndIndex) {
					//Got here - we have a full packet to process
					if ((ret = processVintPacket(hub, buffer + readPtr)) != EPHIDGET_OK)
						return (ret);
					//loop around and check for the next packet
					readPtr += (dataCount + 3);
					continue;
				}
			}

			//Got here - it means we have a partial packet to queue up
			while (readPtr < dataEndIndex)
				hub->splitPacketStorage[hub->splitPacketStoragePtr++] = buffer[readPtr++];
		}

		if (killOutstandingPackets) {
			vintloginfo("Killing outstanding packets on hub");
			/*
			 * Mark the TX buffer as unknown size if there were any outstanding packets
			 * (because we may have missed the packet return)
			 */
			PhidgetLock(hub);
			for (i = 0; i < hub->devChannelCnts.numVintPorts; i++) {
				if (hub->outstandingPacketCnt[i]) {
					hub->outstandingPacketCnt[i] = PUNK_SIZE;
					vintlogdebug("outstandingPacketCnt changed from %d|%d (used|free) to UNK, Port %d", TXBUF_USED(hub, i), TXBUF_FREE(hub, i), i);
				}
				_setPacketsReturnCode((PhidgetDeviceHandle)hub, i, EPHIDGET_INTERRUPTED);
			}
			PhidgetUnlock(hub);
		}

		break;
	default:
		MOS_PANIC("Unexpected Device");
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetHubDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetHubDeviceHandle phid = (PhidgetHubDeviceHandle)ch->parent;
	unsigned char buffer[3];
	PhidgetReturnCode res;
	uint32_t timeout;
	int hubPort;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_HUB);
	assert(ch->class == PHIDCHCLASS_HUB);
	assert(ch->index == 0);

	switch (bp->vpkt) {
	case BP_SETFIRMWAREUPGRADEFLAG:
		hubPort = getBridgePacketInt32(bp, 0);
		timeout = getBridgePacketUInt32(bp, 1);
		buffer[0] = timeout & 0xFF;
		buffer[1] = (timeout >> 8) & 0xFF;
		return (sendHubPortPacket(bp->iop, phid, hubPort, VINTHUB_HUBPACKET_UPGRADE_FIRMWARE, buffer, 2));
	case BP_SETPORTMODE:
		hubPort = getBridgePacketInt32(bp, 0);
		return (PhidgetHubDevice_setPortMode(bp->iop, phid, hubPort, getBridgePacketInt32(bp, 1)));
	case BP_SETPORTPOWER:
		hubPort = getBridgePacketInt32(bp, 0);
		buffer[0] = getBridgePacketInt32(bp, 1);
		res = sendHubPortPacket(bp->iop, phid, hubPort, VINTHUB_HUBPACKET_PORTPOWER, buffer, 1);
		if (res == EPHIDGET_OK)
			phid->portPowered[hubPort] = getBridgePacketInt32(bp, 1);
		return (res);
	case BP_SETPORTAUTOSETSPEED:
		hubPort = getBridgePacketInt32(bp, 0);
		if (!phid->portSupportsAutoSetSpeed[hubPort])
			return (EPHIDGET_UNSUPPORTED);
		buffer[0] = getBridgePacketInt32(bp, 1);
		res = sendHubPortPacket(bp->iop, phid, hubPort, VINTHUB_HUBPACKET_PORTAUTOSETSPEED, buffer, 1);
		return (res);
	case BP_SETCALIBRATIONVALUES:
		return (_setADCCalibrationValues(bp->iop, phid, getBridgePacketDoubleArray(bp, 0),
			getBridgePacketDoubleArray(bp, 1)));
	case BP_SETCALIBRATIONVALUES2:
		assert(getBridgePacketArrayCnt(bp, 0) == phid->devChannelCnts.numVintPorts);
		assert(getBridgePacketArrayCnt(bp, 1) == phid->devChannelCnts.numVintPorts);
		assert(getBridgePacketArrayCnt(bp, 2) == phid->devChannelCnts.numVintPorts);
		assert(getBridgePacketArrayCnt(bp, 3) == phid->devChannelCnts.numVintPorts);
		return (_setADCCalibrationValuesGainOffset(bp->iop, phid, getBridgePacketArrayCnt(bp, 0), getBridgePacketDoubleArray(bp, 0),
			getBridgePacketDoubleArray(bp, 1), getBridgePacketDoubleArray(bp, 2), getBridgePacketDoubleArray(bp, 3)));
	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

void
PhidgetHubDevice_releasePacketSpace(PhidgetHubDeviceHandle phid, int hubPort, size_t packetSize) {

	PhidgetLock(phid);
	if (phid->outstandingPacketCnt[hubPort] != PUNK_SIZE) {
		// When a packet is lost, things can get out of sync. Make sure we don't go below 0!
		if (phid->outstandingPacketCnt[hubPort] < packetSize)
			phid->outstandingPacketCnt[hubPort] = 0;
		else
			phid->outstandingPacketCnt[hubPort] -= packetSize;
		// Ensure we don't go outside of spec
		MOS_ASSERT(phid->outstandingPacketCnt[hubPort] <= phid->internalPacketInBufferLen);
		vintlogverbose("Releasing %d bytes, %d|%d used|free, Port %d", (int)packetSize,
			TXBUF_USED(phid, hubPort), TXBUF_FREE(phid, hubPort), hubPort);
		PhidgetBroadcast(phid);
	} else {
		vintlogverbose("Trying to release %d bytes while outstandingPacketCnt is UNK, Port %d", (int)packetSize,
			TXBUF_USED(phid, hubPort), TXBUF_FREE(phid, hubPort), hubPort);
	}
	PhidgetUnlock(phid);
}

PhidgetReturnCode
PhidgetHubDevice_claimPacketSpace(PhidgetHubDeviceHandle phid, int hubPort, size_t packetSize) {
	size_t pktCnt;
	mostime_t now;
	mostime_t tm;

	PhidgetLock(phid);

	if (!_ISATTACHED((&phid->phid))) {
		PhidgetUnlock(phid);
		return (EPHIDGET_NOTATTACHED);
	}

	tm = mos_gettime_usec() + (2 * 1000000);

	for (;;) {
		pktCnt = phid->outstandingPacketCnt[hubPort];

		// < because our limit is internalPacketInBufferLen - 1
		if (pktCnt != PUNK_SIZE && (pktCnt + packetSize < phid->internalPacketInBufferLen))
			break;

		now = mos_gettime_usec();
		if (now < tm) {
			PhidgetTimedWait(phid, (uint32_t)(tm - now) / 1000);
		} else {
			PhidgetUnlock(phid);
			vintlogverbose("Timed out claiming packet space, Port %d", hubPort);
			return (EPHIDGET_TIMEOUT);
		}
	}

	phid->outstandingPacketCnt[hubPort] += packetSize;
	vintlogverbose("Claiming %d bytes, %d|%d used|free, Port %d", (int)packetSize,
		TXBUF_USED(phid, hubPort), TXBUF_FREE(phid, hubPort), hubPort);

	PhidgetUnlock(phid);

	return (EPHIDGET_OK);
}

/**
 * makePacket - constructs a packet using current device state
 * Packet protocol notes:
 * buffer[0] |76543210|
 *	bit 7: M3 Phidget General Packet Protocol bit
 *         1: GPP Packet (for setting label, rebooting, upgrading hub firmware etc.)
 *         0: Normal packet
 *  bit 6-4: packet type
 *         100: Hub packet - for configuring the hub
 *         010: Device packet - data/commands passed on to VINT devices or configure hub ports
 *  bit 3-0: Device packet: Destination hub port.
 *           Hub packet: hub packet type (PhidgetHubDevice_HubPacketType)
 */
PhidgetReturnCode
PhidgetHubDevice_makePacket(PhidgetHubDeviceHandle phid, PhidgetDeviceHandle vintDevice, int packetID,
  const uint8_t *bufferIn, size_t bufferInLen, uint8_t *buffer, size_t *bufferLen) {

	assert(vintDevice);
	assert(bufferLen);
	assert(bufferIn);
	assert(buffer);
	assert(phid);
	assert(*bufferLen >= getMaxOutPacketSize((PhidgetDeviceHandle)phid));
	assert(getMaxOutPacketSize((PhidgetDeviceHandle)phid) >= bufferInLen + 4);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_HUB0000:
#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDUID_MESHHUB:
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
	case PHIDUID_HUB5000:
		buffer[0] = VINTHUB_PACKET_DEVICE | vintDevice->deviceInfo.hubPort;
		break;
	case PHIDUID_HUB0000_PHIDUSB:
#if PHIDUID_HUB0000_1_SUPPORTED
	case PHIDUID_HUB0000_1:
#endif
	case PHIDUID_HUB5000_PHIDUSB:
	case PHIDUID_HUB0001:
#if PHIDUID_HUB0001_AUTOSETSPEED_SUPPORTED
	case PHIDUID_HUB0001_AUTOSETSPEED:
#endif
#if PHIDUID_HUB0002_SUPPORTED
	case PHIDUID_HUB0002:
#endif
#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
#endif
		// NOTE: packetOutCounter is incremented AFTER the packet is sent successfully
		buffer[0] = ((phid->packetOutCounter[vintDevice->deviceInfo.hubPort] << 4) & 0xF0) | (vintDevice->deviceInfo.hubPort & 0x0F);
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	buffer[1] = vintDevice->deviceInfo.UDD->vintID & 0xFF;
	buffer[2] = (vintDevice->deviceInfo.UDD->vintID >> 4) & 0xF0;
	buffer[3] = packetID;
	memcpy(buffer + 4, bufferIn, bufferInLen);

	*bufferLen = bufferInLen + 4;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
sendHubPacket(mosiop_t iop, PhidgetHubDeviceHandle hub, PhidgetHubDevice_HubPacketType hubPacketType,
	uint8_t *bufferIn, size_t bufferInLen) {
	uint8_t buffer[MAX_OUT_PACKET_SIZE];
	size_t packetLen;

	assert(hub);
	assert(bufferIn || !bufferInLen);

	assert(getMaxOutPacketSize((PhidgetDeviceHandle)hub) <= sizeof(buffer));

	vintlogverbose("Sending VINT Hub Packet\nDevice: %"PRIphid"\nHub Packet Type: %s (0x%02x)\nPacket: "LOGBUFFER_STR,
		hub, Phidget_strPhidgetHubDevice_HubPacketType(hubPacketType), hubPacketType, LOGBUFFER_ARGS(bufferInLen, bufferIn));

	switch (hub->phid.deviceInfo.UDD->uid) {
	case PHIDUID_HUB0000:
#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDUID_MESHHUB:
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
	case PHIDUID_HUB5000:
		buffer[0] = VINTHUB_PACKET_HUB | hubPacketType;
		packetLen = bufferInLen + 1;

		assert(packetLen <= getMaxOutPacketSize((PhidgetDeviceHandle)hub));

		memcpy(buffer + 1, bufferIn, bufferInLen);

		return PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)hub, buffer, packetLen);

	case PHIDUID_HUB0000_PHIDUSB:
#if PHIDUID_HUB0000_1_SUPPORTED
	case PHIDUID_HUB0000_1:
#endif
	case PHIDUID_HUB5000_PHIDUSB:
	case PHIDUID_HUB0001:
#if PHIDUID_HUB0001_AUTOSETSPEED_SUPPORTED
	case PHIDUID_HUB0001_AUTOSETSPEED:
#endif
#if PHIDUID_HUB0002_SUPPORTED
	case PHIDUID_HUB0002:
#endif
#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
#endif
		packetLen = bufferInLen;
		return PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)hub, PHIDGETUSB_REQ_DEVICE_WRITE,
			(VINTHUB_PACKET_HUB | hubPacketType), 0, bufferIn, &packetLen, 100);

	default:
		MOS_PANIC("Unexpected device");
	}
}

PhidgetReturnCode
sendHubPortPacket(mosiop_t iop, PhidgetHubDeviceHandle hub, int hubPort, PhidgetHubDevice_HubPacketType hubPacketType,
	uint8_t *bufferIn, size_t bufferInLen) {
	uint8_t buffer[MAX_OUT_PACKET_SIZE];
	size_t packetLen;

	assert(hub);
	assert(bufferIn || !bufferInLen);

	assert(getMaxOutPacketSize((PhidgetDeviceHandle)hub) <= sizeof(buffer));

	vintlogverbose("Sending VINT Hub Packet\nDevice: %"PRIphid"\nHub Packet Type: %s (0x%02x)\nHub Port: %d\nPacket: "LOGBUFFER_STR,
		hub, Phidget_strPhidgetHubDevice_HubPacketType(hubPacketType), hubPacketType, hubPort, LOGBUFFER_ARGS(bufferInLen, bufferIn));

	switch (hub->phid.deviceInfo.UDD->uid) {
	case PHIDUID_HUB0000:
#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDUID_MESHHUB:
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
	case PHIDUID_HUB5000:

		buffer[0] = VINTHUB_PACKET_HUB | hubPacketType;
		buffer[1] = hubPort;
		packetLen = bufferInLen + 2;

		assert(packetLen <= getMaxOutPacketSize((PhidgetDeviceHandle)hub));

		memcpy(buffer + 2, bufferIn, bufferInLen);

		return PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)hub, buffer, packetLen);

	case PHIDUID_HUB0000_PHIDUSB:
#if PHIDUID_HUB0000_1_SUPPORTED
	case PHIDUID_HUB0000_1:
#endif
	case PHIDUID_HUB5000_PHIDUSB:
	case PHIDUID_HUB0001:
#if PHIDUID_HUB0001_AUTOSETSPEED_SUPPORTED
	case PHIDUID_HUB0001_AUTOSETSPEED:
#endif
#if PHIDUID_HUB0002_SUPPORTED
	case PHIDUID_HUB0002:
#endif
#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
#endif
		packetLen = bufferInLen;
		return PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)hub, PHIDGETUSB_REQ_CHANNEL_WRITE,
			(VINTHUB_PACKET_HUB | hubPacketType), hubPort, bufferIn, &packetLen, 100);

	default:
		MOS_PANIC("Unexpected device");
	}
}

static void CCONV
PhidgetHubDevice_free(PhidgetDeviceHandle *phidG) {
	PhidgetHubDeviceHandle phid;

	phid = (PhidgetHubDeviceHandle)*phidG;

	mos_free(phid, sizeof(*phid));
	*phidG = NULL;
}

static PhidgetReturnCode CCONV
PhidgetHubDevice_close(PhidgetDeviceHandle phidG) {

	waitForAllPendingPackets(phidG);
	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetHubDevice_create(PhidgetHubDeviceHandle *phidp) {
	int i;
	DEVICECREATE_BODY(HubDevice, PHIDCLASS_HUB);

	phid->phid.packetTracking = mallocPhidgetPacketTrackers();
	phid->phid._closing = PhidgetHubDevice_close;
	phid->phid.initAfterCreate = PhidgetHubDevice_initAfterCreate;

	for (i = 0; i < VINTHUB_MAXPORTS; i++) {
		phid->portProtocolVersion[i] = PUNK_UINT8;
		phid->portSupportsSetSpeed[i] = PUNK_BOOL;
		phid->portSupportsAutoSetSpeed[i] = PUNK_BOOL;
		phid->portMaxSpeed[i] = PUNK_UINT32;
		phid->portSpeed[i] = PUNK_UINT32;
		phid->portMode[i] = PUNK_ENUM;
		phid->portPowered[i] = PUNK_BOOL;
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetHubDevice_setPortMode(mosiop_t iop, PhidgetHubDeviceHandle phid, int index, PhidgetHub_PortMode newVal) {
	PhidgetReturnCode res;
	uint8_t buffer[1];

	assert(phid);
	assert(phid->phid.deviceInfo.class == PHIDCLASS_HUB);
	TESTATTACHED(phid);

	buffer[0] = newVal;

	vintloginfo("Setting Port: %d mode to %s on %"PRIphid, index, Phidget_strPhidgetHub_PortMode(newVal), phid);

	res = sendHubPortPacket(iop, phid, index, VINTHUB_HUBPACKET_SETPORTMODE, buffer, 1);
	if (res == EPHIDGET_OK)
		phid->portMode[index] = newVal;

	return (res);
}

static PhidgetReturnCode
_setADCCalibrationValues(mosiop_t iop, PhidgetHubDeviceHandle phid,
	const double voltageInputGain[6], const double voltageRatioGain[6]) {
	PhidgetReturnCode ret;
	int i;

	assert(phid);
	assert(phid->phid.deviceInfo.class == PHIDCLASS_HUB);
	TESTATTACHED(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_HUB0000:
	case PHIDUID_HUB0000_PHIDUSB:
	case PHIDUID_HUB5000_PHIDUSB:
#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDUID_MESHHUB:
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
	case PHIDUID_HUB5000:
	{
		uint8_t buffer[VINTHUB_ADCCalibTable_LENGTH] = { 0 };
		uint32_t header;

		if (!deviceSupportsGeneralPacketProtocol((PhidgetDeviceHandle)phid))
			return (EPHIDGET_UNSUPPORTED);

		header = (((uint32_t)VINTHUB_ADCCalibTable_ID) << 20) | VINTHUB_ADCCalibTable_LENGTH;

		buffer[3] = header >> 24; //header high byte
		buffer[2] = header >> 16;
		buffer[1] = header >> 8;
		buffer[0] = header >> 0; //header low byte

		for (i = 0; i < 6; i++) {
			int int16Offset = i * 2;
			int temp;

			//4-15
			temp = round(voltageInputGain[i] * (double)0x8000);
			buffer[int16Offset + 5] = temp >> 8;
			buffer[int16Offset + 4] = temp >> 0;

			//16-27
			temp = round(voltageRatioGain[i] * (double)0x8000);
			buffer[int16Offset + 17] = temp >> 8;
			buffer[int16Offset + 16] = temp >> 0;
		}

		ret = GPP_setDeviceSpecificConfigTable(iop, (PhidgetDeviceHandle)phid, buffer,
		  VINTHUB_ADCCalibTable_LENGTH, VINTHUB_ADC_CALIB_TABLE_INDEX);
		if (ret != EPHIDGET_OK)
			return (ret);

		return (GPP_writeFlash(iop, (PhidgetDeviceHandle)phid));
	}

	case PHIDUID_HUB0001:
#if PHIDUID_HUB0001_AUTOSETSPEED_SUPPORTED
	case PHIDUID_HUB0001_AUTOSETSPEED:
#endif
	{
		uint8_t buffer[6 * 2 * 2] = { 0 };
		uint8_t *bufPtr = buffer;
		size_t bufferlen;

		pack32(buffer, HUB0001_CALIBRATION_UNLOCK_KEY);
		bufferlen = 4;
		ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
			(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_MODE), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
		if (ret != EPHIDGET_OK)
			return (ret);

		for (i = 0; i < 6; i++) {
			pack16(bufPtr, (uint16_t)round(voltageInputGain[i] * (double)0x8000));
			bufPtr += 2;
		}
		for (i = 0; i < 6; i++) {
			pack16(bufPtr, (uint16_t)round(voltageRatioGain[i] * (double)0x8000));
			bufPtr += 2;
		}
		bufferlen = 24;
		ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
			(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_WRITE), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
		if (ret != EPHIDGET_OK)
			return (ret);

		bufferlen = 0;
		ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
			(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_EXIT), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);

		return (ret);
	}

	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}

static PhidgetReturnCode
_setADCCalibrationValuesGainOffset(mosiop_t iop, PhidgetHubDeviceHandle phid, int portCount,
	const double *voltageInputOffset, const double *voltageInputGain,
	const double *voltageRatioOffset, const double *voltageRatioGain) {
#if PHIDUID_HUB0000_1_SUPPORTED || PHIDUID_HUB0002_SUPPORTED
	PhidgetReturnCode ret;
	int i;
#endif

	assert(phid);
	assert(phid->phid.deviceInfo.class == PHIDCLASS_HUB);
	assert(portCount == phid->devChannelCnts.numVintPorts);
	TESTATTACHED(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {

#if PHIDUID_HUB0000_1_SUPPORTED || PHIDUID_HUB0002_SUPPORTED
#if PHIDUID_HUB0000_1_SUPPORTED
	case PHIDUID_HUB0000_1:
#endif
#if PHIDUID_HUB0002_SUPPORTED
	case PHIDUID_HUB0002:
#endif
	{
		uint8_t buffer[96] = { 0 };
		uint8_t *bufPtr = buffer;
		size_t bufferlen;

		pack32(buffer, HUB0000_HUB0002_CALIBRATION_UNLOCK_KEY);
		bufferlen = 4;
		ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
			(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_MODE), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
		if (ret != EPHIDGET_OK)
			return (ret);

		for (i = 0; i < phid->devChannelCnts.numVintPorts; i++) {
			packfloat(bufPtr, (float)voltageInputOffset[i]);
			bufPtr += 4;
		}
		for (i = 0; i < phid->devChannelCnts.numVintPorts; i++) {
			packfloat(bufPtr, (float)voltageInputGain[i]);
			bufPtr += 4;
		}
		for (i = 0; i < phid->devChannelCnts.numVintPorts; i++) {
			packfloat(bufPtr, (float)voltageRatioOffset[i]);
			bufPtr += 4;
		}
		for (i = 0; i < phid->devChannelCnts.numVintPorts; i++) {
			packfloat(bufPtr, (float)voltageRatioGain[i]);
			bufPtr += 4;
		}
		bufferlen = 96;
		ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
			(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_WRITE), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
		if (ret != EPHIDGET_OK)
			return (ret);

		bufferlen = 0;
		ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
			(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_EXIT), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);

		return (ret);
	}
#endif

#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
		{
			uint8_t buffer[16] = { 0 };
			uint8_t *bufPtr = buffer;
			size_t bufferlen;

			pack32(buffer, HUB0007_CALIBRATION_UNLOCK_KEY);
			bufferlen = 4;
			ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
				(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_MODE), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
			if (ret != EPHIDGET_OK)
				return (ret);

			packfloat(bufPtr, (float)voltageInputOffset[0]);
			bufPtr += 4;

			packfloat(bufPtr, (float)voltageInputGain[0]);
			bufPtr += 4;

			packfloat(bufPtr, (float)voltageRatioOffset[0]);
			bufPtr += 4;

			packfloat(bufPtr, (float)voltageRatioGain[0]);

			bufferlen = 16;
			ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
				(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_WRITE), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
			if (ret != EPHIDGET_OK)
				return (ret);

			bufferlen = 0;
			ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE,
				(VINTHUB_PACKET_HUB | VINTHUB_HUBPACKET_CALIBRATION_EXIT), 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);

			return (ret);
		}
#endif

	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}