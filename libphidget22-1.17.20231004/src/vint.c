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
#include "manager.h"
#include "device/hubdevice.h"
#include "device/vintdevice.h"
#include "vintpackets.h"
#include "network/network.h"

static PhidgetReturnCode
resetVINTPortModeIfNeeded(PhidgetHubDeviceHandle hub, int port, PhidgetHub_PortMode mode) {
	PhidgetChannelHandle channel;
	PhidgetReturnCode res;

	if (mode == PORT_MODE_VINT_PORT)
		return (EPHIDGET_OK);

	/*
	 * Check if we have opened a non-hub-port device and the hub port is in port mode..
	 * in this case switch the port mode. This only applied if we have opened by serial number.
	 */

	res = EPHIDGET_OK;

	PhidgetReadLockChannels();
	FOREACH_CHANNEL(channel) {
		if (ISATTACHED(channel))
			continue;

		if (channel->openInfo->hubPort != port)
			continue;

		if (channel->openInfo->hubPortMode != PORT_MODE_VINT_PORT)
			continue;

		if (!(channel->openInfo->openFlags & (PHIDGETOPEN_SERIAL | PHIDGETOPEN_LABEL)))
			continue;

		if (channel->openInfo->openFlags & PHIDGETOPEN_SERIAL)
			if (channel->openInfo->serialNumber != hub->phid.deviceInfo.serialNumber)
				continue;

		if (channel->openInfo->openFlags & PHIDGETOPEN_LABEL)
			if (strcmp(channel->openInfo->label, hub->phid.deviceInfo.label) != 0)
				continue;

		// Open the hub
		res = openDevice((PhidgetDeviceHandle)hub);
		if (res != EPHIDGET_OK) {
			vintlogwarn("Unable to open Hub: "PRC_FMT, PRC_ARGS(res));
			break;
		}

		// Send out a port mode change message
		res = PhidgetHubDevice_setPortMode(NULL, hub, port, PORT_MODE_VINT_PORT);
		if (res != EPHIDGET_OK) {
			vintlogerr("Setting Hub Port mode failed: "PRC_FMT, PRC_ARGS(res));
			break;
		}

		NotifyCentralThread();
	}

	PhidgetUnlockChannels();

	return (res);
}

/*
 * Called with device list locked.
 */
PhidgetReturnCode
scanVintDevice(PhidgetDeviceHandle device, int childIndex, int id, int version, int port) {
	USBD_VINTDeviceDescStruct vintDeviceDesc;
	PhidgetUSBConnectionHandle usbConn;
	const PhidgetUniqueDeviceDef *pdd;
	PhidgetVINTDeviceHandle vint;
	PhidgetHubDeviceHandle hub;
	uint8_t *props, *propsEnd;
	uint8_t prop, propLen;
	PhidgetReturnCode res;
	size_t bufLen;

	hub = (PhidgetHubDeviceHandle)device;
again:
	for (pdd = Phidget_Unique_Device_Def; (int)(pdd->type) != END_OF_LIST; pdd++) {
		if (pdd->type != PHIDTYPE_VINT)
			continue;
		if (pdd->vintID != id)
			continue;
		if (version >= pdd->versionHigh || version < pdd->versionLow)
			continue;

		/*
		 * If the device is already here, just mark as scanned and move on.
		 * If an old device is here, remove it.
		 */
		vint = (PhidgetVINTDeviceHandle)getChild(device, childIndex);
		if (vint != NULL) {
			if (vint->phid.deviceInfo.UDD == pdd && vint->phid.deviceInfo.version == version) {
				PhidgetSetFlags(vint, PHIDGET_SCANNED_FLAG);
				PhidgetRelease(&vint);
				return (EPHIDGET_OK);
			}
			deviceDetach((PhidgetDeviceHandle)vint);
			setChild(device, childIndex, NULL);
			PhidgetRelease(&vint);
		}

		res = createPhidgetVINTDevice(pdd, version, device->deviceInfo.label,
		  device->deviceInfo.serialNumber, (PhidgetDeviceHandle *)&vint);
		if (res != EPHIDGET_OK)
			return (res);

		// For a VINT Device on a VINT2 capable hub - read out the vint device and port descriptors
		if (id > HUB_PORT_ID_MAX) {
			switch (device->deviceInfo.UDD->uid) {
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
				usbConn = PhidgetUSBConnectionCast(device->conn);
				assert(usbConn);

				bufLen = sizeof(USBD_VINTDeviceDescStruct);
				if (PhidgetUSBGetDeviceDescriptor(usbConn, USB_DESC_TYPE_VINT_DEVICE_DESC, port, (uint8_t *)&vintDeviceDesc, &bufLen) != EPHIDGET_OK) {
					vintlogwarn("Couldn't read VINT Device descriptor from a Hub.");
					break;
				}

				vint->deviceProtocolVersion = vintDeviceDesc.bVINTProtocolVersion;
				vint->deviceSupportsSetSpeed = PFALSE;
				vint->deviceSupportsAutoSetSpeed = PFALSE;
				props = vintDeviceDesc.VINTProperties;
				propsEnd = (uint8_t *)&vintDeviceDesc + vintDeviceDesc.bLength;
				while (props < propsEnd) {
					prop = (props[0] & 0x1F);
					propLen = ((props[0] & 0xE0) >> 5) + 1;
					switch (prop) {
					case VINT_PROP_POWERSOURCE:
						// XXX
						break;
					case VINT_PROP_ISOLATION:
						// XXX
						break;
					case VINT_PROP_SETSPEEDSUPPORT:
						vint->deviceSupportsSetSpeed = PTRUE;
						break;
					case VINT_PROP_SETSPEEDLIMIT:
						vint->deviceMaxSpeed = unpack32(props + 1);
						break;
					case VINT_PROP_AUTOSETSPEEDSUPPORT:
						vint->deviceSupportsAutoSetSpeed = PTRUE;
						break;
					default:
						vintloginfo("Unknown VINT Device property: 0x%0x", prop);
						break;
					}
					props += propLen;
				}

				if (PhidgetHubDevice_updatePortProperties(hub, port) != EPHIDGET_OK)
					vintlogwarn("Couldn't read VINT Port descriptor from a Hub.");

				break;

			default:
				break;
			}
		}

		PhidgetSetFlags(vint, PHIDGET_SCANNED_FLAG);
		vint->phid.deviceInfo.uniqueIndex = childIndex;
		vint->phid.deviceInfo.hubPort = port;
		vint->phid.deviceInfo.isHubPort = id <= HUB_PORT_ID_MAX ? PTRUE : PFALSE;

		// Grab the port properties from the hub
		vint->portProtocolVersion = hub->portProtocolVersion[port];
		vint->portSupportsSetSpeed = hub->portSupportsSetSpeed[port];
		vint->portSupportsAutoSetSpeed = hub->portSupportsAutoSetSpeed[port];
		vint->portMaxSpeed = hub->portMaxSpeed[port];
		vint->vintCommSpeed = hub->portSpeed[port];

		setParent(vint, device);
		setChild(device, childIndex, vint);

		deviceAttach((PhidgetDeviceHandle)vint, 0);
		PhidgetRelease(&vint);	/* release our reference */
		return (EPHIDGET_OK);
	}

	vintlogwarn("A VINT Phidget (ID: 0x%03x Version: %d Hub Port: %d) was found which is "
	  "not supported by the library. A library upgrade is required to work with this Phidget",
	  id, version, port);

	// Attach device as the special unknown device
	if (id != VINTID_0xff0) {
		id = VINTID_0xff0;
		goto again;
	}

	return (EPHIDGET_NOENT);
}

/*
 * Expects devices lock to be held.
 */
PhidgetReturnCode
scanVintDevices(PhidgetDeviceHandle device) {
	USBD_VINTPortsDescStruct_6_4 vintPortsDesc_6port;
#if PHIDUID_HUB0007_SUPPORTED
	USBD_VINTPortsDescStruct_1_4 vintPortsDesc_1port;
#endif
	PhidgetUSBConnectionHandle usbConn;
	PhidgetDeviceHandle child;
	char vintPortString[256];
	char *vintPortStringPtr;
	int i, j, childIndex;
	size_t bufLen;
	int deviceID;
	int version;
	int len;

	if (isNetworkPhidget(device))
		return (EPHIDGET_OK);

	vintPortStringPtr = vintPortString;
	bufLen = 0;
	len = 0;

	switch (device->deviceInfo.UDD->uid) {
	case PHIDUID_HUB0000:
	case PHIDUID_HUB5000:
		usbConn = PhidgetUSBConnectionCast(device->conn);
		assert(usbConn);
		memset(vintPortString, 0, VINTHUB_MAXPORTS * 12 + 1);
		if (PhidgetUSBGetString(usbConn, 5, vintPortString) != EPHIDGET_OK) {
			vintlogwarn("Couldn't get VINT string from a Hub - maybe detaching.");
			return (EPHIDGET_OK);
		}
		len = (int)strlen(vintPortString);
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
		usbConn = PhidgetUSBConnectionCast(device->conn);
		assert(usbConn);
		bufLen = sizeof(USBD_VINTPortsDescStruct_6_4);
		if (PhidgetUSBGetDeviceDescriptor(usbConn, USB_DESC_TYPE_VINT_PORTS_DESC, 0, (uint8_t *)&vintPortsDesc_6port, &bufLen) != EPHIDGET_OK) {
			vintlogwarn("Couldn't get VINT Ports descriptor from a Hub - maybe detaching.");
			return (EPHIDGET_OK);
		}
		break;
#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
		usbConn = PhidgetUSBConnectionCast(device->conn);
		assert(usbConn);
		bufLen = sizeof(USBD_VINTPortsDescStruct_1_4);
		if (PhidgetUSBGetDeviceDescriptor(usbConn, USB_DESC_TYPE_VINT_PORTS_DESC, 0, (uint8_t *)&vintPortsDesc_1port, &bufLen) != EPHIDGET_OK) {
			vintlogwarn("Couldn't get VINT Ports descriptor from a Hub - maybe detaching.");
			return (EPHIDGET_OK);
		}
		break;
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
		memset(vintPortString, 0, VINTHUB_MAXPORTS * 12 + 1);
		if (PhidgetSPIGetVINTDevicesString(vintPortString, sizeof(vintPortString)) != EPHIDGET_OK) {
			vintlogerr("Failed to get port string from SPI attached hub");
			return (EPHIDGET_UNEXPECTED);
		}
		len = (int)strlen(vintPortString);
		break;
#if (PHIDUID_MESHHUB_SUPPORTED || PHIDUID_LIGHTNINGHUB_SUPPORTED)
#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDUID_MESHHUB:
#endif
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
		vintPortStringPtr = ((PhidgetHubDeviceHandle)device)->portDescString;
		len = (int)strlen(vintPortStringPtr);
		break;
#endif /* (PHIDUID_MESHHUB_SUPPORTED || PHIDUID_LIGHTNINGHUB_SUPPORTED) */
	default:
		MOS_PANIC("Unexpected device.");
	}

	switch (device->deviceInfo.UDD->uid) {
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
		if (len != device->dev_hub.numVintPorts * 6 + device->dev_hub.numVintPortModes * 6) {
			vintlogerr("Wrong string length of VINT string from Hub: %d/%d (%s)", len,
			  device->dev_hub.numVintPorts * 6 + device->dev_hub.numVintPortModes * 6, vintPortStringPtr);
			return (EPHIDGET_OK);
		}
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
		if (bufLen != sizeof(USBD_VINTPortsDescStruct_6_4)) {
			vintlogerr("Wrong VINT Ports descriptor length from Hub: %d/%d", bufLen, sizeof(USBD_VINTPortsDescStruct_6_4));
			return (EPHIDGET_OK);
		}
		break;
#if PHIDUID_HUB0007_SUPPORTED
	case PHIDUID_HUB0007:
		if (bufLen != sizeof(USBD_VINTPortsDescStruct_1_4)) {
			vintlogerr("Wrong VINT Ports descriptor length from Hub: %d/%d", bufLen, sizeof(USBD_VINTPortsDescStruct_1_4));
			return (EPHIDGET_OK);
		}
		break;
#endif

	default:
		MOS_PANIC("Unexpected device.");
	}

	// Add/Detach Vint devices
	for (childIndex = 0, i = 0; i < device->dev_hub.numVintPorts; i++) {

		switch (device->deviceInfo.UDD->uid) {
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
			deviceID = ((unsigned int)(vintPortStringPtr[0] - '0') << 8)
				+ ((unsigned int)(vintPortStringPtr[1] - '0') << 4)
				+ (vintPortStringPtr[2] - '0');

			version = ((unsigned int)(vintPortStringPtr[3] - '0') * 100)
				+ ((unsigned int)(vintPortStringPtr[4] - '0') << 4)
				+ (vintPortStringPtr[5] - '0');

			vintPortStringPtr += 6;

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
			deviceID = vintPortsDesc_6port.port[i].wID;
			version = ((vintPortsDesc_6port.port[i].wVersion >> 8) * 100) + ((vintPortsDesc_6port.port[i].wVersion & 0xff));

			break;
#if PHIDUID_HUB0007_SUPPORTED
		case PHIDUID_HUB0007:
			deviceID = vintPortsDesc_1port.port[i].wID;
			version = ((vintPortsDesc_1port.port[i].wVersion >> 8) * 100) + ((vintPortsDesc_1port.port[i].wVersion & 0xff));

			break;
#endif

		default:
			MOS_PANIC("Unexpected device.");
		}

		// deviceID is going to either be 0x000 for nothing attached, or > HUB_PORT_ID_MAX for a VINT device plugged in
		if (deviceID > HUB_PORT_ID_MAX) {
			scanVintDevice(device, childIndex, deviceID, version, i);
		} else {
			// Nothing plugged in - detach previous VINT device if any
			child = getChild(device, childIndex);
			if (child) {
				chlog("vint replace %"PRIphid"", child);
				deviceDetach(child);
				setChild(device, childIndex, NULL);
				PhidgetRelease(&child);
			}
		}

		childIndex++;

		if (deviceID > PORT_MODE_VINT_PORT && deviceID <= HUB_PORT_ID_MAX)
			resetVINTPortModeIfNeeded((PhidgetHubDeviceHandle)device, i, (PhidgetHub_PortMode)deviceID);
	}

	// Add port devices
	for (j = 0; j < device->dev_hub.numVintPortModes; j++) {

		switch (device->deviceInfo.UDD->uid) {
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
			deviceID = ((unsigned int)(vintPortStringPtr[0] - '0') << 8)
				+ ((unsigned int)(vintPortStringPtr[1] - '0') << 4)
				+ (vintPortStringPtr[2] - '0');

			version = ((unsigned int)(vintPortStringPtr[3] - '0') * 100)
				+ ((unsigned int)(vintPortStringPtr[4] - '0') << 4)
				+ (vintPortStringPtr[5] - '0');

			vintPortStringPtr += 6;

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
			deviceID = vintPortsDesc_6port.portModeDevice[j].wID;
			version = ((vintPortsDesc_6port.portModeDevice[j].wVersion >> 8) * 100) + ((vintPortsDesc_6port.portModeDevice[j].wVersion & 0xff));

			break;
#if PHIDUID_HUB0007_SUPPORTED
		case PHIDUID_HUB0007:
			deviceID = vintPortsDesc_1port.portModeDevice[j].wID;
			version = ((vintPortsDesc_1port.portModeDevice[j].wVersion >> 8) * 100) + ((vintPortsDesc_1port.portModeDevice[j].wVersion & 0xff));

			break;
#endif

		default:
			MOS_PANIC("Unexpected device.");
		}

		for (i = 0; i < device->dev_hub.numVintPorts; i++)
			scanVintDevice(device, childIndex++, deviceID, version, i);
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
sendVINTPacketTransaction(mosiop_t iop, PhidgetChannelHandle channel, VINTDeviceCommand command,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen, PhidgetTransactionHandle trans) {
	unsigned char bufferOut[MAX_OUT_PACKET_SIZE] = { 0 };
	PhidgetVINTDeviceHandle vintDevice;
	PhidgetReturnCode ret;
	size_t bufferOutLen;

	bufferOutLen = sizeof(bufferOut);

	vintDevice = (PhidgetVINTDeviceHandle)channel->parent;
	assert(vintDevice);

	ret = PhidgetVINTDevice_makePacket(vintDevice, channel, command, devicePacketType, buffer,
	  bufferLen, bufferOut, &bufferOutLen);
	if (ret != EPHIDGET_OK)
		return (ret);

	if (command == VINT_CMD_DATA) {
		vintlogverbose("Sending VINT Packet\nChannel: %"PRIphid"\nPacket Type: %s (0x%02x)\nPacket: "LOGBUFFER_STR,
			channel, Phidget_strVINTPacketType(devicePacketType), devicePacketType, LOGBUFFER_ARGS(bufferLen, buffer));
	} else {
		vintlogverbose("Sending VINT Packet\nDevice: %"PRIphid"\nCommand: %s (0x%02x)\nPacket: "LOGBUFFER_STR,
			vintDevice, Phidget_strVINTDeviceCommand(command), command, LOGBUFFER_ARGS(bufferLen, buffer));
	}
	return (PhidgetDevice_sendpacketTransaction(iop, (PhidgetDeviceHandle)vintDevice, bufferOut, bufferOutLen, trans));
}

PhidgetReturnCode
sendVINTPacket(mosiop_t iop, PhidgetChannelHandle channel, VINTDeviceCommand command,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen) {

	return (sendVINTPacketTransaction(iop, channel, command, devicePacketType, buffer, bufferLen, NULL));
}

PhidgetReturnCode
sendVINTDataPacket(mosiop_t iop, PhidgetChannelHandle channel,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen) {

	return (sendVINTPacketTransaction(iop, channel,
		(VINTDeviceCommand)VINT_DATA, devicePacketType, buffer, bufferLen, NULL));
}

PhidgetReturnCode
sendVINTDataPacketTransaction(mosiop_t iop, PhidgetChannelHandle channel,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen, PhidgetTransactionHandle trans) {

	return (sendVINTPacketTransaction(iop, channel,
		(VINTDeviceCommand)VINT_DATA, devicePacketType, buffer, bufferLen, trans));
}

PhidgetReturnCode
VINTPacketStatusCode_to_PhidgetReturnCode(VINTPacketStatusCode code, const PhidgetUniqueDeviceDef *udd) {
	switch (code) {
	case VINTPacketStatusCode_DEVICE_ERR0:
	case VINTPacketStatusCode_DEVICE_ERR1:
	case VINTPacketStatusCode_DEVICE_ERR2:
	case VINTPacketStatusCode_DEVICE_ERR3:
		return VINTPacketStatusCodeDeviceCode_to_PhidgetReturnCode(code, udd);
	case VINTPacketStatusCode_ACK:
		return (EPHIDGET_OK);
	case VINTPacketStatusCode_NOSPACE:
		return (EPHIDGET_NOSPC);
	case VINTPacketStatusCode_NAK:
		return (EPHIDGET_BUSY);
	case VINTPacketStatusCode_TOOBIG:
		return (EPHIDGET_FBIG);
	case VINTPacketStatusCode_NOTATTACHED:
		return (EPHIDGET_NOTATTACHED);
	case VINTPacketStatusCode_INVALIDSEQUENCE:
		return (EPHIDGET_NOTCONFIGURED);
	case VINTPacketStatusCode_INVALIDARG:
		return (EPHIDGET_INVALIDARG);
	case VINTPacketStatusCode_INVALIDCOMMAND:
	case VINTPacketStatusCode_MALFORMED:
		return (EPHIDGET_INVALID);
	case VINTPacketStatusCode_INVALIDPACKETTYPE:
		return (EPHIDGET_INVALIDPACKET);
	case VINTPacketStatusCode_FAILSAFE:
		return (EPHIDGET_FAILSAFE);
	case VINTPacketStatusCode_UNEXPECTED:
	default:
		return (EPHIDGET_UNEXPECTED);
	}
}

const char *
Phidget_strVINTPacketStatusCode(VINTPacketStatusCode code) {

	switch (code) {
	case VINTPacketStatusCode_ACK:
		return ("Success");
	case VINTPacketStatusCode_NAK:
		return ("Not ready");
	case VINTPacketStatusCode_INVALIDARG:
		return ("Invalid Argument");
	case VINTPacketStatusCode_INVALIDPACKETTYPE:
		return ("Invalid Packet Type");
	case VINTPacketStatusCode_INVALIDSEQUENCE:
		return ("Invalid Sequence");
	case VINTPacketStatusCode_INVALIDCOMMAND:
		return ("Invalid Command");
	case VINTPacketStatusCode_MALFORMED:
		return ("Malformed");
	case VINTPacketStatusCode_NOSPACE:
		return ("No Space");
	case VINTPacketStatusCode_UNEXPECTED:
		return ("Unexpected");
	case VINTPacketStatusCode_NOTATTACHED:
		return ("Not Attached");
	case VINTPacketStatusCode_TOOBIG:
		return ("Packet Too Big");
	case VINTPacketStatusCode_FAILSAFE:
		return ("Failsafe Tripped");
	default:
		return ("<Unknown Code>");
	}
}

const char *
Phidget_strPhidgetHub_PortMode(PhidgetHub_PortMode mode) {

	switch (mode) {
	case PORT_MODE_VINT_PORT:
		return ("VINT");
	case PORT_MODE_DIGITAL_INPUT:
		return ("Digital Input");
	case PORT_MODE_DIGITAL_OUTPUT:
		return ("Digital Output");
	case PORT_MODE_VOLTAGE_INPUT:
		return ("Voltage Input");
	case PORT_MODE_VOLTAGE_RATIO_INPUT:
		return ("Voltage Ratio Input");
	default:
		return ("<Unknown Mode>");
	}
}

const char *
Phidget_strVINTPacketType(VINTPacketType packetType) {

	switch (packetType) {
	case VINT_PACKET_TYPE_PHIDGET_RESET:
		return ("Reset");
	case VINT_PACKET_TYPE_PHIDGET_ENABLE:
		return ("Enable");

	case VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL:
		return ("Set Data Interval");
	// XXX add more

	default:
		return ("");
	}
}

const char *
Phidget_strVINTDeviceCommand(VINTDeviceCommand command) {

	switch (command) {
	case VINT_CMD_DATA:
		return ("Data");
	case VINT_CMD_RESET:
		return ("Reboot");
	case VINT_CMD_UPGRADE_FIRMWARE:
		return ("Upgrade Firmware");
	case VINT_CMD_FIRMWARE_UPGRADE_DONE:
		return ("Firmware Upgrade Done");
	case VINT_CMD_SETSPEED2:
		return ("Set Speed");
	default:
		return ("<Unknown Command>");
	}
}

static void
PhidgetVINTConnectionDelete(PhidgetVINTConnectionHandle *phid) {

	mos_free(*phid, sizeof(PhidgetVINTConnection));
}

PhidgetReturnCode
PhidgetVINTConnectionCreate(PhidgetVINTConnectionHandle *phid) {

	assert(phid);

	*phid = mos_zalloc(sizeof(PhidgetVINTConnection));
	phidget_init((PhidgetHandle)*phid, PHIDGET_VINT_CONNECTION, (PhidgetDelete_t)PhidgetVINTConnectionDelete);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
Phidget_setHubPortSpeed_internal(mosiop_t iop, PhidgetChannelHandle channel, uint32_t speed) {
	PhidgetVINTDeviceHandle vint;
	PhidgetHubDeviceHandle hub;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;
	unsigned char buf[4];
	int i;

	#define NUM_VINT_SUPPORTED_SPEEDS	7
	uint32_t supportedSpeeds[NUM_VINT_SUPPORTED_SPEEDS] = { 100000, 160000, 250000, 400000, 500000, 800000, 1000000 };

	device = getParent(channel);
	TESTPTR(device);

	if (device->deviceInfo.class != PHIDCLASS_VINT) {
		PhidgetRelease(&device);
		return (MOS_ERROR(iop, EPHIDGET_UNSUPPORTED, "Must be a VINT device."));
	}

	vint = (PhidgetVINTDeviceHandle)device;

	hub = NULL;
	Phidget_getHub((PhidgetHandle)channel, (PhidgetHandle*)&hub);
	TESTPTR(hub);

	// Both the port and device need to support >= VINT2
	if (vint->portProtocolVersion == PUNK_UINT8 || vint->portProtocolVersion < 2
	  || vint->deviceProtocolVersion == PUNK_UINT8 || vint->deviceProtocolVersion < 2) {
		PhidgetRelease(&device);
		PhidgetRelease(&hub);
		return (MOS_ERROR(iop, EPHIDGET_UNSUPPORTED, "VINT Port and Device must support VINT2 protocol."));
	}

	if (vint->portSupportsSetSpeed != PTRUE) {
		PhidgetRelease(&device);
		PhidgetRelease(&hub);
		return (MOS_ERROR(iop, EPHIDGET_UNSUPPORTED, "VINT Port does not support High Speed."));
	}

	if (vint->deviceSupportsSetSpeed != PTRUE) {
		PhidgetRelease(&device);
		PhidgetRelease(&hub);
		return (MOS_ERROR(iop, EPHIDGET_UNSUPPORTED, "VINT Device does not support High Speed."));
	}

	if (speed > vint->portMaxSpeed) {
		PhidgetRelease(&device);
		PhidgetRelease(&hub);
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Speed must be <= Port max speed of: %dHz.", vint->portMaxSpeed));
	}

	if (speed == PHIDGET_HUBPORTSPEED_AUTO && !hub->portSupportsAutoSetSpeed[device->deviceInfo.hubPort]) {
		PhidgetRelease(&device);
		PhidgetRelease(&hub);
		return (MOS_ERROR(iop, EPHIDGET_UNSUPPORTED, "VINT Port does not support Auto Set Speed."));
	}

	if (vint->deviceSupportsSetSpeed != PTRUE) {
		PhidgetRelease(&device);
		PhidgetRelease(&hub);
		return (MOS_ERROR(iop, EPHIDGET_UNSUPPORTED, "VINT Device does not support High Speed."));
	}

	if (speed == PHIDGET_HUBPORTSPEED_AUTO && !vint->deviceSupportsAutoSetSpeed) {
		PhidgetRelease(&device);
		PhidgetRelease(&hub);
		return (MOS_ERROR(iop, EPHIDGET_UNSUPPORTED, "VINT Device does not support Auto Set Speed."));
	}

	/*
	 * Support a subset of the speeds that can be perfectly supported by all Hubs/Devices
	 *  100000, 160000, 250000, 400000, 500000, 800000, 1000000
	 * This is determined by finding the common factors of the various clock (UART CLK) speeds:
	 *  4MHz, 8MHz, 12MHz, 16MHz, 36MHz, 48MHz, 64MHz, 80MHz
	 */

	if (speed != PHIDGET_HUBPORTSPEED_AUTO) {
		// Round down to nearest supported speed
		for (i = (NUM_VINT_SUPPORTED_SPEEDS - 1); i >= 0; i--) {
			if (speed >= supportedSpeeds[i])
				break;
		}
		if (speed != supportedSpeeds[i])
			vintloginfo("Requested VINT speed of %dHz is not supported. Setting nearest lower speed of %dHz instead.", speed, supportedSpeeds[i]);
		speed = supportedSpeeds[i];
	}

	pack32(buf, speed);
	res = sendVINTPacket(iop, channel, VINT_CMD_SETSPEED2, (VINTPacketType)0, buf, 4);

	// Update the speed at the device level
	if (res == EPHIDGET_OK) {
		if (speed == PHIDGET_HUBPORTSPEED_AUTO) {
			// When setting to auto, we need to actually read back to current speed to see what the hub did
			PhidgetHubDevice_updatePortProperties(hub, device->deviceInfo.hubPort);
			vint->vintCommSpeed = hub->portSpeed[device->deviceInfo.hubPort];
		} else {
			vint->vintCommSpeed = speed;
		}
	}

	if (res != EPHIDGET_OK)
		vintlogwarn("VINT_CMD_SETSPEED2 failed in Phidget_setHubPortSpeed: "PRC_FMT, PRC_ARGS(res));

	PhidgetRelease(&device);
	PhidgetRelease(&hub);

	return (res);
}

/**
 * Exported API Below
 */



API_PRETURN
Phidget_getDeviceVINTID(PhidgetHandle deviceOrChannel, uint32_t *VINTID) {
	PhidgetDeviceHandle device;

	TESTPTR_PR(deviceOrChannel);
	TESTPTR_PR(VINTID);
	TESTATTACHEDORDETACHING_PR(deviceOrChannel);

	GETDEVICE(device, deviceOrChannel);

	*VINTID = device->deviceInfo.UDD->vintID;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_getHubPort(PhidgetHandle _phid, int *port) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle device;
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	TESTPTR_PR(phid);

	channel = PhidgetChannelCast(_phid);
	GETDEVICE(device, phid);

	if (ISATTACHEDORDETACHING(phid)) {
		*port = device->deviceInfo.hubPort;
	} else if (channel) {
		*port = channel->openInfo->hubPort;
	} else {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNEXPECTED));
	}

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_setHubPort(PhidgetHandle phid, int newVal) {
	PhidgetChannelHandle channel;

	CHANNELNOTDEVICE_PR(channel, phid);
	TESTPTR_PR(channel->openInfo);
	TESTRANGE_PR(newVal, "%d", PHIDGET_HUBPORT_ANY, VINTHUB_MAXPORTS);

	channel->openInfo->hubPort = newVal;
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_getHubPortCount(PhidgetHandle _phid, int *portCount) {
	PhidgetDeviceHandle pdevice;
	PhidgetDeviceHandle device;
	PhidgetHandle phid;

	TESTPTR_PR(portCount);

	phid = PhidgetCast(_phid);
	TESTPTR_PR(phid);
	TESTATTACHEDORDETACHING_PR(phid);

	GETDEVICE(device, phid);

	while (device != NULL) {
		if (device->deviceInfo.class == PHIDCLASS_HUB)
			break;

		pdevice = getParent(device);
		PhidgetRelease(&device);
		device = pdevice;
	}

	if (device && device->deviceInfo.class == PHIDCLASS_HUB) {
		*portCount = device->deviceInfo.UDD->channelCnts.hub.numVintPorts;
		PhidgetRelease(&device);
		return (EPHIDGET_OK);
	}

	if (device)
		PhidgetRelease(&device);

	return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
}

/*
 * Caller must release the device.
 */
API_PRETURN
Phidget_getHub(PhidgetHandle _phid, PhidgetHandle *hub) {
	PhidgetDeviceHandle pdevice;
	PhidgetDeviceHandle device;
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	TESTPTR_PR(phid);

	device = getParent(phid);
	while (device != NULL) {
		if (device->deviceInfo.class == PHIDCLASS_HUB)
			break;

		pdevice = getParent(device);
		PhidgetRelease(&device);
		device = pdevice;
	}

	if (device && device->deviceInfo.class == PHIDCLASS_HUB) {
		*hub = (PhidgetHandle)device;
		return (EPHIDGET_OK);
	}

	if (device)
		PhidgetRelease(&device);

	return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
}

API_PRETURN
Phidget_getIsHubPortDevice(PhidgetHandle _phid, int *isHubPortDevice) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle device;
	PhidgetHandle phid;

	TESTPTR_PR(isHubPortDevice);
	phid = PhidgetCast(_phid);
	TESTPTR_PR(phid);

	GETDEVICE(device, phid);
	channel = PhidgetChannelCast(phid);

	if (ISATTACHEDORDETACHING(phid)) {
		*isHubPortDevice = device->deviceInfo.isHubPort;
	} else if (channel && channel->openInfo) {
		*isHubPortDevice = channel->openInfo->isHubPort;
	} else {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNEXPECTED));
	}

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_setIsHubPortDevice(PhidgetHandle phid, int newVal) {
	PhidgetChannelHandle channel;

	CHANNELNOTDEVICE_PR(channel, phid);
	TESTPTR_PR(channel->openInfo);

	if (newVal) {
		switch (channel->class) {
		case PHIDCHCLASS_DIGITALINPUT:
		case PHIDCHCLASS_DIGITALOUTPUT:
		case PHIDCHCLASS_VOLTAGEINPUT:
		case PHIDCHCLASS_VOLTAGERATIOINPUT:
			break;
		default:
			return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "IsHubPortDevice cannot be set to true for this channel class."));
		}
	}

	channel->openInfo->isHubPort = newVal;
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_getHubPortSpeed(PhidgetHandle deviceOrChannel, uint32_t *speed) {
	PhidgetDeviceHandle device;
	PhidgetHubDeviceHandle hub;

	TESTPTR_PR(deviceOrChannel);
	TESTPTR_PR(speed);
	TESTATTACHEDORDETACHING_PR(deviceOrChannel);

	GETDEVICE(device, deviceOrChannel);

	if (device->deviceInfo.UDD->class != PHIDCLASS_VINT) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
	}

	// Read the speed back out of the hub every time - it could be changing all over the place because of autoSetSpeed. We can only do this for local hubs..
	if (!isNetworkPhidget(device)) {
		hub = NULL;
		Phidget_getHub(deviceOrChannel, (PhidgetHandle*)&hub);
		TESTPTR(hub);
		if (PhidgetHubDevice_updatePortProperties(hub, device->deviceInfo.hubPort) == EPHIDGET_OK)
			((PhidgetVINTDeviceHandle)device)->vintCommSpeed = hub->portSpeed[device->deviceInfo.hubPort];
	}

	if (((PhidgetVINTDeviceHandle)device)->vintCommSpeed == PUNK_UINT32) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}

	*speed = ((PhidgetVINTDeviceHandle)device)->vintCommSpeed;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_setHubPortSpeed(PhidgetHandle phid, uint32_t speed) {
	PhidgetChannelHandle channel;

	CHANNELNOTDEVICE_PR(channel, phid);
	TESTATTACHED_PR(channel);
	if (!ISOPEN(channel))
		return (PHID_RETURN(EPHIDGET_CLOSED));

	if (!isVintChannel(channel))
		return (PHID_RETURN(EPHIDGET_WRONGDEVICE));

	return (bridgeSendToDevice(channel, BP_SETVINTSPEED, NULL, NULL, 1, "%u", speed));
}

API_PRETURN
Phidget_getHubPortSupportsSetSpeed(PhidgetHandle deviceOrChannel, int *supportsSetSpeed) {
	PhidgetDeviceHandle device;

	TESTPTR_PR(deviceOrChannel);
	TESTPTR_PR(supportsSetSpeed);
	TESTATTACHEDORDETACHING_PR(deviceOrChannel);

	GETDEVICE(device, deviceOrChannel);

	if (device->deviceInfo.UDD->class != PHIDCLASS_VINT) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
	}

	if (((PhidgetVINTDeviceHandle)device)->portSupportsSetSpeed == PUNK_BOOL) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}

	*supportsSetSpeed = ((PhidgetVINTDeviceHandle)device)->portSupportsSetSpeed;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_getHubPortSupportsAutoSetSpeed(PhidgetHandle deviceOrChannel, int *supportsAutoSetSpeed) {
	PhidgetDeviceHandle device;

	TESTPTR_PR(deviceOrChannel);
	TESTPTR_PR(supportsAutoSetSpeed);
	TESTATTACHEDORDETACHING_PR(deviceOrChannel);

	GETDEVICE(device, deviceOrChannel);

	if (device->deviceInfo.UDD->class != PHIDCLASS_VINT) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
	}

	if (((PhidgetVINTDeviceHandle)device)->portSupportsAutoSetSpeed == PUNK_BOOL) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}

	*supportsAutoSetSpeed = ((PhidgetVINTDeviceHandle)device)->portSupportsAutoSetSpeed;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_getMaxHubPortSpeed(PhidgetHandle deviceOrChannel, uint32_t *maxSpeed) {
	PhidgetDeviceHandle device;

	TESTPTR_PR(deviceOrChannel);
	TESTPTR_PR(maxSpeed);
	TESTATTACHEDORDETACHING_PR(deviceOrChannel);

	GETDEVICE(device, deviceOrChannel);

	if (device->deviceInfo.UDD->class != PHIDCLASS_VINT) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
	}

	if (((PhidgetVINTDeviceHandle)device)->portMaxSpeed == PUNK_UINT32) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}

	*maxSpeed = ((PhidgetVINTDeviceHandle)device)->portMaxSpeed;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_getVINTDeviceSupportsSetSpeed(PhidgetHandle deviceOrChannel, int *supportsSetSpeed) {
	PhidgetDeviceHandle device;

	TESTPTR_PR(deviceOrChannel);
	TESTPTR_PR(supportsSetSpeed);
	TESTATTACHEDORDETACHING_PR(deviceOrChannel);

	GETDEVICE(device, deviceOrChannel);

	if (device->deviceInfo.UDD->class != PHIDCLASS_VINT) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
	}

	if (((PhidgetVINTDeviceHandle)device)->deviceSupportsSetSpeed == PUNK_BOOL) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}

	*supportsSetSpeed = ((PhidgetVINTDeviceHandle)device)->deviceSupportsSetSpeed;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_getVINTDeviceSupportsAutoSetSpeed(PhidgetHandle deviceOrChannel, int *supportsAutoSetSpeed) {
	PhidgetDeviceHandle device;

	TESTPTR_PR(deviceOrChannel);
	TESTPTR_PR(supportsAutoSetSpeed);
	TESTATTACHEDORDETACHING_PR(deviceOrChannel);

	GETDEVICE(device, deviceOrChannel);

	if (device->deviceInfo.UDD->class != PHIDCLASS_VINT) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
	}

	if (((PhidgetVINTDeviceHandle)device)->deviceSupportsAutoSetSpeed == PUNK_BOOL) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}

	*supportsAutoSetSpeed = ((PhidgetVINTDeviceHandle)device)->deviceSupportsAutoSetSpeed;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_getMaxVINTDeviceSpeed(PhidgetHandle deviceOrChannel, uint32_t *maxSpeed) {
	PhidgetDeviceHandle device;

	TESTPTR_PR(deviceOrChannel);
	TESTPTR_PR(maxSpeed);
	TESTATTACHEDORDETACHING_PR(deviceOrChannel);

	GETDEVICE(device, deviceOrChannel);

	if (device->deviceInfo.UDD->class != PHIDCLASS_VINT) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_WRONGDEVICE));
	}

	if (((PhidgetVINTDeviceHandle)device)->deviceMaxSpeed == PUNK_UINT32) {
		PhidgetRelease(&device);
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}

	*maxSpeed = ((PhidgetVINTDeviceHandle)device)->deviceMaxSpeed;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);
}
