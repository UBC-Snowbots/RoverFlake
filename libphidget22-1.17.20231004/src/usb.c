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
#include "device/interfacekitdevice.h"
#include "manager.h"
#ifdef _LINUX
#include <libusb-1.0/libusb.h>
#endif
#ifdef _FREEBSD
#include <libusb.h>
#endif

#include "gpp.h"
#include "stats.h"

/* used by OSX to pause traffic during sleep */

PhidgetReturnCode CCONV
UTF16toUTF8(char *in, int inLen, char *out) {
#ifdef USE_INTERNAL_UNICONV
	UTF8 *utf8string = (UTF8 *)out;
	UTF8 *utf8stringEnd = utf8string + 255;
	const UTF16 *utf16string = (const UTF16 *)in;
	const UTF16 *utf16stringEnd = utf16string + (inLen / 2);
	ConversionResult resp;
	resp = ConvertUTF16toUTF8(&utf16string, utf16stringEnd, &utf8string, utf8stringEnd, strictConversion);

	if (resp != conversionOK) {
		switch (resp) {
		case sourceExhausted:
			logwarn("source exhausted error.");
			break;
		case targetExhausted:
			logwarn("target exhausted error.");
			break;
		default:
			logwarn("unexpected error.");
			return (EPHIDGET_UNEXPECTED);
		}
		return (EPHIDGET_INVALIDARG);
	}
#else
#ifndef _WINDOWS
	char *utf16string = in;
	char *utf8string = (char *)out;
	size_t inBytes = inLen;
	size_t outBytes = (255);
	iconv_t conv;
	size_t resp;
	conv= iconv_open("UTF-8", "UTF-16LE");
	if (conv == (iconv_t)(-1))
		return (EPHIDGET_UNEXPECTED);

	resp = iconv(conv, &utf16string, &inBytes, &utf8string, &outBytes);

	// Verify the output is NULL terminated
	out[255 - outBytes] = '\0';

	iconv_close(conv);

	if (resp == (size_t)-1) {
		switch (errno) {
		case EILSEQ:
		case EINVAL:
		case E2BIG:
		default:
			logerr("Unexpected error converting string to UTF-8: %s.", strerror(errno));
			return (EPHIDGET_UNEXPECTED);
		}
	}
#else
	   //stringData in NULL terminated
	int bytesWritten = WideCharToMultiByte(CP_UTF8, 0, (wchar_t *)in, -1, out, 255 + 1, NULL, NULL);

	//Error
	if (!bytesWritten) {
		logerr("Unable to convert string to UTF-8!");
		return (EPHIDGET_UNEXPECTED);
	}
#endif
#endif
	return (EPHIDGET_OK);
}

//Verifies that the UTF-8 label will fit in 20 UTF-16 bytes
//Also gives you a handle to the output label - since we had to make it anyways...
PhidgetReturnCode
encodeLabelString(char *buffer, char *out, size_t *outLen) {
	char buffer2[(MAX_LABEL_SIZE * 2)];
	size_t len;

#ifdef USE_INTERNAL_UNICONV
	const UTF8 *utf8label = (UTF8 *)buffer;
	const UTF8 *utf8labelEnd = utf8label + strlen(buffer);
	UTF16 *utf16label = (UTF16 *)buffer2;
	UTF16 *utf16labelEnd = utf16label + (MAX_LABEL_SIZE);
	ConversionResult resp;

	resp = ConvertUTF8toUTF16(&utf8label, utf8labelEnd, &utf16label, utf16labelEnd, strictConversion);

	if (resp != conversionOK) {
		switch (resp) {
		case sourceExhausted:
			logwarn("source exhausted error.");
			break;
		case targetExhausted:
			logwarn("target exhausted error.");
			break;
		default:
			logwarn("unexpected error.");
			return (EPHIDGET_UNEXPECTED);
		}
		return (EPHIDGET_INVALIDARG);
	}
	len = (size_t)utf16label - (size_t)buffer2;

#else

#ifndef _WINDOWS
   //Mac and Linux compatible UTF-8 to UTF-16LE conversion
	char *utf8label = (char *)buffer;
	char *utf16label = buffer2;
	size_t inBytes = strlen(buffer); // Up to MAX_LABEL_STORAGE bytes read
	size_t outBytes = (MAX_LABEL_SIZE * 2); //UTF-16 characters are two bytes each.
	iconv_t conv;
	size_t resp;

	conv= iconv_open("UTF-16LE", "UTF-8");
	if (conv == (iconv_t)(-1))
		return (EPHIDGET_UNEXPECTED);

	resp = iconv(conv, &utf8label, &inBytes, (char **)&utf16label, &outBytes);

	iconv_close(conv);

	if (resp == (size_t)-1) {
		switch (errno) {
		case EILSEQ:
		case EINVAL:
			logwarn("Malformed UTF8 string used for label.");
			break;
		case E2BIG:
			logwarn("Label string is too long.");
			break;
		default:
			logerr("Unexpected error in parsing label string: %s.", strerror(errno));
			return (EPHIDGET_UNEXPECTED);
		}
		return (EPHIDGET_INVALIDARG);
	}
	//length of descriptor = string length in bytes plus header (buffer[0] and buffer[1])
	len = ((MAX_LABEL_SIZE * 2) - outBytes);
#else
   //Windows compatible UTF-8 to UTF-16LE conversion
	int charsWritten=0;
	wchar_t *outLabel = (wchar_t *)buffer2;
	//don't try to convert a nothing string
	if (strlen(buffer) > 0) {
		charsWritten = MultiByteToWideChar(CP_UTF8, 0, buffer, (int)strlen(buffer), outLabel, MAX_LABEL_SIZE);

		//Error
		if (!charsWritten) {
			switch (GetLastError()) {
			case ERROR_INSUFFICIENT_BUFFER:
				logwarn("Label string is too long.");
				break;
			case ERROR_NO_UNICODE_TRANSLATION:
				logwarn("Malformed UTF8 string used for label.");
				break;
			case ERROR_INVALID_PARAMETER:
			default:
				logerr("Unexpected error in parsing label string.");
				return (EPHIDGET_UNEXPECTED);
			}
			return (EPHIDGET_INVALIDARG);
		}
	}

	len = charsWritten * 2;
#endif

#endif

	if (out && outLen) {
		if (len <= *outLen)
			*outLen = len;
		memcpy(out, buffer2, *outLen);
	}
	return (EPHIDGET_OK);
}

//detect if this label descriptor exhibits the wraparound error
//ie bytes 16-21 will match bytes 0-5 of the serial number string descriptor
BOOL
labelHasWrapError(int serialNumber, char *labelBuf) {
	char serialString[8] = { 0 };
	char errorBytes[6];
	int serialLen;
	int compareSize;
	int i;

	//only applies when the label descriptor is > 16 bytes
	if (labelBuf[0] <= 16)
		return (PFALSE);

	//only applies when the first 7 digits are ascii (set by old label functions)
	for (i = 3; i < 16; i += 2)
		if (labelBuf[i] != 0x00)
			return PFALSE;

	memset(errorBytes, 0, 6);

	//construct the 1st 6 bytes of the serial number descriptor
	snprintf(serialString, sizeof(serialString), "%d", serialNumber);
	serialLen = (int)strlen(serialString);

	errorBytes[0] = serialLen * 2 + 2; //length
	errorBytes[1] = 0x03;	//type

	//serial number 1st digit
	if (serialLen >= 1)
		errorBytes[2] = serialString[0];
	else
		errorBytes[2] = 0x00;
	errorBytes[3] = 0x00;

	//serial number 2nd digit
	if (serialLen >= 2)
		errorBytes[4] = serialString[1];
	else
		errorBytes[4] = 0x00;
	errorBytes[5] = 0x00;

	//compare the end of the label buffer with the string descriptor
	compareSize = labelBuf[0] - 16;
	if (!strncmp(&labelBuf[16], errorBytes, compareSize))
		return (PTRUE);

	return (PFALSE);
}

//takes the label string buffer from the USB device and outputs a UTF-8 version
PhidgetReturnCode
decodeLabelString(char *labelBuf, char *out, int serialNumber) {
	int i;

	//out NEEDS to be zeroed out, or we'll end up with a UTF-8 string with no terminating NULL
	memset(out, 0, MAX_LABEL_STORAGE);

	//this returns true only if our descriptor is > 16 bytes and has the error, so we truncate
	if (labelHasWrapError(serialNumber, labelBuf)) {
		for (i = 16; i < labelBuf[0]; i++)
			labelBuf[i] = 0x00;
		labelBuf[0] = 16;
		logwarn("Detected getLabel error - "
		  "label is being truncated to first 7 characters. Please setLabel again to correct this.");
	}

	//check if the label is stored as UTF-8 directly
	if (labelBuf[0] > 4 && (uint8_t)labelBuf[2] == 0xFF && (uint8_t)labelBuf[3] == 0xFF) {
		logdebug("Found a wrap-around bug style label.");
		memcpy(out, &labelBuf[4], labelBuf[0] - 4);
		out[labelBuf[0] - 4] = '\0';
	} else { //otherwise it's stored as UTF-16LE
		return (UTF16toUTF8(&labelBuf[2], labelBuf[0] - 2, out));
	}

	return (EPHIDGET_OK);
}

void
usbErrDetach(PhidgetDeviceHandle device) {
#ifdef _MACOSX
	logwarn("Resetting device because of USB error.");
	PhidgetUSBResetDevice(device);
#else
	PhidgetDeviceHandle phid;
	int serial;

	MOS_ASSERT(mos_tlock_islocked(devicesLock));

	serial = device->deviceInfo.serialNumber;
	// make sure it's attached and not detaching
	if (PhidgetCKFlags(device, PHIDGET_ATTACHED_FLAG | PHIDGET_DETACHING_FLAG) == PHIDGET_ATTACHED_FLAG) {
		logwarn("Detaching because of USB error: %"PRIphid, device);
		deviceDetach(device);
	}
	FOREACH_DEVICE(phid) {
		// Look for a 2nd interface of this device to detach as well. This assumes unique serial numbers.
		// make sure it's attached and not detaching, and it actually a USB device
		if (serial == phid->deviceInfo.serialNumber && !isNetworkPhidget(phid) && phid->deviceInfo.UDD->type == PHIDTYPE_USB
		  && PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG | PHIDGET_DETACHING_FLAG) == PHIDGET_ATTACHED_FLAG) {
			logwarn("Detaching because of USB error: %"PRIphid, phid);
			deviceDetach(phid);
			break; //only one, stop looking
		}
	}
#endif
}

static void
__commonDelete(PhidgetUSBConnectionHandle *conn) {

#if USB_ASYNC_READS
#ifndef _WINDOWS
	PhidgetUSBFreeAsyncBuffers(*conn);

	// make sure xferQueue is empty
	PhidgetUSBTransferHandle n;
	mos_mutex_lock(&(*conn)->xferQueueLock);
	while (!MSMTAILQ_EMPTY(&(*conn)->xferQueue)) {
		n = MSMTAILQ_FIRST(&(*conn)->xferQueue);
		MSMTAILQ_REMOVE_HEAD(&(*conn)->xferQueue, link);
		mos_free(n, sizeof(PhidgetUSBTransfer));
	}
	(*conn)->queueCnt = 0;
	mos_mutex_unlock(&(*conn)->xferQueueLock);

	mos_mutex_destroy(&(*conn)->xferQueueLock);
	mos_cond_destroy(&(*conn)->xferQueueCond);
#endif
#endif

#ifdef _WINDOWS
	CloseHandle((*conn)->closeReadEvent);
	CloseHandle((*conn)->asyncRead.hEvent);
	CloseHandle((*conn)->asyncWrite.hEvent);
	MOS_ASSERT(((*conn)->deviceHandle == INVALID_HANDLE_VALUE));
#endif

#if defined(_LINUX) || defined (_FREEBSD)
	if ((*conn)->deviceHandle)
		libusb_close((libusb_device_handle *)(*conn)->deviceHandle); //close handle
	if ((*conn)->dev)
		libusb_unref_device((*conn)->dev); //decrease reference cnt.
#endif

	mos_mutex_destroy(&(*conn)->usbwritelock);
	mos_mutex_destroy(&(*conn)->readLock);
	mos_cond_destroy(&(*conn)->readCond);
}

static void
PhidgetHIDUSBConnectionDelete(PhidgetHIDUSBConnectionHandle *conn) {

	__commonDelete((PhidgetUSBConnectionHandle *)conn);
	mos_free(*conn, sizeof(PhidgetHIDUSBConnection));
}

static void
PhidgetPHIDUSBConnectionDelete(PhidgetPHIDUSBConnectionHandle *conn) {

#if USB_ASYNC_READS
#ifdef _WINDOWS
	PhidgetUSBFreeAsyncBuffers(*conn);
#endif
#endif

	__commonDelete((PhidgetUSBConnectionHandle *)conn);
	mos_free(*conn, sizeof(PhidgetPHIDUSBConnection));
}

static PhidgetReturnCode
__commonCreate(PhidgetUSBConnectionHandle *conn) {

#if USB_ASYNC_READS
#ifndef _WINDOWS
	MSMTAILQ_INIT(&(*conn)->xferQueue);
	mos_mutex_init(&(*conn)->xferQueueLock);
	mos_cond_init(&(*conn)->xferQueueCond);
#endif
#endif

#ifdef _WINDOWS
	(*conn)->asyncRead.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	(*conn)->closeReadEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	(*conn)->asyncWrite.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	(*conn)->readPending = FALSE;
	(*conn)->deviceHandle = INVALID_HANDLE_VALUE;
	if ((!(*conn)->asyncRead.hEvent) || (!(*conn)->asyncWrite.hEvent) || (!(*conn)->closeReadEvent))
		return (EPHIDGET_UNEXPECTED);
#endif

	mos_mutex_init(&(*conn)->usbwritelock);
	mos_mutex_init(&(*conn)->readLock);
	mos_cond_init(&(*conn)->readCond);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetHIDUSBConnectionCreate(PhidgetHIDUSBConnectionHandle *conn) {

	assert(conn);

	*conn = mos_zalloc(sizeof(PhidgetHIDUSBConnection));
	phidget_init((PhidgetHandle)*conn, PHIDGET_HIDUSB_CONNECTION, (PhidgetDelete_t)PhidgetHIDUSBConnectionDelete);

	return (__commonCreate((PhidgetUSBConnectionHandle *)conn));
}

PhidgetReturnCode
PhidgetPHIDUSBConnectionCreate(PhidgetPHIDUSBConnectionHandle *conn) {

	assert(conn);

	*conn = mos_zalloc(sizeof(PhidgetPHIDUSBConnection));
	phidget_init((PhidgetHandle)*conn, PHIDGET_PHIDUSB_CONNECTION, (PhidgetDelete_t)PhidgetPHIDUSBConnectionDelete);

#ifdef _WINDOWS
	(*conn)->winusbHandle = INVALID_HANDLE_VALUE;
#endif

	return (__commonCreate((PhidgetUSBConnectionHandle *)conn));
}

/*
 * The device needs to be retained by the caller.
 */
MOS_TASK_RESULT
PhidgetUSBReadThreadFunction(void *_device) {
	PhidgetUSBConnectionHandle conn;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;
	int usbError = 0;

	res = EPHIDGET_OK;
	conn = NULL;

	device = PhidgetDeviceCast(_device);
	if (device == NULL) {
		logerr("Invalid device handle");
		goto exit;
	}

	mos_task_setname("Phidget22 USB Read Thread - %s (%d)", device->deviceInfo.UDD->SKU, device->deviceInfo.serialNumber);
	loginfo("%"PRIphid": USB read thread started: 0x%08x", _device, mos_self());

	conn = PhidgetUSBConnectionCast(device->conn);
	assert(conn);

	while (ISATTACHED(device)) {
		mos_mutex_lock(&conn->readLock);
		if (conn->readRun != 1) {
			mos_mutex_unlock(&conn->readLock);
			break;
		}
		mos_mutex_unlock(&conn->readLock);

		res = PhidgetDevice_read(device);
		switch (res) {
		case EPHIDGET_OK:
		case EPHIDGET_AGAIN:
			break;
		case EPHIDGET_NOTATTACHED:
			loginfo("%"PRIphid": ReadThread exiting normally (Phidget detach detected in PhidgetDevice_read)", device);
			// Ensure that the device gets detached
			queueDeviceDetach(device);
			goto exit;
		case EPHIDGET_INTERRUPTED:
		case EPHIDGET_CLOSED:
			loginfo("%"PRIphid": ReadThread exiting normally (signaled by Phidget_close)", device);
			goto exit;
		case EPHIDGET_TIMEOUT:
			// PhidUSB devices can NAK if they aren't using an Interrupt endpoint
			if (device->connType == PHIDCONN_PHIDUSB && ((PhidgetPHIDUSBConnectionHandle)conn)->pusbParams.ep1type != PHID_EP_INTERRUPT)
				break;

			// Interrupt endpoints can NAK if we explicitely allow it in Devices.js
			if (device->deviceInfo.UDD->canNak) {
				//verbose because it could happen a LOT
				logverbose("%"PRIphid": PhidgetDevice_read() expected time out", device);
				break;
			}

			logerr("%"PRIphid": ReadThread exiting - unexpected timeout (could be an ESD event)", device);
			usbError = 1;
			goto exit;

		case EPHIDGET_UNEXPECTED:
		default:
			logerr("%"PRIphid": ReadThread exiting - PhidgetDevice_read() returned: "PRC_FMT, device, PRC_ARGS(res));
			usbError = 1;
			goto exit;
		}
	}

	loginfo("%"PRIphid": ReadThread exiting normally", device);

exit:

	if (conn) {
		// Make sure any async reads started by this thread are cancelled before the thread exits.
		// Otherwise, we hit a windows bug sometimes where all devices on the host controller get reset.
#if USB_ASYNC_READS && defined(_WINDOWS)
		if (conn->type == PHIDGET_PHIDUSB_CONNECTION)
			PhidgetUSBStopAsyncReads((PhidgetPHIDUSBConnectionHandle)conn);
#endif
		mos_mutex_lock(&conn->readLock);
		conn->readRun = 0;
		mos_cond_broadcast(&conn->readCond);
		mos_mutex_unlock(&conn->readLock);
	}

	/* Do this here because PhidgetUSBCloseHandle joins the read thread */
	if (usbError) {
		PhidgetWriteLockDevices();
		usbErrDetach(device);
		PhidgetUnlockDevices();
	}

	if (device)
		PhidgetRelease(&device);

	decPhidgetStat("usb.readthreads");
	MOS_TASK_EXIT(res);
}

PhidgetReturnCode
openAttachedUSBDevice(PhidgetDeviceHandle device) {
	PhidgetUSBConnectionHandle conn;
	PhidgetReturnCode res;

	conn = PhidgetUSBConnectionCast(device->conn);
	assert(conn);

	res = PhidgetUSBOpenHandle(device);
	if (res != EPHIDGET_OK)
		return res;

	if (device->connType == PHIDCONN_PHIDUSB) {
		res = GPP_open_reset(NULL, device);
		if (res != EPHIDGET_OK) {

			logerr("Open Reset failed: "PRC_FMT, PRC_ARGS(res));
			return res;
		}
	}

	//don't start read thread if it's not needed
	switch (device->deviceInfo.UDD->uid) {
	case PHIDUID_1000_OLD1:
	case PHIDUID_1000_OLD2:
	case PHIDUID_1000_NO_ECHO:
	case PHIDUID_1001_OLD1:
	case PHIDUID_1001_OLD2:
	case PHIDUID_1001_NO_ECHO:
	case PHIDUID_1014_NO_ECHO:
	case PHIDUID_1030:
	case PHIDUID_1202_TEXTLCD:
	case PHIDUID_1202_TEXTLCD_BRIGHTNESS:
	case PHIDUID_1215:
	case PHIDUID_FIRMWARE_UPGRADE_STM32F_USB:
	case PHIDUID_FIRMWARE_UPGRADE_STM32G_USB:
	case PHIDUID_FIRMWARE_UPGRADE_STM32L_USB:
		break;

	default:
#ifdef _WINDOWS
		conn->readPending = PFALSE;
#endif
		PhidgetRetain(device);
		mos_mutex_lock(&conn->readLock);

#if USB_ASYNC_READS
		res = PhidgetUSBStartAsyncReads(device);
		if (res != EPHIDGET_OK) {
			mos_mutex_unlock(&conn->readLock);
			logerr("PhidgetUSBStartAsyncReads failed: "PRC_FMT, PRC_ARGS(res));
			//Call device specific close function if it exists
			if (device->_closing)
				device->_closing(device);
			PhidgetUSBCloseHandle(conn);
			PhidgetRelease(&device);
			return (EPHIDGET_UNEXPECTED);
		}
#endif

		conn->readRun = 1;
		if (mos_task_create(&conn->readThread, PhidgetUSBReadThreadFunction, device)) {
			conn->readRun = 0;
			mos_mutex_unlock(&conn->readLock);
			logwarn("unable to create read thread");
			//Call device specific close function if it exists
			if (device->_closing)
				device->_closing(device);
			PhidgetUSBCloseHandle(conn);
			PhidgetRelease(&device);
			return (EPHIDGET_UNEXPECTED);
		}
		mos_mutex_unlock(&conn->readLock);
		incPhidgetStat("usb.readthreads_ever");
		incPhidgetStat("usb.readthreads");

		break;
	}

	PhidgetSetFlags(device, PHIDGET_ATTACHING_FLAG);
	if ((res = device->initAfterOpen(device)) != 0) {
		logerr("Device Initialization functions failed: "PRC_FMT, PRC_ARGS(res));
		if (res == EPHIDGET_BADVERSION)
			logwarn("This Phidget requires a newer library - please upgrade.");
		PhidgetCLRFlags(device, PHIDGET_ATTACHING_FLAG);
		//Call device specific close function if it exists
		if (device->_closing)
			device->_closing(device);
		PhidgetUSBCloseHandle(conn);
		return (res);
	}
	PhidgetCLRFlags(device, PHIDGET_ATTACHING_FLAG);

	return (EPHIDGET_OK);
}

void
stopUSBReadThread(PhidgetUSBConnectionHandle conn) {

	mos_mutex_lock(&conn->readLock);
	if (conn->readRun != 0)
		conn->readRun = 2;
	mos_cond_broadcast(&conn->readCond);
	mos_mutex_unlock(&conn->readLock);
}

void
joinUSBReadThread(PhidgetUSBConnectionHandle conn) {

	mos_mutex_lock(&conn->readLock);
	while (conn->readRun != 0) {
		conn->readRun = 2;
		/* Because on macOS, ReadPipe has no timeout */
#if defined(_MACOSX) && !defined(_IPHONE)
		IOUSBInterfaceInterface300 **intf = conn->intf;
		(*intf)->AbortPipe(intf, 1);
#endif
		mos_cond_wait(&conn->readCond, &conn->readLock);
	}
	mos_mutex_unlock(&conn->readLock);
}
