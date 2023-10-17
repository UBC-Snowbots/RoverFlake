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

#ifndef CPHIDGET_CONSTANTS_INTERNAL
#define CPHIDGET_CONSTANTS_INTERNAL

#define	PHIDGET_ATTACHED_FLAG 			0x00000001	/* Phidget is attached (channel to device) */
#define	PHIDGET_DETACHING_FLAG			0x00000002	/* Phidget device is detaching */
#define	PHIDGET_OPEN_FLAG				0x00000004	/* Phidget is open */
#define	PHIDGET_OPENBYNETCLIENT_FLAG	0x00000008	/* (Server side) channel is open over the network */
#define	PHIDGET_SCANNED_FLAG			0x00000010	/* Devices was seen by USB code */
#define PHIDGET_DISPATCHIN_FLAG			0x00000020	/* Scheduled for dispatching inbound (from user) events */
#define PHIDGET_DISPATCHOUT_FLAG		0x00000040	/* Scheduled for dispatching outbound (to user) events */
#define PHIDGET_ONDISPATCH_FLAG			0x00000080	/* Referenced by the dispatch thread */
#define	PHIDGET_INDEVICELIST_FLAG		0x00000100	/* Device is in the device list */
#define	PHIDGET_NETWORK_FLAG			0x00000200	/* (Client side) Network Attached Phidget */
#define	PHIDGET_ERROR_FLAG				0x00000400	/* Error state: will be detached */
#define PHIDGET_ATTACHING_FLAG			0x00000800	/* Phidget device is attaching */
#define PHIDGET_CLOSING_FLAG			0x00001000	/* Phidget device is closing */
#define PHIDGET_HASINITIALSTATE_FLAG	0x00002000	/* Phidget channel initial state is available */
#define PHIDGET_INITIALIZED_FLAG		0x00004000	/* Channel has been reset and defaults set */
#define PHIDGET_OBJFLAG_MASK			0xFF000000	/* Flags reserved for other objects */

#define BP_FLAG_NOFORWARD				0x01000000	/* Do not forward to network connections */

/*
 * Connection types
 */
typedef enum {
	PHIDCONN_HIDUSB = 1,
	PHIDCONN_VINT,
	PHIDCONN_MESH,
	PHIDCONN_SPI,
	PHIDCONN_LIGHTNING,
	PHIDCONN_VIRTUAL,
	PHIDCONN_NETWORK,
	PHIDCONN_PHIDUSB
} PhidgetConnectionType;

/* 10 characters UTF-8, so up to 4 bytes per character */
#define MAX_LABEL_SIZE					10
#define MAX_LABEL_STORAGE				MAX_LABEL_SIZE*4+1

/* For now, these are the bigger of USB or SPI max transfer sizes */
#define MAX_IN_PACKET_SIZE				128
#define MAX_OUT_PACKET_SIZE				128

#define DEFAULT_TRANSFER_TIMEOUT		1000

/*
 * Open flags
 */

#define PHIDGETOPEN_SERIAL  0x01
#define PHIDGETOPEN_LABEL   0x02
#define PHIDGETOPEN_NETWORK	0x04

#define HUB_PORT_ID_MAX				0x0F
#define END_OF_LIST 0x8000

#define PHIDGET_MAXCHANNELS		64	/* Max number of channels on any device (current winner LED-64) */
#define PHIDGET_MAXCHILDREN		50	/* Max number of channels on any device (current winner is Mesh Dongle) */

#define MESH_DEVICE_VID				0x10000


#endif
