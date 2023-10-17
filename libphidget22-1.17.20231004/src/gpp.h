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

#ifndef __PHIDGETGPP_H
#define __PHIDGETGPP_H
#include "phidget.h"

//Some constants and function for new M3 Phidgets (General Packet Protocol)

//Bit 7 (MSB)
#define PHID_GENERAL_PACKET_FLAG 0x80

//Bit 6 (used for return value on IN stream)
#define PHID_GENERAL_PACKET_SUCCESS	0x00
#define PHID_GENERAL_PACKET_FAIL	0x40

//Bits 0-5 (0x00 - 0x1F) (up to 31 types)
#define PHID_GENERAL_PACKET_IGNORE 0x00
#define PHID_GENERAL_PACKET_REBOOT_FIRMWARE_UPGRADE	0x01
#define PHID_GENERAL_PACKET_REBOOT_ISP 0x02
#define PHID_GENERAL_PACKET_CONTINUATION 0x03	//This is more data for when we need to send more then will fit in a single packet.
#define PHID_GENERAL_PACKET_ZERO_CONFIG 0x04
#define PHID_GENERAL_PACKET_WRITE_FLASH 0x05
#define PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_WRITE_SECTOR 0x06
#define PHID_GENERAL_PACKET_SET_DS_TABLE 0x07
#define PHID_GENERAL_PACKET_SET_DW_TABLE 0x08
#define PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_ERASE 0x09
#define PHID_GENERAL_PACKET_ERASE_CONFIG 0x0A

// PHIDUSB device only - these can extend to 0xFF
#define PHID_GENERAL_PACKET_OPEN_RESET 0x20
#define PHID_GENERAL_PACKET_CLOSE_RESET 0x21
#define PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_WRITE	0x22
#define PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_DONE	0x23
#define PHID_GENERAL_PACKET_WRITE_LABEL				0x24

//Internal API
PhidgetReturnCode PhidgetGPP_reboot_firmwareUpgrade(mosiop_t iop, PhidgetChannelHandle channel);
PhidgetReturnCode PhidgetGPP_setLabel(mosiop_t iop, PhidgetChannelHandle channel, const char *buffer);
PhidgetReturnCode PhidgetGPP_upgradeFirmware(mosiop_t iop, PhidgetChannelHandle channel, const unsigned char *data, size_t length);
PhidgetReturnCode PhidgetGPP_eraseFirmware(mosiop_t iop, PhidgetChannelHandle channel);
PhidgetReturnCode PhidgetGPP_dataInput(PhidgetDeviceHandle device, unsigned char *buffer, size_t length);
BOOL deviceSupportsGeneralPacketProtocol(PhidgetDeviceHandle device);
BOOL deviceSupportsGeneralPacketProtocolDataInput(PhidgetDeviceHandle device);
PhidgetReturnCode GPP_reboot_firmwareUpgrade(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_reboot_ISP(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_setLabel(mosiop_t iop, PhidgetDeviceHandle device, const char *buffer);
PhidgetReturnCode GPP_setDeviceSpecificConfigTable(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, int index);
PhidgetReturnCode GPP_setDeviceWideConfigTable(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, int index);
PhidgetReturnCode GPP_upgradeFirmware(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, PhidgetChannelHandle channel);
PhidgetReturnCode GPP_eraseFirmware(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_eraseConfig(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_writeFlash(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_open_reset(mosiop_t iop, PhidgetDeviceHandle device);
PhidgetReturnCode GPP_close_reset(mosiop_t iop, PhidgetDeviceHandle device);
#endif
