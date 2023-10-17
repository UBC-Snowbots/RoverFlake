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

#ifndef __CPHIDGETTEXTLCDDEVICE
#define __CPHIDGETTEXTLCDDEVICE

typedef struct _PhidgetTextLCDDevice *PhidgetTextLCDDeviceHandle;
PhidgetReturnCode PhidgetTextLCDDevice_create(PhidgetTextLCDDeviceHandle *phid);

void PhidgetTextLCDDevice_setWidthHeightFromScreenSize(PhidgetLCD_ScreenSize size, int *width, int *height);

#define TEXTLCD_MAXROWS 4
#define TEXTLCD_MAXCOLS 40
#define TEXTLCD_MAXSCREENS 2
#define TEXTLCD_MAXINPUTS 8
#define TEXTLCD_MAXOUTPUTS 8

#define TEXTLCD_CURSOR_PACKET			0x00
#define TEXTLCD_DIGITALOUTPUT_PACKET	0x02
#define TEXTLCD_BACKLIGHT_PACKET		0x11
#define TEXTLCD_CONTRAST_PACKET			0x12
#define TEXTLCD_INIT_PACKET				0x13

#define TEXTLCD_SCREEN(x)			(x << 5)
#define TEXTLCD_CGRAM_ADDR(x)		(x << 3)	//each custom character takes 8 bytes of CGRAM storage

#define TEXTLCD_ESCAPE_CHAR			0x00
#define TEXTLCD_COMMAND_MODE		0x01
#define TEXTLCD_DATA_MODE			0x02

//HD44780 commands
#define HD44780_CLEAR_DISPLAY	0x01
#define HD44780_CURSOR_HOME		0x02

//These are ORed together
#define HD44780_DISPLAY_CNTRL	0x08
#define HD44780_DISPLAY_ON		0x04
#define HD44780_CURSOR_ON		0x02
#define HD44780_CURSOR_BLINK_ON	0x01

#define HD44780_SET_CGRAM_ADDR	0x40
#define HD44780_SET_DDRAM_ADDR	0x80

struct _PhidgetTextLCDDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.textlcd
	PhidgetDevice phid;

	/* Public Members */

	uint8_t cursorOn[TEXTLCD_MAXSCREENS];
	uint8_t cursorBlink[TEXTLCD_MAXSCREENS];
	double contrast[TEXTLCD_MAXSCREENS];
	double backlight[TEXTLCD_MAXSCREENS];
	int height[TEXTLCD_MAXSCREENS];
	int width[TEXTLCD_MAXSCREENS];
	PhidgetLCD_ScreenSize screenSize[TEXTLCD_MAXSCREENS];

	uint8_t inputState[TEXTLCD_MAXINPUTS];
	uint8_t outputState[TEXTLCD_MAXOUTPUTS];

	/* Private Members */

	int cursorLocation[TEXTLCD_MAXSCREENS], cursorColumn[TEXTLCD_MAXSCREENS];
	int cursorScreen[TEXTLCD_MAXSCREENS], cursorLastScreen[TEXTLCD_MAXSCREENS];
	int lastScreenWritten;

	double _contrastSet[TEXTLCD_MAXSCREENS], _backlightSet[TEXTLCD_MAXSCREENS];

	uint8_t fullStateEcho;

	uint8_t internalFramebuffer[TEXTLCD_MAXSCREENS][TEXTLCD_MAXROWS][TEXTLCD_MAXCOLS];
	uint8_t displayFramebuffer[TEXTLCD_MAXSCREENS][TEXTLCD_MAXROWS][TEXTLCD_MAXCOLS];
	int displayFramebufferValid[TEXTLCD_MAXSCREENS][TEXTLCD_MAXROWS];

	uint8_t init[TEXTLCD_MAXSCREENS];
} typedef PhidgetTextLCDDeviceInfo;

#endif
