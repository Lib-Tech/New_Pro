/********************** BEGIN LICENSE BLOCK ************************************
 *
 * JZ4725  mobile_tv  Project  V1.0.0
 * INGENIC CONFIDENTIAL--NOT FOR DISTRIBUTION IN SOURCE CODE FORM
 * Copyright (c) Ingenic Semiconductor Co. Ltd 2005. All rights reserved.
 *
 * This file, and the files included with this file, is distributed and made
 * available on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND REALNETWORKS HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 *
 * http://www.ingenic.cn
 *
 ********************** END LICENSE BLOCK **************************************
 *
 *	Author:  <jbyu@ingenic.cn>
 *
 *	Create: 2008-09-26, by jbyu
 *
 *******************************************************************************
 */

#include <jz4725lcd.h>
#include "lcd.h"

// TV parameter
#define TVE_WIDTH_PAL 				( 720 )
#define TVE_HEIGHT_PAL 				( 573 )
#define DISP_WIDTH_PAL				( 704 - 32)
#define DISP_HEIGHT_PAL				( 573 - 21 )
#define TVE_FREQ_PAL 				( 50 )
#define TVE_WIDTH_NTSC 				( 720 )
#define TVE_HEIGHT_NTSC 			( 482 )
#define DISP_WIDTH_NTSC				( 704 - 32 )
#define DISP_HEIGHT_NTSC			( 482 - 12 )
#define TVE_FREQ_NTSC 				( 60 )

LCD_PARAM lcd_param =
{
	.lcd_mode =
	{
		.vendor = "truly",
		.type = "slcd",
		320, 240, 500, 0, 0, 0, 0, 0, 0,
		.mode_bitmask = (unsigned int *)0,
		.cfg = LCDCFG_SLCD_PIN  | LCDCFG_NEWDES |LCDCFG_MODE_SLCD,
		.ctrl = LCD_CTRL_BST_16 |LCD_CTRL_BPP_16,
#if (LCD_MODEL == OTM3225C)
		.slcd_cfg = LCD_MCFG_DWIDTH_8BIT_TWICE | LCD_MCFG_CWIDTH_8BIT | LCD_MCFG_CS_ACTIVE_LOW | LCD_MCFG_RS_CMD_LOW |LCD_MCFG_CLK_ACTIVE_FALLING | LCD_MCFG_TYPE_PARALLEL,
#elif (LCD_MODEL == OTM4802A)
		.slcd_cfg = LCD_MCFG_DWIDTH_8BIT_ONCE | LCD_MCFG_CWIDTH_8BIT | LCD_MCFG_CS_ACTIVE_LOW | LCD_MCFG_RS_CMD_LOW |LCD_MCFG_CLK_ACTIVE_RASING | LCD_MCFG_TYPE_PARALLEL,
#endif
	},
	.osd_mode =
	{
		.osd_cfg = LCD_OSDC_OSDEN |		   // Use OSD mode
					LCD_OSDC_F0EN,					  // enable Foreground0

		.osd_ctrl = 0,					   // disable ipu,
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,			   // set background color Black
		.colorkey0 = 0,					   // disable colorkey
		.colorkey1 = 0,					   // disable colorkey
		.alpha = 0xa0,					   // alpha value
		.ipu_restart = 0x80000100,		   // ipu restart
		.fg_change = FG_CHANGE_ALL,		   // change all initially

		.foreground0 = {16, 0, 0, 320, 240},	   // bpp, x, y, w, h
		.foreground1 = {16, 0, 0, 320, 240},	   // bpp, x, y, w, h
	}
};

LCD_PARAM tv_param_pal =
{
	.lcd_mode =
	{
		.vendor = (char *)0,
		.type = "tv",
		// width, height, freq, hsync, vsync, elw,	blw, efw, bfw
		TVE_WIDTH_PAL, TVE_HEIGHT_PAL, TVE_FREQ_PAL, 0, 0, 0, 0, 0, 0,
		.mode_bitmask = (unsigned int *)0,
		.cfg =	LCDCFG_TVEN	| LCDCFG_TVEPEH /*| LCDCFG_RECOVER */| LCDCFG_NEWDES | MODE_CCIR656_INT,
		.ctrl = LCD_CTRL_BST_16,
	},
	.osd_mode =
	{
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
		LCD_OSDC_ALPHAEN          | 		// enable alpha
		LCD_OSDC_ALPHAMD          |	    	// alpha mode: 0, alpha register; 1, each pixel
		LCD_OSDC_F0EN	          |
		LCD_OSDC_F1EN,						// enable Foreground1
		.osd_ctrl = 0,						// disable ipu,
	 	.rgb_ctrl = 0,//LCD_RGBC_YCC, 		// enable RGB => YUV
		.bgcolor = 0x008080, 				// set background color Black
		.colorkey0 = 0,						// disable colorkey
		.colorkey1 = 0,						// disable colorkey
		.alpha = 0xff,						// alpha value
		.ipu_restart = 0x80000100,			// ipu restart
		.fg_change = FG_CHANGE_ALL, 		// change all initially
		.foreground0 = {16, 0, 0, 0, 0}, 	// bpp, x, y, w, h
		.foreground1 = {16, (TVE_WIDTH_PAL - DISP_WIDTH_PAL) / 2, (TVE_HEIGHT_PAL - DISP_HEIGHT_PAL) / 2, DISP_WIDTH_PAL, DISP_HEIGHT_PAL}, 		// bpp, x, y, w, h
	}
};

LCD_PARAM tv_param_ntsc =
{
	.lcd_mode =
	{
		.vendor = (char *)0,
		.type = "tv",
		TVE_WIDTH_NTSC, TVE_HEIGHT_NTSC, TVE_FREQ_NTSC, 0, 0, 0, 0, 0, 0,
		.mode_bitmask = (unsigned int *)0,
		.cfg = LCDCFG_TVEN | /*LCDCFG_RECOVER | */LCDCFG_NEWDES | MODE_CCIR656_INT,
		.ctrl = LCD_CTRL_BST_16,
	},
	.osd_mode =
	{
		.osd_cfg = LCD_OSDC_OSDEN	| 		// Use OSD mode
		LCD_OSDC_ALPHAEN			|		// enable alpha
		LCD_OSDC_ALPHAMD			|		// alpha mode: 0, alpha register; 1, each pixel
		LCD_OSDC_F0EN	            |
		LCD_OSDC_F1EN,						// enable Foreground1
		.osd_ctrl = 0,						// disable ipu,
		.rgb_ctrl = 0,//LCD_RGBC_YCC,
		.bgcolor = 0x008080, 				// set background color Black
		.colorkey0 = 0,						// disable colorkey
		.colorkey1 = 0,						// disable colorkey
		.alpha = 0xff,						// alpha value
		.ipu_restart = 0x80000100,			// ipu restart
		.fg_change = FG_CHANGE_ALL, 		// change all initially
		.foreground0 = {16, 0, 0, 0, 0}, 	// bpp, x, y, w, h
		.foreground1 = {16, (TVE_WIDTH_NTSC - DISP_WIDTH_NTSC) / 2, (TVE_HEIGHT_NTSC - DISP_HEIGHT_NTSC) / 2, DISP_WIDTH_NTSC, DISP_HEIGHT_NTSC}, // bpp, x, y, w, h
	}
};



