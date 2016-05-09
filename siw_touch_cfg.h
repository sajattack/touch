/*
 * SiW touch core driver
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef __SIW_TOUCH_CFG_H
#define __SIW_TOUCH_CFG_H

//#define __SIW_SUPPORT_PINCTRL
//#define __SIW_SUPPORT_PWRCTRL

//#define __SIW_SUPPORT_ASC

#define __SIW_SUPPORT_ABT
#define __SIW_SUPPORT_PRD
#define __SIW_SUPPORT_WATCH

#define __SIW_SUPPORT_XFER

//#define __SIW_SUPPORT_DEBUG_OPTION

#if defined(CONFIG_OF)
#define __SIW_CONFIG_OF
#endif

#if !defined(__SIW_CONFIG_OF)
#pragma message("[SiW - Warning] No COFIG_OF")
#endif

#define SIW_TOUCH_NAME				"siw_touch"
#define SIW_TOUCH_CORE				"siw_touch_core"
#define SIW_TOUCH_INPUT				"siw_touch_input"
#define MAX_FINGER					10
#define MAX_LPWG_CODE				128

enum _SIW_CHIP_TYPE {
	CHIP_LG4894 = 0,
	CHIP_LG4895 = 1,
	//
	CHIP_LG4946 = 0x10,
	//
	CHIP_SW1828 = 0x80,
	//
	CHIP_SW49105,
};

//#define __SIW_TEST_IRQ_OFF


#endif	/* __SIW_TOUCH_CFG_H */

