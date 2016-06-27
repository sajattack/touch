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

#include <linux/version.h>

#ifndef __SIW_TOUCH_CFG_H
#define __SIW_TOUCH_CFG_H

#define __SIW_SUPPORT_PINCTRL
#define __SIW_SUPPORT_PWRCTRL

//#define __SIW_SUPPORT_MISC

//#define __SIW_SUPPORT_ASC

#define __SIW_SUPPORT_ABT
#define __SIW_SUPPORT_PRD

#if defined(CONFIG_TOUCHSCREEN_SIW_LG4895) || defined(CONFIG_TOUCHSCREEN_SIW_LG4946)
#define __SIW_SUPPORT_WATCH
#endif

#define __SIW_SUPPORT_XFER

//#define __SIW_SUPPORT_DEBUG_OPTION

#if defined(CONFIG_ANDROID)
#define __SIW_SUPPORT_WAKE_LOCK
#endif

//#define __SIW_SUPPORT_PM_QOS

#if defined(CONFIG_OF)
#define __SIW_CONFIG_OF
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
#define __SIW_CONFIG_EARLYSUSPEND
#elif defined(CONFIG_FB)
#define __SIW_CONFIG_FB
#endif

#if defined(CONFIG_NET)
#define __SIW_CONFIG_NET
#endif

#define __SIW_ATTR_PERMISSION_ALL

#define SIW_TOUCH_NAME				"siw_touch"
#define SIW_TOUCH_CORE				"siw_touch_core"
#define SIW_TOUCH_INPUT				"siw_touch_input"
#define SIW_TOUCH_EXT_WATCH			"siw_ext_watch"

#define MAX_FINGER					10
#define MAX_LPWG_CODE				128

enum _SIW_CHIP_TYPE {
	CHIP_NONE = 0,
	//
	CHIP_LG4894 = 1,
	CHIP_LG4895 = 2,
	//
	CHIP_LG4946 = 0x10,
	//
	CHIP_SW1828 = 0x80,
	//
	CHIP_SW49105,
};

//#define __SIW_TEST_IRQ_OFF

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 0))
#define mod_delayed_work	queue_delayed_work
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
#define subsys_system_register(_subsys, _group)	bus_register(_subsys)
#endif

#endif	/* __SIW_TOUCH_CFG_H */

