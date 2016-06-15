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
#endif

#if defined(CONFIG_FB)
#define __SIW_CONFIG_FB
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

/*
 * __PRD_ROW_SIZE & __PRD_COL_SIZE is also used in ABT layer
 */
#if defined(__SIW_SUPPORT_PRD) || defined(__SIW_SUPPORT_ABT)

#if defined(CONFIG_TOUCHSCREEN_SIW_SW1828)
#define __PRD_TYPE_S1
#elif defined(CONFIG_TOUCHSCREEN_SIW_LG4946)
#define __PRD_TYPE_L2
#elif defined(CONFIG_TOUCHSCREEN_SIW_LG4895)
#define __PRD_TYPE_L2
#elif defined(CONFIG_TOUCHSCREEN_SIW_LG4894)
#define __PRD_TYPE_L1
#endif

#if defined(__PRD_TYPE_S1)
#define __M1_RAWDATA_TEST_CNT	1
#define __M2_RAWDATA_TEST_CNT	1
#define __PRD_ROW_SIZE			32
#define __PRD_COL_SIZE			20
#define __PRD_COL_ADD			0
#define __TC_TOTAL_CH_SIZE		32
enum {
	__DELTA_DATA_OFFSET			= 0xD95,
	__LABLE_DATA_OFFSET			= 0xE83,
	__AIT_RAW_DATA_OFFSET		= 0xA8C,
	__AIT_BASE_DATA_ODD_OFFSET	= 0xC0F,
};
#elif defined(__PRD_TYPE_L2)
#define __M1_RAWDATA_TEST_CNT	1
#define __M2_RAWDATA_TEST_CNT	1
#define __PRD_ROW_SIZE			32
#define __PRD_COL_SIZE			18
#define __PRD_COL_ADD			0
#define __TC_TOTAL_CH_SIZE		34
enum {
	__DELTA_DATA_OFFSET			= 0xF80,
	__LABLE_DATA_OFFSET			= 0x10E8,
	__AIT_RAW_DATA_OFFSET		= 0xD1C,
	__AIT_BASE_DATA_ODD_OFFSET	= 0xE4E,
};
#elif defined(__PRD_TYPE_L1)
#define __M1_RAWDATA_TEST_CNT	2
#define __M2_RAWDATA_TEST_CNT	2
#define __PRD_ROW_SIZE			26
#define __PRD_COL_SIZE			15
#define __PRD_COL_ADD			1
#define __TC_TOTAL_CH_SIZE		32
enum {
	__DELTA_DATA_OFFSET			= 0xD95,
	__LABLE_DATA_OFFSET			= 0xE83,
	__AIT_RAW_DATA_OFFSET		= 0xA8C,
	__AIT_BASE_DATA_ODD_OFFSET	= 0xC0F,
};
#else
	#error Wrong PRD row and col size! check again!
#define __PRD_ROW_SIZE		-1
#define __PRD_COL_SIZE		1
#endif

#endif

//#define __SIW_TEST_IRQ_OFF

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 0))
#define mod_delayed_work	queue_delayed_work
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
#define subsys_system_register(_subsys, _group)	bus_register(_subsys)
#endif

#endif	/* __SIW_TOUCH_CFG_H */

