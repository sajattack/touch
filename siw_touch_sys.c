/*
 * siw_touch_sys.c - SiW touch system interface
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#if defined(CONFIG_PLAT_SAMSUNG)
#include <plat/cpu.h>
#include <plat/gpio-core.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-cfg-helpers.h>
#include <plat/pm.h>
#endif

#include "siw_touch.h"
#include "siw_touch_sys.h"

int siw_touch_sys_bus_use_dma(struct device *dev)
{
#if defined(__If_the_bus_of_your_chipset_needs_dma_control__)
	{
		return 1;
	}
#elif defined(CONFIG_ARCH_EXYNOS5)
	{
		return 0;
	}
#endif

	return 0;
}

int siw_touch_get_boot_mode(void)
{
#if defined(CONFIG_SIW_GET_BOOT_MODE)
	return (sys_get_boot_mode() == BOOT_MODE_CHARGERLOGO)
#else
	return 0;
#endif
}

int siw_touch_boot_mode_check(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = NORMAL_BOOT;

#if defined(CONFIG_SIW_GET_FACTORY_MODE)
	ret = sys_get_factory_boot();
#endif

	if (ret != NORMAL_BOOT) {
		switch (atomic_read(&ts->state.mfts)) {
			case MFTS_NONE :
				ret = MINIOS_AAT;
				break;
			case MFTS_FOLDER :
				ret = MINIOS_MFTS_FOLDER;
				break;
			case MFTS_FLAT :
				ret = MINIOS_MFTS_FLAT;
				break;
			case MFTS_CURVED :
				ret = MINIOS_MFTS_CURVED;
				break;
			default :
				ret = MINIOS_AAT;
				break;
		}
	}

	return ret;
}

int siw_touch_sys_gpio_setpull(int pin, int value)
{
	int ret = 0;

#if defined(CONFIG_PLAT_SAMSUNG)
/*
	{
		int pull_val;

		pull_val = (value == GPIO_PULL_UP)? S3C_GPIO_PULL_UP :
					(value == GPIO_PULL_DOWN)? S3C_GPIO_PULL_DOWN :
					S3C_GPIO_PULL_NONE;
		ret = s3c_gpio_setpull(pin, pull_val);
	}
*/
#endif

	return ret;
}

int siw_touch_sys_panel_reset(struct device *dev)
{
	return 0;
}

int siw_touch_sys_osc(struct device *dev, int onoff)
{
	return 0;
}


