/*
 * siw_touch_gpio.c - SiW touch gpio driver
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/irq.h>
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
/*#include <linux/regulator/machine.h>*/
#include <linux/regulator/consumer.h>

#include "siw_touch.h"
#include "siw_touch_gpio.h"
#include "siw_touch_sys.h"

int siw_touch_gpio_init(struct device *dev,
							int pin, const char *name)
{
	int ret = 0;

	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "pin %d, name %s\n", pin, name);
		ret = gpio_request(pin, name);
		if (ret) {
			t_dev_err(dev, "gpio_request[%s] failed, %d",
				name, ret);
		}
	}

	return ret;
}

void siw_touch_gpio_free(struct device *dev, int pin)
{
	if (gpio_is_valid(pin)) {
		gpio_free(pin);
	}
}

void siw_touch_gpio_direction_input(struct device *dev,
								int pin)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) input mode", pin);
		gpio_direction_input(pin);
	}
}

void siw_touch_gpio_direction_output(
		struct device *dev, int pin, int value)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) output mode(%d)", pin, value);
		gpio_direction_output(pin, value);
	}
}

void siw_touch_gpio_set_pull(struct device *dev, int pin, int value)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) pull(%d)", pin, value);

		siw_touch_sys_gpio_setpull(pin, value);
	}

	return;
}

#if defined(__SIW_SUPPORT_PWRCTRL)

#else	/* __SIW_SUPPORT_PWRCTRL */
int siw_touch_power_init(struct device *dev)
{
	t_dev_dbg_gpio(dev, "power init, nop ...\n");

	return 0;
}

void siw_touch_power_vdd(struct device *dev, int value)
{

}

void siw_touch_power_vio(struct device *dev, int value)
{

}
#endif	/* __SIW_SUPPORT_PWRCTRL */



