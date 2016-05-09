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

#if 1
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

void siw_touch_gpio_direction_input(struct device *dev, int pin)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) input mode", pin);
		gpio_direction_input(pin);
	}
}

void siw_touch_gpio_direction_output(struct device *dev, int pin, int value)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) output mode(%d)", pin, value);
		gpio_direction_output(pin, value);
	}
}
#else
/* just test */
int siw_touch_gpio_init(struct device *dev,	int pin, const char *name){ return 0; }
void siw_touch_gpio_free(struct device *dev, int pin){ }
void siw_touch_gpio_direction_input(struct device *dev, int pin)
{
	if (gpio_is_valid(pin)) {
		gpio_request(pin, NULL);
		t_dev_dbg_gpio(dev, "set pin(%d) input mode", pin);
		gpio_direction_input(pin);
		gpio_free(pin);
	}
}

void siw_touch_gpio_direction_output(struct device *dev, int pin, int value)
{
	if (gpio_is_valid(pin)) {
		gpio_request(pin, NULL);
		t_dev_dbg_gpio(dev, "set pin(%d) output mode(%d)", pin, value);
		gpio_direction_output(pin, value);
		gpio_free(pin);
	}
}

#endif

void siw_touch_gpio_set_pull(struct device *dev, int pin, int value)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) pull(%d)", pin, value);

		siw_touch_sys_gpio_set_pull(pin, value);
	}

	return;
}

#if defined(__SIW_SUPPORT_PWRCTRL)
int siw_touch_power_init(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int vdd_pin = touch_vdd_pin(ts);
	int vio_pin = touch_vio_pin(ts);
	void *vdd, *vio;

	t_dev_dbg_gpio(dev, "power init\n");

	if (gpio_is_valid(vdd_pin)) {
		gpio_request(vdd_pin, "touch-vdd");
	} else {
		vdd = regulator_get(dev, "vdd");
		if (IS_ERR(vdd)) {
			t_dev_info(dev, "regulator \"vdd\" not exist\n");
		} else {
			touch_set_vdd(ts, vdd);
		}
	}

	if (gpio_is_valid(vio_pin)) {
		gpio_request(vio_pin, "touch-vio");
	} else {
		vio = regulator_get(dev, "vio");
		if (IS_ERR(vio)) {
			t_dev_info(dev, "regulator \"vio\" not exist\n");
		} else {
			touch_set_vio(ts, vio);
		}
	}

	return 0;
}

void siw_touch_power_vdd(struct device *dev, int value)
{
	struct siw_ts *ts = to_touch_core(dev);
	int vdd_pin = touch_vdd_pin(ts);
	void *vdd = touch_get_vdd(ts);
	char *op;
	int ret = 0;

	if (gpio_is_valid(vdd_pin)) {
		op = "gpio out control";
		ret = gpio_direction_output(vdd_pin, value);
	} else if (!IS_ERR_OR_NULL(vdd)) {
		if (value) {
			op = "enable regulator";
			ret = regulator_enable((struct regulator *)vdd);
		} else {
			op = "disable regulator";
			ret = regulator_disable((struct regulator *)vdd);
		}
	}

	if (ret)
		t_dev_err(dev, "vdd - %s, %d", op, ret);
}

void siw_touch_power_vio(struct device *dev, int value)
{
	struct siw_ts *ts = to_touch_core(dev);
	int vio_pin = touch_vio_pin(ts);
	void *vio = touch_get_vio(ts);
	char *op;
	int ret = 0;

	if (gpio_is_valid(vio_pin)) {
		op = "gpio out control";
		ret = gpio_direction_output(vio_pin, value);
	} else if (!IS_ERR_OR_NULL(vio)) {
		if (value) {
			op = "enable regulator";
			ret = regulator_enable((struct regulator *)vio);
		} else {
			op = "disable regulator";
			ret = regulator_disable((struct regulator *)vio);
		}
	}

	if (ret)
		t_dev_err(dev, "vio - %s, %d", op, ret);
}
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



