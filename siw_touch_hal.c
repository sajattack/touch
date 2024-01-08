/*
 * siw_touch_hal.c - SiW touch hal driver
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
#include <linux/firmware.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"
#include "siw_touch_event.h"
#include "siw_touch_gpio.h"
#include "siw_touch_irq.h"
#include "siw_touch_sys.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif


extern int siw_hal_sysfs(struct device *dev, int on_off);
extern int siw_hal_sysfs_post(struct device *dev, int on_off);

/*
 * weak(dummy) function for ABT control
 * These are deactivated by enabling __SIW_SUPPORT_ABT
 * and the actual functions can be found in siw_touch_hal_abt.c
 */
int __weak siw_hal_abt_init(struct device *dev)
{
	t_dev_info_once(dev, "ABT disabled\n");
	return 0;
}
int __weak siw_hal_abt_sysfs(struct device *dev, int on_off)
{
	t_dev_info_once(dev, "ABT disabled\n");
	return 0;
}

/*
 * weak(dummy) function for PRD control
 * These are deactivated by enabling __SIW_SUPPORT_PRD
 * and the actual functions can be found in siw_touch_hal_prd.c
 */
int __weak siw_hal_prd_sysfs(struct device *dev, int on_off)
{
	t_dev_info_once(dev, "PRD disabled\n");
	return 0;
}

#if defined(__SIW_SUPPORT_WATCH)
#define t_warn_weak_watch(_dev, fmt, args...)	\
		t_dev_info_once(_dev, "Watch disabled: "fmt, ##args)
#else
#define t_warn_weak_watch(_dev, fmt, args...)	do { }while(0)
#endif
/*
 * weak(dummy) function for Watch control
 * These are deactivated by enabling __SIW_SUPPORT_WATCH
 * and the actual functions can be found in siw_touch_hal_watch.c
 */
int __weak siw_hal_watch_sysfs(struct device *dev, int on_off)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
	return 0;
}
int __weak siw_hal_watch_init(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
	return 0;
}
int __weak siw_hal_watch_chk_font_status(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
	return 0;
}
int __weak siw_hal_watch_get_curr_time(struct device *dev, char *buf, int *len)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
	return 0;
}
int __weak siw_hal_watch_display_off(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
	return 0;
}
int __weak siw_hal_watch_is_disp_waton(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
	return 0;
}
int __weak siw_hal_watch_is_rtc_run(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
	return 0;
}
void __weak siw_hal_watch_set_rtc_run(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
}
void __weak siw_hal_watch_set_rtc_clear(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
}
void __weak siw_hal_watch_set_font_empty(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
}
void __weak siw_hal_watch_set_cfg_blocked(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
}
void __weak siw_hal_watch_rtc_on(struct device *dev)
{
	t_warn_weak_watch(dev, "%s\n", __func__);
}


static int siw_hal_reset_ctrl(struct device *dev, int ctrl);

static int siw_hal_tc_driving(struct device *dev, int mode);

static void siw_hal_deep_sleep(struct device *dev);

static int siw_hal_lpwg_mode(struct device *dev);

static void siw_hal_power_init(struct device *dev)
{
	siw_touch_power_init(dev);
}

static void siw_hal_power_free(struct device *dev)
{
	siw_touch_power_free(dev);
}

static void siw_hal_power_vdd(struct device *dev, int value)
{
	siw_touch_power_vdd(dev, value);
}

static void siw_hal_power_vio(struct device *dev, int value)
{
	siw_touch_power_vio(dev, value);
}

#define SIW_HAL_GPIO_RST		"siw_hal_reset"
#define SIW_HAL_GPIO_IRQ		"siw_hal_irq"
#define SIW_HAL_GPIO_MAKER		"siw_hal_maker_id"

static int __siw_hal_gpio_skip_reset(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int reset_pin = touch_reset_pin(ts);

	if (touch_flags(ts) & TOUCH_SKIP_RESET_PIN) {
		return 1;
	}

	if (!gpio_is_valid(reset_pin)) {
		t_dev_err(dev, "reset_pin invalid, %d\n", reset_pin);
		return 1;
	}

	return 0;
}

static void __siw_hal_init_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);
	int ret = 0;

	if (__siw_hal_gpio_skip_reset(ts)) {
		return;
	}

	ret = siw_touch_gpio_init(dev,
			reset_pin,
			SIW_HAL_GPIO_RST);
	if (ret)
		return;

	siw_touch_gpio_direction_output(dev,
			reset_pin, GPIO_OUT_ONE);
	t_dev_dbg_gpio(dev, "set %s(%d) as output\n",
			SIW_HAL_GPIO_RST, reset_pin);

	siw_touch_gpio_set_pull(dev,
			reset_pin, GPIO_PULL_UP);
	t_dev_dbg_gpio(dev, "set %s(%d) as pull-up(%d)\n",
			SIW_HAL_GPIO_RST,
			reset_pin, GPIO_NO_PULL);
}

static void __siw_hal_free_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);

	if (__siw_hal_gpio_skip_reset(ts)) {
		return;
	}

	siw_touch_gpio_free(dev, reset_pin);
}

static void __siw_hal_set_gpio_reset(struct device *dev, int val)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);

	if (__siw_hal_gpio_skip_reset(ts)) {
		return;
	}

	siw_touch_gpio_direction_output(dev,
			reset_pin, !!(val));
	t_dev_dbg_gpio(dev, "set %s(%d) : %d\n",
			SIW_HAL_GPIO_RST,
			reset_pin, !!(val));
}

static void siw_hal_init_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_init_reset) {
		ret = fquirks->gpio_init_reset(dev);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_init_gpio_reset(dev);
}

static void siw_hal_free_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_free_reset) {
		ret = fquirks->gpio_free_reset(dev);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_free_gpio_reset(dev);
}

static void siw_hal_set_gpio_reset(struct device *dev, int val)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_set_reset) {
		ret = fquirks->gpio_set_reset(dev, val);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_set_gpio_reset(dev, val);
}

static void siw_hal_trigger_gpio_reset(struct device *dev, int delay)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);
	touch_msleep(chip->drv_reset_low + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_0));
	siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);

	t_dev_info(dev, "trigger gpio reset\n");

	touch_msleep((delay) ? delay : ts->caps.hw_reset_delay);
}

static int __siw_hal_gpio_skip_irq(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int irq_pin = touch_irq_pin(ts);

	if (!gpio_is_valid(irq_pin)) {
		t_dev_err(dev, "irq_pin inavlid, %d\n", irq_pin);
		return 1;
	}

	return 0;
}

static void __siw_hal_init_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int irq_pin = touch_irq_pin(ts);
	int ret = 0;

	if (__siw_hal_gpio_skip_irq(ts)) {
		return;
	}

	ret = siw_touch_gpio_init(dev,
			irq_pin,
			SIW_HAL_GPIO_IRQ);
	if (ret)
		return;

	siw_touch_gpio_direction_input(dev,
			irq_pin);
	t_dev_dbg_gpio(dev, "set %s(%d) as input\n",
			SIW_HAL_GPIO_IRQ,
			irq_pin);

	siw_touch_gpio_set_pull(dev,
			irq_pin, GPIO_PULL_UP);
	t_dev_dbg_gpio(dev, "set %s(%d) as pull-up(%d)\n",
			SIW_HAL_GPIO_IRQ,
			irq_pin, GPIO_PULL_UP);
}

static void __siw_hal_free_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int irq_pin = touch_irq_pin(ts);

	if (__siw_hal_gpio_skip_irq(ts)) {
		return;
	}

	siw_touch_gpio_free(dev, irq_pin);
}

static void siw_hal_init_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_init_irq) {
		ret = fquirks->gpio_init_irq(dev);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_init_gpio_irq(dev);
}

static void siw_hal_free_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_free_irq) {
		ret = fquirks->gpio_free_irq(dev);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_free_gpio_irq(dev);
}

static void siw_hal_init_gpio_maker_id(struct device *dev)
{
#if 0
	struct siw_ts *ts = to_touch_core(dev);
	int maker_id_pin = touch_maker_id_pin(ts);
	int ret = 0;

	if (!gpio_is_valid(maker_id_pin)) {
		return;
	}

	ret = siw_touch_gpio_init(dev,
			maker_id_pin,
			SIW_HAL_GPIO_MAKER);
	if (ret)
		return;

	siw_touch_gpio_direction_input(dev,
			maker_id_pin);
#endif
}

static void siw_hal_free_gpio_maker_id(struct device *dev)
{
#if 0
	struct siw_ts *ts = to_touch_core(dev);
	int maker_id_pin = touch_maker_id_pin(ts);

	if (!gpio_is_valid(maker_id_pin)) {
		return;
	}

	siw_touch_gpio_free(dev, maker_id_pin);
#endif
}

static void siw_hal_init_gpios(struct device *dev)
{
	siw_hal_init_gpio_reset(dev);

	siw_hal_init_gpio_irq(dev);

	siw_hal_init_gpio_maker_id(dev);
}

static void siw_hal_free_gpios(struct device *dev)
{
	siw_hal_free_gpio_reset(dev);

	siw_hal_free_gpio_irq(dev);

	siw_hal_free_gpio_maker_id(dev);
}

u32 t_bus_dbg_mask = 0;

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/bus_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko bus_dbg_mask=<value>
 */
module_param_named(bus_dbg_mask, t_bus_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);

#define SIW_HAL_BUS_TAG 		"hal(bus): "
#define SIW_HAL_BUS_TAG_ERR 	"hal(bus)(E): "
#define SIW_HAL_BUS_TAG_WARN	"hal(bus)(W): "
#define SIW_HAL_BUS_TAG_DBG		"hal(bus)(D): "

#if 1
#define t_hal_bus_info(_dev, fmt, args...)	\
		__t_dev_info(_dev, SIW_HAL_BUS_TAG fmt, ##args)

#define t_hal_bus_warn(_abt, fmt, args...)	\
		__t_dev_warn(_dev, SIW_HAL_BUS_TAG_WARN fmt, ##args)
#else
#define t_hal_bus_info(_dev, fmt, args...)	__t_dev_none(_dev, fmt, ##args)
#define t_hal_bus_warn(_dev, fmt, args...)	__t_dev_none(_dev, fmt, ##args)
#endif

#define t_hal_bus_err(_dev, fmt, args...)	\
		__t_dev_err(_dev, SIW_HAL_BUS_TAG_ERR fmt, ##args)

#define t_hal_bus_dbg(condition, _dev, fmt, args...)			\
		do {							\
			if (unlikely(t_bus_dbg_mask & (condition)))	\
				__t_dev_info(_dev, SIW_HAL_BUS_TAG_DBG fmt, ##args);	\
		} while (0)

#define t_hal_bus_dbg_base(_dev, fmt, args...)	\
		t_hal_bus_dbg(DBG_BASE, _dev, fmt, ##args)

#define t_hal_bus_dbg_trace(_dev, fmt, args...)	\
		t_hal_bus_dbg(DBG_TRACE, _dev, fmt, ##args)

#define DBG_BUS_ERR_TRACE	DBG_GET_DATA

static void __siw_hal_bus_err(struct device *dev,
		u32 addr, u8 *buf, int size, int wr)
{
	int prt_len = 0;
	int prt_idx = 0;
	int prd_sz = size;

	if (!unlikely(t_bus_dbg_mask & DBG_BUS_ERR_TRACE)) {
		return;
	}

	while (size) {
		prt_len = min(size, 16);

		t_hal_bus_err(dev,
				"%s 0x%04X, 0x%04X buf[%3d~%3d] %*ph\n",
				(wr) ? "wr" : "rd",
				(u32)addr, (u32)prd_sz,
				prt_idx, prt_idx + prt_len - 1,
				prt_len, &buf[prt_idx]);

		size -= prt_len;
		prt_idx += prt_len;
	}
}

static void __siw_hal_bus_dbg(struct device *dev,
		u32 addr, u8 *buf, int size, int wr)
{
	int prt_len = 0;
	int prt_idx = 0;
	int prd_sz = size;

	if (!unlikely(t_bus_dbg_mask & DBG_TRACE)) {
		return;
	}

	while (size) {
		prt_len = min(size, 16);

		t_hal_bus_dbg_trace(dev,
				"%s 0x%04X, 0x%04X buf[%3d~%3d] %*ph\n",
				(wr) ? "wr" : "rd",
				(u32)addr, (u32)prd_sz,
				prt_idx, prt_idx + prt_len - 1,
				prt_len, &buf[prt_idx]);

		size -= prt_len;
		prt_idx += prt_len;
	}
}

static void *__siw_hal_get_curr_buf(struct siw_ts *ts, dma_addr_t *dma, int tx)
{
	struct siw_touch_buf *t_buf;
	int *idx;
	void *buf = NULL;

	idx = (tx) ? &ts->tx_buf_idx : &ts->rx_buf_idx;
	t_buf = (tx) ? &ts->tx_buf[(*idx)] : &ts->rx_buf[(*idx)];

	buf = t_buf->buf;
	if (dma)
		*dma = t_buf->dma;

	(*idx)++;
	(*idx) %= SIW_TOUCH_MAX_BUF_IDX;

	return buf;
}

static int __siw_hal_reg_quirk_addr(struct device *dev,
			u32 addr, int size, int wr)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int t_rw_opt = chip->opt.t_rw_opt;
	int bus_type = touch_bus_type(ts);
	int last = addr + size - 1;
	int invalid_s = 0;
	int invalid_e = 0;
	int detected = 0;

	switch (t_rw_opt) {
	case 1:
		invalid_s = (bus_type == BUS_IF_I2C) ? 0x400 : 0x200;
		invalid_e = invalid_s + 0x200;

		if ((addr >= invalid_s) && (addr < invalid_e)) {
			detected |= 0x1;
		}
		if ((last >= invalid_s) && (last < invalid_e)) {
			detected |= (0x1<<1);
		}

		if (detected) {
			t_dev_info(dev,
				"invalid access(%s) : %04Xh, %04Xh (%X, %04Xh, %04Xh)\n",
				(wr) ? "wr" : "rd",
				addr, last,
				detected, invalid_s, invalid_e);
			return -EINVAL;
		}
		break;
	}

	return 0;
}

static void __siw_hal_reg_quirk_rd_i2c(struct device *dev,
			u32 *addr, int *size,
			int *hdr_sz, int *dummy_sz, int *hdr_flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int t_i2c_opt = chip->opt.t_i2c_opt;
	u32 __addr = 0;
	int __size = 0;
	int __hdr_sz = 0;
	int __dummy_sz = 0;
	int __hdr_flag = 0;

	if ((touch_bus_type(ts) != BUS_IF_I2C) || !t_i2c_opt) {
		return;
	}

	__addr = (*addr);
	__size = (*size);
	__hdr_sz = (*hdr_sz);
	__dummy_sz = (*dummy_sz);
	__hdr_flag = (*hdr_flag);

	switch (t_i2c_opt) {
	case 1:
		/*
		 * If 0x200 and burst, change to 0x201
		 * If 0x201, change to 0x202
		 */
		if (__addr == reg->tc_ic_status) {
			__addr += !!(__size > 4);
		} else {
			__addr += !!(__addr == reg->tc_status);
		}
		break;
	}

	(*addr) = __addr;
	(*size) = __size;
	(*hdr_sz) = __hdr_sz;
	(*dummy_sz) = __dummy_sz;
	(*hdr_flag) = __hdr_flag;
}

static void __siw_hal_reg_quirk_rd_spi(struct device *dev,
			u32 *addr, int *size,
			int *hdr_sz, int *dummy_sz, int *hdr_flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int t_spi_opt = chip->opt.t_spi_opt;
	u32 __addr = 0;
	int __size = 0;
	int __hdr_sz = 0;
	int __dummy_sz = 0;
	int __hdr_flag = 0;
	int dummy_idx = 0;

	if ((touch_bus_type(ts) != BUS_IF_SPI) || !t_spi_opt) {
		return;
	}

	__addr = (*addr);
	__size = (*size);
	__hdr_sz = (*hdr_sz);
	__dummy_sz = (*dummy_sz);
	__hdr_flag = (*hdr_flag);

	switch (t_spi_opt) {
	case 1:
		if (touch_max_freq(ts) > __CLOCK_MHZ(10)) {
			dummy_idx = !!((__addr >= 0xC00) && (__addr < 0xF00));
		}

		switch (dummy_idx) {
		case 1:
			__hdr_sz = SPI_BUS_RX_HDR_SZ_128BIT;
			__dummy_sz = SPI_BUS_RX_DUMMY_SZ_128BIT;
			__hdr_flag = SPI_BUS_RX_DUMMY_FLAG_128BIT;
			break;
		default:
			__hdr_sz = SPI_BUS_RX_HDR_SZ_32BIT;
			__dummy_sz = SPI_BUS_RX_DUMMY_SZ_32BIT;
			break;
		}
		break;
	}

	(*addr) = __addr;
	(*size) = __size;
	(*hdr_sz) = __hdr_sz;
	(*dummy_sz) = __dummy_sz;
	(*hdr_flag) = __hdr_flag;
}

static int __siw_hal_reg_quirk_rd(struct device *dev,
			u32 *addr, int *size,
			int *hdr_sz, int *dummy_sz, int *hdr_flag)
{
	int ret;

	ret = __siw_hal_reg_quirk_addr(dev, (*addr), (*size), 0);
	if (ret < 0) {
		return -EINVAL;
	}

	__siw_hal_reg_quirk_rd_i2c(dev, addr, size, hdr_sz, dummy_sz, hdr_flag);
	__siw_hal_reg_quirk_rd_spi(dev, addr, size, hdr_sz, dummy_sz, hdr_flag);

	return 0;
}

static int __siw_hal_reg_quirk_wr(struct device *dev,
			u32 *addr, int *size, int *hdr_sz)
{
	int ret = 0;

	ret = __siw_hal_reg_quirk_addr(dev, (*addr), (*size), 1);
	if (ret < 0) {
		return -EINVAL;
	}

	return 0;
}

//#define __SIW_CONFIG_CLR_RX_BUFFER

static int __used __siw_hal_do_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int bus_tx_hdr_size = touch_tx_hdr_size(ts);
	int bus_rx_hdr_size = touch_rx_hdr_size(ts);
//	int bus_tx_dummy_size = touch_tx_dummy_size(ts);
	int bus_rx_dummy_size = touch_rx_dummy_size(ts);
	int bus_rd_hdr_flag = 0;
	struct touch_bus_msg _msg = {0, };
	struct touch_bus_msg *msg = &_msg;
	int tx_size = bus_tx_hdr_size;
	u8 *tx_buf;
	u8 *rx_buf;
	dma_addr_t tx_dma;
	dma_addr_t rx_dma;
	int ret = 0;

#if 0
	if (!addr) {
		t_dev_err(dev, "NULL addr\n");
		return -EFAULT;
	}
#endif
	if (!data) {
		t_dev_err(dev, "NULL data(0x%04X, 0x%04X)\n", addr, size);
		return -EFAULT;
	}

	ret = __siw_hal_reg_quirk_rd(dev, &addr, &size,
		&bus_rx_hdr_size, &bus_rx_dummy_size, &bus_rd_hdr_flag);
	if (ret < 0) {
		return -EINVAL;
	}

//	t_dev_info(dev, "addr %04Xh, size %d\n", addr, size);

	tx_buf = __siw_hal_get_curr_buf(ts, &tx_dma, 1);
	rx_buf = __siw_hal_get_curr_buf(ts, &rx_dma, 0);

#if defined(__SIW_CONFIG_CLR_RX_BUFFER)
	if (touch_bus_type(ts) == BUS_IF_I2C) {
		memset(&rx_buf[bus_rx_hdr_size], 0xFF, min(8, size));
	}
#endif

	switch (chip->opt.t_bus_opt) {
	case 1:
		tx_buf[0] = ((addr >> 8) & 0xff);
		tx_buf[1] = (addr & 0xff);
		break;
	default:
		tx_buf[0] = bus_rd_hdr_flag | ((size > 4) ? 0x20 : 0x00);
		tx_buf[0] |= ((addr >> 8) & 0x0f);
		tx_buf[1] = (addr & 0xff);
		break;
	}

//	while (bus_tx_dummy_size--) {
	while (bus_rx_dummy_size--) {
		tx_buf[tx_size++] = 0;
	}

	msg->tx_buf = tx_buf;
	msg->tx_size = tx_size;
	msg->rx_buf = rx_buf;
	msg->rx_size = bus_rx_hdr_size + size;
	msg->tx_dma = tx_dma;
	msg->rx_dma = rx_dma;
	msg->bits_per_word = 8;
	msg->priv = 0;

	ret = siw_touch_bus_read(dev, msg);
	if (ret < 0) {
		t_hal_bus_err(dev, "read reg error(0x%04X, 0x%04X, %*ph), %d\n",
				(u32)addr, (u32)size,
				tx_size, tx_buf,
				ret);
		__siw_hal_bus_err(dev, addr, (u8 *)msg->rx_buf, msg->rx_size, 0);
		return ret;
	}

	memcpy(data, &rx_buf[bus_rx_hdr_size], size);

	__siw_hal_bus_dbg(dev, addr, (u8 *)data, size, 0);

	return size;
}

static int __used __siw_hal_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	if (siw_addr_is_skip(addr)) {
		t_hal_bus_dbg_base(dev, "rd: skip by ADDR_SKIP_MASK\n");
		return 0;
	}

	mutex_lock(&chip->bus_lock);
	ret = __siw_hal_do_reg_read(dev, addr, data, size);
	mutex_unlock(&chip->bus_lock);

	return ret;
}

static int __used __siw_hal_do_reg_write(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int bus_tx_hdr_size = touch_tx_hdr_size(ts);
//	int bus_rx_hdr_size = touch_rx_hdr_size(ts);
	struct touch_bus_msg _msg = {0, };
	struct touch_bus_msg *msg = &_msg;
	u8 *tx_buf;
	dma_addr_t tx_dma;
	int is_spi = !!(touch_bus_type(ts) == BUS_IF_SPI);
	int ret = 0;

#if 0
	if (!addr) {
		t_dev_err(dev, "NULL addr\n");
		return -EFAULT;
	}
#endif
	if (!data) {
		t_dev_err(dev, "NULL data(0x%04X, 0x%04X)\n", addr, size);
		return -EFAULT;
	}

	ret = __siw_hal_reg_quirk_wr(dev, &addr, &size, &bus_tx_hdr_size);
	if (ret < 0) {
		return -EINVAL;
	}

	tx_buf = __siw_hal_get_curr_buf(ts, &tx_dma, 1);

	switch (chip->opt.t_bus_opt) {
	case 1:
		tx_buf[0] = ((addr >> 8) & 0xff);
		tx_buf[1] = (addr & 0xff);
		break;
	default:
		tx_buf[0] = (is_spi || (size > 4)) ? 0x60 : 0x40;
		tx_buf[0] |= ((addr >> 8) & 0x0f);
		tx_buf[1] = (addr & 0xff);
		break;
	}

	msg->tx_buf = tx_buf;
	msg->tx_size = bus_tx_hdr_size + size;
	msg->rx_buf = NULL;
	msg->rx_size = 0;
	msg->tx_dma = tx_dma;
	msg->rx_dma = 0;
	msg->bits_per_word = 8;
	msg->priv = 0;

	memcpy(&tx_buf[bus_tx_hdr_size], data, size);

	ret = siw_touch_bus_write(dev, msg);
	if (ret < 0) {
		t_hal_bus_err(dev, "write reg error(0x%04X, 0x%04X, %*ph - %*ph%s), %d\n",
				(u32)addr, (u32)size,
				bus_tx_hdr_size, tx_buf,
				min(size, 8), data,
				(size > 8) ? " ..." : "",
				ret);
		__siw_hal_bus_err(dev, addr, (u8 *)data, size, 1);
		return ret;
	}

	__siw_hal_bus_dbg(dev, addr, (u8 *)data, size, 1);

	return size;
}

static int __used __siw_hal_reg_write(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	if (siw_addr_is_skip(addr)) {
		t_hal_bus_dbg_base(dev, "wr: skip by ADDR_SKIP_MASK\n");
		return 0;
	}

	mutex_lock(&chip->bus_lock);
	ret = __siw_hal_do_reg_write(dev, addr, data, size);
	mutex_unlock(&chip->bus_lock);

	return ret;
}

int siw_hal_read_value(struct device *dev, u32 addr, u32 *value)
{
	return __siw_hal_reg_read(dev, addr, value, sizeof(u32));
}

int siw_hal_write_value(struct device *dev, u32 addr, u32 value)
{
	return __siw_hal_reg_write(dev, addr, &value, sizeof(u32));
}

int siw_hal_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	return __siw_hal_reg_read(dev, addr, data, size);
}

int siw_hal_reg_write(struct device *dev, u32 addr, void *data, int size)
{
	return __siw_hal_reg_write(dev, addr, data, size);
}

int siw_hal_reg_read_single(struct device *dev, u32 addr, void *data, int size)
{
	u32 *__data = (u32 *)data;
	int __size;
	int ret = 0;

	while (size) {
		__size = min(4, size);
		ret = siw_hal_reg_read(dev, addr, __data, __size);
		if (ret < 0) {
			break;
		}

		addr++;
		__data++;
		size -= __size;
	}

	return ret;
}

int siw_hal_reg_write_single(struct device *dev, u32 addr, void *data, int size)
{
	u32 *__data = (u32 *)data;
	int __size;
	int ret = 0;

	while (size) {
		__size = min(4, size);
		ret = siw_hal_reg_write(dev, addr, __data, __size);
		if (ret < 0) {
			break;
		}

		addr++;
		__data++;
		size -= __size;
	}

	return ret;
}

int siw_hal_reg_rw_multi(struct device *dev,
		struct siw_hal_rw_multi *multi, char *title)
{
	int (*func)(struct device *dev, u32 addr, void *data, int size);
	int ret = 0;

	while (1) {
		if ((multi->wr == -1) ||
			(multi->addr == -1) ||
			(multi->data == NULL)) {
			break;
		}

		func = (multi->wr) ? siw_hal_reg_write : siw_hal_reg_read;

		ret = func(dev, multi->addr, multi->data, multi->size);
		if (ret < 0) {
			t_dev_err(dev, "%s: %s %s failed, %d\n",
				title, (multi->name) ? multi->name : "",
				(multi->wr) ? "write" : "read",
				ret);
			break;
		}

		multi++;
	}

	return ret;
}

static int siw_hal_cmd_write(struct device *dev, u8 cmd)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct touch_bus_msg msg = {0, };
	u8 input[2] = {0, };
	int ret = 0;

	input[0] = cmd;
	input[1] = 0;

	msg.tx_buf = input;
	msg.tx_size = 2;

	msg.rx_buf = NULL;
	msg.rx_size = 0;
	msg.bits_per_word = 8;

	ret = siw_touch_bus_write(dev, &msg);
	if (ret < 0) {
		t_dev_err(dev, "touch cmd(0x%02X) write error, %d\n",
				cmd, ret);
		return ret;
	}
	return 0;
}

static int siw_hal_condition_wait(struct device *dev,
				    u16 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry)
{
	u32 data = 0;
	int ret = 0;

	do {
		touch_msleep(delay);

		ret = siw_hal_read_value(dev, addr, &data);
		switch (expect) {
		case FLASH_CODE_DNCHK_VALUE:
		case FLASH_CONF_DNCHK_VALUE:
			t_dev_dbg_base(dev,
				"wait read: addr[%04Xh] data[%08Xh], "
				"mask[%08Xh], expect[%08Xh], %d\n",
				addr, data, mask, expect, retry);
			break;
		}
		if ((ret >= 0) && ((data & mask) == expect)) {
			if (value)
				*value = data;
		#if 0
			t_dev_info(dev,
				"wait done: addr[%04Xh] data[%08Xh], "
				"mask[%08Xh], expect[%08Xh], %d\n",
				addr, data, mask, expect, retry);
		#endif
			return 0;
		}
	} while (--retry);

	if (value)
		*value = data;

	t_dev_err(dev,
		"wait fail: addr[%04Xh] data[%08Xh], "
		"mask[%08Xh], expect[%08Xh]\n",
		addr, data, mask, expect);

	return -EPERM;
}

enum {
	EQ_COND = 0,
	NOT_COND,
};

static int __used siw_hal_condition_wait_cond(struct device *dev,
					u32 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry, int not_cond)
{
	u32 data = 0;
	int match = 0;
	int ret = 0;

	do {
		touch_msleep(delay);

		ret = siw_hal_read_value(dev, addr, &data);
		if (ret >= 0) {
			match = (not_cond == NOT_COND) ?	\
				!!((data & mask) != expect) : !!((data & mask) == expect);

			if (match) {
				if (value)
					*value = data;

				return 0;
			}
		}
	} while (--retry);

	if (value) {
		*value = data;
	}

	t_dev_err(dev,
		"wait fail: addr[%04Xh] data[%08Xh], "
		"mask[%08Xh], expect[%s%08Xh]\n",
		addr, data, mask,
		(not_cond == NOT_COND) ? "not " : "",
		expect);

	return -EPERM;
}

static int siw_hal_flash_wp(struct device *dev, int wp)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->flash_wp) {
		ret = fquirks->flash_wp(dev, wp);
		t_dev_info(dev, "flash_wp(%s) %s\n",
			touch_chip_name(ts), (wp) ? "enabled" : "disabled");
	}

	return ret;
}

int siw_hal_enable_flash_wp(struct device *dev)
{
	return siw_hal_flash_wp(dev, 1);
}

int siw_hal_disable_flash_wp(struct device *dev)
{
	return siw_hal_flash_wp(dev, 0);
}

int siw_hal_access_not_allowed(struct device *dev, char *title, int skip_flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	char *sub = "";

	if (!(skip_flag & HAL_ACCESS_CHK_SKIP_SLEEP)) {
		if (atomic_read(&ts->state.sleep) != IC_NORMAL) {
			sub = "not IC_NORMAL";
			goto out;
		}
	}

	if (!(skip_flag & HAL_ACCESS_CHK_SKIP_FB)) {
		if (atomic_read(&ts->state.fb) != FB_RESUME) {
			sub = "not FB_RESUME";
			goto out;
		}
	}

	if (!(skip_flag & HAL_ACCESS_CHK_SKIP_PM)) {
		if (atomic_read(&ts->state.pm) != DEV_PM_RESUME) {
			sub = "not DEV_PM_RESUME";
			goto out;
		}
	}

	if (!(skip_flag & HAL_ACCESS_CHK_SKIP_INIT)) {
		if (atomic_read(&chip->init) != IC_INIT_DONE) {
			sub = "not IC_INIT_DONE";
			goto out;
		}
	}

	return 0;

out:
	if (title) {
		t_dev_warn(dev, "%s: %s\n", title, sub);
	}

	return 1;
}

static int siw_hal_tc_not_allowed(struct device *dev, char *title)
{
#if defined(__SIW_CONFIG_SYSTEM_PM)
	int skip_flag = 0;
#else	/* __SIW_CONFIG_SYSTEM_PM */
	int skip_flag = HAL_ACCESS_CHK_SKIP_PM | HAL_ACCESS_CHK_SKIP_FB;
#endif	/* __SIW_CONFIG_SYSTEM_PM */

	return siw_hal_access_not_allowed(dev, title, skip_flag);
}

static void siw_hal_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct siw_touch_chip *chip =
			container_of(to_delayed_work(fb_notify_work),
				struct siw_touch_chip, fb_notify_work);
	int type = FB_RESUME;

	switch (chip->lcd_mode) {
	case LCD_MODE_U0:
	case LCD_MODE_U2:
		type = FB_SUSPEND;
		break;
	}

	siw_touch_notifier_call_chain(NOTIFY_FB, &type);
}

static void siw_hal_init_works(struct siw_touch_chip *chip)
{
	INIT_DELAYED_WORK(&chip->fb_notify_work, siw_hal_fb_notify_work_func);
}

static void siw_hal_free_works(struct siw_touch_chip *chip)
{
	cancel_delayed_work(&chip->fb_notify_work);
}

static void siw_hal_init_locks(struct siw_touch_chip *chip)
{
	mutex_init(&chip->bus_lock);
}

static void siw_hal_free_locks(struct siw_touch_chip *chip)
{
	mutex_destroy(&chip->bus_lock);
}

static u32 siw_hal_get_subdisp_sts(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 rdata = LCD_MODE_U3;	//dummy value

	siw_hal_read_value(dev, reg->spr_subdisp_status, &rdata);

	return rdata;
}

int siw_hal_get_boot_status(struct device *dev, u32 *boot_st)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	struct siw_hal_ops_quirk *ops_quirk = &chip->ops_quirk;
	struct siw_hal_reg *reg = chip->reg;
	u32 bootmode = 0;
	int ret = 0;

	if (fquirks->boot_status) {
		ret = fquirks->boot_status(dev, boot_st);
		if (ret != -EAGAIN) {
			return ret;
		}
	}

	if (ops_quirk->boot_status) {
		ret = ops_quirk->boot_status(dev, boot_st);
		if (ret != -EAGAIN) {
			return ret;
		}
	}

	ret = siw_hal_read_value(dev, reg->spr_boot_status, &bootmode);
	if (ret < 0) {
		return ret;
	}

	if (boot_st) {
		*boot_st = bootmode;
	}

	return 0;
}

#if defined(__SIW_CONFIG_KNOCK)
#define __TCI_INFO_SET(_idx, _t, _mit, _mat, _ts, _td, _id)	\
		[_idx] = {	\
			.tap_count = _t,	\
			.min_intertap = _mit, .max_intertap = _mat,	\
			.touch_slop = _ts, .tap_distance = _td,	\
			.intr_delay = _id,	\
		}

const struct tci_info siw_hal_tci_info_default[2] = {
	__TCI_INFO_SET(TCI_1, 2, 6, 70, 100, 10, 0),
	__TCI_INFO_SET(TCI_2, 0, 6, 70, 100, 255, 20),
};

static void siw_prt_tci_info(struct device *dev, char *name, struct tci_info *info)
{
	t_dev_info(dev,
		"tci info[%s] tap_count %d, min_intertap %d, max_intertap %d\n",
		name, info->tap_count, info->min_intertap, info->max_intertap);

	t_dev_info(dev,	\
		"tci info[%s] touch_slop %d, tap_distance %d, intr_delay %d\n",
		name, info->touch_slop, info->tap_distance, info->intr_delay);
}

static void siw_prt_active_area_info(struct device *dev, char *name, struct active_area *area)
{
	t_dev_info(dev, "tci active_area %s %4d %4d %4d %4d\n",
		name, area->x1, area->y1, area->x2, area->y2);
}

static void siw_prt_reset_area_info(struct device *dev, char *name, struct reset_area *area, int shift)
{
	t_dev_info(dev, "tci reset_area %s %4d %4d %4d %4d\n",
		name,
		(area->x1 >> shift) & 0xFFFF,
		(area->y1 >> shift) & 0xFFFF,
		(area->x2 >> shift) & 0xFFFF,
		(area->y2 >> shift) & 0xFFFF);
}

static void siw_hal_prt_tci_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct tci_ctrl *tci = &ts->tci;
	struct tci_info *info;
	struct active_area *area;
	struct reset_area *tci_qcover;

	if (!ts->role.use_lpwg) {
		return;
	}

	info = &tci->info[TCI_1];
	siw_prt_tci_info(dev, "TCI_1", info);
	info = &tci->info[TCI_2];
	siw_prt_tci_info(dev, "TCI_2", info);

	area = &tci->area;
	siw_prt_active_area_info(dev, "tci active area ", area);

	tci_qcover = &tci->qcover_open;
	siw_prt_reset_area_info(dev, "tci qcover_open ", tci_qcover, 0);

	tci_qcover = &tci->qcover_close;
	siw_prt_reset_area_info(dev, "tci qcover_close", tci_qcover, 0);
}

static void siw_hal_get_tci_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct tci_ctrl *tci = &ts->tci;
	void *tci_src;
	struct reset_area *tci_qcover;

	//Set default
	tci->area.x1 = 0;
	tci->area.y1 = 0;
	tci->area.x2 = ts->caps.max_x;
	tci->area.y2 = ts->caps.max_y;

	tci->info[TCI_1].tap_count = 2;

	if (chip->opt.t_knock) {
		siw_prt_active_area_info(dev, "tci active area ", &tci->area);
		return;
	}

	tci_src = pdata_tci_info(ts->pdata);
	if (tci_src == NULL) {
		tci_src = (void *)siw_hal_tci_info_default;
	}
	memcpy(tci->info, tci_src, sizeof(tci->info));

	tci_qcover = pdata_tci_qcover_open(ts->pdata);
	if (tci_qcover != NULL) {
		memcpy(&tci->qcover_open, tci_qcover, sizeof(struct reset_area));
	}

	tci_qcover = pdata_tci_qcover_close(ts->pdata);
	if (tci_qcover != NULL) {
		memcpy(&tci->qcover_close, tci_qcover, sizeof(struct reset_area));
	}

	siw_hal_prt_tci_info(dev);
}

static void siw_hal_set_tci_debug_type_1(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 wdata = chip->tci_debug_type;
	int ret = 0;

	t_dev_info(dev, "TCI-Debug: %s\n", (wdata) ? "Enable" : "Disable");

	ret = siw_hal_write_value(dev, reg->tci_debug_fail_ctrl, wdata);
	if (ret < 0) {
		t_dev_err(dev, "TCI-Debug: ctrl failed, %d\n", ret);
	}
}

static void siw_hal_set_tci_debug(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	switch (chip->opt.t_chk_tci_debug) {
	case 1:
		siw_hal_set_tci_debug_type_1(dev);
		break;
	default:
		break;
	}
}

static int siw_hal_tci_knock(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data[7];
	int ret = 0;

	siw_hal_set_tci_debug(dev);

	lpwg_data[0] = ts->tci.mode;
	lpwg_data[1] = info1->tap_count | (info2->tap_count << 16);
	lpwg_data[2] = info1->min_intertap | (info2->min_intertap << 16);
	lpwg_data[3] = info1->max_intertap | (info2->max_intertap << 16);
	lpwg_data[4] = info1->touch_slop | (info2->touch_slop << 16);
	lpwg_data[5] = info1->tap_distance | (info2->tap_distance << 16);
	lpwg_data[6] = info1->intr_delay | (info2->intr_delay << 16);

	t_dev_dbg_base(dev, "lpwg_data[0] : %08Xh\n", lpwg_data[0]);
	t_dev_dbg_base(dev, "lpwg_data[1] : %08Xh\n", lpwg_data[1]);
	t_dev_dbg_base(dev, "lpwg_data[2] : %08Xh\n", lpwg_data[2]);
	t_dev_dbg_base(dev, "lpwg_data[3] : %08Xh\n", lpwg_data[3]);
	t_dev_dbg_base(dev, "lpwg_data[4] : %08Xh\n", lpwg_data[4]);
	t_dev_dbg_base(dev, "lpwg_data[5] : %08Xh\n", lpwg_data[5]);
	t_dev_dbg_base(dev, "lpwg_data[6] : %08Xh\n", lpwg_data[6]);

	ret = siw_hal_reg_write_single(dev,
				reg->tci_enable_w,
				(void *)lpwg_data, sizeof(lpwg_data));

	return ret;
}

static int siw_hal_tci_password(struct device *dev)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);

	siw_hal_set_tci_debug(dev);

	return siw_hal_tci_knock(dev);
}

static int siw_hal_tci_active_area(struct device *dev,
		u32 x1, u32 y1, u32 x2, u32 y2, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int margin = touch_senseless_margin(ts);
	u32 area[4] = { 0, };
	int i;
	int ret = 0;

	t_dev_info(dev, "tci_active_area[%d]: x1[%Xh], y1[%Xh], x2[%Xh], y2[%Xh]\n",
		type, x1, y1, x2, y2);

	area[0] = (x1 + margin) & 0xFFFF;
	area[1] = (y1 + margin) & 0xFFFF;
	area[2] = (x2 - margin) & 0xFFFF;
	area[3] = (y2 - margin) & 0xFFFF;

	for (i = 0; i < ARRAY_SIZE(area); i++) {
		area[i] = (area[i]) | (area[i]<<16);
	}

	ret = siw_hal_reg_write_single(dev,
				reg->act_area_x1_w,
				(void *)area, sizeof(area));

	return ret;
}

static int siw_hal_tci_area_set(struct device *dev, int cover_status)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct reset_area *qcover;
	const char *msg;
	int qcover_invalid = 0;

	if (chip->opt.t_knock)
		return 0;

	if (!chip->mode_allowed_qcover) {
		return 0;
	}

	qcover = (cover_status == QUICKCOVER_CLOSE) ?
			&ts->tci.qcover_close : &ts->tci.qcover_open;
	msg = (cover_status == QUICKCOVER_CLOSE) ?
			"close" : "open";

	if (!qcover->x2 || !qcover->y2) {
		/* deactivated */
		return 0;
	}

	qcover_invalid |= (qcover->x1 >= qcover->x2);
	qcover_invalid |= (qcover->y1 >= qcover->y2);

	if (!qcover_invalid) {
		siw_hal_tci_active_area(dev,
				qcover->x1, qcover->y1,
				qcover->x2, qcover->y2,
				0);
	}

	t_dev_info(dev,
		"lpwg active area - qcover %s %4d %4d %4d %4d%s\n",
		msg, qcover->x1, qcover->y1, qcover->x2, qcover->y2,
		(qcover_invalid) ? " (invalid)": "");

	return 0;
}

static int siw_hal_tci_control(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct tci_ctrl *tci = &ts->tci;
	struct active_area *area = &tci->area;
	struct tci_info *info1 = &tci->info[TCI_1];
	struct tci_info *info2 = &tci->info[TCI_2];
	u32 reg_w = ~0;
	u32 data;
	int ret = 0;

	switch (type) {
	case ENABLE_CTRL:
		reg_w = reg->tci_enable_w;
		data = tci->mode;
		break;

	case TAP_COUNT_CTRL:
		reg_w = reg->tap_count_w;
		data = info1->tap_count | (info2->tap_count << 16);
		break;

	case MIN_INTERTAP_CTRL:
		reg_w = reg->min_intertap_w;
		data = info1->min_intertap | (info2->min_intertap << 16);
		break;

	case MAX_INTERTAP_CTRL:
		reg_w = reg->max_intertap_w;
		data = info1->max_intertap | (info2->max_intertap << 16);
		break;

	case TOUCH_SLOP_CTRL:
		reg_w = reg->touch_slop_w;
		data = info1->touch_slop | (info2->touch_slop << 16);
		break;

	case TAP_DISTANCE_CTRL:
		reg_w = reg->tap_distance_w;
		data = info1->tap_distance | (info2->tap_distance << 16);
		break;

	case INTERRUPT_DELAY_CTRL:
		reg_w = reg->int_delay_w;
		data = info1->intr_delay | (info2->intr_delay << 16);
		break;

	case ACTIVE_AREA_CTRL:
		ret = siw_hal_tci_active_area(dev,
					area->x1, area->y1,
					area->x2, area->y2,
					type);
		break;

	default:
		break;
	}

	if (reg_w != ~0) {
		ret = siw_hal_write_value(dev,
					reg_w,
					data);
	}

	return ret;
}

static int siw_hal_lpwg_control(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	int ret = 0;

	if (chip->opt.t_knock)
		return 0;

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		if (touch_senseless_margin(ts)) {
			ret = siw_hal_tci_control(dev, ACTIVE_AREA_CTRL);
			if (ret < 0) {
				break;
			}
		}

		ret = siw_hal_tci_knock(dev);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x01 | (0x01 << 16);
		info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
		info1->tap_distance = 7;

		if (touch_senseless_margin(ts)) {
			ret = siw_hal_tci_control(dev, ACTIVE_AREA_CTRL);
			if (ret < 0) {
				break;
			}
		}

		ret = siw_hal_tci_password(dev);
		break;

	default:
		ts->tci.mode = 0;
		ret = siw_hal_tci_control(dev, ENABLE_CTRL);
		break;
	}

	t_dev_info(dev, "lpwg_control mode = %d\n", mode);

	return ret;
}

static const char *siw_hal_tci_debug_type_1_str[] = {
	"SUCCESS",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCH_SLOP",
	"MIN_TIMEOUT_INTER_TAP",
	"MAX_TIMEOUT_INTER_TAP",
	"LONG_PRESS_TIME_OUT",
	"MULTI_FINGER",
	"DELAY_TIME",
	"PALM_STATE",
	"OUT_OF_AREA",
};

#define TCI_FAIL_TYPE_1_NUM		ARRAY_SIZE(siw_hal_tci_debug_type_1_str)

static void siw_hal_debug_tci_type_1(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	char **str = (char **)siw_hal_tci_debug_type_1_str;
	u32 rdata = 0;
	u32 buffer = 0;
	u32 index = 0;
	int ret = 0;

	ret = siw_hal_read_value(dev,
			reg->tci_debug_fail_status,
			&rdata);
	if (ret < 0) {
		t_dev_err(dev, "failed to read tci debug fail status, %d\n", ret);
		return;
	}

	ret = siw_hal_read_value(dev,
			reg->tci_debug_fail_buffer,
			&buffer);
	if (ret < 0) {
		t_dev_err(dev, "failed to read tci debug fail buffer, %d\n", ret);
		return;
	}

	t_dev_info(dev,
		"Status[%04Xh] = %08Xh, Buffer[%04Xh] = %08Xh\n",
		reg->tci_debug_fail_status, rdata,
		reg->tci_debug_fail_buffer, buffer);

	/* Knock On fail */
	if (rdata & 0x01) {
		index = (buffer & 0xFFFF);
		t_dev_info(dev, "TCI-Debug: on fail reason: %s(%08Xh)\n",
			(index < TCI_FAIL_TYPE_1_NUM) ? str[index] : "(unknown)",
			buffer);
	}

	/* Knock Code fail */
	if (rdata & 0x02) {
		index = ((buffer>>16) & 0xFFFF);
		t_dev_info(dev, "TCI-Debug: code fail reason: %s(%08Xh)\n",
			(index < TCI_FAIL_TYPE_1_NUM) ? str[index] : "(unknown)",
			buffer);
	}
}

static void siw_hal_debug_tci(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (chip->opt.t_knock)
		return;

	switch (chip->opt.t_chk_tci_debug) {
	case 1:
		siw_hal_debug_tci_type_1(dev);
		break;
	default:
		break;
	}
}

#define TCI_INFO_CNT (sizeof(struct tci_info) / 2)

static int siw_hal_show_ic_tci_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	struct tci_info *info;
	u16 lpwg_data[TCI_INFO_CNT + 1][2];
	u16 lpwg_data_tci_1[TCI_INFO_CNT];
	u16 lpwg_data_tci_2[TCI_INFO_CNT];
	int i = 0;
	int ret = 0;

	if (chip->opt.t_knock)
		return 0;

	if (atomic_read(&chip->init) == IC_INIT_NEED) {
		t_dev_warn(dev, "Not Ready, Need IC init (show_ic_tci_info)\n");
		return 0;
	}

	t_dev_info(dev, "[ IC tci info ]\n");

	ret = siw_hal_reg_read_single(dev,
				reg->tci_enable_w,
				(void *)lpwg_data, sizeof(lpwg_data));
	if (ret < 0) {
		t_dev_err(dev, "tci info read fail\n");
		goto out;
	}

	for (i = 0; i < TCI_INFO_CNT; i++) {
		lpwg_data_tci_1[i] = lpwg_data[i + 1][0];
		lpwg_data_tci_2[i] = lpwg_data[i + 1][1];
	}

	info = (struct tci_info *)lpwg_data_tci_1;
	siw_prt_tci_info(dev, "TCI_1", info);

	info = (struct tci_info *)lpwg_data_tci_2;
	siw_prt_tci_info(dev, "TCI_2", info);

out:
	return ret;
}

static void siw_hal_show_driver_tci_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct tci_ctrl *tci = &ts->tci;
	struct tci_info *info;

	if (chip->opt.t_knock)
		return;

	t_dev_info(dev, "[ driver tci info ]\n");

	info = &tci->info[TCI_1];
	siw_prt_tci_info(dev, "TCI_1", info);

	info = &tci->info[TCI_2];
	siw_prt_tci_info(dev, "TCI_2", info);
}

static int siw_hal_lpwg_ext_tci_info(struct device *dev, void *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u16 *info_addr;
	int *data = (int *)param;
	int tci_type = 0;
	int index = 0;
	u16 value = 0;
	u16 write_en = 0;
	u16 value_2 = 0;
	u32 send_buf = 0;
	int ret = 0;

	if (chip->opt.t_knock)
		return 0;

	if (atomic_read(&chip->init) == IC_INIT_NEED) {
		t_dev_warn(dev, "Not Ready, Need IC init (lpwg_ext_tci_info)\n");
		return 0;
	}

	tci_type = data[0];
	index = data[1];
	value = data[2];
	write_en = data[3];

	if ((tci_type > TCI_2) || (0 > tci_type)) {
		t_dev_err(dev, "invaild tci_type value [%d]\n", tci_type);
		goto out_invaild;
	}

	if ((index > (TCI_INFO_CNT - 1)) || (0 > index)) {
		t_dev_err(dev, "invaild index value [%d]\n", index);
		goto out_invaild;
	}

	info_addr = (u16 *)&ts->tci.info[!tci_type];
	value_2 = info_addr[index];

	send_buf = (value << (tci_type ? 16 : 0)) | (value_2 << (tci_type ? 0 : 16));

	if (write_en == 1) {
		t_dev_info(dev, "tci info write addr[%Xh] value[%Xh]\n",
			reg->tap_count_w + index, send_buf);

		ret = siw_hal_write_value(dev,
			reg->tap_count_w + index, send_buf);
		if (ret < 0) {
			t_dev_err(dev, "tci info write fail, so don't setting driver tci info\n");
			goto out_write_fail;
		}
	}

	info_addr = (u16 *)&ts->tci.info[tci_type];
	info_addr[index] = value;

	siw_hal_show_ic_tci_info(dev);

out_write_fail:
	siw_hal_show_driver_tci_info(dev);

out_invaild:
	return ret;
}

static void siw_hal_show_driver_tci_area(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct active_area *area;
	struct reset_area *tci_qcover;

	if (chip->opt.t_knock)
		return;

	t_dev_info(dev, "[ driver tci area ]\n");

	area = &ts->tci.area;
	siw_prt_active_area_info(dev, "[active      ]", area);

	tci_qcover = &ts->tci.qcover_open;
	siw_prt_reset_area_info(dev, "[qcover open ]", tci_qcover, 0);

	tci_qcover = &ts->tci.qcover_close;
	siw_prt_reset_area_info(dev, "[qcover close]", tci_qcover, 0);
}

#define TCI_AREA_CNT (sizeof(struct active_area) / 2)

static int siw_hal_show_ic_tci_area(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	struct reset_area *area;
	u32 area_data_buf[TCI_AREA_CNT] = {0, };
	int ret = 0;

	if (chip->opt.t_knock)
		return 0;

	if (atomic_read(&chip->init) == IC_INIT_NEED) {
		t_dev_warn(dev, "Not Ready, Need IC init (show_ic_tci_area)\n");
		return 0;
	}

	t_dev_info(dev, "[ IC tci area ]\n");

	ret = siw_hal_reg_read_single(dev,
				reg->act_area_x1_w,
				(void *)area_data_buf, sizeof(area_data_buf));
	if (ret < 0) {
		t_dev_err(dev, "tci area read fail\n");

		goto out;
	}

	area = (struct reset_area *)area_data_buf;
	siw_prt_reset_area_info(dev, "[TCI_1]", area, 0);
	siw_prt_reset_area_info(dev, "[TCI_2]", area, 16);

out:
	return ret;
}

#else	/* __SIW_CONFIG_KNOCK */
static inline void siw_hal_get_tci_info(struct device *dev)
{

}

static inline int siw_hal_tci_area_set(struct device *dev, int cover_status)
{
	return 0;
}
static inline int siw_hal_lpwg_control(struct device *dev, int mode)
{
	return 0;
}

static inline void siw_hal_debug_tci(struct device *dev)
{

}

static inline int siw_hal_show_ic_tci_info(struct device *dev)
{
	return 0;
}

static inline int siw_hal_show_driver_tci_info(struct device *dev)
{
	return 0;
}

static inline int siw_hal_lpwg_ext_tci_info(struct device *dev, void *param)
{
	return 0;
}

static inline void siw_hal_show_driver_tci_area(struct device *dev)
{

}

static inline int siw_hal_show_ic_tci_area(struct device *dev)
{
	return 0;
}
#endif	/* __SIW_CONFIG_KNOCK */

#if defined(__SIW_CONFIG_SWIPE)
#define __SWIPE_INFO_SET(_idx, _d, _r, _rd, _rp, _mt, _mxt,	\
			_ax1, _ay1, _ax2, _ay2)	\
		[_idx] = {	\
			.distance = _d,	.ratio_thres = _r,	\
			.ratio_distance = _rd, .ratio_period = _rp,	\
			.min_time = _mt, .max_time = _mxt,	\
			.area.x1 = _ax1, .area.y1 = _ay1,	\
			.area.x2 = _ax2, .area.y2 = _ay2,	\
		}

const struct siw_hal_swipe_ctrl siw_hal_swipe_info_default = {
	.mode	= SWIPE_LEFT_BIT | SWIPE_RIGHT_BIT,
	.info = {
		__SWIPE_INFO_SET(SWIPE_R, 5, 100, 2, 5, 0, 150,	401, 0, 1439, 159),
		__SWIPE_INFO_SET(SWIPE_L, 5, 100, 2, 5, 0, 150, 401, 0, 1439, 159),
	},
};

static void siw_prt_swipe_info(struct device *dev,
			char *name, struct siw_hal_swipe_info *info)
{
	t_dev_info(dev,
		"swipe info[%s] distance %d, ratio_thres %d, ratio_distance %d\n",
		name, info->distance, info->ratio_thres, info->ratio_distance);

	t_dev_info(dev,
		"swipe info[%s] ratio_period %d, min_time %d, max_time %d\n",
		name, info->ratio_period, info->min_time, info->max_time);

	t_dev_info(dev,
		"swipe info[%s] area.x1 %d, area.y1 %d, area.x2 %d, area.y2 %d\n",
		name, info->area.x1, info->area.y1, info->area.x2, info->area.y2);
}

static void siw_hal_prt_swipe_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_swipe_ctrl *swipe = &chip->swipe;
	struct siw_hal_swipe_info *info;

	t_dev_info(dev, "swipe mode %08Xh\n", swipe->mode);
	info = &swipe->info[SWIPE_R];
	siw_prt_swipe_info(dev, "SWIPE_R", info);
	info = &swipe->info[SWIPE_L];
	siw_prt_swipe_info(dev, "SWIPE_L", info);
}

static void siw_hal_get_swipe_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	void *swipe_src;

	if (chip->opt.t_swipe)
		return;

	swipe_src = pdata_swipe_ctrl(ts->pdata);
	if (!swipe_src)
		swipe_src = (void *)&siw_hal_swipe_info_default;

	memcpy(&chip->swipe, swipe_src, sizeof(struct siw_hal_swipe_ctrl));

	siw_hal_prt_swipe_info(dev);
}

static int siw_hal_swipe_active_area(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_swipe_info *left = &chip->swipe.info[SWIPE_L];
	struct siw_hal_swipe_info *right = &chip->swipe.info[SWIPE_R];
	u32 active_area[4] = {
		(right->area.x1) | (left->area.x1 << 16),
		(right->area.y1) | (left->area.y1 << 16),
		(right->area.x2) | (left->area.x2 << 16),
		(right->area.y2) | (left->area.y2 << 16),
	};
	int ret = 0;

	ret = siw_hal_reg_write_single(dev,
				reg->swipe_act_area_x1_w,
				(void *)active_area, sizeof(active_area));
	if (ret < 0) {
		t_dev_err(dev, "swipe failed: active area, %d\n", ret);
		return ret;
	}

	t_dev_dbg_base(dev, "swipe done: active area\n");

	return 0;
}

static int siw_hal_swipe_control(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_swipe_info *left = &chip->swipe.info[SWIPE_L];
	struct siw_hal_swipe_info *right = &chip->swipe.info[SWIPE_R];
	u32 addr = ~0;
	u32 data = 0;
	char *name = NULL;
	int ret = 0;

	switch (type) {
	case SWIPE_ENABLE_CTRL:
	case SWIPE_DISABLE_CTRL:
		addr = reg->swipe_enable_w;
		data = (type == SWIPE_ENABLE_CTRL) ? chip->swipe.mode : 0;
		name = "mode";
		break;
	case SWIPE_DIST_CTRL:
		addr = reg->swipe_dist_w;
		data = (right->distance) | (left->distance << 16);
		name = "distance";
		break;
	case SWIPE_RATIO_THR_CTRL:
		addr = reg->swipe_ratio_thr_w;
		data = (right->ratio_thres) | (left->ratio_thres << 16);
		name = "ratio_threshold";
		break;
	case SWIPE_RATIO_PERIOD_CTRL:
		addr = reg->swipe_ratio_period_w;
		data = (right->ratio_period) | (left->ratio_period << 16);
		name = "ratio_period";
		break;
	case SWIPE_RATIO_DIST_CTRL:
		addr = reg->swipe_ratio_dist_w;
		data = (right->ratio_distance) | (left->ratio_distance << 16);
		name = "ratio_distance";
		break;
	case SWIPE_TIME_MIN_CTRL:
		addr = reg->swipe_time_min_w;
		data = (right->min_time) | (left->min_time << 16);
		name = "min_time";
		break;
	case SWIPE_TIME_MAX_CTRL:
		addr = reg->swipe_time_max_w;
		data = (right->max_time) | (left->max_time << 16);
		name = "max_time";
		break;
	case SWIPE_AREA_CTRL:
		ret = siw_hal_swipe_active_area(dev);
		break;
	default:
		break;
	}

	if (siw_addr_is_skip(addr)) {
		goto out;
	}

	ret = siw_hal_write_value(dev, addr, data);
	if (ret < 0) {
		t_dev_err(dev, "swipe failed: %s(%04Xh, %08Xh), %d\n",
			name, addr, data, ret);
		goto out;
	}

	t_dev_dbg_base(dev, "swipe done: %s(%04Xh, %08Xh)\n",
			name, addr, data);

out:
	return ret;
}

static int siw_hal_swipe_mode(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int swipe_ctrls[12] = { 0, };
	int i = 0;
	int ret = 0;

	if (chip->opt.t_swipe)
		return 0;

	if (!chip->swipe.mode || (mode != LCD_MODE_U2)) {
		ret = siw_hal_swipe_control(dev, SWIPE_DISABLE_CTRL);
		if (ret < 0) {
			t_dev_err(dev, "swipe failed to disable\n");
		} else {
			t_dev_info(dev, "swipe disabled\n");
		}
		return ret;
	}

	swipe_ctrls[i++] = SWIPE_ENABLE_CTRL;
	swipe_ctrls[i++] = SWIPE_DIST_CTRL;
	swipe_ctrls[i++] = SWIPE_RATIO_THR_CTRL;
	swipe_ctrls[i++] = SWIPE_RATIO_DIST_CTRL;
	swipe_ctrls[i++] = SWIPE_RATIO_PERIOD_CTRL;
	swipe_ctrls[i++] = SWIPE_TIME_MIN_CTRL;
	swipe_ctrls[i++] = SWIPE_TIME_MAX_CTRL;
	swipe_ctrls[i++] = SWIPE_AREA_CTRL;
	swipe_ctrls[i++] = -1;

	i = 0;
	while (swipe_ctrls[i] != -1) {
		ret = siw_hal_swipe_control(dev, swipe_ctrls[i]);
		if (ret < 0) {
			goto out;
		}

		i++;
	}

	t_dev_info(dev, "swipe enabled\n");

out:
	return ret;
}

#define SWIPE_FAIL_NUM 7
static const char const *siw_hal_swipe_debug_str[SWIPE_FAIL_NUM] = {
	"ERROR",
	"1FINGER_FAST_RELEASE",
	"MULTI_FINGER",
	"FAST_SWIPE",
	"SLOW_SWIPE",
	"OUT_OF_AREA",
	"RATIO_FAIL",
};

static void siw_hal_debug_swipe(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u8 debug_reason_buf[SWIPE_MAX_NUM][SWIPE_DEBUG_MAX_NUM];
	u32 rdata[5] = {0 , };
	u8 count[2] = {0, };
	u8 count_max = 0;
	u32 i, j = 0;
	u8 buf = 0;
	int ret = 0;

	if (chip->opt.t_swipe)
		return;

	if (!chip->swipe_debug_type)
		return;

	ret = siw_hal_reg_read_single(dev,
				reg->swipe_debug_r,
				(void *)rdata, sizeof(rdata));

	count[SWIPE_R] = (rdata[0] & 0xFFFF);
	count[SWIPE_L] = ((rdata[0] >> 16) & 0xFFFF);
	count_max = (count[SWIPE_R] > count[SWIPE_L]) ?
			count[SWIPE_R] : count[SWIPE_L];

	if (count_max == 0)
		return;

	if (count_max > SWIPE_DEBUG_MAX_NUM) {
		count_max = SWIPE_DEBUG_MAX_NUM;
		if (count[SWIPE_R] > SWIPE_DEBUG_MAX_NUM)
			count[SWIPE_R] = SWIPE_DEBUG_MAX_NUM;
		if (count[SWIPE_L] > SWIPE_DEBUG_MAX_NUM)
			count[SWIPE_L] = SWIPE_DEBUG_MAX_NUM;
	}

	for (i = 0; i < ((count_max-1)>>2)+1; i++) {
		memcpy(&debug_reason_buf[SWIPE_R][i<<2], &rdata[i+1], sizeof(u32));
		memcpy(&debug_reason_buf[SWIPE_L][i<<2], &rdata[i+3], sizeof(u32));
	}

	for (i = 0; i < SWIPE_MAX_NUM; i++) {
		for (j = 0; j < count[i]; j++) {
			buf = debug_reason_buf[i][j];
			t_dev_info(dev, "SWIPE_%s - DBG[%d/%d]: %s\n",
					i == SWIPE_R ? "Right" : "Left",
					j + 1, count[i],
					(buf > 0 && buf < SWIPE_FAIL_NUM) ?
					siw_hal_swipe_debug_str[buf] :
					siw_hal_swipe_debug_str[0]);
		}
	}
}
#else	/* __SIW_CONFIG_SWIPE */
static inline void siw_hal_get_swipe_info(struct device *dev)
{

}

static inline int siw_hal_swipe_mode(struct device *dev, int mode)
{
	return 0;
}

static inline void siw_hal_debug_swipe(struct device *dev)
{

}
#endif	/* __SIW_CONFIG_SWIPE */

static const char *siw_hal_pwr_name[] = {
	[POWER_OFF]		= "Power off",
	[POWER_SLEEP]	= "Power sleep",
	[POWER_WAKE]	= "Power wake",
	[POWER_ON]		= "Power on",
};

static int siw_hal_power_core(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	switch (ctrl) {
	case POWER_OFF:
		t_dev_dbg_pm(dev, "power core: power off\n");
		atomic_set(&chip->init, IC_INIT_NEED);

		siw_hal_power_vio(dev, 0);
		siw_hal_power_vdd(dev, 0);

		touch_msleep(chip->drv_reset_low + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_0));
		break;

	case POWER_ON:
		t_dev_dbg_pm(dev, "power core: power on\n");
		siw_hal_power_vdd(dev, 1);
		siw_hal_power_vio(dev, 1);
		break;

	case POWER_SLEEP:
		break;

	case POWER_WAKE:
		break;

	case POWER_HW_RESET:
		break;
	}

	return 0;
}

static int siw_hal_power(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if ((ctrl < 0) || (ctrl > POWER_ON)) {
		t_dev_err(dev, "power ctrl: wrong ctrl value, %d\n", ctrl);
		return -EINVAL;
	}

	t_dev_dbg_pm(dev, "power ctrl: %s - %s\n",
			touch_chip_name(ts), siw_hal_pwr_name[ctrl]);

	switch (ctrl) {
	case POWER_OFF:
		t_dev_dbg_pm(dev, "power ctrl: power off\n");
		atomic_set(&chip->init, IC_INIT_NEED);

		siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);

		siw_hal_power_core(dev, ctrl);
		break;

	case POWER_ON:
		t_dev_dbg_pm(dev, "power ctrl: power on\n");

		siw_hal_power_core(dev, ctrl);

		siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);
		break;

	case POWER_SLEEP:
		t_dev_dbg_pm(dev, "power ctrl: sleep\n");
		break;

	case POWER_WAKE:
		t_dev_dbg_pm(dev, "power ctrl: wake\n");
		break;

	case POWER_HW_RESET:
		t_dev_info(dev, "power ctrl: reset\n");
		siw_hal_reset_ctrl(dev, HW_RESET_ASYNC);
		break;
	}

	return 0;
}

enum {
	BOOT_CHK_SKIP = (1<<16),
};

static int siw_hal_chk_boot_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	u32 boot_failed = 0;
	u32 bootmode = 0;
	u32 boot_chk_offset_busy = siw_hal_boot_sts_pos_busy(chip);
	u32 boot_chk_offset_err = siw_hal_boot_sts_pos_dump_err(chip);
	u32 boot_chk_empty_mask = siw_hal_boot_sts_mask_empty(chip);
	int ret = 0;

	ret = siw_hal_get_boot_status(dev, &bootmode);
	if (ret < 0) {
		return ret;
	}

	/* maybe nReset is low state */
	if (!bootmode || (bootmode == ~0)) {
		return BOOT_CHK_SKIP;
	}

	/* booting... need to wait */
	if ((bootmode >> boot_chk_offset_busy) & 0x1) {
		return BOOT_CHK_SKIP;
	}

	boot_failed |= !!((bootmode >> boot_chk_offset_err) & 0x1);	/* CRC error */
	boot_failed |= (!!(bootmode & boot_chk_empty_mask))<<1;

	if (boot_failed) {
		t_dev_err(dev, "boot fail: boot sts  = %08Xh(%02Xh)\n",
			bootmode, boot_failed);
	}

	return boot_failed;
}

static int siw_hal_chk_boot_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_status_mask_bit *mask_bit = &chip->status_mask_bit;
	u32 boot_failed = 0;
	u32 tc_status = 0;
	u32 valid_cfg_crc_mask = 0;
	u32 valid_code_crc_mask = 0;
	int ret = 0;

	valid_cfg_crc_mask = mask_bit->valid_cfg_crc;
	valid_code_crc_mask = mask_bit->valid_code_crc;

	ret = siw_hal_read_value(dev,
			reg->tc_status,
			&tc_status);
	if (ret < 0) {
		return ret;
	}

	/* maybe nReset is low state */
	if (!tc_status || (tc_status == ~0)) {
		return BOOT_CHK_SKIP;
	}

	if (valid_cfg_crc_mask && !(tc_status & valid_cfg_crc_mask)) {
		boot_failed |= (1<<5);
	}
	if (valid_code_crc_mask && !(tc_status & valid_code_crc_mask)) {
		boot_failed |= (1<<4);
	}
	if (boot_failed) {
		t_dev_err(dev, "boot fail: tc_status = %08Xh(%02Xh)\n",
			tc_status, boot_failed);
	}

	return boot_failed;
}

enum {
	BOOT_CHK_MODE_RETRY = 2,
	BOOT_CHK_STS_RETRY	= 2,
	/* */
	BOOT_CHK_MODE_DELAY	= 10,
	BOOT_CHK_STS_DELAY	= 10,
};

static int siw_hal_chk_boot(struct device *dev)
{
	u32 boot_failed = 0;
	int retry;
	int ret = 0;

	retry = BOOT_CHK_MODE_RETRY;
	while (retry--) {
		ret = siw_hal_chk_boot_mode(dev);
		if (ret < 0) {
			return ret;
		}
		if (ret == BOOT_CHK_SKIP) {
			return 0;
		}
		if (!ret) {
			break;
		}
		touch_msleep(BOOT_CHK_MODE_DELAY);
	}
	boot_failed |= ret;

	retry = BOOT_CHK_STS_RETRY;
	while (retry--) {
		ret = siw_hal_chk_boot_status(dev);
		if (ret < 0) {
			return ret;
		}
		if (ret == BOOT_CHK_SKIP) {
			return boot_failed;
		}
		if (!ret) {
			break;
		}
		touch_msleep(BOOT_CHK_STS_DELAY);
	}
	boot_failed |= ret;

	return boot_failed;
}

//#define __SIW_SUPPORT_STATUS_OPT_IGNORE_ABNORMAL
#define __SIW_SUPPORT_STATUS_OPT_IGNORE_DISP_ERR

#if defined(__SIW_PANEL_CLASS_MOBILE_OLED)

#else	/* !__SIW_PANEL_CLASS_MOBILE_OLED */
#define __SIW_SUPPORT_STATUS_ERROR_CFG
#endif	/* __SIW_PANEL_CLASS_MOBILE_OLED */

#if defined(CONFIG_TOUCHSCREEN_SIW_SW49408)
#define __SIW_SUPPORT_STATUS_ERROR_MEM
#endif

#if defined(CONFIG_TOUCHSCREEN_SIW_SW49106) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49408) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49409) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49501)
#define __SIW_SUPPORT_STATUS_ERROR_DISP
#endif

enum {
	IC_DEBUG_SIZE		= 16,	// byte
	//
	IC_CHK_LOG_MAX		= (1<<9),
	//
	INT_IC_ABNORMAL_STATUS	= (1<<0),
	//
	INT_IC_ERROR_STATUS = ((1<<5) | (1<<3)),
};

static const struct siw_hal_status_filter status_filter_type_0[] = {
	_STS_FILTER(STS_ID_VALID_DEV_CTL, 1, STS_POS_VALID_DEV_CTL,
		0, "device ctl not set"),
	_STS_FILTER(STS_ID_VALID_CODE_CRC, 1, STS_POS_VALID_CODE_CRC_TYPE_0,
		0, "code crc invalid"),
	_STS_FILTER(STS_ID_ERROR_ABNORMAL, 1, STS_POS_ERROR_ABNORMAL,
		STS_FILTER_FLAG_TYPE_ERROR | STS_FILTER_FLAG_CHK_FAULT,
		"abnormal status detected"),
	_STS_FILTER(STS_ID_ERROR_SYSTEM, 1, STS_POS_ERROR_SYSTEM,
		STS_FILTER_FLAG_TYPE_ERROR | STS_FILTER_FLAG_ESD_SEND,
		"system error detected"),
	_STS_FILTER(STS_ID_ERROR_MISMTACH, 2, STS_POS_ERROR_MISMTACH,
		STS_FILTER_FLAG_TYPE_ERROR,
		"display mode mismatch"),
	_STS_FILTER(STS_ID_VALID_IRQ_PIN, 1, STS_POS_VALID_IRQ_PIN,
		0, "irq pin invalid"),
	_STS_FILTER(STS_ID_VALID_IRQ_EN, 1, STS_POS_VALID_IRQ_EN,
		0, "irq status invalid"),
	/* end mask */
	_STS_FILTER(STS_ID_NONE, 0, 0, 0, NULL),
};

static const struct siw_hal_status_filter status_filter_type_1[] = {
	_STS_FILTER(STS_ID_VALID_DEV_CTL, 1, STS_POS_VALID_DEV_CTL,
		0, "device ctl not set"),
	_STS_FILTER(STS_ID_VALID_CODE_CRC, 1, STS_POS_VALID_CODE_CRC,
		0, "code crc invalid"),
#if defined(__SIW_SUPPORT_STATUS_ERROR_CFG)
	_STS_FILTER(STS_ID_VALID_CFG_CRC, 1, STS_POS_VALID_CFG_CRC,
		0, "cfg crc invalid"),
#endif
	_STS_FILTER(STS_ID_ERROR_ABNORMAL, 1, STS_POS_ERROR_ABNORMAL,
		STS_FILTER_FLAG_TYPE_ERROR | STS_FILTER_FLAG_CHK_FAULT,
		"abnormal status detected"),
	_STS_FILTER(STS_ID_ERROR_SYSTEM, 1, STS_POS_ERROR_SYSTEM,
		STS_FILTER_FLAG_TYPE_ERROR | STS_FILTER_FLAG_ESD_SEND,
		"system error detected"),
	_STS_FILTER(STS_ID_ERROR_MISMTACH, 1, STS_POS_ERROR_MISMTACH,
		STS_FILTER_FLAG_TYPE_ERROR,
		"display mode mismatch"),
	_STS_FILTER(STS_ID_VALID_IRQ_PIN, 1, STS_POS_VALID_IRQ_PIN,
		0, "irq pin invalid"),
	_STS_FILTER(STS_ID_VALID_IRQ_EN, 1, STS_POS_VALID_IRQ_EN,
		0, "irq status invalid"),
	_STS_FILTER(STS_ID_VALID_TC_DRV, 1, STS_POS_VALID_TC_DRV,
		0, "driving invalid"),
#if defined(__SIW_SUPPORT_STATUS_ERROR_MEM)
	_STS_FILTER(STS_ID_ERROR_MEM, 1, STS_POS_ERROR_MEM,
		STS_FILTER_FLAG_TYPE_ERROR,
		"memory error detected"),
#endif
#if defined(__SIW_SUPPORT_STATUS_ERROR_DISP)
	_STS_FILTER(STS_ID_ERROR_DISP, 1, STS_POS_ERROR_DISP,
		STS_FILTER_FLAG_TYPE_ERROR | STS_FILTER_FLAG_ESD_SEND,
		"display error detected"),
#endif
	/* end mask */
	_STS_FILTER(STS_ID_NONE, 0, 0, 0, NULL),
};

static const struct siw_hal_status_filter status_filter_type_2[] = {
	_STS_FILTER(STS_ID_VALID_DEV_CTL, 1, STS_POS_VALID_DEV_CTL,
		0, "device ctl not set"),
	_STS_FILTER(STS_ID_ERROR_ABNORMAL, 1, STS_POS_ERROR_ABNORMAL,
		STS_FILTER_FLAG_TYPE_ERROR | STS_FILTER_FLAG_CHK_FAULT,
		"re-init required"),
	_STS_FILTER(STS_ID_VALID_IRQ_PIN, 1, STS_POS_VALID_IRQ_PIN,
		0, "irq pin invalid"),
	_STS_FILTER(STS_ID_VALID_IRQ_EN, 1, STS_POS_VALID_IRQ_EN,
		0, "irq status invalid"),
	_STS_FILTER(STS_ID_VALID_TC_DRV, 1, STS_POS_VALID_TC_DRV,
		0, "driving invalid"),
	/* end mask */
	_STS_FILTER(STS_ID_NONE, 0, 0, 0, NULL),
};

static u32 siw_hal_get_status_mask(struct device *dev, int id)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_status_filter *filter = chip->status_filter;
	u32 mask = 0;

	if (filter == NULL)
		goto out;

	while (1) {
		if (!filter->id || !filter->width) {
			break;
		}

		if (filter->id == id) {
			mask = ((1<<filter->width)-1)<<filter->pos;
			break;
		}
		filter++;
	}

out:
	return mask;
}

static int siw_hal_chk_report_type(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	int invalid_pid = fw->invalid_pid;

	if (chip->report_type) {
		return 0;
	}

	if (invalid_pid) {
		return -EINVAL;
	}

	switch (touch_chip_type(ts)) {
	case CHIP_SW42000:
	case CHIP_SW42000A:
	case CHIP_SW82905:
		chip->report_type = CHIP_REPORT_TYPE_1;
		break;
	default:
		chip->report_type = CHIP_REPORT_TYPE_0;
		break;
	}

	t_dev_info(dev, "report type  : %d\n", chip->report_type);

	return 0;
}

static void siw_hal_chk_status_type_quirks(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;

	switch (touch_chip_type(ts)) {
	case CHIP_SW42103:
		if (!strncmp(fw->product_id, "LA145WF1", 8)) {
			chip->status_filter++;
			t_dev_info(dev, "%s[%s] status quirk adopted\n",
				touch_chip_name(ts), fw->product_id);
		}
		break;
	}
}

static int siw_hal_chk_status_type(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_status_mask_bit *mask_bit = &chip->status_mask_bit;
	int t_sts_mask = chip->opt.t_sts_mask;
	int invalid_pid = fw->invalid_pid;

	siw_hal_chk_report_type(dev);

	if (chip->status_type) {
		return 0;
	}

	if (invalid_pid) {
		return -EINVAL;
	}

	switch (touch_chip_type(ts)) {
	case CHIP_SW42101:
		chip->status_type = CHIP_STATUS_TYPE_2;
		break;
	case CHIP_LG4894:
		if (!strncmp(fw->product_id, "L0W53K6P", 8)) {
			chip->status_type = CHIP_STATUS_TYPE_0;
			break;
		}
		chip->status_type = CHIP_STATUS_TYPE_1;
		break;
	default:
		chip->status_type = CHIP_STATUS_TYPE_1;
		break;
	}

	switch (chip->status_type) {
	case CHIP_STATUS_TYPE_2:
		chip->status_filter = (struct siw_hal_status_filter *)status_filter_type_2;
		break;
	case CHIP_STATUS_TYPE_0:
		chip->status_filter = (struct siw_hal_status_filter *)status_filter_type_0;
		break;
	default:
		chip->status_filter = (struct siw_hal_status_filter *)status_filter_type_1;
		break;
	}

	siw_hal_chk_status_type_quirks(dev);

	mask_bit->valid_dev_ctl = siw_hal_get_status_mask(dev, STS_ID_VALID_DEV_CTL);
	mask_bit->valid_code_crc = siw_hal_get_status_mask(dev, STS_ID_VALID_CODE_CRC);
	mask_bit->valid_cfg_crc = siw_hal_get_status_mask(dev, STS_ID_VALID_CFG_CRC);;
	mask_bit->error_abnormal = siw_hal_get_status_mask(dev, STS_ID_ERROR_ABNORMAL);
	mask_bit->error_system = siw_hal_get_status_mask(dev, STS_ID_ERROR_SYSTEM);
	mask_bit->error_mismtach = siw_hal_get_status_mask(dev, STS_ID_ERROR_MISMTACH);
	mask_bit->valid_irq_pin = siw_hal_get_status_mask(dev, STS_ID_VALID_IRQ_PIN);
	mask_bit->valid_irq_en = siw_hal_get_status_mask(dev, STS_ID_VALID_IRQ_EN);
	mask_bit->error_mem = siw_hal_get_status_mask(dev, STS_ID_ERROR_MEM);
	mask_bit->valid_tv_drv = siw_hal_get_status_mask(dev, STS_ID_VALID_TC_DRV);
	mask_bit->error_disp = siw_hal_get_status_mask(dev, STS_ID_ERROR_DISP);

#if defined(__SIW_SUPPORT_STATUS_OPT_IGNORE_ABNORMAL)
	mask_bit->error_abnormal = 0;
	mask_bit->error_system = 0;
#endif

#if defined(__SIW_SUPPORT_STATUS_OPT_IGNORE_DISP_ERR)
	mask_bit->error_disp = 0;
#endif

	t_dev_dbg_base(dev, "mask[v_dev]  : %08Xh\n", mask_bit->valid_dev_ctl);
	t_dev_dbg_base(dev, "mask[v_code] : %08Xh\n", mask_bit->valid_code_crc);
	t_dev_dbg_base(dev, "mask[v_cfg]  : %08Xh\n", mask_bit->valid_cfg_crc);
	t_dev_dbg_base(dev, "mask[e_abn]  : %08Xh\n", mask_bit->error_abnormal);
	t_dev_dbg_base(dev, "mask[e_sys]  : %08Xh\n", mask_bit->error_system);
	t_dev_dbg_base(dev, "mask[e_mis]  : %08Xh\n", mask_bit->error_mismtach);
	t_dev_dbg_base(dev, "mask[v_i_p]  : %08Xh\n", mask_bit->valid_irq_pin);
	t_dev_dbg_base(dev, "mask[v_i_e]  : %08Xh\n", mask_bit->valid_irq_en);
	t_dev_dbg_base(dev, "mask[e_mem]  : %08Xh\n", mask_bit->error_mem);
	t_dev_dbg_base(dev, "mask[v_tc]   : %08Xh\n", mask_bit->valid_tv_drv);
	t_dev_dbg_base(dev, "mask[e_disp] : %08Xh\n", mask_bit->error_disp);

	chip->status_mask_normal = mask_bit->valid_dev_ctl |
						mask_bit->valid_code_crc |
						mask_bit->valid_cfg_crc |
						mask_bit->valid_irq_pin |
						mask_bit->valid_irq_en |
						mask_bit->valid_tv_drv |
						0;

	chip->status_mask_logging = mask_bit->error_mismtach |
						mask_bit->valid_irq_pin |
						mask_bit->valid_irq_en |
						mask_bit->valid_tv_drv |
						0;

	chip->status_mask_reset = mask_bit->valid_dev_ctl |
						mask_bit->valid_code_crc |
						mask_bit->valid_cfg_crc |
						mask_bit->error_abnormal |
						mask_bit->error_system |
						mask_bit->error_mem |
						mask_bit->error_disp |
						0;

	chip->status_mask = chip->status_mask_normal |
						chip->status_mask_logging |
						chip->status_mask_reset |
						0;

	chip->status_mask_ic_abnormal = INT_IC_ABNORMAL_STATUS;
	chip->status_mask_ic_error = INT_IC_ERROR_STATUS;
	chip->status_mask_ic_disp_err = 0;

	switch (t_sts_mask) {
	case 1:
		chip->status_mask_ic_valid = 0xFFFF;
		chip->status_mask_ic_disp_err = (0x3<<6);
		break;
	case 4:
	case 2:
		chip->status_mask_ic_abnormal |= (0x3<<1);
		chip->status_mask_ic_error = ((1<<7) | (1<<5));
		chip->status_mask_ic_valid = (t_sts_mask == 4) ? 0x1FFFFF : 0x7FFFF;
		chip->status_mask_ic_disp_err = (0x3<<8);
		break;
	case 5:
	case 3:
		chip->status_mask_ic_abnormal = 0;
		chip->status_mask_ic_error = ((1<<3) | (1<<1));
		chip->status_mask_ic_valid = (t_sts_mask == 5) ? 0x3FF : 0x7FFFF;
		break;
	default:
		chip->status_mask_ic_valid = 0xFF;
		break;
	}

	chip->status_mask_ic_normal = chip->status_mask_ic_valid;
	chip->status_mask_ic_normal &= ~chip->status_mask_ic_abnormal;
	chip->status_mask_ic_normal &= ~chip->status_mask_ic_error;
	chip->status_mask_ic_normal &= ~chip->status_mask_ic_disp_err;

#if defined(__SIW_SUPPORT_STATUS_OPT_IGNORE_ABNORMAL)
	chip->status_mask_ic_abnormal = 0;
#endif

#if defined(__SIW_SUPPORT_STATUS_OPT_IGNORE_DISP_ERR)
	chip->status_mask_ic_disp_err = 0;
#endif

	t_dev_info(dev, "status type  : %d\n", chip->status_type);
	t_dev_info(dev, "status mask  : %08Xh\n", chip->status_mask);
	t_dev_info(dev, " normal      : %08Xh\n", chip->status_mask_normal);
	t_dev_info(dev, " logging     : %08Xh\n", chip->status_mask_logging);
	t_dev_info(dev, " reset       : %08Xh\n", chip->status_mask_reset);
	t_dev_info(dev, " ic normal   : %08Xh\n", chip->status_mask_ic_normal);
	t_dev_info(dev, " ic abnormal : %08Xh\n", chip->status_mask_ic_abnormal);
	t_dev_info(dev, " ic error    : %08Xh\n", chip->status_mask_ic_error);
	t_dev_info(dev, " ic valid    : %08Xh\n", chip->status_mask_ic_valid);
	t_dev_info(dev, " ic disp err : %08Xh\n", chip->status_mask_ic_disp_err);

	return 0;
}

struct siw_ic_info_chip_proto {
	int chip_type;
	int vchip;
	int vproto;
};

static const struct siw_ic_info_chip_proto siw_ic_info_chip_protos[] = {
	{ CHIP_LG4894, 4, 4 },
	{ CHIP_LG4895, 8, 4 },
	{ CHIP_LG4946, 7, 4 },
	{ CHIP_LG4951, 7, 4 },
	{ CHIP_SW1828, 9, 4 },
	{ CHIP_SW46104, 15, 4 },
	{ CHIP_SW49105, 10, 4 },
	{ CHIP_SW49106, 11, 4 },
	{ CHIP_SW49406, 7, 4 },
	{ CHIP_SW49407, 9, 4 },
	{ CHIP_SW49408, 9, 4 },
	{ CHIP_SW49501, 14, 4 },
	{ CHIP_SW42000, 10, 4 },
	{ CHIP_SW42000A, 10, 4 },
	{ CHIP_SW82905, 10, 4 },
	{ CHIP_SW42101, 15, 4 },
	{ CHIP_SW42103, 17, 4 },
	{ CHIP_SW17700, 18, 4 },
	{ CHIP_NONE, 0, 0 },	//End mark
};

static int siw_hal_ic_info_ver_check(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_ic_info_chip_proto *chip_proto;
//	u32 version = fw->v.version_raw;
	u32 vchip = fw->v.version.chip;
	u32 vproto = fw->v.version.protocol;
//	int ret = 0;

	chip_proto = (struct siw_ic_info_chip_proto *)siw_ic_info_chip_protos;
	while (1) {
		if (chip_proto->chip_type == CHIP_NONE) {
			break;
		}

		if (touch_chip_type(ts) == chip_proto->chip_type) {
			if ((chip_proto->vchip != vchip) ||
				(chip_proto->vproto != vproto)) {
				break;
			}

			t_dev_info(dev, "[%s] IC info is good: %d, %d\n",
					touch_chip_name(ts), vchip, vproto);

			return 0;
		}

		chip_proto++;
	}

	t_dev_err(dev, "[%s] IC info is abnormal: %d, %d\n",
			touch_chip_name(ts), vchip, vproto);

	return -EINVAL;
}

static int siw_hal_ic_info_boot(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int boot_fail_cnt = chip->boot_fail_cnt;
	int ret = 0;

	ret = siw_hal_chk_boot(dev);
	if (ret < 0) {
		return ret;
	}

	if (ret) {
		atomic_set(&chip->boot, IC_BOOT_FAIL);

		/* Limit to avoid infinite repetition */
		if (boot_fail_cnt >= BOOT_FAIL_RECOVERY_MAX) {
			t_dev_err(dev, "Boot fail can't be recovered(%d) - %02Xh\n",
				boot_fail_cnt, ret);
			return -EFAULT;
		}

		t_dev_err(dev, "Boot fail detected(%d) - %02Xh\n",
			boot_fail_cnt, ret);

		chip->boot_fail_cnt++;

		/* return special flag to let the core layer know */
		return -ETDBOOTFAIL;
	}
	chip->boot_fail_cnt = 0;

	return 0;
}

static int siw_hal_hw_reset_quirk(struct device *dev, int pwr_con, int delay);

static int siw_hal_ic_info_quirk(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;

	switch (touch_chip_type(ts)) {
	case CHIP_SW17700:
		if (atomic_read(&ts->state.core) != CORE_PROBE) {
			break;
		}

		if (touch_flags(ts) & TOUCH_SKIP_RESET_PIN) {
			break;
		}

		if (fw->revision) {
			break;
		}

		if (chip->ops_quirk.hw_reset != NULL) {
			break;
		}

		if (touch_fquirks(ts)->gpio_set_reset != NULL) {
			break;
		}

		chip->ops_quirk.hw_reset = siw_hal_hw_reset_quirk;

		t_dev_info(dev, "[%s] reset quirk activated\n",
			touch_chip_name(ts));

		break;
	}

	return 0;
}

static int __siw_hal_get_sys_id(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_reg *reg = chip->reg;
	u32 sys_id_addr = fw->sys_id_addr;
	u32 data = 0;
	int ret = 0;

	fw->sys_id_raw = 0;
	memset(fw->sys_id, 0, sizeof(fw->sys_id));

	if (!sys_id_addr) {
		return 0;
	}

	ret = siw_hal_write_value(dev, reg->spr_code_offset, sys_id_addr);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_read_value(dev, reg->code_access_addr, (void *)&data);

	/* ignore return value */
	siw_hal_write_value(dev, reg->spr_code_offset, 0);

	if (ret < 0) {
		goto out;
	}

	if (!data) {
		t_dev_warn(dev, "%s has no chip info\n", touch_chip_name(ts));
		goto out;
	}

	fw->sys_id_raw = data;
	snprintf(fw->sys_id, sizeof(fw->sys_id) - 1, "%X", data);

out:
	return ret;
}

static int siw_hal_do_ic_info_more(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	int ret = 0;

	ret = __siw_hal_get_sys_id(dev);
	if (ret < 0) {
		return ret;
	}

	if (chip->opt.f_info_more) {
		struct siw_hal_rw_multi multi[] = {
			{ 0, reg->info_fpc_type, &fw->fpc, sizeof(fw->fpc), "fpc" },
			{ 0, reg->info_wfr_type, &fw->wfr, sizeof(fw->wfr), "wft" },
			{ 0, reg->info_cg_type, &fw->cg, sizeof(fw->cg), "cg" },
			{ 0, reg->info_lot_num, &fw->lot, sizeof(fw->lot), "lot" },
			{ 0, reg->info_serial_num, &fw->sn, sizeof(fw->sn), "sn" },
			{ 0, reg->info_date, &fw->date, sizeof(fw->date), "date" },
			{ 0, reg->info_time, &fw->time, sizeof(fw->time), "time" },
			{ -1, -1, NULL, }
		};

		ret = siw_hal_reg_rw_multi(dev, multi, "ic_info(2)");

		fw->wfr &= WAFER_TYPE_MASK;
	}

	return ret;
}

static int siw_hal_do_ic_info(struct device *dev, int prt_on)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	char sys_id_str[16] = { 0, };
	u32 product[2] = {0};
	u32 chip_id = 0;
	u32 version = 0;
	u32 version_ext = 0;
	u32 revision = 0;
	u32 bootmode = 0;
	u32 boot_chk_offset = 0;
	int invalid_pid = 0;
	struct siw_hal_rw_multi multi[] = {
		{ 0, reg->spr_chip_id, &chip_id, sizeof(chip_id), "chip_id" },
		{ 0, reg->tc_version, &version, sizeof(version), "version" },
		{ 0, reg->info_chip_version, &revision, sizeof(revision), "revision" },
		{ 0, reg->tc_product_id1, product, sizeof(product), "product_id" },
		{ -1, -1, NULL, },
	};
	int ret = 0;

	ret = siw_hal_reg_rw_multi(dev, multi, "ic_info(1)");
	if (ret < 0) {
		return ret;
	}

	if (chip->opt.f_ver_ext) {
		ret = siw_hal_read_value(dev, reg->tc_version_ext, &version_ext);
		if (ret < 0) {
			t_dev_err(dev, "ic_info(1): version_ext failed, %d\n", ret);
			return ret;
		}
	}

	ret = siw_hal_get_boot_status(dev, &bootmode);
	if (ret < 0) {
		t_dev_err(dev, "ic_info(1): failed to get boot status, %d\n", ret);
		return ret;
	}

	ret = siw_hal_do_ic_info_more(dev);
	if (ret < 0) {
		return ret;
	}

	if (fw->sys_id_raw) {
		snprintf(sys_id_str, sizeof(sys_id_str) - 1, "(%s)", fw->sys_id);
	}

	siw_hal_fw_set_chip_id(fw, chip_id);
	siw_hal_fw_set_version(fw, version, version_ext);
	siw_hal_fw_set_revision(fw, revision);
	siw_hal_fw_set_prod_id(fw, (u8 *)product, sizeof(product));

	invalid_pid = fw->invalid_pid;
	if (invalid_pid) {
		t_dev_err(dev, "[info] invalid PID - \"%s\" (%03Xh)\n",
			fw->product_id, invalid_pid);
	}

	siw_hal_chk_status_type(dev);

	if (fw->version_ext) {
		int ferr;

		ferr = siw_hal_fw_chk_version_ext(fw->version_ext,
									fw->v.version.ext);
		t_dev_info_sel(dev, prt_on,
				"[T] chip id %s%s, version %08X(%u.%02u) (0x%02X) %s\n",
				chip->fw.chip_id,
				(sys_id_str[0]) ? sys_id_str : "",
				fw->version_ext,
				fw->v.version.major, fw->v.version.minor,
				fw->revision,
				(ferr < 0) ? "(invalid)" : "");
	} else {
		t_dev_info_sel(dev, prt_on,
				"[T] chip id %s%s, version v%u.%02u (0x%08X, 0x%02X)\n",
				fw->chip_id,
				(sys_id_str[0]) ? sys_id_str : "",
				fw->v.version.major, fw->v.version.minor,
				version, fw->revision);
	}

	boot_chk_offset = siw_hal_boot_sts_pos_busy(chip);
	t_dev_info_sel(dev, prt_on,
			"[T] product id %s, flash boot %s(%s), crc %s (0x%08X)\n",
			fw->product_id,
			((bootmode >> boot_chk_offset) & 0x1) ? "BUSY" : "idle",
			((bootmode >> (boot_chk_offset + 1)) & 0x1) ? "done" : "booting",
			((bootmode >> (boot_chk_offset + 2)) & 0x1) ? "ERROR" : "ok",
			bootmode);

	ret = siw_hal_ic_info_boot(dev);
	if (ret) {
		return ret;
	}

	if (chip->opt.f_info_more) {
		t_dev_info_sel(dev, prt_on,
			"[T] fpc %d, wfr %d, cg %d, lot %d\n",
			fw->fpc, fw->wfr, fw->cg, fw->lot);
		t_dev_info_sel(dev, prt_on,
			"[T] sn %Xh, "
			"date %04d.%02d.%02d, time %02d:%02d:%02d site%d\n",
			fw->sn,
			fw->date & 0xFFFF, ((fw->date>>16) & 0xFF), ((fw->date>>24) & 0xFF),
			fw->time & 0xFF, ((fw->time>>8) & 0xFF), ((fw->time>>16) & 0xFF),
			((fw->time>>24) & 0xFF));
	}

	if (strcmp(fw->chip_id, touch_chip_id(ts))) {
		t_dev_err(dev, "Invalid chip id(%s), shall be %s\n",
			fw->chip_id, touch_chip_id(ts));
		return -EINVAL;
	}

	ret = siw_hal_ic_info_ver_check(dev);
	if (ret < 0) {
		return ret;
	}

	siw_hal_ic_info_quirk(dev);

	if (invalid_pid) {
		return -EINVAL;
	}

	return 0;
}

static int siw_hal_ic_info(struct device *dev)
{
	return siw_hal_do_ic_info(dev, 1);
}

static int siw_hal_init_reg_verify(struct device *dev,
				u32 addr, u32 data, int retry,
				const char *name)
{
	u32 rdata = ~0;
	int i;
	int ret = 0;

	for (i = 0; i < retry; i++) {
		ret = siw_hal_write_value(dev, addr, data);
		if (ret < 0) {
			t_dev_err(dev, "failed to write %s, %d\n",
				name, ret);
			return ret;
		}

		ret = siw_hal_read_value(dev, addr, &rdata);
		if (ret < 0) {
			t_dev_err(dev, "failed to read %s, %d\n",
				name, ret);
			return ret;
		}

		if (data == rdata) {
			t_dev_dbg_base(dev, "init reg done: %s(%08Xh)\n", name, rdata);
			return 0;
		}
	}

	t_dev_err(dev, "init reg failed: %08Xh, %08Xh\n", data, rdata);

	return -EFAULT;
}

static int siw_hal_init_reg_set_pre(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int is_spi = !!(touch_bus_type(ts) == BUS_IF_SPI);
	u32 addr_spi = 0xFE4;
	u32 addr_i2c = 0xFE5;
	u32 addr_set = 0xFF3;
	u32 value_spi = is_spi;
	u32 value_i2c = !is_spi;
	u32 wdata = ABNORMAL_IC_DETECTION;
	int wr_only = 0;
	int ret = 0;

	if (!chip->opt.f_attn_opt) {
		goto out;
	}

	switch (touch_chip_type(ts)) {
	case CHIP_SW46104:
		addr_set = ADDR_SKIP_MASK;
		break;
	case CHIP_SW42000:
		wdata = (1<<is_spi);
		wr_only = 1;
		break;
	case CHIP_SW42000A:
	case CHIP_SW82905:
		addr_set = ADDR_SKIP_MASK;
		break;
	default:
		break;
	}

	/* for spi : 1, for i2c : 0 */
	ret = siw_hal_write_value(dev, addr_spi, value_spi);
	if (ret < 0) {
		goto out;
	}

	/* for spi : 0, for i2c : 1 */
	ret = siw_hal_write_value(dev, addr_i2c, value_i2c);
	if (ret < 0) {
		goto out;
	}

	if (siw_addr_is_skip(addr_set)) {
		goto out;
	}

	if (wr_only) {
		ret = siw_hal_write_value(dev, addr_set, wdata);
	} else {
		ret = siw_hal_init_reg_verify(dev,
				addr_set, wdata, 3,
				"spi_tattn_opt");
	}
	if (ret < 0) {
		t_dev_err(dev,
			"failed to set spi_tattn_opt, %d\n",
			ret);
		goto out;
	}

out:
	return ret;
}

static int siw_hal_init_reg_set_post(struct device *dev)
{
	return 0;
}

static int siw_hal_init_reg_set(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data = 1;
	int ret = 0;

	siw_hal_init_reg_set_pre(dev);

	ret = siw_hal_write_value(dev,
				reg->tc_device_ctl,
				1);
	if (ret < 0) {
		t_dev_err(dev, "failed to start chip, %d\n", ret);
		goto out;
	}

	ret = siw_hal_write_value(dev,
				reg->tc_interrupt_ctl,
				1);
	if (ret < 0) {
		t_dev_err(dev, "failed to start chip irq, %d\n", ret);
		goto out;
	}

#if 0
	ret = siw_hal_write_value(dev,
				reg->spr_charger_status,
				chip->charger);
	if (ret < 0) {
		goto out;
	}
#endif

	data = atomic_read(&ts->state.ime);

	ret = siw_hal_write_value(dev,
				reg->ime_state,
				data);
	if (ret < 0) {
		goto out;
	}

	siw_hal_init_reg_set_post(dev);

out:
	return ret;
}

#if defined(__SIW_SUPPORT_WATCH)
static int siw_hal_check_watch(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	siw_hal_watch_chk_font_status(chip->dev);

	if ((chip->lcd_mode == LCD_MODE_U2) &&
		siw_hal_watch_is_disp_waton(dev) &&
		siw_hal_watch_is_rtc_run(dev)) {
		siw_hal_watch_get_curr_time(dev, NULL, NULL);
	}

	return 0;
}

static int siw_hal_check_mode_type_0(struct device *dev,
				int lcd_mode, int chk_mode)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int ret = 0;

	if (lcd_mode == LCD_MODE_U3) {
		goto out;
	}

	if (lcd_mode == LCD_MODE_U2) {
		if (chk_mode == LCD_MODE_U2_UNBLANK) {
			t_dev_info(dev, "U1 -> U2 : watch on\n");
			siw_hal_watch_init(dev);
			// knockon mode change + swipe enable
			ret = siw_hal_tc_driving(dev, LCD_MODE_U2);
			if (!ret)
				ret = 1;
		} else {
			t_dev_info(dev, "U2 mode change\n");
		}
		goto out;
	}

	if (lcd_mode == LCD_MODE_U2_UNBLANK) {
		switch (chk_mode) {
		case LCD_MODE_STOP:
			t_dev_info(dev, "Skip mode change : LCD_MODE_STOP -> U1\n");
			siw_hal_watch_display_off(dev);
			ret = 1;
			break;
		case LCD_MODE_U2:
			t_dev_info(dev, "U2 -> U1 : watch off\n");
			siw_hal_watch_display_off(dev);
			// abs mode change + swipe disable
			ret = siw_hal_tc_driving(dev, LCD_MODE_U2_UNBLANK);
			if (!ret)
				ret = 1;
			break;
		case LCD_MODE_U0:
			t_dev_info(dev, "U0 -> U1 mode change\n");
			break;
		default:
			t_dev_info(dev, "Not Defined Mode, %d\n", chk_mode);
			break;
		}
		goto out;
	}

	if (lcd_mode == LCD_MODE_U0) {
		t_dev_info(dev, "U0 mode change\n");
		goto out;
	}

	t_dev_info(dev, "Not defined mode, %d\n", lcd_mode);

out:
	return ret;
}

static int siw_hal_check_mode_type_1(struct device *dev,
				int lcd_mode, int chk_mode)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int ret = 0;

	if (lcd_mode == LCD_MODE_U3) {
		goto out;
	}

	if (lcd_mode == LCD_MODE_U2) {
		if (chk_mode == LCD_MODE_U2_UNBLANK) {
			t_dev_info(dev, "U1 -> U2 : watch on\n");
			siw_hal_watch_init(dev);
			ret = 1;
		} else {
			t_dev_info(dev, "U2 mode change\n");
			siw_hal_watch_init(dev);
		}
		goto out;
	}

	if (lcd_mode == LCD_MODE_U2_UNBLANK) {
		switch (chk_mode) {
		case LCD_MODE_U2:
			t_dev_info(dev, "U2 -> U1\n");
			ret = 1;
			break;
		case LCD_MODE_U0:
			t_dev_info(dev, "U0 -> U1 mode change\n");
			siw_hal_watch_init(dev);
			break;
		default:
			t_dev_info(dev, "Not Defined Mode, %d\n", chk_mode);
			break;
		}
		goto out;
	}

	if (lcd_mode == LCD_MODE_U0) {
		t_dev_info(dev, "U0 mode change\n");
		goto out;
	}

	t_dev_info(dev, "Not defined mode, %d\n", lcd_mode);

out:
	return ret;
}

static int siw_hal_check_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int ret = 0;

	switch (chip->opt.t_chk_mode) {
	case 1:
		ret = siw_hal_check_mode_type_1(dev,
					chip->lcd_mode, chip->prev_lcd_mode);
		break;
	default:
		ret = siw_hal_check_mode_type_0(dev,
					chip->lcd_mode, chip->driving_mode);
		break;
	}

	return ret;
}

static void siw_hal_lcd_event_read_reg(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 rdata[5] = {0, 0};
	struct siw_hal_rw_multi multi[] = {
		{ 0, reg->tc_ic_status, &rdata[0], sizeof(rdata[0]), "ic_status" },
		{ 0, reg->tc_status, &rdata[1], sizeof(rdata[1]), "tc_status" },
		{ 0, reg->tc_version, &rdata[3], sizeof(rdata[3]), "version" },
		{ 0, reg->spr_chip_id, &rdata[4], sizeof(rdata[4]), "chip_id" },
		{ -1, -1, NULL, },
	};
	int ret = 0;

	ret = siw_hal_reg_rw_multi(dev, multi, "read_reg");
	if (ret < 0) {
		return;
	}

	rdata[2] = siw_hal_get_subdisp_sts(dev);

	t_dev_info(dev,
		"r[%04X] %08Xh, r[%04X] %08Xh, r[%04X] %08Xh, r[%04X] %08Xh, r[%04X] %08Xh\n",
		reg->tc_ic_status, rdata[0],
		reg->tc_status, rdata[1],
		reg->spr_subdisp_status, rdata[2],
		reg->tc_version, rdata[3],
		reg->spr_chip_id, rdata[4]);
	t_dev_info(dev,
		"v%d.%02d\n",
		(rdata[3] >> 8) & 0xF, rdata[3] & 0xFF);
}
#else	/* __SIW_SUPPORT_WATCH */
static int siw_hal_check_watch(struct device *dev){ return 0; }
static int siw_hal_check_mode(struct device *dev){ return 0; }
static void siw_hal_lcd_event_read_reg(struct device *dev){ }
#endif	/* __SIW_SUPPORT_WATCH */

#define DIC_ERR_TYPE	0x10

#if defined(__SIW_PANEL_CLASS_MOBILE)
static int siw_hal_send_abnormal_notifier_mobile(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int esd = type;
	int ret = 0;

	if (chip->opt.t_oled)
		return 0;

	if (touch_flags(ts) & TOUCH_SKIP_ESD_EVENT) {
		if (type < DIC_ERR_TYPE) {
			t_dev_info(dev, "skip sending abnormal notifier\n");
			return 0;
		}
	}

	t_dev_info(dev, "trigger abnormal notifier, %Xh\n", type);

	ret = siw_touch_atomic_notifier_call(LCD_EVENT_TOUCH_ESD_DETECTED, (void*)&esd);
	if (ret)
		t_dev_err(dev, "check the value, %d\n", ret);

	/*
	 * return value
	 * 0    : reset touch
	 * else : do not reset touch
	 */
	return 1;
}
#endif	/* __SIW_PANEL_CLASS_MOBILE */

static int siw_hal_send_abnormal_notifier(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->abnormal_notifier) {
		t_dev_info(dev, "trigger abnormal notifier quirk, %Xh\n", type);

		/*
		 * type = -1 : cancel
		 * else      : event operation
		 */
		ret = fquirks->abnormal_notifier(dev, type);
		if (ret != -EAGAIN) {
			return ret;
		}
	}

	if (type == -1)
		return 0;

#if defined(__SIW_PANEL_CLASS_MOBILE)
	return siw_hal_send_abnormal_notifier_mobile(dev, type);
#else
	/*
	 * return value
	 * 0    : reset touch
	 * else : do not reset touch
	 */
	return 0;
#endif
}

enum {
	IC_TEST_ADDR_NOT_VALID = 0x8000,
};

int siw_hal_ic_test_unit(struct device *dev, u32 data)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data_rd;
	int ret;

	if (!reg->spr_chip_test) {
		t_dev_warn(dev, "ic test addr not valid, skip\n");
		return IC_TEST_ADDR_NOT_VALID;
	}

	ret = siw_hal_write_value(dev,
				reg->spr_chip_test,
				data);
	if (ret < 0) {
		t_dev_err(dev, "ic test wr err, %08Xh, %d\n", data, ret);
		goto out;
	}

	ret = siw_hal_read_value(dev,
				reg->spr_chip_test,
				&data_rd);
	if (ret < 0) {
		t_dev_err(dev, "ic test rd err: %08Xh, %d\n", data, ret);
		goto out;
	}

	if (data != data_rd) {
		t_dev_err(dev, "ic test cmp err, %08Xh, %08Xh\n", data, data_rd);
		ret = -EFAULT;
		goto out;
	}

out:
	return ret;
}

static int siw_hal_ic_test(struct device *dev)
{
	u32 data[] = {
		0x5A5A5A5A,
		0xA5A5A5A5,
		0xF0F0F0F0,
		0x0F0F0F0F,
		0xFF00FF00,
		0x00FF00FF,
		0xFFFF0000,
		0x0000FFFF,
		0xFFFFFFFF,
		0x00000000,
	};
	int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		ret = siw_hal_ic_test_unit(dev, data[i]);
		if ((ret == IC_TEST_ADDR_NOT_VALID) || (ret < 0)) {
			break;
		}
	}

	if (ret >= 0) {
		t_dev_dbg_base(dev, "ic bus r/w test done\n");
	}

	return ret;
}

static int siw_hal_init_quirk(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u8 temp[10+4];
	int ret = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_SW42101:
		memset(temp, 0, sizeof(temp));
		ret = siw_hal_reg_read(dev, 0x150, temp, 10);
		if (ret < 0) {
			t_dev_err(dev, "init quirk for SW42101 failed, %d\n", ret);
			return ret;
		}
		t_dev_info(dev, "init quirk for SW42101: %s\n", temp);
		break;
	}

	return 0;
}

static void siw_hal_init_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (chip->ops_quirk.hw_reset != NULL) {
		chip->ops_quirk.hw_reset(dev, 1, 0);
		return;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);
	siw_hal_power(dev, POWER_OFF);
	siw_hal_power(dev, POWER_ON);
	touch_msleep(ts->caps.hw_reset_delay);
}

static int siw_hal_init_pre(struct device *dev)
{
	int ret = 0;

	ret = siw_hal_init_quirk(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_ic_test(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_init_charger(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	if (!ts->is_charger) {
		return 0;
	}

	ret = siw_hal_init_pre(dev);
	if (ret < 0) {
		return ret;
	}

	if (chip->mode_allowed_partial) {
		ret = siw_hal_tc_driving(dev, LCD_MODE_U3_PARTIAL);
		if (ret < 0) {
			return ret;
		}
		touch_msleep(100);
	}

	/* Deep Sleep */
	siw_hal_deep_sleep(dev);

	return 0;
}

static int siw_hal_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int is_probe = !!(atomic_read(&ts->state.core) == CORE_PROBE);
	int init_retry = (is_probe) ? CHIP_INIT_RETRY_PROBE : CHIP_INIT_RETRY_MAX;
	int i;
	int ret = 0;

	atomic_set(&chip->boot, IC_BOOT_DONE);

	ret = siw_hal_init_pre(dev);
	if (ret < 0) {
		siw_hal_init_reset(dev);
		goto out;
	}

	t_dev_dbg_base(dev, "charger_state = 0x%02X\n", chip->charger);

	if (atomic_read(&ts->state.debug_tool) == DEBUG_TOOL_ENABLE) {
		siw_hal_abt_init(dev);
	}

	for (i = 0; i < init_retry; i++) {
		ret = siw_hal_ic_info(dev);
		if (ret >= 0) {
			break;
		}
		/*
		 * When boot fail detected
		 *
		 * 1. At the fisrt detection,
		 *    it sends esd noti for LCD recovery(full reset procedure)
		 *    and skip fw_upgrade.
		 * 2. LCD driver is suppsed to send lcd mode notifier
		 *    back to touch side after its recovery.
		 * 3. The lcd mode notifier restarts init work again
		 *    via siw_touch_resume.
		 * 4. If boot fail detected again(counted by boot_fail_cnt)
		 *    it goes to fw_upgrade stage.
		 * (See siw_touch_init_work_func in siw_touch.c)
		 */
		if (ret == -ETDBOOTFAIL) {
			/* For the probe stage */
			if (atomic_read(&ts->state.core) == CORE_PROBE) {
				break;
			}

			/* Don't do recovery twice continuously */
			if (atomic_read(&chip->abnormal_noti_sent)) {
				break;
			}

			/* At the first boot fail */
			if (chip->boot_fail_cnt > 1) {
				break;
			}

			if (siw_hal_send_abnormal_notifier(dev, 2)) {
				ret = -ETDSENTESD;
				break;
			}
		}

		t_dev_dbg_base(dev, "retry getting ic info (%d)\n", i);

		siw_hal_init_reset(dev);
	}
	if (ret < 0) {
		goto out;
	}

	atomic_set(&chip->abnormal_noti_sent, 0);

	siw_hal_init_reg_set(dev);

	siw_hal_watch_rtc_on(dev);

	atomic_set(&chip->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);

	ret = siw_hal_lpwg_mode(dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to set lpwg control, %d\n", ret);
		goto out;
	}

	ret = siw_hal_check_watch(dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to check watch, %d\n", ret);
		goto out;
	}

	siw_hal_sysfs_post(dev, DRIVER_INIT);

out:
	if (ret < 0) {
		t_dev_err(dev, "%s init failed, %d\n",
			touch_chip_name(ts), ret);
	} else {
		t_dev_info(dev, "%s init done\n",
			touch_chip_name(ts));
	}

	siwmon_submit_ops_step_chip_wh_name(dev, "%s init done",
			touch_chip_name(ts), ret);

	return ret;
}

static int siw_hal_reinit(struct device *dev,
					int pwr_con,
					int delay,
					int irq_enable,
					int (*do_call)(struct device *dev))
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;
	int skip_quirk = !!(pwr_con & 0x10);

	pwr_con &= 0x0F;

	if (!skip_quirk && (chip->ops_quirk.hw_reset != NULL)) {
		ret = chip->ops_quirk.hw_reset(dev, pwr_con, delay);
		goto reset_done;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	atomic_set(&chip->init, IC_INIT_NEED);

	if (pwr_con) {
		siw_hal_power(dev, POWER_OFF);
		siw_hal_power(dev, POWER_ON);
	} else {
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);
		touch_msleep(chip->drv_reset_low + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_0));
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);
	}

	touch_msleep(delay);
reset_done:

	if (do_call) {
		ret = do_call(dev);
	}

	if (irq_enable)
		siw_touch_irq_control(dev, INTERRUPT_ENABLE);

	return ret;
}

#define SIW_SW_RST_TYPE_NONE	0x0F
#define SIW_SW_RST_TYPE_MAX		5

static int siw_hal_sw_reset_type_5(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = reg->spr_rst_ctl;
	u32 value = 1;

	t_dev_dbg_trace(dev, "spr_rst_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	touch_msleep(20);

	value = 0;
	t_dev_dbg_trace(dev, "spr_rst_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	return 0;
}

static int siw_hal_sw_reset_type_4(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = 0;
	u32 value = 0;

	addr = reg->spr_boot_ctl;
	t_dev_dbg_trace(dev, "spr_boot_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	addr = 0x081;
	value = 0xF83;
	t_dev_dbg_trace(dev, "spr_clk_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	addr = reg->spr_rst_ctl;
	value = 0x13;
	t_dev_dbg_trace(dev, "spr_rst_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	touch_msleep(10);

	return 0;
}

static int siw_hal_sw_reset_type_3(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = 0;
	u32 value = 0x03;

	if (chip->ops_quirk.hw_reset != NULL) {
		value |= 0x08;
	}

	if (value & 0x08) {
		addr = reg->spr_boot_ctl;
		t_dev_dbg_trace(dev, "spr_boot_ctl[%04Xh] = %08Xh\n", addr, 0);
		siw_hal_write_value(dev, addr, 0);
	}

	addr = reg->spr_rst_ctl;
	t_dev_dbg_trace(dev, "spr_rst_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	touch_msleep(10);

	return 0;
}

#define SIW_SW_RST_CTL_T2		0xFE0

static int siw_hal_sw_reset_type_2(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;

	siw_hal_write_value(dev, SIW_SW_RST_CTL_T2, 0);

	touch_msleep(1 + hal_dbg_delay(chip, HAL_DBG_DLY_SW_RST_0));

	siw_hal_write_value(dev, SIW_SW_RST_CTL_T2, 1);

	return 0;
}

static int siw_hal_sw_reset_type_1(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;

	siw_hal_cmd_write(dev, CMD_ENA);
	siw_hal_cmd_write(dev, CMD_RESET_LOW);

	touch_msleep(1 + hal_dbg_delay(chip, HAL_DBG_DLY_SW_RST_0));

	siw_hal_cmd_write(dev, CMD_RESET_HIGH);
	siw_hal_cmd_write(dev, CMD_DIS);

	return 0;
}

static int siw_hal_sw_reset_default(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 chk_resp;
	u32 data;
	int ret = 0;

	/******************************************************
	* Siliconworks does not recommend to use SW reset    *
	* due to ist limitation in stability in LG4894.      *
	******************************************************/

	siw_hal_write_value(dev, reg->spr_rst_ctl, 7);
	siw_hal_write_value(dev, reg->spr_rst_ctl, 0);

	/* Boot Start */
	siw_hal_write_value(dev, reg->spr_boot_ctl, 1);

	/* firmware boot done check */
	chk_resp = FLASH_BOOTCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, ~0,
				10 + hal_dbg_delay(chip, HAL_DBG_DLY_SW_RST_0),
				200);
	if (ret < 0) {
		t_dev_err(dev, "failed : boot check(%Xh), %Xh\n",
			chk_resp, data);
		goto out;
	}

out:
	return ret;
}

static int __siw_hal_sw_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int type = chip->opt.t_sw_rst;
	int ret = 0;

	if (type > SIW_SW_RST_TYPE_MAX) {
		t_dev_warn(dev, "sw reset not supported\n");
		ret = -EPERM;
		goto out;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	atomic_set(&chip->init, IC_INIT_NEED);

	t_dev_info(dev, "SW Reset(%d)\n", type);

	touch_msleep(chip->drv_reset_low + 10);

	switch (type) {
	case 5:
		ret = siw_hal_sw_reset_type_5(dev);
		break;
	case 4:
		ret = siw_hal_sw_reset_type_4(dev);
		break;
	case 3:
		ret = siw_hal_sw_reset_type_3(dev);
		break;
	case 2:
		ret = siw_hal_sw_reset_type_2(dev);
		break;
	case 1:
		ret = siw_hal_sw_reset_type_1(dev);
		break;
	case 0:
		ret = siw_hal_sw_reset_default(dev);
		break;
	default:
		t_dev_warn(dev, "unknown sw reset type, %d\n", type);
		ret = -ESRCH;
		break;
	}

out:
	return ret;
}

static int siw_hal_sw_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	ret = __siw_hal_sw_reset(dev);

	touch_msleep(hal_dbg_delay(chip, HAL_DBG_DLY_SW_RST_1));

	if (chip->ops_quirk.hw_reset != NULL) {
		siw_touch_qd_init_work_hw(ts);
	} else {
		siw_touch_qd_init_work_sw(ts);
	}

	return ret;
}

static int siw_hal_hw_reset_quirk(struct device *dev, int pwr_con, int delay)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	t_dev_info(dev, "run sw reset (reset gpio deactivated)\n");

	if (pwr_con) {
		siw_touch_irq_control(dev, INTERRUPT_DISABLE);

		atomic_set(&chip->init, IC_INIT_NEED);

		siw_hal_power_core(dev, POWER_OFF);
		siw_hal_power_core(dev, POWER_ON);
		touch_msleep((delay) ? delay : ts->caps.hw_reset_delay);
	}

	ret = __siw_hal_sw_reset(dev);

	touch_msleep((delay) ? delay : ts->caps.hw_reset_delay);

	return ret;
}

static int siw_hal_hw_reset(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int pwr_con = 0;

	pwr_con = !!(ctrl & 0x80);
	pwr_con |= (ctrl & 0x10);
	ctrl &= 0x0F;

	t_dev_info(dev, "HW Reset(%s)\n",
		(ctrl == HW_RESET_ASYNC) ? "Async" : "Sync");

	if (ctrl == HW_RESET_ASYNC) {
		siw_hal_reinit(dev, pwr_con, hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_1), 0, NULL);
		siw_touch_qd_init_work_hw(ts);
		return 0;
	}

	siw_hal_reinit(dev, pwr_con, ts->caps.hw_reset_delay, 1, siw_hal_init);

	return 0;
}

static int siw_hal_reset_ctrl(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ctrl_low = ctrl & 0xF;
	int ret = -EINVAL;

	mutex_lock(&ts->reset_lock);

	t_dev_info(dev, "%s reset control(0x%X)\n",
			touch_chip_name(ts), ctrl);

	siw_hal_watch_set_rtc_clear(dev);

	switch (ctrl_low) {
	case SW_RESET:
		ret = siw_hal_sw_reset(dev);
		break;
	case HW_RESET_ASYNC:
	case HW_RESET_SYNC:
		ret = siw_hal_hw_reset(dev, ctrl);
		break;

	default:
		t_dev_err(dev, "unknown reset type, 0x%X\n", ctrl);
		break;
	}

	mutex_unlock(&ts->reset_lock);

	return ret;
}

static u32 siw_hal_fw_act_buf_size(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int buf_size = (touch_get_act_buf_size(ts) - SIW_TOUCH_BUF_MARGIN) & (~0x3FF);

	return buf_size;
}

static int siw_hal_fw_rd_value(struct device *dev,
				u32 addr, u32 *value)
{
	u32 data;
	int ret;

	ret = siw_hal_read_value(dev, addr, &data);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_fwup(dev, "FW upgrade: reg rd: addr[%04Xh], value[%08Xh], %d\n",
			addr, data, ret);

	if (value)
		*value = data;

	return 0;
}

static int siw_hal_fw_wr_value(struct device *dev,
				u32 addr, u32 value)
{
	int ret;

	ret = siw_hal_write_value(dev, addr, value);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_fwup(dev, "FW upgrade: reg wr: addr[%04Xh], value[%08Xh], %d\n",
			addr, value, ret);

	return 0;
}

static int siw_hal_fw_wr_seq(struct device *dev,
				u32 addr, u8 *data, int size)
{
	int ret;

	ret = siw_hal_reg_write(dev, addr, (void *)data, size);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_fwup(dev, "FW upgrade: reg wr: addr[%04Xh], data[%02X ...], %d\n",
			addr, data[0], ret);

	return 0;
}

static int siw_hal_fw_wr_data(struct device *dev,
				u32 addr, u8 *dn_buf, int dn_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 offset = reg->serial_data_offset;
	u32 data_access = reg->data_i2cbase_addr;
	int ret = 0;

	if (!dn_size)
		goto out;

	ret = siw_hal_fw_wr_value(dev, offset, addr);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_wr_seq(dev, data_access, (void *)dn_buf, dn_size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

#if defined(__SIW_FW_TYPE_OLED_BASE)
#if defined(__SIW_FLASH_CRC_PASS)
static int siw_hal_oled_fw_read_crc_pass(struct device *dev, u32 *data)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 __data = 1;
	int ret = 0;

	if (touch_flags(ts) & TOUCH_SKIP_RESET_PIN) {
		goto out;
	}

	if (chip->ops_quirk.hw_reset != NULL) {
		goto out;
	}

	ret = siw_hal_fw_rd_value(dev, fw->gdma_crc_pass, &__data);
	if (ret < 0) {
		return ret;
	}

out:
	if (data)
		*data = __data;

	return 0;
}
#else	/* __SIW_FLASH_CRC_PASS */
static int siw_hal_oled_fw_read_crc_pass(struct device *dev, u32 *data)
{
	if (data)
		*data = 1;

	return 0;
}
#endif	/* __SIW_FLASH_CRC_PASS */

#define LOG_SZ	64

static int siw_hal_oled_fwup_rst_ctl(struct device *dev, int val, const char *str)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 spr_rst_ctl = reg->spr_rst_ctl;
	char log[LOG_SZ] = { 0, };
	char *name = NULL;
	int ret = 0;

	switch (val) {
	case 2:
		name = "system hold";
		break;
	case 1:
		name = "release cm3";
		break;
	default:
		name = "system release";
		break;
	}

	if (str == NULL) {
		snprintf(log, LOG_SZ, "%s", name);
	} else {
		snprintf(log, LOG_SZ, "%s for %s", name, str);
	}

	ret = siw_hal_fw_wr_value(dev, spr_rst_ctl, val);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - spr_rst_ctl(%d) - %s, %d\n", val, log, ret);
		goto out;
	}

	t_dev_dbg_fwup(dev, "FW upgrade: spr_rst_ctl(%d) - %s\n", val, log);

out:
	return ret;
}

static int __used siw_hal_oled_fwup_flash_crc(struct device *dev, u32 *crc_val)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 gdma_saddr = fw->gdma_saddr;
	u32 gdma_ctrl = fw->gdma_ctrl;
	u32 gmda_start = fw->gdma_start;
	u32 gdma_crc_result = fw->gdma_crc_result;
	u32 gdma_crc_pass = fw->gdma_crc_pass;
	u32 data = 0;
	u32 ctrl_data = 0;
	int ret = 0;

	ret = siw_hal_fw_wr_value(dev, gdma_saddr, 0);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_SADDR(%04Xh) set zero, %d\n",
			gdma_saddr, ret);
		goto out;
	}

	ctrl_data = (fw->sizeof_flash>>2) - 1;
	ctrl_data |= fw->gdma_ctrl_en | fw->gdma_ctrl_ro;
	ret = siw_hal_fw_wr_value(dev, gdma_ctrl, ctrl_data);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_CTL(%04Xh) write %08Xh, %d\n",
			gdma_ctrl, ctrl_data, ret);
		goto out;
	}
	touch_msleep(10);

	ret = siw_hal_fw_wr_value(dev, gmda_start, 1);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_START(%04Xh) on, %d\n",
			gmda_start, ret);
		goto out;
	}
	touch_msleep(100);

	ret = siw_hal_fw_rd_value(dev, gdma_crc_result, &data);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_CRC_RESULT(%04Xh) read, %d\n",
			gdma_crc_result, ret);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: flash crc result(%04Xh) %08Xh\n",
		gdma_crc_result, data);

	if (crc_val != NULL) {
		*crc_val = data;
	}

	ret = siw_hal_oled_fw_read_crc_pass(dev, &data);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_CRC_PASS(%04Xh) read, %d\n",
			gdma_crc_pass, ret);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: flash crc pass(%04Xh) %Xh\n",
		gdma_crc_pass, data);

out:
	touch_msleep(100);

	return ret;
}

static int siw_hal_oled_fwup_flash_mass_erase(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 fc_addr = fw->fc_addr;
	u32 fc_ctrl = fw->fc_ctrl;
	u32 fc_start = fw->fc_start;
	u32 flash_status = fw->flash_status;
	int fc_err = 0;
	int busy_time = fw->fc_erase_wait_time;
	int busy_cnt = (fw->flash_page_size<<1)/busy_time;
	u32 chk_resp, data;
	int ret = 0;

	ret = siw_hal_fw_wr_value(dev, fc_addr, 0);
	if (ret < 0) {
		fc_err = 1;
		goto out;
	}

	ret = siw_hal_fw_wr_value(dev, fc_ctrl, fw->fc_ctrl_mass_erase);
	if (ret < 0) {
		fc_err = 2;
		goto out;
	}

	ret = siw_hal_fw_wr_value(dev, fc_start, 1);
	if (ret < 0) {
		fc_err = 3;
		goto out;
	}

	chk_resp = 1;
	ret = siw_hal_condition_wait_cond(dev, flash_status, &data,
			chk_resp, ~0, busy_time, busy_cnt, NOT_COND);
	if (ret < 0) {
		fc_err = 4;
		t_dev_err(dev, "FW upgrade: failed - flash erase wait(%Xh), %Xh\n",
			chk_resp, data);
		goto out;
	}

out:
	touch_msleep(10);

	siw_hal_fw_wr_value(dev, fc_ctrl, 0);

	if (fc_err) {
		t_dev_err(dev, "FW upgrade: failed - flash mass erase error, %d, %d\n",
			fc_err, ret);
	} else {
		t_dev_info(dev, "FW upgrade: flash mass erase done\n");
	}

	return ret;
}

static int siw_hal_oled_fwup_flash_write_en(struct device *dev, int en)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 fc_ctrl = fw->fc_ctrl;
	u32 fc_ctrl_wr_en = fw->fc_ctrl_wr_en;
	int ret = 0;

	if (!fc_ctrl)
		goto out;

	ret = siw_hal_fw_wr_value(dev, fc_ctrl, (en) ? fc_ctrl_wr_en : 0);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_write_data(struct device *dev, int addr, u8 *dn_buf, int dn_size)
{
	int ret = 0;

	ret = siw_hal_fw_wr_data(dev, addr, dn_buf, dn_size);
	if (ret < 0) {
		goto out;
	}

out:
	touch_msleep(5);

	return ret;
}

static int siw_hal_oled_fwup_bdma_saddr(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 bdma_saddr = fw->bdma_saddr;
	u32 datasram_addr = fw->datasram_addr;
	int ret = 0;

	if (!bdma_saddr)
		goto out;

	ret = siw_hal_fw_wr_value(dev, bdma_saddr, datasram_addr);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_bdma_daddr(struct device *dev, int dst)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 bdma_daddr = fw->bdma_daddr;
	int ret = 0;

	if (!bdma_daddr)
		goto out;

	ret = siw_hal_fw_wr_value(dev, bdma_daddr, dst);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_bdma_ctrl(struct device *dev, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 bdma_ctrl = fw->bdma_ctrl;
	u32 bdma_ctrl_en = fw->bdma_ctrl_en;
	int ret = 0;

	if (!bdma_ctrl)
		goto out;

	ret = siw_hal_fw_wr_value(dev, bdma_ctrl, bdma_ctrl_en | (size>>2));
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_bdma_cal(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 bdma_cal_op = fw->bdma_cal_op;
	u32 bdma_cal_op_ctrl = fw->bdma_cal_op_ctrl;
	int ret = 0;

	if (!bdma_cal_op)
		goto out;

	ret = siw_hal_fw_wr_value(dev, bdma_cal_op, bdma_cal_op_ctrl);

out:
	return ret;
}

static int siw_hal_oled_fwup_bdma_start(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 bdma_start = fw->bdma_start;
	int ret = 0;

	if (!bdma_start)
		goto out;

	ret = siw_hal_fw_wr_value(dev, bdma_start, 1);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_bdma_sts(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 bdma_sts = fw->bdma_sts;
	u32 bdma_sts_tr_busy = fw->bdma_sts_tr_busy;
	int ret = 0;

	if (!bdma_sts)
		goto out;

	ret = siw_hal_condition_wait_cond(dev, bdma_sts, NULL,
		bdma_sts_tr_busy, bdma_sts_tr_busy, 10, 2000, NOT_COND);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_fw_pre(struct device *dev)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	int ret = 0;

	ret = siw_hal_oled_fwup_rst_ctl(dev, 2, NULL);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_flash_mass_erase(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_rst_ctl(dev, 0, NULL);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_fw_core(struct device *dev, u8 *dn_buf, int dn_size)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_fw_info *fw = &chip->fw;
	u8 *fw_data = NULL;
	int fw_size = 0;
	int fw_addr = 0;
	int curr_size = 0;
	int fw_size_org = dn_size;
	int fw_dn_size = 0, fw_dn_percent;
	int buf_size = 0;
	int ret = 0;

	buf_size = min(FW_DN_LOG_UNIT, (int)siw_hal_fw_act_buf_size(dev));

	ret = siw_hal_oled_fwup_rst_ctl(dev, 2, "flash write");
	if (ret < 0) {
		goto out;
	}

#if 0
	ret = siw_hal_oled_fwup_flash_write_en(dev, 1);
	if (ret < 0) {
		goto out;
	}
#endif

	ret = siw_hal_oled_fwup_bdma_sts(dev);
	if (ret < 0) {
		goto out;
	}

	fw_data = dn_buf;
	fw_size = dn_size;
	fw_addr = 0;

	while (fw_size) {
		curr_size = min(fw_size, buf_size);

		t_dev_dbg_fwup(dev, "FW upgrade: flash write %08Xh %04Xh\n", fw_addr, curr_size);

		ret = siw_hal_oled_fwup_write_data(dev, 0, fw_data, curr_size);
		if (ret < 0) {
			t_dev_err(dev, "FW upgrade: failed - flash write %08Xh %04Xh, %d\n",
				fw_addr, curr_size, ret);
			break;
		}

		ret = siw_hal_oled_fwup_bdma_saddr(dev);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_oled_fwup_bdma_ctrl(dev, curr_size);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_oled_fwup_bdma_cal(dev);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_oled_fwup_bdma_daddr(dev, fw_addr);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_oled_fwup_flash_write_en(dev, 1);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_oled_fwup_bdma_start(dev);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_oled_fwup_bdma_sts(dev);
		if (ret < 0) {
			goto out;
		}

		fw_addr += curr_size;
		fw_data += curr_size;
		fw_size -= curr_size;

		fw_dn_size += curr_size;
		if (!fw_size || !(fw_dn_size & (FW_DN_LOG_UNIT-1))) {
			fw_dn_percent = (fw_dn_size * 100);
			fw_dn_percent /= fw_size_org;

			t_dev_info(dev, "FW upgrade: downloading...(%d%c)\n",
				fw_dn_percent, '%');
		}
	}

out:
	siw_hal_oled_fwup_flash_write_en(dev, 0);

//	siw_hal_oled_fwup_rst_ctl(dev, 0, NULL);

	return ret;
}

static int siw_hal_oled_fwup_fw_post(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	u32 boot_code_addr = chip->fw.boot_code_addr;
	u32 chk_resp, data;
	int ret = 0;

	if (!boot_code_addr)
		goto out;

	ret = siw_hal_fw_wr_value(dev, boot_code_addr, FW_BOOT_LOADER_INIT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - FW_BOOT_LOADER_INIT, %d\n", ret);
		goto out;
	}
	touch_msleep(100);

	ret = siw_hal_oled_fwup_rst_ctl(dev, 0, NULL);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - system release, %d\n", ret);
		goto out;
	}
	touch_msleep(200);

	chk_resp = FW_BOOT_LOADER_CODE;
	ret = siw_hal_condition_wait_cond(dev, boot_code_addr, &data,
			chk_resp, ~0, 10, 20, EQ_COND);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - boot check(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: boot loader ready\n");

out:
	return ret;
}

static int __used siw_hal_oled_fwup_fw(struct device *dev,
				u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max = touch_fw_size(ts);
	int ret = 0;

	ret = siw_hal_oled_fwup_fw_pre(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_fw_core(dev, fw_buf, fw_size_max);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_fw_post(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

#if defined(__SIW_FLASH_CFG)
static int siw_hal_oled_fwup_conf_pre(struct device *dev,
				u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 conf_idx_addr = fw->conf_idx_addr;
	int fw_size_max = touch_fw_size(ts);
	u32 cfg_min_s_conf_idx = OLED_MIN_S_CONF_IDX;
	u32 cfg_max_s_conf_idx = OLED_MIN_S_CONF_IDX;
	u32 cfg_c_size = OLED_CFG_C_SIZE;
	u32 cfg_s_size = OLED_CFG_S_SIZE;
	u32 index = 0;
	int required_size = 0;
	int ret = 0;

	if (!conf_idx_addr || !fw->cfg_chip_id)
		goto out;

	ret = siw_hal_oled_fwup_rst_ctl(dev, 2, "conf");
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_rd_value(dev, conf_idx_addr, &index);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - read conf_idx_addr, %d\n", ret);
		goto out;
	}

	if ((index < cfg_min_s_conf_idx) || (index > cfg_max_s_conf_idx)) {
		t_dev_err(dev, "FW upgrade: failed - wrong cfg index, %d\n", index);
		return -EFAULT;
	}

	required_size = fw_size_max + cfg_c_size + (index * cfg_s_size);
	if (fw_size < required_size) {
		t_dev_err(dev, "FW upgrade: failed - too big index, %d (%Xh < %Xh)\n",
			index, fw_size, required_size);
		return -EFAULT;
	}

	t_dev_info(dev, "FW upgrade: conf_index %d", index);

	chip->fw.conf_index = index;

out:
	return ret;
}

static int siw_hal_oled_fwup_do_conf_core(struct device *dev,
			int dn_addr, u8 *dn_buf, int dn_size, const char *name)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	int buf_size = siw_hal_fw_act_buf_size(dev);
	int fw_size_max = touch_fw_size(ts);
	int ret = 0;

	if (!fw->conf_idx_addr || !fw->cfg_chip_id)
		goto out;

	t_dev_info(dev, "FW upgrade: %s write %Xh %Xh\n", name, dn_addr, dn_size);

	if (dn_size > buf_size) {
		t_dev_err(dev, "FW upgrade: buffer overflow, dn_size %d > %d\n",
			dn_size, buf_size);
		ret = -EOVERFLOW;
		goto out;
	}

	ret = siw_hal_oled_fwup_bdma_sts(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_write_data(dev, 0, dn_buf, dn_size);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - %s write, %d\n", name, ret);
		goto out;
	}

	ret = siw_hal_oled_fwup_bdma_saddr(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_bdma_ctrl(dev, dn_size);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_bdma_cal(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_bdma_daddr(dev, fw_size_max);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_flash_write_en(dev, 1);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_bdma_start(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_bdma_sts(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_conf_core(struct device *dev, u8 *fw_buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	int conf_index = fw->conf_index;
	int fw_size_max = touch_fw_size(ts);
	u32 cfg_pow_s_conf = OLED_POW_S_CONF;
	u32 cfg_c_size = OLED_CFG_C_SIZE;
	u32 cfg_s_size = OLED_CFG_S_SIZE;
	u8 *fw_data = NULL;
	int fw_size = 0;
	int fw_addr = 0;
	u32 data = 0;
	int ret = 0;

	if (!fw->conf_idx_addr || !fw->cfg_chip_id)
		goto out;

	ret = siw_hal_oled_fwup_rst_ctl(dev, 2, "flash write - conf");
	if (ret < 0) {
		goto out;
	}

#if 0
	ret = siw_hal_oled_fwup_flash_write_en(dev, 1);
	if (ret < 0) {
		goto out;
	}
#endif

	fw_addr = fw_size_max;

	if (cfg_c_size) {
		fw_data = &fw_buf[fw_size_max];
		fw_size = cfg_c_size;

		ret = siw_hal_oled_fwup_do_conf_core(dev, fw_addr, fw_data, fw_size, "ccfg");
		if (ret < 0) {
			goto out_conf;
		}
	}

	fw_addr += cfg_c_size;
	data = fw_size_max + cfg_c_size + ((conf_index - 1)<<cfg_pow_s_conf);
	fw_data = &fw_buf[data];
	fw_size = cfg_s_size;

	ret = siw_hal_oled_fwup_do_conf_core(dev, fw_addr, fw_data, fw_size, "scfg");
	if (ret < 0) {
		goto out_conf;
	}

out_conf:
	siw_hal_oled_fwup_flash_write_en(dev, 0);

out:
	return ret;
}

static int __used siw_hal_oled_fwup_conf(struct device *dev,
				u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	int ret = 0;

	if (!fw->conf_idx_addr || !fw->cfg_chip_id)
		goto out;

	ret = siw_hal_oled_fwup_conf_pre(dev, fw_buf, fw_size);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_oled_fwup_conf_core(dev, fw_buf);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_oled_fwup_verify_cfg(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct oled_cfg_head *head = (struct oled_cfg_head *)buf;
	u32 cfg_c_size = OLED_CFG_C_SIZE;
	u32 cfg_s_size = OLED_CFG_S_SIZE;

	if (head->chip_id != chip->fw.cfg_chip_id) {
		t_dev_warn(dev, "FW chk_img: invalid chip ID, %dh\n", head->chip_id);
		return -EFAULT;
	}

	if (cfg_c_size) {
		if (head->c_size.b.common_size != cfg_c_size) {
			t_dev_warn(dev, "FW chk_img: invalid c_cfg size, %04Xh\n", head->c_size.b.common_size);
			return -EFAULT;
		}
	}

	if (head->c_size.b.specific_size != cfg_s_size) {
		t_dev_warn(dev, "FW chk_img: invalid s_cfg size, %04Xh\n", head->c_size.b.specific_size);
		return -EFAULT;
	}

	t_dev_dbg_fwup(dev, "FW chk_img: magic_code  : %08Xh\n", head->magic_code);
	t_dev_dbg_fwup(dev, "FW chk_img: chip ID     : %d\n", head->chip_id);
	t_dev_dbg_fwup(dev, "FW chk_img: c_cfg size  : %04X\n", head->c_size.b.common_size);
	t_dev_dbg_fwup(dev, "FW chk_img: s_cfg size  : %04X\n", head->c_size.b.specific_size);

	return 0;
}

static int siw_hal_oled_fwup_verify_s_cfg(struct device *dev, char *buf, int index)
{
	struct oled_cfg_head *head = (struct oled_cfg_head *)buf;
	struct oled_s_cfg_head *s_head = &head->s_cfg_head;
	int chip_rev = s_head->info_1.b.chip_rev;

	if (chip_rev > 10) {
		t_dev_warn(dev, "FW chk_img: invalid s_cfg rev, %d\n", chip_rev);
		return -EFAULT;
	}

	t_dev_dbg_fwup(dev, "FW chk_img: s-chip_rev  : %d\n", s_head->info_1.b.chip_rev);
	t_dev_dbg_fwup(dev, "FW chk_img: s-model_id  : %d\n", s_head->info_1.b.model_id);
	t_dev_dbg_fwup(dev, "FW chk_img: s-lcm_id    : %d\n", s_head->info_1.b.lcm_id);
	t_dev_dbg_fwup(dev, "FW chk_img: s-fpc_id    : %d\n", s_head->info_1.b.fpc_id);
	t_dev_dbg_fwup(dev, "FW chk_img: s-lot_id    : %d\n", s_head->info_2.b.lot_id);

	return 0;
}
#else	/* !__SIW_FLASH_CFG */
static int siw_hal_oled_fwup_conf(struct device *dev,
				u8 *fw_buf, int fw_size)
{
	return 0;
}
#endif	/* __SIW_FLASH_CFG */

static int siw_hal_oled_fwup_post(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 crc_fixed_value = fw->crc_fixed_value;
	u32 crc_val;
	int ret = 0;

	ret = siw_hal_oled_fwup_rst_ctl(dev, 0, NULL);
	if (ret < 0) {
		goto out;
	}
	touch_msleep(100);

	ret = siw_hal_oled_fwup_rst_ctl(dev, 2, "crc");
	if (ret < 0) {
		goto out;
	}
	touch_msleep(100);

	ret = siw_hal_oled_fwup_flash_crc(dev, &crc_val);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - flash crc, %d\n", ret);
		goto out;
	}
	if (crc_val != crc_fixed_value) {
		t_dev_err(dev, "FW upgrade: flash crc error %08Xh != %08Xh, %d\n",
			crc_fixed_value, crc_val, ret);
		ret = -EFAULT;
		goto out;
	}
	t_dev_info(dev, "FW upgrade: flash crc check done\n");

	ret = siw_hal_oled_fwup_rst_ctl(dev, 1, NULL);
	if (ret < 0) {
		goto out;
	}
	touch_msleep(100);

	ret = siw_hal_oled_fwup_rst_ctl(dev, 0, NULL);
	if (ret < 0) {
		goto out;
	}
	touch_msleep(100);

	t_dev_dbg_fwup(dev, "FW upgrade: post done\n");

	return 0;

out:
	siw_hal_oled_fwup_rst_ctl(dev, 0, NULL);

	touch_msleep(100);

	return ret;
}

static int siw_hal_oled_fwup_verify(struct device *dev, u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
#if defined(__SIW_FLASH_CFG)
	struct siw_hal_fw_info *fw = &chip->fw;
#endif
	int fw_size_max = touch_fw_size(ts);
	u32 fw_code_crc = *(u32 *)&fw_buf[fw_size_max - 4];
	u32 fw_code_size = *(u32 *)&fw_buf[fw_size_max - 8];

	if (fw_size < fw_size_max) {
		t_dev_err(dev, "FW chk_img: too small img size(%Xh), must be >= fw_size_max(%Xh)\n",
			fw_size, fw_size_max);
		return OLED_E_FW_CODE_SIZE_ERR;
	}

	t_dev_info(dev, "FW chk_img: code size %Xh, code crc %Xh\n",
		fw_code_size, fw_code_crc);

	if (fw_code_size > fw_size_max) {
		t_dev_err(dev, "FW chk_img: invalid code_size(%Xh), must be <= fw_size_max(%Xh)\n",
			fw_code_size, fw_size_max);
		return OLED_E_FW_CODE_SIZE_ERR;
	}

	if (fw_size == fw_size_max) {
		return OLED_E_FW_CODE_ONLY_VALID;
	}

#if defined(__SIW_FLASH_CFG)
	if (fw->conf_idx_addr && fw->cfg_chip_id) {
		struct oled_cfg_head *head = NULL;
		char *cfg_base = NULL;
		char *s_cfg_base = NULL;
		u32 cfg_pow_s_conf = OLED_POW_S_CONF;
		u32 cfg_magic_code = OLED_CFG_MAGIC_CODE;
		u32 cfg_offset = 0;
		u32 cfg_pos = 0;
		int s_cfg_cnt = 0;
		int i;
		int ret = 0;

		cfg_offset = *(u32 *)&fw_buf[BIN_CFG_OFFSET_POS];
		if (!cfg_offset) {
			t_dev_info(dev, "FW chk_img: cfg offset zero\n");
			return OLED_E_FW_CODE_CFG_ERR;
		}

		cfg_pos = *(u32 *)&fw_buf[cfg_offset];

		t_dev_info(dev, "FW chk_img: cfg pos %Xh (cfg offset %Xh)\n",
			cfg_pos, cfg_offset);

		if (cfg_pos >= fw_size) {
			t_dev_err(dev, "FW chk_img: invalid cfg_pos(%Xh), must be < img size(%Xh)\n",
				cfg_pos, fw_size);
			return OLED_E_FW_CODE_CFG_ERR;
		}

		if (cfg_pos >= fw->sizeof_flash) {
			t_dev_err(dev, "FW chk_img: invalid cfg_pos(%Xh), must be < SIZEOF_FLASH\n",
				cfg_pos);
			return OLED_E_FW_CODE_CFG_ERR;
		}

		cfg_base = (char *)&fw_buf[cfg_pos];

		head = (struct oled_cfg_head *)cfg_base;
		if (head->magic_code != cfg_magic_code) {
			t_dev_warn(dev, "FW chk_img: unknown data in cfg\n");
			return OLED_E_FW_CODE_ONLY_VALID;
		}

		t_dev_info(dev, "FW chk_img: cfg detected\n");
		ret = siw_hal_oled_fwup_verify_cfg(dev, (char *)head);
		if (ret < 0) {
			t_dev_warn(dev, "FW chk_img: invalid cfg\n");
			return OLED_E_FW_CODE_ONLY_VALID;
		}

		s_cfg_base = cfg_base + head->c_size.b.common_size;
		s_cfg_cnt = ((fw_size - fw_size_max) - head->c_size.b.common_size)>>cfg_pow_s_conf;
		for (i = 0; i < s_cfg_cnt; i++) {
			ret = siw_hal_oled_fwup_verify_s_cfg(dev, (char *)s_cfg_base, i);
			if (ret < 0) {
				t_dev_err(dev, "FW chk_img: invalid s_cfg\n");
				return OLED_E_FW_CODE_CFG_ERR;
			}
			s_cfg_base += head->c_size.b.specific_size;
		}

		return OLED_E_FW_CODE_AND_CFG_VALID;
	}
#endif	/* __SIW_FLASH_CFG */

	return -EINVAL;
}

#define t_dev_dbg_fwup_setup(_dev, _format, _fw, _element, args...)	\
		t_dev_dbg_fwup(_dev, "%-20s = " _format "\n", #_element, _fw->_element, ##args)

static void __used siw_hal_oled_fwup_show_setup(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;

	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, sizeof_flash);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, flash_page_offset);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, flash_page_size);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, flash_max_rw_size);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, boot_code_addr);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, gdma_saddr);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, gdma_ctrl);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, gdma_ctrl_en);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, gdma_ctrl_ro);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, gdma_start);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, fc_ctrl);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, fc_ctrl_page_erase);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, fc_ctrl_mass_erase);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, fc_ctrl_wr_en);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, fc_start);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, fc_addr);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, flash_status);
	t_dev_dbg_fwup_setup(dev, "%d", fw, fc_erase_wait_cnt);
	t_dev_dbg_fwup_setup(dev, "%d", fw, fc_erase_wait_time);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, bdma_saddr);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, bdma_daddr);
	if (fw->bdma_cal_op) {
		t_dev_dbg_fwup_setup(dev, "0x%04X", fw, bdma_cal_op);
	}
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, bdma_ctrl);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, bdma_ctrl_en);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, bdma_ctrl_bst);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, bdma_start);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, bdma_sts);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, bdma_sts_tr_busy);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, datasram_addr);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, info_ptr);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, gdma_crc_result);
	t_dev_dbg_fwup_setup(dev, "0x%04X", fw, gdma_crc_pass);
	t_dev_dbg_fwup_setup(dev, "0x%08X", fw, crc_fixed_value);

	if (fw->conf_idx_addr) {
		t_dev_dbg_fwup_setup(dev, "0x%04X", fw, conf_idx_addr);
		t_dev_dbg_fwup_setup(dev, "%d(0x%08X)", fw, cfg_chip_id, fw->cfg_chip_id);
	}
}

static void __used siw_hal_oled_fwup_setup(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;

	fw->conf_index = 0;
	fw->conf_idx_addr = 0;
	fw->boot_code_addr = 0x01A;
	fw->sizeof_flash = (128<<10);
	fw->flash_page_offset = (0x200);
	fw->flash_page_size = (2<<10);
	fw->flash_max_rw_size = (1<<10);
	fw->gdma_saddr = 0x056;
	fw->gdma_ctrl = 0x058;
	fw->gdma_ctrl_en = GDMA_CTRL_EN;
	fw->gdma_ctrl_ro = GDMA_CTRL_READONLY;
	fw->gdma_start = 0x059;
	fw->fc_ctrl = 0x06B;
	fw->fc_ctrl_page_erase = FC_CTRL_PAGE_ERASE;
	fw->fc_ctrl_mass_erase = FC_CTRL_MASS_ERASE;
	fw->fc_ctrl_wr_en = FC_CTRL_WR_EN;
	fw->fc_start = 0x06C;
	fw->fc_addr = 0x06D;
	fw->flash_status = 0xFE2;
	fw->fc_erase_wait_cnt = 200;
	fw->fc_erase_wait_time = 5;
	fw->bdma_saddr = 0x072;
	fw->bdma_daddr = 0x073;
	fw->bdma_cal_op = 0;
	fw->bdma_cal_op_ctrl = 0;
	fw->bdma_ctrl = 0x074;
	fw->bdma_ctrl_en = BDMA_CTRL_EN;
	fw->bdma_ctrl_bst = BDMA_CTRL_BST;
	fw->bdma_start = 0x075;
	fw->bdma_sts = 0x077;
	fw->bdma_sts_tr_busy = BDMA_STS_TR_BUSY;
	fw->datasram_addr = 0x20000000;
	fw->cfg_chip_id = 0;

	switch (chip->opt.t_oled) {
	case 2:
	//	fw->boot_code_addr = 0x1F;
		fw->boot_code_addr = 0;
		fw->gdma_saddr = 0x085;
		fw->gdma_ctrl = 0x087;
		fw->gdma_start = 0x088;
		fw->fc_ctrl = 0x09A;
		fw->fc_start = 0x09B;
		fw->fc_addr = 0x09C;
		fw->bdma_saddr = 0x0A1;
		fw->bdma_daddr = 0x0A3;
		fw->bdma_cal_op = 0xA5;
		fw->bdma_cal_op_ctrl = 3 | (1024<<2) | (1024<<13);
		fw->bdma_ctrl = 0x0A6;
		fw->bdma_ctrl_en = BDMA_CTRL_EN | (2<<23) | (2<<25) | (2<<27);
		fw->bdma_ctrl_bst = BIT(17);
		fw->bdma_start = 0x0A7;
		fw->bdma_sts = 0x0A9;
		fw->datasram_addr = 0x20000000;
		break;
	}

	switch (touch_chip_type(ts)) {
	case CHIP_SW42000:
		fw->conf_idx_addr = 0x646;
		fw->cfg_chip_id = (42000);
		break;
	}
}

static int __used siw_hal_oled_fwup_upgrade(struct device *dev,
					u8 *fw_buf, int fw_size, int retry)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max = touch_fw_size(ts);
	u32 include_conf = !!(fw_size > fw_size_max);
	int ret = 0;

	if (!retry) {
		siw_hal_oled_fwup_setup(dev);
		siw_hal_oled_fwup_show_setup(dev);
	}

	chip->fw.conf_index = 0;

	t_dev_info(dev, "FW upgrade:%s include conf data\n",
			(include_conf) ? "" : " not");

	t_dev_dbg_fwup(dev, "FW upgrade: fw size %08Xh, fw_size_max %08Xh\n",
			fw_size, fw_size_max);

	ret = siw_hal_oled_fwup_verify(dev, fw_buf, fw_size);
	switch (ret) {
	case OLED_E_FW_CODE_ONLY_VALID:
		include_conf = 0;
		break;
	case OLED_E_FW_CODE_AND_CFG_VALID:
		break;
	case OLED_E_FW_CODE_SIZE_ERR:
	case OLED_E_FW_CODE_CFG_ERR:
	default:
		ret = -EPERM;
		goto out;
	}

	ret = siw_hal_oled_fwup_fw(dev, fw_buf, fw_size);
	if (ret < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_CODE);
		goto out;
	}

	if (include_conf) {
		ret = siw_hal_oled_fwup_conf(dev, fw_buf, fw_size);
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_CFG);
		if (ret < 0) {
			goto out;
		}
	}

	ret = siw_hal_oled_fwup_post(dev);

out:
	return ret;
}

static int __used siw_hal_oled_boot_status(struct device *dev, u32 *boot_st)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 gdma_crc_result = fw->gdma_crc_result;
	u32 crc_fixed_value = fw->crc_fixed_value;
	u32 info_ptr = fw->info_ptr;
	u32 rdata_crc = 0;
	u32 rdata_pass = 0;
	u32 rdata_ptr = 0;
	int err = 0;
	int ret = 0;

	/*
	 * Check CRC
	 */
	ret = siw_hal_fw_rd_value(dev, gdma_crc_result, &rdata_crc);
	if (ret < 0) {
		t_dev_err(dev, "boot status: read %04Xh failed, %d\n",
			gdma_crc_result, ret);
		return ret;
	}
	err |= (rdata_crc != crc_fixed_value);

	ret = siw_hal_oled_fw_read_crc_pass(dev, &rdata_pass);
	if (ret < 0) {
		return ret;
	}
	err |= (!rdata_pass)<<1;

	/*
	 * Check Boot
	 */
	ret = siw_hal_fw_rd_value(dev, info_ptr, &rdata_ptr);
	if (ret < 0) {
		t_dev_err(dev, "boot status: read %04Xh failed, %d\n",
			info_ptr, ret);
		return ret;
	}
	err |= (!rdata_ptr)<<2;

	if (boot_st != NULL) {
		(*boot_st) = (err) ? BIT(BOOT_STS_POS_DUMP_ERR) : BIT(BOOT_STS_POS_DUMP_DONE);
	}

	if (err) {
		t_dev_err(dev, "boot status: %Xh(%Xh, %Xh, %Xh)\n",
			err, rdata_crc, rdata_pass, rdata_ptr);
	}

	return 0;
}
#else	/* !__SIW_FW_TYPE_OLED_BASE */
static void siw_hal_oled_fwup_setup(struct device *dev)
{

}

static int siw_hal_oled_fwup_upgrade(struct device *dev,
					u8 *fw_buf, int fw_size, int retry)
{
	t_dev_info(dev, "FW upgrade: noop\n");

	return 0;
}

static int siw_hal_oled_boot_status(struct device *dev, u32 *boot_st)
{
	t_dev_info(dev, "OLED boot status: noop\n");

	if (boot_st != NULL) {
		(*boot_st) = BIT(BOOT_STS_POS_DUMP_DONE);
	}

	return 0;
}
#endif 	/* __SIW_FW_TYPE_OLED_BASE */

static int siw_hal_fw_sram_wr_enable(struct device *dev, int onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data;
	int ret = 0;

#if 0
	ret = siw_hal_fw_rd_value(dev, reg->spr_sram_ctl, &data);
	if (ret < 0) {
		goto out;
	}

	if (onoff)
		data |= 0x01;
	else
		data &= ~0x01;

	ret = siw_hal_fw_wr_value(dev, reg->spr_sram_ctl, data);
	if (ret < 0) {
		goto out;
	}
#else
//	data = !!onoff;
	data = (onoff) ? 0x03 : 0x00;
	ret = siw_hal_fw_wr_value(dev, reg->spr_sram_ctl, data);
	if (ret < 0) {
		goto out;
	}
#endif

out:
	return ret;
}

static int siw_hal_fw_upgrade_fw_core(struct device *dev, u8 *dn_buf, int dn_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int is_i2c = !!(touch_bus_type(ts) == BUS_IF_I2C);
	u8 *fw_data;
	int fw_size;
	int fw_pos, curr_size;
	int fw_size_org = dn_size;
	int fw_dn_size = 0, fw_dn_percent;
	int buf_size = min(MAX_RW_SIZE, (int)siw_hal_fw_act_buf_size(dev));
	int ret = 0;

	fw_data = dn_buf;
	fw_size = dn_size;
	fw_pos = 0;
	while (fw_size) {
		t_dev_dbg_fwup(dev, "FW upgrade: fw_pos[%06Xh ...] = %02X %02X %02X %02X ...\n",
				fw_pos,
				fw_data[0], fw_data[1], fw_data[2], fw_data[3]);

		curr_size = min(fw_size, buf_size);

		/* code sram base address write */
		ret = siw_hal_fw_wr_value(dev, reg->spr_code_offset, fw_pos>>2);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_fw_wr_seq(dev, reg->code_access_addr,
					(void *)fw_data, curr_size);
		if (ret < 0) {
			goto out;
		}

		fw_data += curr_size;
		fw_pos += curr_size;
		fw_size -= curr_size;

		/*
		 * Show progress log for slow I2C case
		 */
		if (!is_i2c) {
			continue;
		}

		fw_dn_size += curr_size;
		if (!fw_size || !(fw_dn_size & (FW_DN_LOG_UNIT-1))) {
			fw_dn_percent = (fw_dn_size * 100);
			fw_dn_percent /= fw_size_org;

			t_dev_info(dev, "FW upgrade: downloading...(%d%c)\n",
				fw_dn_percent, '%');
		}
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf_core(struct device *dev,
				u32 addr, u8 *dn_buf, int dn_size)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int buf_size = (int)siw_hal_fw_act_buf_size(dev);
	int ret = 0;

	if (dn_size > buf_size) {
		t_dev_err(dev, "FW upgrade: buffer overflow, dn_size %d > %d\n",
			dn_size, buf_size);
		ret = -EOVERFLOW;
		goto out;
	}

	ret = siw_hal_fw_wr_data(dev, addr, dn_buf, dn_size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

#if defined(__SIW_FW_TYPE_1)
static int siw_hal_fw_upgrade_fw_post_quirk(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 chk_resp, data;
	u32 boot_code_addr = chip->fw.boot_code_addr;
	int ret;

	ret = siw_hal_fw_wr_value(dev, boot_code_addr, FW_BOOT_LOADER_INIT);
	if (ret < 0) {
		goto out;
	}

	/* Set Serial Dump Done */
	ret = siw_hal_fw_wr_value(dev, reg->spr_boot_ctl, 1);
	if (ret < 0) {
		goto out;
	}

	/* Release CM3 core */
	ret = siw_hal_fw_wr_value(dev, reg->spr_rst_ctl, 0);
	if (ret < 0) {
		goto out;
	}

	/* firmware boot done check */
	chk_resp = FW_BOOT_LOADER_CODE;
	ret = siw_hal_condition_wait(dev, boot_code_addr, &data,
				chk_resp, ~0,
				FW_POST_QUIRK_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_0),
				FW_POST_QUIRK_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - boot check(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: boot check done\n");

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf_quirk(struct device *dev,
			     u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max;
	u32 conf_index = chip->fw.conf_index;
	u32 conf_dn_addr;
	u32 data;
	int ret = 0;

	if (!conf_index) {
		goto out;
	}

	fw_size_max = touch_fw_size(ts);

	conf_dn_addr = chip->fw.conf_dn_addr;
	ret = siw_hal_fw_rd_value(dev, conf_dn_addr, &data);
	if (ret < 0) {
		goto out;
	}

	conf_dn_addr = (data & 0xFFFF);
	t_dev_dbg_fwup(dev, "FW upgrade: s_conf_dn_addr %04Xh (%08Xh)\n",
			conf_dn_addr, data);

	data = fw_size_max +	\
		(NUM_C_CONF<<POW_C_CONF) +	\
		((conf_index - 1)<<POW_S_CONF);
	ret = siw_hal_fw_upgrade_conf_core(dev, conf_dn_addr,
				(u8 *)&fw_buf[data], FLASH_CONF_SIZE);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static void siw_hal_fw_var_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;

	fw->conf_index = 0;
	fw->conf_idx_addr = FW_S_CONF_IDX_ADDR;
	fw->conf_dn_addr = FW_S_CONF_DN_ADDR;
	fw->boot_code_addr = FW_BOOT_CODE_ADDR;
	fw->conf_skip = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_SW46104:
		fw->conf_idx_addr = 0x316;
		fw->conf_dn_addr = 0x31D;
		fw->boot_code_addr = 0x0BD;
		break;
	case CHIP_SW49501:
		fw->conf_idx_addr = 0x316;
		fw->conf_dn_addr = 0x31D;
		fw->boot_code_addr = 0x03F;
		break;
	case CHIP_SW42103:
		fw->boot_code_addr = 0x03F;
		break;
	case CHIP_SW17700:
		fw->conf_idx_addr = 0x246;
		fw->conf_dn_addr = 0x24D;
		fw->boot_code_addr = 0x0BB;
		break;
	}

	t_dev_info(dev, "FW upgrade: idx %Xh, dn %Xh, code %Xh\n",
		fw->conf_idx_addr, fw->conf_dn_addr, fw->boot_code_addr);
}

static int siw_hal_fw_size_check(struct device *dev, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	int fw_size_max = touch_fw_size(ts);
	int size_min = (fw_size_max + (NUM_C_CONF<<POW_C_CONF) + (MIN_S_CONF<<POW_S_CONF));
	int size_max = (fw_size_max + (NUM_C_CONF<<POW_C_CONF) + (MAX_S_CONF<<POW_S_CONF));

	siw_hal_fw_var_init(dev);

	switch (touch_chip_type(ts)) {
	case CHIP_SW42103:
		fw->conf_skip = !!(fw_size == fw_size_max);

		if (!strncmp(fw->product_id, "LA145WF1", 8)) {
			fw->conf_skip = 1;
		}
		break;
	case CHIP_SW17700:
		fw->conf_skip = !!(fw_size == fw_size_max);

		if (!strncmp(fw->product_id, "LA103WF5", 8)) {
			fw->conf_skip = 1;
		}
		break;
	}

	if (fw->conf_skip) {
		return 0;
	}

	if ((fw_size < size_min) || (fw_size > size_max)) {
		t_dev_err(dev, "FW upgrade: wrong file size - %Xh,\n",
			fw_size);
		t_dev_err(dev, "			shall be '%Xh <= x <= %Xh'\n",
			size_min, size_max);
		return -EFAULT;
	}

	return 0;
}

static int siw_hal_fw_size_check_post(struct device *dev, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max = touch_fw_size(ts);
	int required_size;
	u32 index = 0;
	u32 conf_idx_addr = chip->fw.conf_idx_addr;
	int ret = 0;

	if (chip->fw.conf_skip) {
		return 0;
	}

#if (S_CFG_DBG_IDX != 0)
	index = S_CFG_DBG_IDX;
	t_dev_warn(dev, "FW upgrade: conf_index fixed for debugging: %d\n", index);
#else
	ret = siw_hal_fw_rd_value(dev, conf_idx_addr, &index);
#endif
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - conf_index(%04Xh) read, %d\n",
			conf_idx_addr, ret);
		return ret;
	}
	if ((index < MIN_S_CONF_IDX) || (index > MAX_S_CONF_IDX)) {
		t_dev_err(dev, "FW upgrade: failed - wrong cfg index, %d\n", index);
		return -EFAULT;
	}
	t_dev_info(dev, "FW upgrade: conf_index: %d\n", index);

	required_size = fw_size_max + (NUM_C_CONF<<POW_C_CONF) + (index<<POW_S_CONF);
	if (fw_size < required_size) {
		t_dev_err(dev, "FW upgrade: wrong file size - %Xh < %Xh,\n",
			fw_size, required_size);
		return -EFAULT;
	}

	chip->fw.conf_index = index;

	return 0;
}
#else	/* __SIW_FW_TYPE_1 */
static int siw_hal_fw_upgrade_fw_post_quirk(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 chk_resp, data;
	int ret;

	/* Release CM3 core */
	ret = siw_hal_fw_wr_value(dev, reg->spr_rst_ctl, 0);
	if (ret < 0) {
		goto out;
	}

	/* Set Serial Dump Done */
	ret = siw_hal_fw_wr_value(dev, reg->spr_boot_ctl, 1);
	if (ret < 0) {
		goto out;
	}

	/* firmware boot done check */
	chk_resp = FLASH_BOOTCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, ~0,
				FW_POST_QUIRK_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_0),
				FW_POST_QUIRK_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - boot check(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: boot check done\n");

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf_quirk(struct device *dev,
			     u8 *fw_buf, int fw_size)
{
	return 0;
}

static int siw_hal_fw_size_check(struct device *dev, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max = touch_fw_size(ts);

	chip->fw.conf_index = 0;

	if ((fw_size != fw_size_max) &&
		(fw_size != (fw_size_max + FLASH_CONF_SIZE)))
	{
		t_dev_err(dev, "FW upgrade: wrong file size - %Xh,\n",
			fw_size);
		t_dev_err(dev, "            shall be '%Xh' or '%Xh + %Xh'\n",
			fw_size_max, fw_size_max, FLASH_CONF_SIZE);
		return -EFAULT;
	}

	return 0;
}

static int siw_hal_fw_size_check_post(struct device *dev, int fw_size)
{
	return 0;
}
#endif	/* __SIW_FW_TYPE_1 */

static int siw_hal_fw_compare(struct device *dev, u8 *fw_buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	struct siw_hal_ops_quirk *ops_quirk = &chip->ops_quirk;
	struct siw_hal_tc_version_bin *bin_ver;
	int fw_max_size = touch_fw_size(ts);
	u32 bin_ver_offset = 0;
	u32 bin_ver_ext_offset = 0;
	u32 bin_pid_offset = 0;
	u32 dev_major = 0;
	u32 dev_minor = 0;
	char pid[12] = {0, };
	u32 bin_major = 0;
	u32 bin_minor = 0;
//	u32 bin_raw = 0;
	u32 bin_raw_ext = 0;
	int bin_diff = 0;
	int update = 0;
	int boot_failed = !!(atomic_read(&chip->boot) == IC_BOOT_FAIL);
//	int ret = 0;

	if (fquirks->fwup_check) {
		update = fquirks->fwup_check(dev, fw_buf);
		if (update != -EAGAIN) {
			if (update < 0) {
				return update;
			}
			goto out;
		}
		update = 0;
	}

	if (ops_quirk->fwup_check) {
		update = ops_quirk->fwup_check(dev, fw_buf);
		if (update != -EAGAIN) {
			if (update < 0) {
				return update;
			}
			goto out;
		}
		update = 0;
	}

	if (!boot_failed) {
		if (fw->version_ext) {
			dev_major = fw->version_ext >> 8;
			dev_minor = fw->version_ext & 0xFF;
		} else {
			dev_major = fw->v.version.major;
			dev_minor = fw->v.version.minor;
		}

		if (!dev_major && !dev_minor){
			t_dev_err(dev, "fw can not be 0.0!! Check your panel connection!!\n");
			return 0;
		}
	}

	bin_ver_offset = *((u32 *)&fw_buf[BIN_VER_OFFSET_POS]);
	if (!bin_ver_offset) {
		t_dev_err(dev, "FW compare: zero ver offset\n");
		return -EINVAL;
	}

	if (chip->opt.f_ver_ext) {
		bin_ver_ext_offset = *((u32 *)&fw_buf[BIN_VER_EXT_OFFSET_POS]);
	} else {
		bin_ver_ext_offset = 0;
	}

	if (!boot_failed) {
		if ((fw->version_ext && !bin_ver_ext_offset) ||
			(!fw->version_ext && bin_ver_ext_offset)) {
			if (!ts->force_fwup) {
				t_dev_warn(dev,
					"FW compare: different version format, "
					"use force update %s",
					(fw->version_ext) ? "(ext)" : "");
				return -EINVAL;
			}
			bin_diff = 1;
		}
	}

	bin_pid_offset = *((u32 *)&fw_buf[BIN_PID_OFFSET_POS]);
	if (!bin_pid_offset) {
		t_dev_err(dev, "FW compare: zero pid offset\n");
		return -EINVAL;
	}

	if (((bin_ver_offset + 4) > fw_max_size) ||
		((bin_ver_ext_offset + 4) > fw_max_size) ||
		((bin_pid_offset + 8) > fw_max_size)) {
		t_dev_err(dev, "FW compare: invalid offset - ver %06Xh, ver_ext %06Xh pid %06Xh, max %06Xh\n",
			bin_ver_offset, bin_ver_ext_offset, bin_pid_offset, fw_max_size);
		return -EINVAL;
	}

	t_dev_dbg_fwup(dev, "ver %06Xh, ver_ext %06Xh, pid %06Xh\n",
			bin_ver_offset, bin_ver_ext_offset, bin_pid_offset);

	memcpy(pid, &fw_buf[bin_pid_offset], 8);
	t_dev_dbg_fwup(dev, "pid %s\n", pid);

	if (siw_hal_fw_check_pid(pid)) {
		t_dev_err(dev, "[fw-bin] invalid pid - \"%s\"\n", pid);
		return -EINVAL;
	}

	if (boot_failed) {
		update |= (1<<7);
		goto out;
	}

	bin_ver = (struct siw_hal_tc_version_bin *)&fw_buf[bin_ver_offset];
	bin_major = bin_ver->major;
	bin_minor = bin_ver->minor;

	if (bin_ver_ext_offset) {
		if (!bin_ver->ext) {
			t_dev_err(dev, "FW compare: (no ext flag in binary)\n");
			return -EINVAL;
		}

		memcpy(&bin_raw_ext, &fw_buf[bin_ver_ext_offset], sizeof(bin_raw_ext));
		bin_major = bin_raw_ext >> 8;
		bin_minor = bin_raw_ext & 0xFF;

		t_dev_info(dev,
			"FW compare: bin-ver: %08X (%s)(%d)\n",
			bin_raw_ext, pid, bin_diff);

		if (siw_hal_fw_chk_version_ext(bin_raw_ext,
					bin_ver->ext) < 0) {
			t_dev_err(dev, "FW compare: (invalid extension in binary)\n");
			return -EINVAL;
		}
	} else {
		t_dev_info(dev,
			"FW compare: bin-ver: %d.%02d (%s)(%d)\n",
			bin_major, bin_minor, pid, bin_diff);
	}

	if (fw->version_ext) {
		t_dev_info(dev, "FW compare: dev-ver: %08X (%s)\n",
				fw->version_ext, fw->product_id);
	} else {
		t_dev_info(dev, "FW compare: dev-ver: %d.%02d (%s)\n",
				dev_major, dev_minor, fw->product_id);
	}

	if (ts->force_fwup) {
		update |= (1<<0);
	} else {
		if (bin_major > dev_major) {
			update |= (1<<1);
		} else if (bin_major == dev_major) {
			if (bin_minor > dev_minor) {
				update |= (1<<2);
			}
		}
	}

	if (ts->force_fwup & FORCE_FWUP_SKIP_PID) {
		t_dev_warn(dev, "FW compare: skip pid check\n");
		goto out;
	}

	if (memcmp(pid, fw->product_id, 8)) {
		if (fw->invalid_pid) {
			t_dev_err(dev,
				"FW compare: bin-pid[%s], dev-pid invalid, halted (up %02X, fup %02X)\n",
				pid, update, ts->force_fwup);
			return -EINVAL;

		}

		t_dev_err(dev,
			"FW compare: bin-pid[%s] != dev-pid[%s], halted (up %02X, fup %02X)\n",
			pid, fw->product_id, update, ts->force_fwup);
		return -EINVAL;
	}

out:
	t_dev_info(dev,
		"FW compare: up %02X, fup %02X\n",
		update, ts->force_fwup);

	return update;
}

static int siw_hal_fw_upgrade_fw_pre(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ret;

	/* Reset CM3 core */
	ret = siw_hal_fw_wr_value(dev, reg->spr_rst_ctl, 2);
	if (ret < 0) {
		goto out;
	}

	/* Disable SRAM write protection */
	ret = siw_hal_fw_sram_wr_enable(dev, 1);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade_fw_post(struct device *dev, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dn_cmd, chk_resp, data;
	int ret;

	/* Enable SRAM write protection */
	ret = siw_hal_fw_sram_wr_enable(dev, 0);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_upgrade_fw_post_quirk(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_size_check_post(dev, fw_size);
	if (ret < 0) {
		goto out;
	}

	/* Firmware Download Start */
	dn_cmd = (FLASH_KEY_CODE_CMD << 16) | 1;
	ret = siw_hal_fw_wr_value(dev, reg->tc_flash_dn_ctl, dn_cmd);
	if (ret < 0) {
		goto out;
	}

	touch_msleep(ts->caps.hw_reset_delay);

	/* download check */
	chk_resp = FLASH_CODE_DNCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, 0xFFFF,
				FW_POST_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_1),
				FW_POST_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - code check(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}
	t_dev_info(dev, "FW upgrade: code check done\n");

out:
	return ret;
}

static int siw_hal_fw_upgrade_fw(struct device *dev,
				u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u8 *fw_data;
	int fw_size_max;
	int ret = 0;

	/*
	 * Stage 1-1 : download code data
	 */
	fw_size_max = touch_fw_size(ts);

	ret = siw_hal_fw_upgrade_fw_pre(dev);
	if (ret < 0) {
		goto out;
	}

	/*
	 * [Caution]
	 * The size for F/W upgrade is fw_size_max, not fw->size
	 * because the fw file can have config area.
	 */
	fw_data = fw_buf;
	ret = siw_hal_fw_upgrade_fw_core(dev, fw_data, fw_size_max);
	if (ret < 0) {
		goto out;
	}

	/*
	 * Stage 1-2: upgrade code data
	 */
	ret = siw_hal_fw_upgrade_fw_post(dev, fw_size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf_pre(struct device *dev, u32 *value)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data = 0;
	int ret = 0;

	ret = siw_hal_fw_rd_value(dev, reg->tc_confdn_base_addr, &data);
	if (ret < 0) {
		goto out;
	}

out:
	if (value)
		*value = data;

	return ret;
}

static int siw_hal_fw_upgrade_release_cm3(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ret;

	/* Release & Reset CM3 */
	ret = siw_hal_fw_wr_value(dev, reg->spr_rst_ctl, 1);

	return ret;
}

static int siw_hal_fw_upgrade_conf_post(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dn_cmd, chk_resp, data;
	int ret;

	/* Conf Download Start */
	dn_cmd = (FLASH_KEY_CONF_CMD << 16) | 2;
	ret = siw_hal_fw_wr_value(dev, reg->tc_flash_dn_ctl, dn_cmd);
	if (ret < 0) {
		goto out;
	}

	/* Conf check */
	chk_resp = FLASH_CONF_DNCHK_VALUE_TYPE_X;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, 0xFFFF,
				CONF_POST_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_2),
				CONF_POST_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - conf check(%Xh), %X\n",
			chk_resp, data);
		ret = -EPERM;
		goto out;
	}

	t_dev_info(dev, "FW upgrade: conf check done\n");

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf(struct device *dev,
			     u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max;
	u32 conf_dn_addr;
	u32 data;
	int ret;

	fw_size_max = touch_fw_size(ts);

	/*
	 * Stage 2-1: download config data
	 */
	ret = siw_hal_fw_upgrade_conf_pre(dev, &data);
	if (ret < 0) {
		goto out;
	}

	conf_dn_addr = ((data >> 16) & 0xFFFF);

	t_dev_dbg_fwup(dev, "FW upgrade: conf_dn_addr %04Xh (%08Xh)\n",
		conf_dn_addr, data);
#if 0
	if (conf_dn_addr >= (0x1200) || conf_dn_addr < (0x8C0)) {
		t_dev_err(dev, "FW upgrade: failed - conf base invalid\n");
		ret = -EPERM;
		goto out;
	}
#endif

	/* C_CFG */
	ret = siw_hal_fw_upgrade_conf_core(dev, conf_dn_addr,
				(u8 *)&fw_buf[fw_size_max], FLASH_CONF_SIZE_TYPE_X);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_upgrade_conf_quirk(dev, fw_buf, fw_size);
	if (ret < 0) {
		goto out;
	}

	/*
	 * Stage 2-2: upgrade config data
	 */
	ret = siw_hal_fw_upgrade_conf_post(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade(struct device *dev,
				u8 *fw_buf, int fw_size, int retry)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	struct siw_hal_ops_quirk *ops_quirk = &chip->ops_quirk;
	int fw_size_max;
	u32 include_conf;
	int ret = 0;

	t_dev_info(dev, "===== FW upgrade: start (%d) =====\n", retry);

	if (fquirks->fwup_upgrade) {
		ret = fquirks->fwup_upgrade(dev, fw_buf, fw_size, retry);
		if (ret < 0) {
			goto out;
		}
		goto out_done;
	}

	if (ops_quirk->fwup_upgrade) {
		ret = ops_quirk->fwup_upgrade(dev, fw_buf, fw_size, retry);
		if (ret < 0) {
			goto out;
		}
		goto out_done;
	}

	fw_size_max = touch_fw_size(ts);

	ret = siw_hal_fw_size_check(dev, fw_size);
	if (ret < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_CHK);
		goto out;
	}

	include_conf = (chip->fw.conf_skip) ? 0 : !!(fw_size > fw_size_max);
	t_dev_info(dev, "FW upgrade:%s include conf data\n",
			(include_conf) ? "" : " not");

	t_dev_dbg_fwup(dev, "FW upgrade: fw size %08Xh, fw_size_max %08Xh\n",
			fw_size, fw_size_max);

	ret = siw_hal_fw_upgrade_fw(dev, fw_buf, fw_size);
	if (ret < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_CODE);
		goto out;
	}

	if (include_conf) {
		ret = siw_hal_fw_upgrade_conf(dev, fw_buf, fw_size);
		if (ret < 0) {
			siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_CFG);
			goto out;
		}
	}

	ret = siw_hal_fw_upgrade_release_cm3(dev);
	if (ret < 0) {
		goto out;
	}

out_done:
	t_dev_info(dev, "===== FW upgrade: done (%d) =====\n", retry);

out:
	return ret;
}

static int siw_hal_fw_do_get_fw_abs(const struct firmware **fw_p,
				const char *name,
                struct device *dev)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct firmware *fw = NULL;
	struct file *filp = NULL;
	char *buf = NULL;
	loff_t size;
	int rd_size;
	int ret = 0;

	fw = kzalloc(sizeof(*fw), GFP_KERNEL);
	if (fw == NULL) {
		dev_err(dev, "can't allocate fw(struct firmware)\n");
		return -ENOMEM;
	}

	filp = filp_open(name, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		dev_err(dev, "can't open %s\n", name);
		kfree(fw);
		return (int)PTR_ERR(filp);
	}

	size = vfs_llseek(filp, 0, SEEK_END);
	if (size < 0)	 {
		t_dev_err(dev, "invalid file size, %d\n", (int)size);
		ret = -EINVAL;
		goto out;
	}

	buf = kzalloc((size_t)size, GFP_KERNEL);
	if (buf == NULL) {
		t_dev_err(dev, "can't allocate firm buf\n");
		ret = -ENOMEM;
		goto out;
	}

	rd_size = kernel_read(filp, 0,
				(char *)buf,
				(unsigned long)size);
	if (rd_size != (int)size) {
		t_dev_err(dev, "can't read[%d], %d\n",
			(int)size, (int)rd_size);
		ret = (rd_size < 0) ? rd_size : -EFAULT;
		goto out;
	}

	fw->data = buf;
	fw->size = size;

	if (fw_p) {
		*fw_p = fw;
	}

	filp_close(filp, NULL);

	return 0;

out:
	if (buf)
		kfree(buf);

	if (fw)
		kfree(fw);

	filp_close(filp, NULL);

	return ret;
}

static int siw_hal_fw_do_get_file(const struct firmware **fw_p,
				const char *name,
                struct device *dev,
                int abs_path)
{
	if (abs_path) {
		return siw_hal_fw_do_get_fw_abs(fw_p, name, dev);
	}

	return request_firmware(fw_p, name, dev);
}

static int siw_hal_fw_get_file(const struct firmware **fw_p,
				char *fwpath,
				struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	const struct firmware *fw = NULL;
	char *src_path;
	int src_len;
	int abs_path = 0;
	int ret = 0;

	if (ts->test_fwpath[0]) {
		src_path = (char *)ts->test_fwpath;
	} else if (ts->def_fwcnt) {
		src_path = (char *)ts->def_fwpath[0];
	} else {
		t_dev_err(dev, "no target fw defined\n");
		ret = -ENOENT;
		goto out;
	}

	/*
	 * Absolute path option
	 * ex) echo {root}/.../target_fw_img > fw_upgrade
	 *          ++++++~~~~~~~~~~~~~~~~~~
	 *          flag  |
	 *                absolute path
	 */
	src_len = strlen(src_path);
	if (strncmp(src_path, "{root}", 6) == 0) {
		abs_path = 1;
		src_path += 6;
		src_len -= 6;
	}
	chip->fw_abs_path = abs_path;

	strncpy(fwpath, src_path, src_len);
	fwpath[src_len] = 0;

	t_dev_info(dev, "target fw: %s (%s)\n",
		fwpath,
		(abs_path) ? "abs" : "rel");

	ret = siw_hal_fw_do_get_file(&fw,
				(const char *)fwpath,
				dev, abs_path);
	if (ret < 0) {
		if (ret == -ENOENT) {
			t_dev_err(dev, "can't find fw: %s\n", fwpath);
		} else {
			t_dev_err(dev, "can't %s fw: %s, %d\n",
				(abs_path) ? "read" : "request",
				fwpath, ret);
		}
		goto out;
	}

	if (fw_p) {
		*fw_p = fw;
	}

out:
	return ret;
}

static void siw_hal_fw_release_firm(struct device *dev,
			const struct firmware *fw)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;

	if (fw == NULL) {
		return;
	}

	if (chip->fw_abs_path) {
		chip->fw_abs_path = 0;
		kfree(fw->data);
		kfree(fw);
		return;
	}

	release_firmware(fw);
}

/*
 * FW upgrade option
 *
 * 1. If TOUCH_USE_FW_BINARY used
 * 1-1 Default upgrade (through version comparison)
 *     do upgarde using binary header link
 * 1-2 echo {bin} > fw_upgrade
 *     do force-upgrade using binary header link (same as 1-1)
 * 1-3 echo /.../fw_img > fw_upgrade
 *     do force-upgrade using request_firmware (relative path)
 * 1-4 echo {root}/.../fw_img > fw_upgrade
 *     do force-upgrade using normal file open control (absolute path)
 *
 * 2. Else
 * 2-1 Default upgrade (through version comparison)
 *     do upgarde using request_firmware (relative path)
 * 2-2 echo /.../fw_img > fw_upgrade
 *     do force-upgrade using request_firmware (relative path)
 * 2-3 echo {root}/.../fw_img > fw_upgrade
 *     do force-upgrade using normal file open control (absolute path)
 */
static int siw_hal_upgrade_not_allowed(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (atomic_read(&chip->boot) == IC_BOOT_FAIL) {
		return 0;
	}

	if (chip->lcd_mode != LCD_MODE_U3) {
		t_dev_warn(dev, "FW upgrade: not U3 mode, %s(%d)\n",
			siw_lcd_driving_mode_str(chip->lcd_mode),
			chip->lcd_mode);
		return 1;
	}

	if (siw_hal_access_not_allowed(dev, "FW_Upgrade", HAL_ACCESS_CHK_SKIP_INIT)) {
		return 1;
	}

	return 0;
}

static int siw_hal_upgrade_pre(struct device * dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ctrl = chip->tc_cmd_table[LCD_MODE_STOP];
	u32 rdata;
	int ret = 0;

	if ((ctrl < 0) || !touch_mode_allowed(ts, LCD_MODE_STOP)) {
		goto out;
	}

	/*
	 * TC_STOP before fw upgrade
	 * to avoid unexpected IRQ drop by internal watchdog
	 */
	rdata = reg->tc_drive_ctl;

	ret = siw_hal_write_value(dev, rdata, ctrl);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: TC stop(%04Xh, 0x%08X) failed\n",
				rdata, ctrl);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: TC stop(%04Xh, 0x%08X)\n",
			rdata, ctrl);

	rdata = chip->drv_delay + hal_dbg_delay(chip, HAL_DBG_DLY_TC_DRIVING_1);
	touch_msleep(rdata);

out:
	return ret;
}

static int siw_hal_upgrade(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fw_bin *fw_bin = NULL;
	const struct firmware *fw = NULL;
	char *fwpath = NULL;
	u8 *fw_buf = NULL;
	int fw_max_size = touch_fw_size(ts);
	int fw_size = 0;
	int fw_up_binary = 0;
	int i = 0;
	int ret_val = 0;
	int ret = 0;

	siw_hal_set_fwup_status(chip, FWUP_STATUS_BUSY);

	chip->fw_abs_path = 0;

	if (siw_hal_upgrade_not_allowed(dev)) {
		t_dev_warn(dev, "FW upgrade: not granted\n");
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_OP);
		return EACCES;
	}

	t_dev_info(dev, "fw type: %s\n", FW_TYPE_STR);

	fwpath = touch_getname();
	if (fwpath == NULL) {
		t_dev_err(dev, "failed to allocate name buffer - fwpath\n");
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_OP);
		return -ENOMEM;
	}

	if (touch_flags(ts) & TOUCH_USE_FW_BINARY) {
		fw_up_binary = 1;

		if (ts->force_fwup & FORCE_FWUP_SYS_STORE) {
			switch (ts->test_fwpath[0]) {
			case 0:
				/* fall through */
			case ' ':	/* ignore space */
				break;

			default:
				/* if target string is not "{bin}" */
				if (strncmp(ts->test_fwpath, "{bin}", 5) != 0) {
					fw_up_binary = 0;
				}
				break;
			}
		}
	}

	if (fw_up_binary) {
		t_dev_info(dev, "getting fw from binary header data\n");
		fw_bin = touch_fw_bin(ts);
		if (fw_bin != NULL) {
			fw_buf = fw_bin->fw_data;
			fw_size = fw_bin->fw_size;
		} else {
			t_dev_warn(dev, "empty fw info\n");
		}
	} else {
		t_dev_info(dev, "getting fw from file\n");
		ret = siw_hal_fw_get_file(&fw, fwpath, dev);
		if (ret < 0) {
			siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_OPEN);
			goto out;
		}
		fw_buf = (u8 *)fw->data;
		fw_size = (int)fw->size;
	}

//	ret = -EINVAL;
	ret = -EPERM;

	if ((fw_buf == NULL) || !fw_size) {
		t_dev_err(dev, "invalid fw info\n");
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_CHK);
		goto out_fw;
	}

	if (fw_size < fw_max_size) {
		t_dev_err(dev, "invalid fw size: %Xh < %Xh\n",
			fw_size, fw_max_size);
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_CHK);
		goto out_fw;
	}

	t_dev_info(dev, "fw size: %d\n", fw_size);

	ret_val = siw_hal_fw_compare(dev, fw_buf);
	if (ret_val < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_CHK);
		ret = ret_val;
		goto out_fw;
	}

	if (!ret_val) {
		goto out_fw;
	}

	ret_val = siw_hal_upgrade_pre(dev);
	if (ret_val < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_IO);
		ret = ret_val;
		goto out_fw;
	}

	touch_msleep(100);

	siw_hal_disable_flash_wp(dev);
	for (i = 0; (i < 2) && (ret < 0); i++) {
		ret = siw_hal_fw_upgrade(dev, fw_buf, fw_size, i);
	}
	siw_hal_enable_flash_wp(dev);

out_fw:
	siw_hal_fw_release_firm(dev, fw);

out:
	if (ret < 0) {
		siwmon_submit_ops_step_chip_wh_name(dev, "%s - FW upgrade halted",
				touch_chip_name(ts), ret);
		if (siw_hal_get_fwup_status(chip) == FWUP_STATUS_BUSY) {
			siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_IO);
		}
	} else {
		siwmon_submit_ops_step_chip_wh_name(dev, "%s - FW upgrade done",
				touch_chip_name(ts), ret);
		siw_hal_set_fwup_status(chip, FWUP_STATUS_OK);
	}

	touch_putname(fwpath);

	return ret;
}

#define SIW_LDO_CTL_T4		0x006
#define SIW_OSC_CTL_T4		0xFE1

static int siw_hal_clock_type_4(struct device *dev, bool onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int is_spi = !!(touch_bus_type(ts) == BUS_IF_SPI);

	if (onoff) {
		atomic_set(&ts->state.sleep, IC_NORMAL);
		if (is_spi) {
			siw_hal_write_value(dev, SIW_OSC_CTL_T4, 1);
			siw_touch_irq_control(ts->dev, INTERRUPT_ENABLE);
		} else {
			//need to be considered more!!
			t_dev_info(dev, "siw_hal_clock(4) -> reset(sync)\n");
			siw_hal_reset_ctrl(dev, HW_RESET_SYNC);
		}
	} else {
		siw_touch_irq_control(ts->dev, INTERRUPT_DISABLE);
		siw_hal_write_value(dev, SIW_LDO_CTL_T4, 0);
		siw_hal_write_value(dev, SIW_OSC_CTL_T4, 0);
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
	}

	t_dev_info(dev, "siw_hal_clock(4) -> %s\n",
		(onoff) ? "ON" : "OFF");

	return 0;
}

#define SIW_OSC_CTL_T2		0xFE1
#define SIW_CLK_CTL_T2		0xFE2

struct siw_clock_setup {
	const int count;
	const u32 *data;
};

static int siw_hal_clock_osc_base(struct device *dev,
			const struct siw_clock_setup *setup, int onoff)
{
	u32 value;
	int i;

	if (setup == NULL) {
		t_dev_info(dev, "osc %s\n",
			(onoff) ? "on" : "off");
		siw_hal_write_value(dev, SIW_OSC_CTL_T2, !!onoff);
		return 0;
	}

	for (i = 0; i < setup->count; i++) {
		value = setup->data[i];
		t_dev_info(dev, "osc %s : %Xh\n",
			(onoff) ? "on" : "off", value);
		siw_hal_write_value(dev, SIW_OSC_CTL_T2, value);
		touch_msleep(1);
	}

	return 0;
}

static const u32 osc_setup_data_on_base[3] = {
	0x01, 0x03, 0x07,
};

static const u32 osc_setup_data_off_base[3] = {
	0x03, 0x01, 0x00,
};

static const struct siw_clock_setup osc_setup_spi_on_base = {
	.count = 3,
	.data = osc_setup_data_on_base,
};

static const struct siw_clock_setup osc_setup_spi_off_base = {
	.count = 3,
	.data = osc_setup_data_off_base,
};

static const struct siw_clock_setup osc_setup_i2c_on_base = {
	.count = 1,
	.data = &osc_setup_data_on_base[2],
};

static const struct siw_clock_setup osc_setup_i2c_off_base = {
	.count = 1,
	.data = &osc_setup_data_off_base[0],
};

static int siw_hal_clock_type_2_osc(struct device *dev, bool onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int is_spi = !!(touch_bus_type(ts) == BUS_IF_SPI);
	const struct siw_clock_setup *osc_setup = NULL;

	switch (touch_chip_type(ts)) {
	case CHIP_SW49501:
		if (onoff) {
			osc_setup = (is_spi) ? &osc_setup_spi_on_base : \
									&osc_setup_i2c_on_base;
		} else {
			osc_setup = (is_spi) ? &osc_setup_spi_off_base : \
									&osc_setup_i2c_off_base;
		}
		break;
	}

	if (onoff) {
		/* I2C needs touch reset. */
		if (!is_spi) {
			return 0;
		}
	}

	siw_hal_clock_osc_base(dev, osc_setup, onoff);

	return 0;
}

static int siw_hal_clock_type_2_clk(struct device *dev, bool onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int is_spi = !!(touch_bus_type(ts) == BUS_IF_SPI);

	if (!is_spi) {
		return 0;
	}

	siw_hal_write_value(dev, SIW_CLK_CTL_T2, !!onoff);

	return 0;
}

static int siw_hal_clock_type_2(struct device *dev, bool onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (onoff) {
		/*
		 * [Notice]
		 * I2C needs touch reset.
		 */
		siw_hal_clock_type_2_osc(dev, onoff);
		siw_hal_clock_type_2_clk(dev, onoff);
		atomic_set(&ts->state.sleep, IC_NORMAL);
	} else {
		if (chip->lcd_mode == LCD_MODE_U0) {
			siw_hal_clock_type_2_clk(dev, onoff);
			siw_hal_clock_type_2_osc(dev, onoff);
			atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
		}
	}

	t_dev_info(dev, "siw_hal_clock(2) -> %s\n",
		(onoff) ? "ON" : (!chip->lcd_mode) ? "OFF" : "SKIP");

	return 0;
}

static int siw_hal_clock_type_1(struct device *dev, bool onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_cmd_write(dev, CMD_ENA);

	if (onoff) {
		siw_hal_cmd_write(dev, CMD_OSC_ON);
		siw_hal_cmd_write(dev, CMD_CLK_ON);
		atomic_set(&ts->state.sleep, IC_NORMAL);
	} else {
		if (chip->lcd_mode == LCD_MODE_U0) {
			siw_hal_cmd_write(dev, CMD_CLK_OFF);
			siw_hal_cmd_write(dev, CMD_OSC_OFF);
			atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
		}
	}

	siw_hal_cmd_write(dev, CMD_DIS);

	t_dev_info(dev, "siw_hal_clock(1) -> %s\n",
		(onoff) ? "ON" : (!chip->lcd_mode) ? "OFF" : "SKIP");

	return 0;
}

static int siw_hal_clock(struct device *dev, bool onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int state = atomic_read(&ts->state.sleep);
	int ret = 0;

	if (onoff) {
		if (state != IC_DEEP_SLEEP) {
			return 0;
		}
	} else {
		if (state == IC_DEEP_SLEEP) {
			t_dev_info(dev, "already sleep state\n");
			return 0;
		}
	}

	siw_touch_sys_osc(dev, onoff);

	switch (chip->opt.t_clock) {
	case 4:
		ret = siw_hal_clock_type_4(dev, onoff);
		break;
	case 2:
		ret = siw_hal_clock_type_2(dev, onoff);
		break;
	case 1:
		ret = siw_hal_clock_type_1(dev, onoff);
		break;
	default:
		atomic_set(&ts->state.sleep,
			(onoff) ? IC_NORMAL : IC_DEEP_SLEEP);
		t_dev_info(dev, "sleep state -> %s\n",
			(onoff) ? "IC_NORMAL" : "IC_DEEP_SLEEP");
		break;
	}

	return ret;
}

static int siw_hal_tc_con_type_g(struct device *dev, u32 addr, int value, char *name)
{
	int ret = 0;

	ret = siw_hal_write_value(dev, addr, value);
	if (ret < 0) {
		t_dev_err(dev, "failed to set %s[%04Xh], %d\n",
			name, addr, ret);
		goto out;
	}

	t_dev_info(dev, "%s[%04Xh]: %s(%08Xh)\n",
		name, addr,
		(value & 0x1) ? "ON" : "OFF",
		value);

out:
	return ret;
}

static int siw_hal_tc_con_glove(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = reg->glove_en;
	int value = atomic_read(&ts->state.glove);
	int ret = 0;

	if (chip->opt.f_glove_en) {
		ret = siw_hal_tc_con_type_g(dev, addr, value, "glove_en");
	}

	return ret;
}

static int siw_hal_tc_con_grab(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = reg->grab_en;
	int value = atomic_read(&ts->state.grab);
	int ret = 0;

	if (chip->opt.f_grab_en) {
		ret = siw_hal_tc_con_type_g(dev, addr, value, "grab_en");
	}

	return ret;
}

static int siw_hal_tc_con(struct device *dev, u32 code, void *param)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int ret = 0;

	if (siw_hal_tc_not_allowed(dev, "tc con not allowed")) {
		return 0;
	}

	switch (code) {
	case TCON_GLOVE:
		ret = siw_hal_tc_con_glove(dev);
		break;
	case TCON_GRAB:
		ret = siw_hal_tc_con_grab(dev);
		break;
	}

	return ret;
}

static void siw_hal_chk_dbg_report(struct device *dev, u32 status, int irq)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = reg->tc_ic_status;
	u32 irq_type = siw_tc_sts_irq_type(status);
	u32 ic_debug[4];
	u32 debug_info = 0;
	u32 debug_len = 0;
	u32 debug_type = 0;
	int ret = 0;

	if (!chip->opt.f_dbg_report) {
		return;
	}

	switch (irq_type) {
	case TC_STS_IRQ_TYPE_ABNORMAL:
	case TC_STS_IRQ_TYPE_DEBUG:
		break;
	default:
		return;
	}

	addr += ((0x100>>2) - 2);

	ret = siw_hal_reg_read(dev, addr, ic_debug, sizeof(ic_debug));
	if (ret < 0) {
		return;
	}

	debug_info = ic_debug[3];
	debug_len = ((debug_info>>24) & 0xFF);
	debug_type = (debug_info & ((1<<24)-1));

	t_dev_info(dev,
			"[%d] ic debug: s %08Xh / m %Xh, l %Xh, t %Xh (%08Xh)\n",
			irq, status, irq_type, debug_len, debug_type, debug_info);

	t_dev_info(dev,
		"[%d] ic debug: log %08Xh %08Xh %08Xh\n",
		irq, ic_debug[0], ic_debug[1], ic_debug[2]);
}

static int siw_hal_tc_driving_post(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 addr[4] = { 0, };
	u32 rdata;
	int ret = 0;

	switch (chip->opt.t_chk_mipi) {
	case 1:
		if (chip->driving_mode != LCD_MODE_U3) {
			break;
		}

		rdata = 0;
		switch (fw->v.version.major) {
		case 0 :
		case 1 :
			rdata = !!(fw->v.version.minor > 14);
			break;
		}

		addr[0] = (rdata) ? 0x284 : 0;
		break;
	default:
		goto out;
	}

	if (addr[0]) {
		rdata = 0;
		ret = siw_hal_read_value(dev, addr[0], &rdata);
		if (ret < 0) {
			goto out;
		}
		if (rdata) {
			t_dev_err(dev, "!! [Warning] !!\n");
			t_dev_err(dev, "   Check MIPI script (%d)\n", rdata);
			t_dev_err(dev, "   Maybe, MIPI(VIDEO) vs. FW(CMD) or vice versa.\n");
		}
	}

out:
	return 0;
}

static int siw_hal_tc_driving_cmd(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int ctrl = 0;

	if ((mode < LCD_MODE_U0) || (mode >= LCD_MODE_MAX)) {
		t_dev_err(dev, "invalid mode, %d\n", mode);
		return -EINVAL;
	}

	ctrl = chip->tc_cmd_table[mode];
	if (ctrl < 0) {
		t_dev_err(dev, "%s(%d) not granted\n",
			siw_lcd_driving_mode_str(mode), mode);
		return -ESRCH;
	}

	chip->driving_ctrl = ctrl;

	return ctrl;
}

static int siw_hal_tc_driving_quirk(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 rdata = 0;
	int ctrl = 0;
	int ret = 0;

	switch (chip->opt.t_tc_quirk) {
	case 1:
		if (chip->driving_mode == LCD_MODE_STOP) {
			break;
		}

		if (mode == LCD_MODE_STOP) {
			break;
		}

		if (mode == chip->driving_mode) {
			break;
		}

		ctrl = siw_hal_tc_driving_cmd(dev, LCD_MODE_STOP);
		if (ctrl < 0) {
			return ctrl;
		}

		rdata = reg->tc_drive_ctl;

		ret = siw_hal_write_value(dev, rdata, ctrl);
		if (ret < 0) {
			t_dev_err(dev, "driving quirk - stop(0x%04X, 0x%08X) before new command(%s) failed\n",
					rdata, ctrl,
					siw_lcd_driving_mode_str(mode));
			return ret;
		}

		t_dev_info(dev, "driving quirk - stop(0x%04X, 0x%08X) before new command(%s)\n",
				rdata, ctrl,
				siw_lcd_driving_mode_str(mode));

		rdata = chip->drv_delay + hal_dbg_delay(chip, HAL_DBG_DLY_TC_DRIVING_1);
		touch_msleep(rdata);
		break;
	}

	return 0;
}

static int siw_hal_tc_driving(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 tc_status = 0;
	u32 running_status = 0;
	u32 rdata;
	int ctrl = 0;
	int re_init = 0;
	int ret = 0;

	if (siw_hal_tc_not_allowed(dev, "tc driving not allowed")) {
		return 0;
	}

	if (touch_mode_not_allowed(ts, mode)) {
		return -EPERM;
	}

	if (atomic_read(&ts->recur_chk)) {
		/* keep the last value for retry case */
		mode = chip->driving_mode;
		t_dev_info(dev, "keep the last mode(%d) for retry\n", mode);
	}

	siw_hal_tc_driving_quirk(dev, mode);

	ctrl = siw_hal_tc_driving_cmd(dev, mode);
	if (ctrl < 0) {
		return ctrl;
	}

	chip->driving_mode = mode;

	/* swipe set */
	ret = siw_hal_swipe_mode(dev, mode);
	if (ret < 0) {
		t_dev_warn(dev, "swipe mode err, %d", ret);
	}

	if ((mode == LCD_MODE_U0) ||
		(mode == LCD_MODE_U2)) {
		touch_msleep(chip->drv_opt_delay);
	}

	touch_msleep(hal_dbg_delay(chip, HAL_DBG_DLY_TC_DRIVING_0));

	t_dev_info(dev, "current driving mode is %s\n",
			siw_lcd_driving_mode_str(mode));

	rdata = siw_hal_get_subdisp_sts(dev);
	t_dev_info(dev, "DDI Display Mode[%04Xh] = 0x%08X\n",
			reg->spr_subdisp_status, rdata);

	rdata = reg->tc_drive_ctl;

	ret = siw_hal_write_value(dev, rdata, ctrl);
	if (ret < 0) {
		t_dev_err(dev, "TC Driving[%04Xh](0x%08X) failed, %d\n",
				rdata, ctrl, ret);
		return ret;
	}
	t_dev_info(dev, "TC Driving[%04Xh] wr 0x%08X\n",
			rdata, ctrl);

	rdata = chip->drv_delay + hal_dbg_delay(chip, HAL_DBG_DLY_TC_DRIVING_1);
	touch_msleep(rdata);
	t_dev_dbg_base(dev, "waiting %d msecs\n", rdata);

	if (siw_touch_boot_mode_tc_check(dev)) {
		goto out;
	}

	if (mode == LCD_MODE_U3_PARTIAL) {
		goto out;
	}

	siw_hal_tc_driving_post(dev);

	ret = siw_hal_read_value(dev,
				reg->tc_status,
				&tc_status);
	if (ret < 0) {
		t_dev_err(dev, "failed to get tc_status\n");
		atomic_set(&ts->recur_chk, 0);
		return ret;
	}

	siw_hal_chk_dbg_report(dev, tc_status, 0);

	running_status = siw_tc_sts_running_sts(tc_status);

	re_init = 0;
	if (mode == LCD_MODE_STOP) {
		re_init = !!running_status;
	} else {
		if (!running_status ||
			(running_status == 0x10) ||
			(running_status == 0x0F)){
			re_init = 1;
		}
	}

	if (re_init) {
		int delay = ts->caps.hw_reset_delay + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_2);

		rdata = siw_hal_get_subdisp_sts(dev);

		if (ts->is_charger || atomic_read(&ts->recur_chk)) {
			t_dev_err(dev, "command failed: mode %d, tc_status %08Xh, DDI %08Xh\n",
				mode, tc_status, rdata);
			atomic_set(&ts->recur_chk, 0);
			return -EFAULT;
		}

		t_dev_err(dev, "command missed: mode %d, tc_status %08Xh, DDI %08Xh\n",
			mode, tc_status, rdata);

		atomic_set(&ts->recur_chk, 1);

		ret = siw_hal_reinit(dev, 1, delay, 1, siw_hal_init);
		if (ret < 0) {
			return ret;
		}
	} else {
		t_dev_info(dev, "command done: mode %d, running_sts %02Xh\n",
			mode, running_status);
	}

out:
	siw_hal_tc_con_glove(dev);
	siw_hal_tc_con_grab(dev);

	atomic_set(&ts->recur_chk, 0);

	return 0;
}

static void siw_hal_deep_sleep(struct device *dev)
{
	t_dev_info(dev, "deep sleep\n");

	siw_hal_tc_driving(dev, LCD_MODE_STOP);
	siw_hal_clock(dev, 0);
}

enum {
	LPWG_SET_SKIP	= -1,
};

struct lpwg_mode_ctrl {
	int clk;
	int qcover;
	int lpwg;
	int lcd;
	int sleep;
};

static void __t_dev_lpwg_info(struct device *dev, int lcd_mode,
				char *label, char *title, char *str)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int drv_mode = chip->driving_mode;
	char *_str = (str) ? str : "";

	if ((drv_mode == lcd_mode) || (lcd_mode == LPWG_SET_SKIP)) {
		t_dev_info(dev, "lpwg %s: %s(%d) %s\n",
			label, title,
			drv_mode,
			_str);
		return;
	}

	t_dev_info(dev, "lpwg %s: %s(%d -> %d) %s\n",
		label, title,
		drv_mode, lcd_mode,
		_str);
}

static void t_dev_lpwg_suspend_info(struct device *dev, int lcd_mode,
				char *title, char *str)
{
	__t_dev_lpwg_info(dev, lcd_mode, "suspend", title, str);
}

static void t_dev_lpwg_resume_info(struct device *dev, int lcd_mode,
				char *title, char *str)
{
	__t_dev_lpwg_info(dev, lcd_mode, "resume", title, str);
}

static void siw_hal_lpwg_ctrl_init(struct lpwg_mode_ctrl *ctrl)
{
	ctrl->clk = LPWG_SET_SKIP;
	ctrl->qcover = LPWG_SET_SKIP;
	ctrl->lpwg = LPWG_SET_SKIP;
	ctrl->lcd = LPWG_SET_SKIP;
	ctrl->sleep = LPWG_SET_SKIP;
}

static int siw_hal_lpwg_ctrl(struct device *dev,
				struct lpwg_mode_ctrl *ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	if (ctrl->clk != LPWG_SET_SKIP) {
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
			siw_hal_clock(dev, ctrl->clk);
		}
	}

	if (ctrl->qcover != LPWG_SET_SKIP) {
		ret = siw_hal_tci_area_set(dev, ctrl->qcover);
		if (ret < 0) {
			goto out;
		}
	}

	if (ctrl->lpwg != LPWG_SET_SKIP) {
		ret = siw_hal_lpwg_control(dev, ctrl->lpwg);
		if (ret < 0) {
			goto out;
		}
	}

	if (ctrl->lcd != LPWG_SET_SKIP) {
		ret = siw_hal_tc_driving(dev, ctrl->lcd);
	}

	if (ctrl->sleep != LPWG_SET_SKIP) {
		siw_hal_deep_sleep(dev);
	}

out:
	return ret;
}

static int siw_hal_lpwg_ctrl_skip(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	t_dev_info(dev, "skip lpwg_mode\n");

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		siw_hal_clock(dev, 1);
	}

	siw_hal_debug_tci(dev);
	siw_hal_debug_swipe(dev);

	return 0;
}

static int siw_hal_lpwg_mode_suspend(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct lpwg_mode_ctrl ctrl;
//	int mode_allowed_partial = chip->mode_allowed_partial;
	int mode_allowed_qcover = chip->mode_allowed_qcover;
	int qcover_near = (ts->lpwg.qcover == HOLE_NEAR);
	int lcd_mode = chip->lcd_mode;
	int changed = 0;
	int ret = 0;

	siw_hal_lpwg_ctrl_init(&ctrl);

	t_dev_info(dev, "lpwg suspend: mode %d, screen %d\n",
			ts->lpwg.mode, ts->lpwg.screen);

	if (ts->role.mfts_lpwg) {
		ctrl.lpwg = LPWG_DOUBLE_TAP,
		ctrl.lcd = chip->lcd_mode;

		t_dev_lpwg_suspend_info(dev, ctrl.lcd, "mfts_lpwg", NULL);
		goto out_con;
	}

	if (ts->lpwg.mode == LPWG_NONE) {
		if (ts->lpwg.screen) {
			ctrl.clk = 1;

			t_dev_lpwg_suspend_info(dev, ctrl.lcd, "mode", NULL);
			goto out_con;
		}
	}

	if (ts->lpwg.screen) {
		t_dev_lpwg_suspend_info(dev, ctrl.lcd, "screen", NULL);
		siw_hal_lpwg_ctrl_skip(dev);
		goto out;
	}

#if defined(__SIW_CONFIG_PROX_ON_SUSPEND)
	if (ts->lpwg.sensor == PROX_NEAR) {
		ctrl.sleep = 1;
		t_dev_lpwg_suspend_info(dev, ctrl.lcd, "sensor", NULL);
		goto out_con;
	}
#endif

	if (mode_allowed_qcover) {
		if (qcover_near) {
			/* knock on/code disable */
			ctrl.clk = 1;
			ctrl.qcover = QUICKCOVER_CLOSE;
			ctrl.lpwg = ts->lpwg.mode;
			ctrl.lcd = lcd_mode;

			t_dev_lpwg_suspend_info(dev, ctrl.lcd, "qcover", NULL);
			goto out_con;
		}
	}

	/* knock on/code */
	ctrl.clk = 1;
	if (mode_allowed_qcover) {
		ctrl.qcover = QUICKCOVER_OPEN;
	}
	ctrl.lpwg = ts->lpwg.mode;

	lcd_mode = chip->lcd_mode;
	if ((ctrl.lpwg == LPWG_NONE) && !chip->swipe.mode) {
		ctrl.sleep = 1;
		lcd_mode = LPWG_SET_SKIP;
	}

	ctrl.lcd = lcd_mode;

	t_dev_lpwg_suspend_info(dev, ctrl.lcd, "default",
		(ctrl.sleep == 1) ? "(sleep)" : "");

out_con:
	ret = siw_hal_lpwg_ctrl(dev, &ctrl);
	changed = 1;

out:
	t_dev_info(dev, "lpwg suspend(%d, %d): lcd_mode %d, driving_mode %d\n",
			changed, ret,
			chip->lcd_mode, chip->driving_mode);

	return ret;
}

static int siw_hal_lpwg_mode_resume(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct lpwg_mode_ctrl ctrl;
	int mode_allowed_partial = chip->mode_allowed_partial;
	int mode_allowed_qcover = chip->mode_allowed_qcover;
	int qcover_near = (ts->lpwg.qcover == HOLE_NEAR);
	int lcd_mode = chip->lcd_mode;
	int changed = 0;
	int ret = 0;

	siw_hal_lpwg_ctrl_init(&ctrl);

	t_dev_info(dev, "lpwg resume: mode %d, screen %d\n",
			ts->lpwg.mode, ts->lpwg.screen);

	siw_touch_report_all_event(ts);		//clear (?)

	ctrl.clk = 1;

#if defined(__SIW_CONFIG_PROX_ON_RESUME)
	if (ts->lpwg.sensor == PROX_NEAR) {
		ctrl.lcd = LCD_MODE_STOP;

		t_dev_lpwg_resume_info(dev, ctrl.lcd, "sensor", NULL);
		goto out_con;
	}
#endif

	if (ts->lpwg.screen) {
		if (mode_allowed_qcover) {
			lcd_mode = (qcover_near) ? LCD_MODE_U3_QUICKCOVER : lcd_mode;

			ctrl.qcover = (qcover_near) ? QUICKCOVER_CLOSE : QUICKCOVER_OPEN;
		}

		ctrl.lpwg = LPWG_NONE;
		ctrl.lcd = lcd_mode;

		t_dev_lpwg_resume_info(dev, ctrl.lcd, "screen", NULL);
		goto out_con;
	}

	if (ts->lpwg.mode == LPWG_NONE) {
		ctrl.lpwg = LPWG_NONE;
		ctrl.lcd = LCD_MODE_STOP;

		t_dev_lpwg_resume_info(dev, ctrl.lcd, "mode", "(LPWG_NONE)");
		goto out_con;
	}

	if (mode_allowed_qcover) {
		if (qcover_near) {
			ctrl.qcover = QUICKCOVER_CLOSE;
			ctrl.lpwg = ts->lpwg.mode;

			if (mode_allowed_partial) {
				lcd_mode = LCD_MODE_U3_PARTIAL;
			}
			ctrl.lcd = lcd_mode;

			t_dev_lpwg_resume_info(dev, ctrl.lcd, "qcover", NULL);
			goto out_con;
		}
	}

	t_dev_lpwg_resume_info(dev, ctrl.lcd,
		(mode_allowed_partial) ? "partial" : "default", NULL);

	if (mode_allowed_qcover) {
		ctrl.qcover = QUICKCOVER_OPEN;
	}
	ctrl.lpwg = ts->lpwg.mode;
	if (mode_allowed_partial) {
		lcd_mode = LCD_MODE_U3_PARTIAL;
	}
	ctrl.lcd = lcd_mode;

out_con:
	ret = siw_hal_lpwg_ctrl(dev, &ctrl);
	changed = 1;

	t_dev_info(dev, "lpwg resume(%d, %d): lcd_mode %d, driving_mode %d\n",
			changed, ret,
			chip->lcd_mode, chip->driving_mode);

	return ret;
}

static int siw_hal_lpwg_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (atomic_read(&chip->init) == IC_INIT_NEED) {
		t_dev_warn(dev, "Not Ready, Need IC init (lpwg_mode)\n");
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		return siw_hal_lpwg_mode_suspend(dev);
	}

	return siw_hal_lpwg_mode_resume(dev);
}

//#define __SKIP_LPWG_UPDATE_ALL_FOR_SAME_INPUT

static int siw_hal_lpwg(struct device *dev, u32 code, void *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct tci_ctrl *tci = &ts->tci;
	struct active_area *area = &tci->area;
	struct reset_area *area_qcover;
	struct lpwg_info *lpwg = &ts->lpwg;
	int *value = (int *)param;
	int changed = 0;
	int ret = 0;

//	if (!touch_test_quirks(ts, CHIP_QUIRK_SUPPORT_LPWG)) {
	if (!ts->role.use_lpwg) {
		t_dev_warn(dev, "LPWG control not supported in %s\n",
				touch_chip_name(ts));
		return 0;
	}

	touch_msleep(hal_dbg_delay(chip, HAL_DBG_DLY_LPWG));

	switch (code) {
	case LPWG_ACTIVE_AREA:
		area->x1 = value[0];
		area->x2 = value[1];
		area->y1 = value[2];
		area->y2 = value[3];
		t_dev_info(dev, "LPWG_ACTIVE_AREA: x1[%d], y1[%d], x2[%d], y2[%d]\n",
				area->x1, area->y1, area->x2, area->y2);
		break;

	case LPWG_TAP_COUNT:
		tci->info[TCI_2].tap_count = value[0];
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		tci->double_tap_check = value[0];
		break;

	case LPWG_UPDATE_ALL:
		changed = (lpwg->mode != value[0]) |
				((lpwg->screen != value[1])<<1) |
				((lpwg->sensor != value[2])<<2) |
				((lpwg->qcover != value[3])<<3);

		lpwg->mode = value[0];
		lpwg->screen = value[1];
		lpwg->sensor = value[2];
		lpwg->qcover = value[3];

		t_lpwg_mode = lpwg->mode;
		t_lpwg_screen = lpwg->screen;
		t_lpwg_sensor = lpwg->sensor;
		t_lpwg_qcover = lpwg->qcover;

		t_dev_info(dev,
				"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s] (%02Xh)\n",
				lpwg->mode,
				lpwg->screen ? "ON" : "OFF",
				lpwg->sensor ? "FAR" : "NEAR",
				lpwg->qcover ? "CLOSE" : "OPEN",
				changed);

#if defined(__SKIP_LPWG_UPDATE_ALL_FOR_SAME_INPUT)
		if (!changed) {
			t_dev_info(dev, "LPWG_UPDATE_ALL: not changed, skip\n");
			break;
		}
#endif

		ret = siw_hal_lpwg_mode(dev);
		break;

	case LPWG_REPLY:
		break;

	case LPWG_EXT_TCI_INFO_STORE:
		t_dev_info(dev,
				"LPWG_EXT_TCI_INFO_STORE: tci[%s], index[%d], value[%d], write_en[%s]\n",
				value[0] ? "TCI_2" : "TCI_1",
				value[1],
				value[2],
				value[3] ? "YES" : "NO");

		ret = siw_hal_lpwg_ext_tci_info(dev, param);
		break;

	case LPWG_EXT_TCI_QOPEN_AREA_STORE:
		area_qcover = &tci->qcover_open;

		area_qcover->x1 = value[0];
		area_qcover->x2 = value[1];
		area_qcover->y1 = value[2];
		area_qcover->y2 = value[3];

		t_dev_info(dev,
				"LPWG_EXT_TCI_QOPEN_AREA_STORE: x1[%d], y1[%d], x2[%d], y2[%d]\n",
				area_qcover->x1, area_qcover->y1, area_qcover->x2, area_qcover->y2);
		break;

	case LPWG_EXT_TCI_QCLOSE_AREA_STORE:
		area_qcover = &tci->qcover_close;

		area_qcover->x1 = value[0];
		area_qcover->x2 = value[1];
		area_qcover->y1 = value[2];
		area_qcover->y2 = value[3];

		t_dev_info(dev,
				"LPWG_EXT_TCI_QCLOSE_AREA_STORE: x1[%d], y1[%d], x2[%d], y2[%d]\n",
				area_qcover->x1, area_qcover->y1, area_qcover->x2, area_qcover->y2);
		break;

	case LPWG_EXT_SWIPE_INFO_STORE:
		t_dev_info(dev, "LPWG_EXT_SWIPE_INFO_STORE\n");
		break;

	case LPWG_EXT_TCI_INFO_SHOW:
		t_dev_info(dev, "LPWG_EXT_TCI_INFO_SHOW\n");

		ret = siw_hal_show_ic_tci_info(dev);

		siw_hal_show_driver_tci_info(dev);
		break;

	case LPWG_EXT_TCI_AREA_SHOW:
		t_dev_info(dev, "LPWG_EXT_TCI_AREA_SHOW\n");

		ret = siw_hal_show_ic_tci_area(dev);

		siw_hal_show_driver_tci_area(dev);
		break;

	case LPWG_EXT_SWIPE_INFO_SHOW:
		break;
	}

	return ret;
}

#define siw_chk_sts_snprintf(_dev, _buf, _buf_max, _size, _fmt, _args...) \
		({	\
			int _n_size = 0;	\
			_n_size = __siw_snprintf(_buf, _buf_max, _size, _fmt, ##_args);	\
			t_dev_dbg_trace(_dev, _fmt, ##_args);	\
			_n_size;	\
		})

static int siw_hal_check_fault_type(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	u32 addr = 0;
	int fault_type = NON_FAULT_INT;
	int ret = 0;

	switch (chip->opt.t_chk_fault) {
	case 1 :
		addr = 0x283;
		break;
	default :
		return NON_FAULT_INT;
	}

	ret = siw_hal_read_value(dev, addr, (u32 *)&fault_type);
	if (ret < 0) {
		return NON_FAULT_INT;
	}

	return fault_type;
}

static u32 siw_hal_check_sys_error_type(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	u32 addr = 0;
	u32 fault_type = NON_FAULT_U32;
	int ret = 0;

	switch (chip->opt.t_chk_sys_error) {
	case 1:
		addr = 0x020;
		break;
	case 2:
		addr = 0x021;
		break;
	case 3:
		addr = 0x01C;
		break;
	default :
		return NON_FAULT_U32;
	}

	ret = siw_hal_read_value(dev, addr, (u32 *)&fault_type);
	if (ret < 0) {
		return NON_FAULT_U32;
	}

	return fault_type;
}

static u32 siw_hal_check_sys_fault_type(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	u32 addr = 0;
	u32 fault_type = NON_FAULT_U32;
	int ret = 0;

	switch (chip->opt.t_chk_sys_fault) {
	case 1:
		addr = 0xFF4;
		break;
	default :
		return NON_FAULT_U32;
	}

	ret = siw_hal_read_value(dev, addr, (u32 *)&fault_type);
	if (ret < 0) {
		return NON_FAULT_U32;
	}

	return fault_type;
}

static int siw_hal_check_status_type_x(struct device *dev,
				u32 status, u32 ic_status, int irq)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_status_filter *filter = chip->status_filter;
	u32 check_mask, detected;
	int type_error, type_esd, type_fault;
	u32 irq_type = siw_tc_sts_irq_type(status);
	u32 log_flag = 0;
	u32 esd_send = 0;
	u32 tc_disp_err = 0;
	u32 ic_abnormal, ic_error, ic_disp_err;
	int log_max = IC_CHK_LOG_MAX;
	char log[IC_CHK_LOG_MAX] = {0, };
	int fault_val;
	int len = 0;
	int ret = 0;

	if (filter == NULL) {
		return -EINVAL;
	}

	if (ic_status & ~chip->status_mask_ic_valid) {
		t_dev_err(dev, "[%d] status %08Xh, ic_status %08Xh, ic_status invalid\n",
			irq, status, ic_status);

		if (chip->lcd_mode != LCD_MODE_U0) {
			if (siw_hal_send_abnormal_notifier(dev, 8)) {
				atomic_set(&chip->abnormal_noti_sent, 1);
				return -ETDSENTESDIRQ;
			}
		}
		return -ERESTART;
	}

	while (1) {
		if (!filter->id || !filter->width) {
			break;
		}

		type_error = !!(filter->flag & STS_FILTER_FLAG_TYPE_ERROR);
		type_esd = !!(filter->flag & STS_FILTER_FLAG_ESD_SEND);
		type_fault = !!(filter->flag & STS_FILTER_FLAG_CHK_FAULT);

		check_mask = ((1<<filter->width)-1)<<filter->pos;

		detected = (type_error) ? (status & check_mask) : !(status & check_mask);

		if (check_mask && detected) {
			log_flag |= check_mask;
			esd_send |= (type_esd) ? check_mask : 0;

			fault_val = (type_fault) ? siw_hal_check_fault_type(dev) : -1;

			if (type_fault && (fault_val >= 0)) {
				len += siw_chk_sts_snprintf(dev, log, log_max, len,
							"[b%d] %s(%Xh) ", filter->pos, filter->str, fault_val);
			} else {
				len += siw_chk_sts_snprintf(dev, log, log_max, len,
							"[b%d] %s ", filter->pos, filter->str);
			}

			tc_disp_err |= !!(filter->id == STS_ID_ERROR_DISP);
		}

		filter++;
	}

	if (log_flag) {
		t_dev_err(dev, "[%d] status %08Xh, ic_status %08Xh, (%08Xh) %s\n",
			irq, status, ic_status, log_flag, log);
	}

	ic_abnormal = ic_status & chip->status_mask_ic_abnormal;
	ic_error = ic_status & chip->status_mask_ic_error;
	ic_disp_err = ic_status & chip->status_mask_ic_disp_err;

	if (ic_abnormal || ic_error || ic_disp_err) {
		u32 err_val[3] = { ic_abnormal, ic_error, ic_disp_err };
		char *err_str_ait[3] = {
			"esd",
			"watchdog",
			"dic"
		};
		char *err_str_oled[3] = {
			"spck",
			"watchdog",
			"dic"
		};
		char **err_str = (chip->opt.t_oled) ? err_str_oled : err_str_ait;
		u32 sys_error, sys_fault;
		int log_add = !log_flag;
		int err_pre, i;

		sys_error = siw_hal_check_sys_error_type(dev);
		sys_fault = siw_hal_check_sys_fault_type(dev);

		log_add |= (!!((sys_error != NON_FAULT_U32) || (sys_fault != NON_FAULT_U32)))<<1;

		len = siw_chk_sts_snprintf(dev, log, log_max, 0,
					"[%d] ", irq);

		err_pre = 0;
		for (i = 0; i < ARRAY_SIZE(err_val) ; i++) {
			if (!err_val[i]) {
				continue;
			}

			if (err_pre) {
				len += siw_chk_sts_snprintf(dev, log, log_max, len, " & ");
			}
			len += siw_chk_sts_snprintf(dev, log, log_max, len, "%s", err_str[i]);
			err_pre |= err_val[i];
		}

		if (log_add) {
			len += siw_chk_sts_snprintf(dev, log, log_max, len,
							" - ");

			if (log_add & 0x01) {
				len += siw_chk_sts_snprintf(dev, log, log_max, len,
							"status %08Xh, ic_status %08Xh%s",
							status, ic_status,
							(log_add & 0x02) ? ", " : " ");
			}

			if (log_add & 0x02) {
				len += siw_chk_sts_snprintf(dev, log, log_max, len,
							"sys_error %08Xh, sys_fault %08Xh",
							sys_error, sys_fault);
			}
		}

		t_dev_err(dev, "%s\n", log);

		if (chip->opt.t_oled) {
			esd_send |= (ic_disp_err);
		} else {
			esd_send |= (ic_abnormal | ic_disp_err);
		}
		if (!esd_send) {
			ret = -ERESTART;	//touch reset
		}
	}

	if (esd_send) {
		ret = -ERESTART;
		if (chip->lcd_mode != LCD_MODE_U0) {
			int esd_type;

			esd_type = (ic_disp_err || tc_disp_err)? DIC_ERR_TYPE : 0;

			if (siw_hal_send_abnormal_notifier(dev, esd_type | 1)) {
				atomic_set(&chip->abnormal_noti_sent, 1);
				return -ETDSENTESDIRQ;
			}
		}
	}

	if (ret == -ERESTART) {
		return ret;
	}

	/*
	 * Check interrupt_type[19:16] in TC_STATUS
	 */
	switch (irq_type) {
	case TC_STS_IRQ_TYPE_INIT_DONE:
		t_dev_info(dev, "[%d] TC Driving OK\n", irq);
		ret = -ERANGE;
		break;
	case TC_STS_IRQ_TYPE_REPORT:	/* Touch report */
		break;
	default:
		t_dev_dbg_trace(dev, "[%d] irq_type %Xh\n",
			irq, irq_type);
		ret = -ERANGE;
		break;
	}

	return ret;
}

#if defined(__SIW_PANEL_CLASS_MOBILE)
#define STS_RET_ERR		ERANGE
#else
#define STS_RET_ERR		ERESTART
#endif

static int siw_hal_do_check_status(struct device *dev,
				u32 status, u32 ic_status, int irq)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	u32 reset_clr_bit = 0;
	u32 logging_clr_bit = 0;
	u32 int_norm_mask = 0;
	u32 status_mask = 0;
	int skip_trace = (irq & 0x80);
	int ret_pre = 0;
	int ret = 0;

	irq &= 0x01;

	if (!status && !ic_status) {
		t_dev_err(dev, "[%d] all low detected\n", irq);
		return -STS_RET_ERR;
	}
	if ((status == ~0) && (ic_status == ~0)) {
		t_dev_err(dev, "[%d] all high detected\n", irq);
		return -STS_RET_ERR;
	}

	reset_clr_bit = chip->status_mask_reset;
	logging_clr_bit = chip->status_mask_logging;
	int_norm_mask = chip->status_mask_normal;

	status_mask = status ^ int_norm_mask;

	if (!skip_trace) {
		t_dev_dbg_trace(dev, "[%d] h/w:%Xh, f/w:%Xh(%Xh)\n",
				irq, ic_status, status, status_mask);
	}

	if (status_mask & reset_clr_bit) {
		t_dev_err(dev,
			"[%d] need reset : status %08Xh, ic_status %08Xh, chk %08Xh (%08Xh)\n",
			irq, status, ic_status, status_mask & reset_clr_bit, reset_clr_bit);
		ret_pre = -ERESTART;
	} else if (status_mask & logging_clr_bit) {
		t_dev_err(dev,
			"[%d] need logging : status %08Xh, ic_status %08Xh, chk %08Xh (%08Xh)\n",
			irq, status, ic_status, status_mask & logging_clr_bit, logging_clr_bit);
		ret_pre = -ERANGE;
	}

	switch (chip->status_type) {
	case CHIP_STATUS_TYPE_2:
	case CHIP_STATUS_TYPE_1:
	case CHIP_STATUS_TYPE_0:
		ret = siw_hal_check_status_type_x(dev, status, ic_status, irq);
		siw_hal_chk_dbg_report(dev, status, irq);
		break;
	default:
		t_dev_warn(dev, "unknown status type, %d\n", chip->status_type);
		break;
	}

	if (ret == -ETDSENTESDIRQ) {
		return ret;
	}

	if (ret_pre) {
		if (ret != -ERESTART) {
			ret = ret_pre;
		}
	}

	return ret;
}

static int siw_hal_check_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	u32 ic_status = chip->info.ic_status;
	u32 status = chip->info.device_status;

	return siw_hal_do_check_status(dev, status, ic_status, 1);
}

static int siw_hal_irq_abs_data_type_1(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_touch_data_type_1 *data;
	struct touch_data *tdata;
	u32 touch_count = chip->info.touch_cnt;
	int finger_index = 0;
	int i = 0;
	int ret = 0;

	data = (struct siw_hal_touch_data_type_1 *)chip->info.data;

	touch_count = chip->info.touch_cnt;
	ts->new_mask = 0;

	/* check if palm detected */
	if (data->track_id == PALM_ID) {
		if (data->event == TOUCHSTS_DOWN) {
			ts->is_cancel = 1;
			t_dev_info(dev, "Palm Detected\n");
		} else if (data->event == TOUCHSTS_UP) {
			ts->is_cancel = 0;
			t_dev_info(dev, "Palm Released\n");
		}
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < touch_count; i++, data++) {
		if (data->track_id >= touch_max_finger(ts)) {
			continue;
		}

		if ((data->event == TOUCHSTS_DOWN) ||
			(data->event == TOUCHSTS_MOVE)) {
			ts->new_mask |= (1 << data->track_id);
			tdata = ts->tdata + data->track_id;

			tdata->id = data->track_id;
			tdata->type = data->tool_type;
			tdata->event = data->event;
			tdata->x = data->x;
			tdata->y = data->y;
			tdata->pressure = data->pressure;
			tdata->width_major = data->width_major;
			tdata->width_minor = data->width_minor;

			if (data->width_major == data->width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = (s8)data->angle;

			finger_index++;

			t_dev_dbg_abs(dev,
					"touch data [id %d, t %d, e %d, x %d, y %d, z %d - %d, %d, %d]\n",
					tdata->id,
					tdata->type,
					tdata->event,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);
		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return ret;
}

static int siw_hal_irq_abs_data_type_0(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_touch_data *data;
	struct touch_data *tdata;
	u32 touch_count = chip->info.touch_cnt;
	int finger_index = 0;
	int i = 0;
	int ret = 0;

	data = (struct siw_hal_touch_data *)chip->info.data;

	touch_count = chip->info.touch_cnt;
	ts->new_mask = 0;

	/* check if palm detected */
	if (data->track_id == PALM_ID) {
		if (data->event == TOUCHSTS_DOWN) {
			ts->is_cancel = 1;
			t_dev_info(dev, "Palm Detected\n");
		} else if (data->event == TOUCHSTS_UP) {
			ts->is_cancel = 0;
			t_dev_info(dev, "Palm Released\n");
		}
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < touch_count; i++, data++) {
		if (data->track_id >= touch_max_finger(ts)) {
			continue;
		}

		if ((data->event == TOUCHSTS_DOWN) ||
			(data->event == TOUCHSTS_MOVE)) {
			ts->new_mask |= (1 << data->track_id);
			tdata = ts->tdata + data->track_id;

			tdata->id = data->track_id;
			tdata->type = data->tool_type;
			tdata->event = data->event;
			tdata->x = data->x;
			tdata->y = data->y;
			tdata->pressure = data->pressure;
			tdata->width_major = data->width_major;
			tdata->width_minor = data->width_minor;

			if (data->width_major == data->width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = (s8)data->angle;

			finger_index++;

			t_dev_dbg_abs(dev,
					"touch data [id %d, t %d, e %d, x %d, y %d, z %d - %d, %d, %d]\n",
					tdata->id,
					tdata->type,
					tdata->event,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);
		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return ret;
}

static int siw_hal_irq_abs_data(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	switch (chip->report_type) {
	case CHIP_REPORT_TYPE_1:
		ret = siw_hal_irq_abs_data_type_1(dev);
		break;
	case CHIP_REPORT_TYPE_0:
		ret = siw_hal_irq_abs_data_type_0(dev);
		break;
	default:
		t_dev_warn(dev, "unknown report type, %d\n", chip->report_type);
		break;
	}

	return ret;
}

static int siw_hal_irq_abs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_touch_info *info = &chip->info;

	/* check if touch cnt is valid */
	if (!info->touch_cnt ||
		(info->touch_cnt > ts->caps.max_id)) {
		struct siw_hal_touch_data *data = info->data;

		t_dev_dbg_abs(dev, "Invalid touch count, %d(%d)\n",
				info->touch_cnt, ts->caps.max_id);

		/* debugging */
		t_dev_dbg_abs(dev, "t %d, ev %d, id %d, x %d, y %d, p %d, a %d, w %d %d\n",
			data->tool_type, data->event, data->track_id,
			data->x, data->y, data->pressure, data->angle,
			data->width_major, data->width_minor);

		return -ERANGE;
	}

	return siw_hal_irq_abs_data(dev);
}

static int siw_hal_get_tci_data(struct device *dev, int count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u8 i = 0;
	u32 rdata[MAX_LPWG_CODE];

	if (!count)
		return 0;

	ts->lpwg.code_num = count;

	memcpy(&rdata, chip->info.data, sizeof(u32) * count);

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = rdata[i] & 0xffff;
		ts->lpwg.code[i].y = (rdata[i] >> 16) & 0xffff;

		if (ts->lpwg.mode == LPWG_PASSWORD)
			t_dev_info(dev, "LPWG data xxxx, xxxx\n");
		else
			t_dev_info(dev, "LPWG data %d, %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int siw_hal_get_swipe_data(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u32 rdata[3];
	int count = 1;

	/* swipe_info */
	/* start (X, Y), end (X, Y), time = 2bytes * 5 = 10 bytes */
	memcpy(&rdata, chip->info.data, sizeof(u32) * 3);

	t_dev_info(dev,
			"Swipe Gesture: start(%4d, %4d) end(%4d, %4d) swipe_time(%dms)\n",
			rdata[0] & 0xffff, rdata[0] >> 16,
			rdata[1] & 0xffff, rdata[1] >> 16,
			rdata[2] & 0xffff);

	ts->lpwg.code_num = count;
	ts->lpwg.code[0].x = rdata[1] & 0xffff;
	ts->lpwg.code[0].x = rdata[1]  >> 16;

	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int siw_hal_irq_lpwg_t1_base(struct siw_ts *ts, int type)
{
	struct device *dev = ts->dev;
	int ret = 0;

	switch (type) {
	case LPWG_T1_KNOCK_1:
		if (ts->lpwg.mode == LPWG_NONE) {
			break;
		}
		t_dev_info(dev, "LPWG: TOUCH_IRQ_KNOCK\n");
		siw_hal_get_tci_data(dev, ts->tci.info[TCI_1].tap_count);
		ts->intr_status = TOUCH_IRQ_KNOCK;
		break;
	case LPWG_T1_KNOCK_2:
		if (ts->lpwg.mode != LPWG_PASSWORD) {
			break;
		}
		t_dev_info(dev, "LPWG: TOUCH_IRQ_PASSWD\n");
		siw_hal_get_tci_data(dev, ts->tci.info[TCI_2].tap_count);
		ts->intr_status = TOUCH_IRQ_PASSWD;
		break;
	case LPWG_T1_SWIPE_LEFT:
		t_dev_info(dev, "LPWG: SWIPE_LEFT\n");
		siw_hal_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
		break;
	case LPWG_T1_SWIPE_RIGHT:
		t_dev_info(dev, "LPWG: SWIPE_RIGHT\n");
		siw_hal_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
		break;
	case LPWG_T1_SWIPE_UP:
		t_dev_info(dev, "LPWG: SWIPE_UP\n");
		siw_hal_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_UP;
		break;
	case LPWG_T1_SWIPE_DOWN:
		t_dev_info(dev, "LPWG: SWIPE_DOWN\n");
		siw_hal_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_DOWN;
		break;
	default:
	//	t_dev_warn(dev, "unkown lpwg: %d\n", type);
		break;
	}

	return ret;
}

static int siw_hal_irq_lpwg_base(struct siw_ts *ts, int type)
{
	struct device *dev = ts->dev;
	int ret = 0;

	switch (type) {
	case KNOCK_1:
		if (ts->lpwg.mode == LPWG_NONE) {
			break;
		}
		t_dev_info(dev, "LPWG: TOUCH_IRQ_KNOCK\n");
		siw_hal_get_tci_data(dev,
			ts->tci.info[TCI_1].tap_count);
		ts->intr_status = TOUCH_IRQ_KNOCK;
		break;
	case KNOCK_2:
		if (ts->lpwg.mode != LPWG_PASSWORD) {
			break;
		}
		t_dev_info(dev, "LPWG: TOUCH_IRQ_PASSWD\n");
		siw_hal_get_tci_data(dev,
			ts->tci.info[TCI_2].tap_count);
		ts->intr_status = TOUCH_IRQ_PASSWD;
		break;
	case SWIPE_RIGHT:
		t_dev_info(dev, "LPWG: SWIPE_RIGHT\n");
		siw_hal_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
		break;
	case SWIPE_LEFT:
		t_dev_info(dev, "LPWG: SWIPE_LEFT\n");
		siw_hal_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
		break;
	default:
	//	t_dev_warn(dev, "unkown lpwg: %d\n", type);
		break;
	}

	return ret;
}

static int siw_hal_irq_lpwg_gesture(struct siw_ts *ts, int type)
{
	struct device *dev = ts->dev;
	int index = 0;
	int ret = 0;

	t_dev_info(dev, "lpwg gesture: %d\n", type);

	ts->intr_status = TOUCH_IRQ_GESTURE;
	ts->intr_gesture = TOUCH_UEVENT_GESTURE_C + index;

	return ret;
}

static int siw_hal_irq_lpwg_dir(struct siw_ts *ts, int type)
{
	struct device *dev = ts->dev;
	int index = 0;
	int ret = 0;

	t_dev_info(dev, "lpwg dir: %d\n", type);

	ts->intr_status = TOUCH_IRQ_GESTURE;
	ts->intr_gesture = TOUCH_UEVENT_GESTURE_DIR_RIGHT + index;

	return ret;
}

static int siw_hal_irq_lpwg_custom_debug(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	t_dev_info(dev, "LPWG: CUSTOM_DEBUG\n");
	siw_hal_debug_tci(dev);
	siw_hal_debug_swipe(dev);

	return 0;
}

static int siw_hal_irq_lpwg_knock_overtap(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	t_dev_info(dev, "LPWG: overtap\n");
//	siw_hal_get_tci_data(dev, 1);
	siw_hal_get_tci_data(dev, ts->tci.info[TCI_2].tap_count + 1);
	ts->intr_status = TOUCH_IRQ_PASSWD;

	return 0;
}

static int siw_hal_irq_lpwg_type_1(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u32 type = chip->info.wakeup_type;
	int ret = 0;

	if (!type) {
		goto out;
	}

	if (type <= LPWG_T1_SWIPE_LAST) {
		ret = siw_hal_irq_lpwg_t1_base(ts, type);
	} else {
		goto out;
	}

	return ret;

out:
	t_dev_err(dev, "LPWG: unknown type, %d\n", type);

	return -EINVAL;
}

static int siw_hal_irq_lpwg_default(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u32 type = chip->info.wakeup_type;
	int ret = 0;

	if (!type || (type > KNOCK_OVERTAP)) {
		goto out;
	}

	if (type == CUSTOM_DEBUG) {
		ret = siw_hal_irq_lpwg_custom_debug(ts);
	} else if (type == KNOCK_OVERTAP) {
		ret = siw_hal_irq_lpwg_knock_overtap(ts);
	} else if (type <= SWIPE_LEFT) {
		ret = siw_hal_irq_lpwg_base(ts, type);
	} else if (type <= GESTURE_Z) {
		ret = siw_hal_irq_lpwg_gesture(ts, type);
	} else if ((type >= GESTURE_DIR_RIGHT) || (type <= GESTURE_DIR_UP)) {
		ret = siw_hal_irq_lpwg_dir(ts, type);
	} else {
		goto out;
	}

	return ret;

out:
	t_dev_err(dev, "LPWG: unknown type, %d\n", type);

	return -EINVAL;
}

static int siw_hal_irq_lpwg(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	switch (chip->opt.t_lpwg) {
	case 1:
		ret = siw_hal_irq_lpwg_type_1(dev);
		break;
	default:
		ret = siw_hal_irq_lpwg_default(dev);
		break;
	}

	return ret;
}

#define GET_REPORT_BASE_PKT		(1)
#define GET_REPORT_BASE_HDR		(3)

static int siw_hal_irq_get_report(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_touch_info *info = &chip->info;
	u32 addr = reg->tc_ic_status;
	char *buf = (char *)&chip->info;
	int size = 0;
	int pkt_unit = 0;
	int pkt_cnt = 0;
	int touch_cnt = 0;
	int ret = 0;

	pkt_unit = sizeof(struct siw_hal_touch_data);
	pkt_cnt = GET_REPORT_BASE_PKT;
	touch_cnt = touch_max_finger(ts) - pkt_cnt;

	size = (GET_REPORT_BASE_HDR<<2);
	size += (pkt_unit * pkt_cnt);

	/*
	 * Dynamic read access
	 */
	if (chip->opt.f_flex_report) {
		ret = siw_hal_reg_read(dev, addr, (void *)buf, size);
		if (ret < 0) {
			return ret;
		}

		if (info->wakeup_type != ABS_MODE) {
			/* No need to read more */
			return 0;
		}

		if ((info->touch_cnt <= pkt_cnt) ||
			(info->touch_cnt > ts->caps.max_id)) {
			/* No need to read more */
			return 0;
		}

		addr += (size>>2);
		buf += size;
		size = 0;

		touch_cnt = chip->info.touch_cnt - pkt_cnt;
	}

	size += (pkt_unit * touch_cnt);

	ret = siw_hal_reg_read(dev, addr, (void *)buf, size);

	return ret;
}

static int siw_hal_irq_exception(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	char *title = NULL;

	if (atomic_read(&chip->init) == IC_INIT_NEED) {
		title = "Not Ready, Need IC init (irq)";
		goto out;
	}

	if (!chip->status_type) {
		title = "No status type";
		goto out;
	}

	if (!chip->report_type) {
		title = "No report type";
		goto out;
	}

	return 0;

out:
	t_dev_warn(dev, "%s\n", title);

	return 1;
}

static int siw_hal_irq_skip_event(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	char *title = NULL;

	switch (chip->driving_mode) {
	case LCD_MODE_STOP:
		title = "stop state";
		goto out;
	}

	return 0;

out:
	t_dev_info(dev, "skip event - %s\n", title);
	siw_touch_report_all_event(ts);

	return 1;
}

static int siw_hal_irq_handler(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	int ret = 0;

	if (siw_hal_irq_exception(dev)) {
		return 0;
	}

	siw_touch_set_pm_qos_req(dev, 10);
	ret = siw_hal_irq_get_report(dev);
	siw_touch_clr_pm_qos_req(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_check_status(dev);
	if (ret < 0) {
		goto out;
	}

	t_dev_dbg_irq(dev, "hal irq handler: wakeup_type %d\n",
			chip->info.wakeup_type);

	if (siw_hal_irq_skip_event(dev)) {
		goto out;
	}

	if (chip->info.wakeup_type == ABS_MODE) {
		ret = siw_hal_irq_abs(dev);
		if (ret) {
			t_dev_err(dev, "siw_hal_irq_abs failed(%d), %d\n",
				chip->info.touch_cnt, ret);
			goto out;
		}
	} else {
		ret = siw_hal_irq_lpwg(dev);
		if (ret) {
			t_dev_err(dev, "siw_hal_irq_lpwg failed, %d\n", ret);
			goto out;
		}
	}

	if (atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_2) {
		/* */
	}

out:
	return ret;
}

static void siw_hal_connect(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int charger_state = atomic_read(&ts->state.connect);
	int wireless_state = atomic_read(&ts->state.wireless);

#if 1
	if (wireless_state) {
		chip->charger = CONNECT_WIRELESS;
	} else {
		if ((charger_state < 0) ||
			(charger_state > CONNECT_OTG)) {
			t_dev_err(dev, "invalid charger status, %d\n", charger_state);
			return;
		}
		chip->charger = charger_state;
	}
#else
	chip->charger = 0;
	switch (charger_state) {
	case CONNECT_INVALID:
		chip->charger = CONNECT_NONE;
		break;
	case CONNECT_DCP:
		/* fall through */
	case CONNECT_PROPRIETARY:
		chip->charger = CONNECT_TA;
		break;
	case CONNECT_HUB:
		chip->charger = CONNECT_OTG;
		break;
	default:
		chip->charger = CONNECT_USB;
		break;
	}

#if 0
	/* code for TA simulator */
	if (atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_4) {
		t_dev_info(dev, "TA simulator mode, set CONNECT_TA\n");
		chip->charger = CONNECT_TA;
	}
#endif

	/* wireless */
	chip->charger |= (wireless_state) ? CONNECT_WIRELESS : 0;
#endif

	t_dev_info(dev,
		"charger_state = %Xh (%Xh, %Xh)\n",
		chip->charger, charger_state, wireless_state);

	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		t_dev_warn(dev, "DEV_PM_SUSPEND - Don't try SPI\n");
		return;
	}

	siw_hal_write_value(dev,
			reg->spr_charger_status,
			chip->charger);

	t_dev_info(dev, "charger_state set done\n");
}

static int siw_hal_lcd_mode(struct device *dev, u32 mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (touch_mode_not_allowed(ts, mode)) {
		return -EPERM;
	}

	if ((chip->lcd_mode == LCD_MODE_U2) &&
		siw_hal_watch_is_disp_waton(dev)) {
		siw_hal_watch_get_curr_time(dev, NULL, NULL);
	}

	if (chip->opt.f_u2_blank_chg) {
		if (mode == LCD_MODE_U2_UNBLANK) {
			mode = LCD_MODE_U2;
		}
	}

	chip->prev_lcd_mode = chip->lcd_mode;
	chip->lcd_mode = mode;

	t_dev_info(dev, "lcd_mode: %d (prev: %d)\n",
		mode, chip->prev_lcd_mode);

	return 0;
}

static int siw_hal_usb_status(struct device *dev, u32 mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	t_dev_info(dev, "TA Type: %d\n", atomic_read(&ts->state.connect));

	siw_hal_connect(dev);

	return 0;
}

static int siw_hal_wireless_status(struct device *dev, u32 onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	t_dev_info(dev, "Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));

	siw_hal_connect(dev);

	return 0;
}

static int siw_hal_earjack_status(struct device *dev, u32 onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	t_dev_info(dev, "Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));

	return 0;
}

#if defined(__SIW_SUPPORT_ABT)
extern void siw_hal_switch_to_abt_irq_handler(struct siw_ts *ts);

static int siw_hal_debug_tool(struct device *dev, u32 value)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (value >= DEBUG_TOOL_MAX) {
		t_dev_err(dev,
			"wrong index, debug tool select failed, %d\n",
			value);
		return -EINVAL;
	}

	mutex_lock(&ts->lock);
	switch (value) {
	case DEBUG_TOOL_ENABLE:
		siw_hal_switch_to_abt_irq_handler(ts);
		break;
	default:
		siw_ops_restore_irq_handler(ts);
		t_dev_info(dev, "restore irq handler\n");
		break;
	}
	mutex_unlock(&ts->lock);

	return 0;
}
#else	/* __SIW_SUPPORT_ABT */
static int siw_hal_debug_tool(struct device *dev, u32 value)
{
	t_dev_info(dev, "Nop ...\n");
	return 0;
}
#endif	/* __SIW_SUPPORT_ABT */

static int siw_hal_notify(struct device *dev, ulong event, void *data)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	char *noti_str = "(unknown)";
	u32 value = 0;
	int ret = 0;

	if (data) {
		value = *((u32 *)data);
	}

	switch (event) {
	case LCD_EVENT_TOUCH_RESET_START:
	case LCD_EVENT_TOUCH_RESET_END:
		break;
	default:
		touch_msleep(hal_dbg_delay(chip, HAL_DBG_DLY_NOTIFY));
		break;
	}

	t_dev_dbg_noti(dev, "notify event(%d)\n", (u32)event);

	switch (event) {
	case NOTIFY_TOUCH_RESET:
	#if 0
		ret = !!(atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_1);
		t_dev_info(dev, "notify: reset, %d\n", ret);
	#else
		t_dev_info(dev, "notify: reset\n");
	#endif

		atomic_set(&chip->init, IC_INIT_NEED);
		atomic_set(&ts->state.hw_reset, event);

		siw_hal_watch_set_rtc_clear(dev);

	#if 0
		siw_hal_watch_set_font_empty(dev);
		siw_hal_watch_set_cfg_blocked(dev);
	#endif

		noti_str = "TOUCH_RESET";
		break;
	case LCD_EVENT_TOUCH_RESET_START:
		t_dev_info(dev, "notify: lcd_event: touch reset start\n");
		atomic_set(&chip->init, IC_INIT_NEED);
		atomic_set(&ts->state.hw_reset, event);

		siw_touch_irq_control(ts->dev, INTERRUPT_DISABLE);
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);
		touch_msleep(chip->drv_reset_low + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_0));

		noti_str = "TOUCH_RESET_START";
		break;
	case LCD_EVENT_TOUCH_RESET_END:
		t_dev_info(dev, "notify: lcd_event: touch reset end\n");
		atomic_set(&ts->state.hw_reset, event);

		siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);

		siw_touch_qd_init_work_hw(ts);

		noti_str = "TOUCH_RESET_END";
		break;
	case LCD_EVENT_LCD_MODE:
		noti_str = "LCD_MODE";

		t_dev_info(dev, "notify: lcd_event: lcd mode\n");
		if (data == NULL) {
			t_dev_err(dev, "data is null, cancled\n");
			break;
		}

		ret = siw_hal_lcd_mode(dev, value);
		if (ret < 0) {
			break;
		}
		ret = siw_hal_check_mode(dev);
		if (!ret) {
			queue_delayed_work(ts->wq, &chip->fb_notify_work, 0);
		}
		ret = 0;
		break;

	case LCD_EVENT_READ_REG:
		t_dev_info(dev, "notify: lcd event: read reg\n");
		siw_hal_lcd_event_read_reg(dev);

		noti_str = "READ_REG";
		break;

	case NOTIFY_CONNECTION:
		t_dev_info(dev, "notify: connection\n");
		ret = siw_hal_usb_status(dev, value);

		noti_str = "CONNECTION";
		break;
	case NOTIFY_WIRELEES:
		t_dev_info(dev, "notify: wireless\n");
		ret = siw_hal_wireless_status(dev, value);

		noti_str = "WIRELESS";
		break;
	case NOTIFY_EARJACK:
		t_dev_info(dev, "notify: earjack\n");
		ret = siw_hal_earjack_status(dev, value);

		noti_str = "EARJACK";
		break;
	case NOTIFY_IME_STATE:
#if 0
		t_dev_info(dev, "notify: ime state\n");
		ret = siw_hal_write_value(dev,
					reg->ime_state,
					value);
#else
		t_dev_info(dev, "notify: do nothing for ime\n");
#endif

		noti_str = "IME_STATE";
		break;
	case NOTIFY_DEBUG_TOOL:
		t_dev_info(dev, "notify: debug tool\n");
		ret = siw_hal_debug_tool(dev, value);

		noti_str = "DEBUG_TOOL";
		break;
	case NOTIFY_CALL_STATE:
		t_dev_info(dev, "notify: call state\n");
		ret = siw_hal_write_value(dev,
					reg->call_state,
					value);

		noti_str = "CALL_STATE";
		break;
	case LCD_EVENT_TOUCH_DRIVER_REGISTERED:
	case LCD_EVENT_TOUCH_DRIVER_UNREGISTERED:
		if (0) {
			/* from siw_touch_probe */
			t_dev_info(dev, "notify: driver %s\n",
					(event == LCD_EVENT_TOUCH_DRIVER_REGISTERED) ?
					"registered" : "unregistered");
		}
		noti_str = "DRV";
		break;
	case LCD_EVENT_TOUCH_WATCH_LUT_UPDATE:
		t_dev_info(dev, "notify: WATCH_LUT_UPDATE(%lu)\n", event);
		noti_str = "WATCH_LUT";
		break;
	case LCD_EVENT_TOUCH_WATCH_POS_UPDATE:
		t_dev_info(dev, "notify: WATCH_POS_UPDATE(%lu)\n", event);
		noti_str = "WATCH_POS";
		break;
	case LCD_EVENT_TOUCH_PROXY_STATUS:
		t_dev_info(dev, "notify: PROXY_STATUS(%lu)\n", event);
		noti_str = "PROXY";
		break;
	case LCD_EVENT_TOUCH_ESD_DETECTED:
		t_dev_info(dev, "notify: ESD_DETECTED(%lu, %d)\n", event, value);
		noti_str = "ESD";
		break;
	default:
		t_dev_err(dev, "notify: %lu is not supported\n", event);
		break;
	}

	siwmon_submit_evt(dev, "NOTIFY", 0, noti_str, event, value, ret);

	return ret;
}

enum {
	SIW_GET_CHIP_NAME	= (1<<0),
	SIW_GET_VERSION		= (1<<1),
	SIW_GET_REVISION	= (1<<2),
	SIW_GET_PRODUCT		= (1<<3),
	/* */
	SIW_GET_OPT1		= (1<<8),
	/* */
	SIW_GET_VER_SIMPLE	= (1<<16),
	/* */
	SIW_GET_ALL			= 0xFFFF,
};

static int siw_hal_get_cmd_version(struct device *dev, char *buf, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	int offset = 0;
//	int ret = 0;

	if (!flag)
		return 0;

	if (flag & SIW_GET_CHIP_NAME) {
		offset += siw_snprintf(buf, offset,
					"chip : %s\n",
					touch_chip_name(ts));
	}

	if (flag & (SIW_GET_VERSION|SIW_GET_VER_SIMPLE)) {
		char *ver_tag = (flag & SIW_GET_VER_SIMPLE) ? "" : "version : ";
		if (fw->version_ext) {
			offset += siw_snprintf(buf, offset,
						"%s%08X(%u.%02u)\n",
						ver_tag,
						fw->version_ext,
						fw->v.version.major, fw->v.version.minor);
		} else {
			offset += siw_snprintf(buf, offset,
						"%sv%u.%02u\n",
						ver_tag,
						fw->v.version.major, fw->v.version.minor);
		}
	}

	if (flag & SIW_GET_REVISION) {
		if (chip->fw.revision == 0xFF) {
			offset += siw_snprintf(buf, offset,
						"revision : Flash Erased(0xFF)\n");
		} else {
			offset += siw_snprintf(buf, offset,
						"revision : %d\n", fw->revision);
		}
	}

	if (flag & SIW_GET_PRODUCT) {
		offset += siw_snprintf(buf, offset,
					"product id : %s\n", fw->product_id);
	}

	if (flag & SIW_GET_OPT1) {
		if (chip->opt.f_info_more) {
			offset += siw_snprintf(buf, offset,
						"fpc : %d\n", fw->fpc);
			offset += siw_snprintf(buf, offset,
						"wfr : %d\n", fw->wfr);
			offset += siw_snprintf(buf, offset,
						"cg  : %d\n", fw->cg);
			offset += siw_snprintf(buf, offset,
						"lot : %d\n", fw->lot);
			offset += siw_snprintf(buf, offset,
						"serial : 0x%X\n", fw->sn);
			offset += siw_snprintf(buf, offset,
						"date : %04d.%02d.%02d %02d:%02d:%02d Site%d\n",
						fw->date & 0xFFFF,
						((fw->date>>16) & 0xFF), ((fw->date>>24) & 0xFF),
						fw->time & 0xFF,
						((fw->time>>8) & 0xFF), ((fw->time>>16) & 0xFF),
						((fw->time>>24) & 0xFF));
		}
	}

	return offset;
}

static int siw_hal_set(struct device *dev, u32 cmd, void *buf)
{
	return 0;
}

static int siw_hal_get(struct device *dev, u32 cmd, void *buf)
{
	int ret = 0;

	t_dev_dbg_base(dev, "cmd %d\n", cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = siw_hal_get_cmd_version(dev, (char *)buf, SIW_GET_ALL);
		break;

	case CMD_ATCMD_VERSION:
		ret = siw_hal_get_cmd_version(dev, (char *)buf, SIW_GET_VER_SIMPLE);
		break;

	default:
		break;
	}

	return ret;
}

u32 t_mon_dbg_mask = 0;

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/mon_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko mon_dbg_mask=<value>
 */
module_param_named(mon_dbg_mask, t_mon_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);

#define SIW_HAL_MON_TAG 		"mon: "
#define SIW_HAL_MON_TAG_ERR 	"mon(E): "
#define SIW_HAL_MON_TAG_WARN	"mon(W): "
#define SIW_HAL_MON_TAG_DBG		"mon(D): "

#if 1
#define t_mon_info(_dev, fmt, args...)		__t_dev_info(_dev, SIW_HAL_MON_TAG fmt, ##args)
#define t_mon_warn(_dev, fmt, args...)		__t_dev_warn(_dev, SIW_HAL_MON_TAG_WARN fmt, ##args)
#else
#define t_mon_info(_dev, fmt, args...)		__t_dev_none(_dev, fmt, ##args)
#define t_mon_warn(_dev, fmt, args...)		__t_dev_none(_dev, fmt, ##args)
#endif

#define t_mon_err(_dev, fmt, args...)		__t_dev_err(_dev, SIW_HAL_MON_TAG_ERR fmt, ##args)

#define t_mon_dbg(condition, _dev, fmt, args...)			\
		do {							\
			if (unlikely(t_mon_dbg_mask & (condition)))	\
				__t_dev_info(_dev, SIW_HAL_MON_TAG_DBG fmt, ##args);	\
		} while (0)

#define t_mon_dbg_base(_dev, fmt, args...)	\
		t_mon_dbg(DBG_BASE, _dev, fmt, ##args)

#define t_mon_dbg_trace(_dev, fmt, args...)	\
		t_mon_dbg(DBG_TRACE, _dev, fmt, ##args)

#if defined(__SIW_SUPPORT_MON_THREAD)
static int siw_hal_mon_handler_chk_frame(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 frame_addr = 0;
	u32 frame_s = 0;
	u32 frame_e = 0;
	u32 delay = 1;
	int cnt = 20;
	int i;
	int ret = 0;

	switch (chip->opt.t_chk_frame) {
	case 3:
		if (!strncmp(fw->product_id, "LA103WF5", 8)) {
			frame_addr = 0x272;
		}
		break;
	case 2:
		if (!strncmp(fw->product_id, "LA145WF1", 8)) {
			if (fw->v.version.major || (fw->v.version.minor > 2)) {
				frame_addr = 0x271;
			}
		}
		break;
	case 1:
		frame_addr = 0x24F;
		break;
	default:
		break;
	}

	if (!frame_addr) {
		return 0;
	}

	ret = siw_hal_read_value(dev,
				frame_addr,
				(void *)&frame_s);
	if (ret < 0){
		goto out;
	}

	for (i = 0; i < cnt; i++) {
		touch_msleep(delay);

		ret = siw_hal_read_value(dev,
					frame_addr,
					(void *)&frame_e);
		if (ret < 0){
			goto out;
		}

		if (frame_e != frame_s) {
			t_mon_dbg_trace(dev, "frame ok: %d(%d), %d x %d ms\n",
				frame_e, frame_s, i, delay);
			return 0;
		}
	}

	t_mon_err(dev, "frame not changed: %d, %d x %d ms\n",
			frame_s, cnt, delay);

	ret = -ERESTART;

out:
	return ret;
}

static int siw_hal_mon_handler_chk_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dbg_mask = (t_mon_dbg_mask & DBG_TRACE) ? 0 : 0x80;
	u32 ic_status;
	u32 status;
	int ret = 0;

	ret = siw_hal_reg_read(dev,
				reg->tc_ic_status,
				(void *)&ic_status, sizeof(ic_status));
	if (ret < 0){
		goto out;
	}

	ret = siw_hal_reg_read(dev,
				reg->tc_status,
				(void *)&status, sizeof(status));
	if (ret < 0){
		goto out;
	}

	status |= 0x8000;	//Valid IRQ
	ret = siw_hal_do_check_status(dev, status, ic_status, dbg_mask);
	if (ret < 0) {
		if (ret == -ERESTART) {
			goto out;
		}
		ret = 0;
	}

out:
	return ret;
}

static int siw_hal_mon_handler_chk_id(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 chip_id;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->spr_chip_id,
				&chip_id);
	if (ret < 0) {
		goto out;
	}

	if (fw->chip_id_raw != chip_id) {
		ret = -ERESTART;
		goto out;
	}

out:
	return ret;
}

struct siw_mon_hanlder_op {
	unsigned int step;
	unsigned int delay;	//msec
	unsigned int retry;
	char *name;
	int (*func)(struct device *dev);
};

#define SIW_MON_HANDLER_OP_SET(_step, _delay, _retry, _name, _func)	\
	[_step] = {	\
		.step = _step,	\
		.delay = _delay,	\
		.retry = _retry,	\
		.name = _name,	\
		.func = _func,	\
	}

static const struct siw_mon_hanlder_op siw_mon_hanlder_ops[] = {
	SIW_MON_HANDLER_OP_SET(0, 10, 3, "id", siw_hal_mon_handler_chk_id),
	SIW_MON_HANDLER_OP_SET(1, 10, 3, "status", siw_hal_mon_handler_chk_status),
	SIW_MON_HANDLER_OP_SET(2, 10, 3, "frame", siw_hal_mon_handler_chk_frame),
};

static int siw_hal_mon_hanlder_do_op(struct device *dev,
				const struct siw_mon_hanlder_op *op, char *p_name)
{
	unsigned int delay = op->delay;
	unsigned int retry = op->retry;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < retry; i++) {
		ret = op->func(dev);
		if (ret >= 0) {
			t_mon_dbg_trace(dev,
				"%s : [%d] %s check done\n",
				p_name, op->step, op->name);
			break;
		}

		t_mon_err(dev,
			"%s : [%d] %s check failed(%d), %d (%d)\n",
			p_name, op->step, op->name, i, ret, op->delay);

		touch_msleep(delay);
	}

	return ret;
}

static int siw_hal_mon_handler_skip(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;

	if (siw_hal_access_not_allowed(dev, NULL, 0)) {
		return 1;
	}

	if (chip->lcd_mode != LCD_MODE_U3) {
		return 1;
	}

	if (chip->driving_mode != LCD_MODE_U3) {
		return 1;
	}

	if (!chip->status_type || !chip->report_type) {
		return 1;
	}

	if (chip->fw.invalid_pid) {
		return 1;
	}

	return 0;
}

static void siw_hal_mon_handler_self_reset(struct device *dev, char *title)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	const struct siw_mon_hanlder_op *ops = siw_mon_hanlder_ops;
	unsigned int ops_num = ARRAY_SIZE(siw_mon_hanlder_ops);
	char name[32];
	int step;
	int ret = 0;

	mutex_lock(&ts->lock);

	mutex_lock(&ts->reset_lock);

	if (siw_hal_mon_handler_skip(dev)) {
		mutex_unlock(&ts->reset_lock);
		mutex_unlock(&ts->lock);
		return;
	}

	snprintf(name, sizeof(name), "%s self-reset", title);

	for (step = 0 ; step<ops_num ; step++, ops++) {
		if ((ops->step >= ops_num) ||
			(ops->name == NULL) ||
			(ops->func == NULL)) {
			break;
		}

		ret = siw_hal_mon_hanlder_do_op(dev, ops, name);
		if (ret < 0){
			break;
		}
	}

	mutex_unlock(&ts->reset_lock);

	if (ret < 0) {
		t_mon_err(dev,
			"%s : recovery begins(hw reset)\n",
			name);

		siw_hal_reset_ctrl(dev, HW_RESET_ASYNC);
	} else {
		t_mon_dbg_trace(dev,
			"%s : check ok\n",
			name);
	}

	mutex_unlock(&ts->lock);
}

static int siw_hal_mon_handler(struct device *dev, u32 opt)
{
	char *name;

	name = (opt & MON_FLAG_RST_ONLY) ? "reset cond" : "mon handler";

	t_mon_dbg_trace(dev, "%s begins\n", name);

	siw_hal_mon_handler_self_reset(dev, name);

	if (opt & MON_FLAG_RST_ONLY) {
		goto out;
	}

	/*
	 * For other controls
	 */

out:
	t_mon_dbg_trace(dev, "%s ends\n", name);

	return 0;
}
#else	/* __SIW_SUPPORT_MON_THREAD */
static int siw_hal_mon_handler(struct device *dev, u32 opt)
{
	t_mon_dbg_trace(dev, "mon handler(noop)\n");
	return 0;
}
#endif	/* __SIW_SUPPORT_MON_THREAD */

static int siw_hal_early_probe(struct device *dev)
{
	return 0;
}

#define LCD_TYPE_A_U3_FINGER	BIT(0)
#define LCD_TYPE_A_STOP			BIT(2)

static void siw_hal_tc_cmd_set_type_a(struct siw_touch_chip *chip)
{
	int *tc_cmd_table = chip->tc_cmd_table;
	u32 ux_cmd = LCD_TYPE_A_U3_FINGER;

	tc_cmd_table[LCD_MODE_U0] = (LCD_MODE_U0<<8) | ux_cmd;
	tc_cmd_table[LCD_MODE_U2] = -1;
	tc_cmd_table[LCD_MODE_U2_UNBLANK] = -1;
	tc_cmd_table[LCD_MODE_U3] = (LCD_MODE_U3<<8) | ux_cmd;
	tc_cmd_table[LCD_MODE_U3_PARTIAL] = -1;
	tc_cmd_table[LCD_MODE_U3_QUICKCOVER] = -1;
	tc_cmd_table[LCD_MODE_STOP] = LCD_TYPE_A_STOP;
}

/*
 * still under test...
 * Bit0 Bit1
 * sync swing
 *   0    0    U3 M2 only
 *   0    1    U3 M1 & M2 swing
 *   1    0    U3 M2 only sync
 *   1    1    U3 M1 & M2 sync swing
 */
int __tc_cmd_t1_u3_opt = 0;

module_param_named(siw_tc_cmd_t1_u3_opt, __tc_cmd_t1_u3_opt, int, S_IRUGO|S_IWUSR|S_IWGRP);

__siw_setup_u32("siw_tc_cmd_t1_u3_opt=", siw_setup_tc_cmd_t1_u3_opt, __tc_cmd_t1_u3_opt);

static void siw_hal_tc_cmd_set_type_1(struct siw_touch_chip *chip)
{
	int *tc_cmd_table = chip->tc_cmd_table;

	tc_cmd_table[LCD_MODE_U0] = (LCD_MODE_U0<<8) | 0x1;
	tc_cmd_table[LCD_MODE_U2] = (LCD_MODE_U2<<8) | 0x1;
	tc_cmd_table[LCD_MODE_U2_UNBLANK] = tc_cmd_table[LCD_MODE_U2];
	tc_cmd_table[LCD_MODE_U3] = (LCD_MODE_U3<<8) | 0x1;
	tc_cmd_table[LCD_MODE_U3_PARTIAL] = 0x385;
	tc_cmd_table[LCD_MODE_U3_QUICKCOVER] = TC_DRIVE_CTL_QCOVER | 0x385;
	tc_cmd_table[LCD_MODE_STOP] = TC_DRIVE_CTL_STOP;

	/* sync option */
	if (__tc_cmd_t1_u3_opt & BIT(0)) {
		tc_cmd_table[LCD_MODE_U3] |= (0x08<<4);
	}

	/* swing option */
	if (__tc_cmd_t1_u3_opt & BIT(1)) {
		tc_cmd_table[LCD_MODE_U3] |= (0x01<<4);
	}
}

static void siw_hal_tc_cmd_set_default(struct siw_touch_chip *chip)
{
	struct siw_ts *ts = chip->ts;
	int *tc_cmd_table = chip->tc_cmd_table;
	int ctrl;

	ctrl = (TC_DRIVE_CTL_DISP_U3 | TC_DRIVE_CTL_MODE_6LHB | TC_DRIVE_CTL_START);

	switch (touch_chip_type(ts)) {
	case CHIP_SW17700:
		ctrl &= ~TC_DRIVE_CTL_DISP_U3;
		ctrl |= (1<<7);
		break;
	}

	if (touch_flags(ts) & TOUCH_USE_VBLANK)
		ctrl &= ~TC_DRIVE_CTL_MODE_6LHB;

	tc_cmd_table[LCD_MODE_U0] = TC_DRIVE_CTL_START;
	tc_cmd_table[LCD_MODE_U2] = (TC_DRIVE_CTL_DISP_U2 | TC_DRIVE_CTL_START);
	tc_cmd_table[LCD_MODE_U2_UNBLANK] = tc_cmd_table[LCD_MODE_U2];
	tc_cmd_table[LCD_MODE_U3] = ctrl;
	tc_cmd_table[LCD_MODE_U3_PARTIAL] = TC_DRIVE_CTL_PARTIAL | ctrl;
	tc_cmd_table[LCD_MODE_U3_QUICKCOVER] = TC_DRIVE_CTL_QCOVER | ctrl;
	tc_cmd_table[LCD_MODE_STOP] = TC_DRIVE_CTL_STOP;
}

static void siw_hal_tc_cmd_set(struct siw_touch_chip *chip)
{
	int t_tc_cmd = chip->opt.t_tc_cmd;

	switch (t_tc_cmd) {
	case 0xA:
		siw_hal_tc_cmd_set_type_a(chip);
		break;
	case 1:
		siw_hal_tc_cmd_set_type_1(chip);
		break;
	default:
		siw_hal_tc_cmd_set_default(chip);
		break;
	}
}

static void siw_hal_show_tc_cmd_set(struct siw_touch_chip *chip)
{
	struct siw_ts *ts = chip->ts;
	struct device *dev = chip->dev;
	int *tc_cmd_table = chip->tc_cmd_table;
	char *mode_str = NULL;
	char *ext_str = NULL;
	int ctrl = 0;
	int i = 0;

	t_dev_info(dev, "[tc cmd set] (mode bit %04Xh)\n",
		ts->mode_allowed);

	for (i = 0; i < LCD_MODE_MAX; i++) {
		mode_str = (char *)siw_lcd_driving_mode_str(i);
		ctrl = tc_cmd_table[i];

		if (ctrl < 0) {
			ext_str = "(not granted)";
		} else if (!touch_mode_allowed(ts, i)) {
			ext_str = "(not allowed)";
		} else {
			ext_str = "";
		}

		t_dev_info(dev, " %04Xh [%-13s] %s\n",
			ctrl, mode_str, ext_str);
	}
}

static void siw_hal_chipset_option(struct siw_touch_chip *chip)
{
//	struct device *dev = chip->dev;
	struct siw_touch_chip_opt *opt = &chip->opt;
	struct siw_ts *ts = chip->ts;
	int is_i2c = !!(touch_bus_type(ts) == BUS_IF_I2C);

	chip->mode_allowed_partial = !!touch_mode_allowed(ts, LCD_MODE_U3_PARTIAL);
	chip->mode_allowed_qcover = !!touch_mode_allowed(ts, LCD_MODE_U3_QUICKCOVER);

	chip->drv_reset_low = 1;
	chip->drv_delay = 20;
	chip->drv_opt_delay = 0;

	opt->t_sw_rst = SIW_SW_RST_TYPE_NONE;

	switch (touch_chip_type(ts)) {
	case CHIP_LG4894:
		chip->drv_delay = 60;

		opt->t_sw_rst = 0;
		opt->t_chk_tci_debug = 1;
		break;

	case CHIP_LG4895:
		chip->drv_opt_delay = 200;

		opt->f_u2_blank_chg = 1;
		opt->t_clock = 1;
		opt->t_sw_rst = 1;
		break;

	case CHIP_LG4946:
		opt->f_info_more = 1;
		opt->f_glove_en = 1;
		opt->t_chk_mode = 1;
		opt->t_clock = 1;
		opt->t_sw_rst = 1;
		break;

	case CHIP_SW46104:
		opt->t_boot_mode = 2;
		opt->t_sts_mask = 4;
		break;

	case CHIP_SW49105:
		opt->f_attn_opt = 1;
		opt->t_sw_rst = 2;
		opt->t_chk_sys_error = 1;
		opt->t_chk_sys_fault = 1;
		opt->t_i2c_opt = 1;
		opt->t_spi_opt = 1;
		break;

	case CHIP_SW49106:
		opt->f_attn_opt = 1;
		opt->f_glove_en = 1;
		opt->f_grab_en = 1;
		opt->t_boot_mode = 1;
		opt->t_sts_mask = 1;
		opt->t_sw_rst = 2;
		opt->t_chk_tci_debug = 1;
		break;

	case CHIP_SW49406:
		opt->f_attn_opt = 1;
		break;

	case CHIP_SW49407:
		opt->f_attn_opt = 1;
		opt->f_glove_en = 1;
		opt->f_grab_en = 1;
		opt->f_dbg_report = 1;
		opt->f_u2_blank_chg = 1;
		opt->t_sw_rst = 2;
		opt->t_clock = 2;
		opt->t_chk_mipi = 1;
		opt->t_chk_sys_error = 1;
		opt->t_chk_sys_fault = 1;
		opt->t_chk_fault = 1;
		opt->t_i2c_opt = 1;
		opt->t_spi_opt = 1;
		break;

	case CHIP_SW49408:
		opt->f_attn_opt = 1;
		opt->t_chk_mode = 1;
		opt->t_sw_rst = 2;
		opt->t_clock = 2;
		opt->t_chk_sys_error = 2;
		opt->t_chk_sys_fault = 1;
		opt->t_i2c_opt = 1;
		opt->t_spi_opt = 1;
		break;

	case CHIP_SW49409:
		opt->t_sw_rst = 2;
		opt->t_clock = 2;
		opt->t_chk_sys_error = 3;
		opt->t_chk_sys_fault = 1;
		opt->t_i2c_opt = 1;
		opt->t_spi_opt = 1;
		break;

	case CHIP_SW49501:
		opt->f_glove_en = 1;
		opt->f_grab_en = 1;
		opt->f_dbg_report = 1;
		opt->t_boot_mode = 2;
		opt->t_sts_mask = 2;
		opt->t_sw_rst = 2;
		opt->t_clock = 2;
		opt->t_chk_tci_debug = 1;
		break;

	case CHIP_SW42000:
	case CHIP_SW42000A:
		chip->drv_delay = 30;

		opt->f_attn_opt = 1;
		opt->t_sw_rst = 5;
		opt->t_clock = 4;
		opt->t_oled = 1;
		opt->t_tc_cmd = 1;
		opt->t_tc_quirk = 1;
		opt->t_lpwg = 1;
		opt->t_knock = 1;
		opt->t_swipe = 1;
		break;

	case CHIP_SW82905:
		chip->fw.sys_id_addr = 0x8014;

		chip->drv_delay = 30;

	//	opt->f_attn_opt = 1;
		opt->f_flex_report = is_i2c;
	//	opt->t_sw_rst = 5;		//TBD
		opt->t_clock = 4;
		opt->t_oled = 2;
		opt->t_tc_cmd = 0xA;
		opt->t_tc_quirk = 1;
		opt->t_lpwg = 1;
		opt->t_knock = 1;
		opt->t_swipe = 1;
		break;

	/* Large panel group */

	case CHIP_SW42101:
		opt->t_bus_opt = 1;
		break;

	case CHIP_SW1828 :
		chip->drv_reset_low = 10;	//following LGD CAS recommendation

		opt->f_ver_ext = 1;
		opt->f_dbg_report = 1;
		opt->t_chk_frame = 1;
		break;

	case CHIP_SW42103:
		chip->drv_reset_low = 10;	//following LGD CAS recommendation

		opt->f_flex_report = is_i2c;
		opt->t_boot_mode = 1;
		opt->t_sts_mask = 3;
		opt->t_sw_rst = 3;
		opt->t_chk_frame = 2;
		opt->t_rw_opt = 1;
		break;

	case CHIP_SW17700:
		chip->drv_reset_low = 10;	//following LGD CAS recommendation

		opt->f_flex_report = is_i2c;
		opt->t_boot_mode = 2;
		opt->t_sts_mask = 5;

		opt->t_sw_rst = 4;
		opt->t_chk_frame = 3;
		break;
	}

	siw_hal_tc_cmd_set(chip);
}

static void siw_hal_show_chipset_option(struct siw_touch_chip *chip)
{
	struct device *dev = chip->dev;
	struct siw_touch_chip_opt *opt = &chip->opt;
	struct chip_options {
		const char *fmt;
		int value;
		int chk;
	} *options, lists[] = {
		{	" f_info_more     : %d\n", opt->f_info_more, 0	},
		{	" f_ver_ext       : %d\n", opt->f_ver_ext, 0	},
		{	" f_attn_opt      : %d\n", opt->f_attn_opt, 0	},
		{	" f_glove_en      : %d\n", opt->f_glove_en, 0	},
		{	" f_grab_en       : %d\n", opt->f_grab_en, 0	},
		{	" f_dbg_report    : %d\n", opt->f_dbg_report, 0	},
		{	" f_u2_blank_chg  : %d\n", opt->f_u2_blank_chg, 0	},
		{	" f_flex_report   : %d\n", opt->f_flex_report, 0	},
		/* */
		{	" t_boot_mode     : %d\n", opt->t_boot_mode, 0	},
		{	" t_sts_mask      : %d\n", opt->t_sts_mask, 0	},
		{	" t_chk_mode      : %d\n", opt->t_chk_mode, 0	},
		{	" t_sw_rst        : %d\n", opt->t_sw_rst, SIW_SW_RST_TYPE_NONE	},
		{	" t_clock         : %d\n", opt->t_clock, 0	},
		{	" t_chk_mipi      : %d\n", opt->t_chk_mipi, 0	},
		{	" t_chk_frame     : %d\n", opt->t_chk_frame, 0	},
		{	" t_chk_tci_debug : %d\n", opt->t_chk_tci_debug, 0	},
		{	" t_chk_sys_error : %d\n", opt->t_chk_sys_error, 0	},
		{	" t_chk_sys_fault : %d\n", opt->t_chk_sys_fault, 0	},
		{	" t_chk_fault     : %d\n", opt->t_chk_fault, 0	},
		{	" t_oled          : %d\n", opt->t_oled, 0	},
		/* */
		{	" t_tc_cmd        : %d\n", opt->t_tc_cmd, 0	},
		{	" t_tc_quirk      : %d\n", opt->t_tc_quirk, 0	},
		{	" t_lpwg          : %d\n", opt->t_lpwg, 0	},
		{	" t_knock         : %d\n", opt->t_knock, 0	},
		{	" t_swipe         : %d\n", opt->t_swipe, 0	},
		/* */
		{	" t_bus_opt       : %d\n", opt->t_bus_opt, 0	},
		{	" t_rw_opt        : %d\n", opt->t_rw_opt, 0	},
		{	" t_i2c_opt       : %d\n", opt->t_i2c_opt, 0	},
		{	" t_spi_opt       : %d\n", opt->t_spi_opt, 0	},
		{ NULL, 0, 0	},
	};

	options = lists;

	t_dev_info(dev, "[opt summary]\n");
	while (options->fmt != NULL) {
		if (options->value != options->chk)
			t_dev_info(dev, options->fmt, options->value);

		options++;
	}

	t_dev_info(dev, " drv_reset_low   : %d ms\n", chip->drv_reset_low);
	t_dev_info(dev, " drv_delay       : %d ms\n", chip->drv_delay);
	t_dev_info(dev, " drv_opt_delay   : %d ms\n", chip->drv_opt_delay);

	t_dev_info(dev, " mode_partial    : %s\n",
		(chip->mode_allowed_partial) ? "enabled" : "disabled");
	t_dev_info(dev, " mode_qcover     : %s\n",
		(chip->mode_allowed_qcover) ? "enabled" : "disabled");

	if (chip->fw.sys_id_addr) {
		t_dev_info(dev, " fw.sys_id_addr  : 0x%X\n", chip->fw.sys_id_addr);
	}

	siw_hal_show_tc_cmd_set(chip);
}

static void siw_hal_chipset_quirk_reset(struct siw_touch_chip *chip)
{
	struct siw_ts *ts = chip->ts;
	struct device *dev = chip->dev;

	if (!(touch_flags(ts) & TOUCH_SKIP_RESET_PIN)) {
		return;
	}

	if (touch_fquirks(ts)->gpio_set_reset != NULL) {
		return;
	}

	t_dev_info(dev, "hw_reset_quirk activated\n");

	chip->ops_quirk.hw_reset = siw_hal_hw_reset_quirk;
}

static void siw_hal_chipset_quirk_fw(struct siw_touch_chip *chip)
{
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct device *dev = chip->dev;

	if (chip->opt.t_oled) {
		/* __SIW_FW_TYPE_OLED_BASE */
		t_dev_info(dev, "fw quirk activated\n");

		siw_hal_oled_fwup_setup(dev);

		fw->info_ptr = 0x01B;
		fw->gdma_crc_result = 0x05D;
		fw->gdma_crc_pass = 0x05E;
		fw->crc_fixed_value = 0x800D800D;

		switch (chip->opt.t_oled) {
		case 2:
			fw->info_ptr = 0x020;
			fw->gdma_crc_result = 0x08C;
			fw->gdma_crc_pass = 0x08D;
			break;
		}

		t_dev_dbg_fwup(dev, "info_ptr            = 0x%04X\n", fw->info_ptr);
		t_dev_dbg_fwup(dev, "gdma_crc_result     = 0x%04X\n", fw->gdma_crc_result);
		t_dev_dbg_fwup(dev, "gdma_crc_pass       = 0x%04X\n", fw->gdma_crc_pass);
		t_dev_dbg_fwup(dev, "crc_fixed_value     = 0x%08X\n", fw->crc_fixed_value);

		chip->ops_quirk.fwup_upgrade = siw_hal_oled_fwup_upgrade;
		chip->ops_quirk.boot_status = siw_hal_oled_boot_status;
	}
}

static void siw_hal_chipset_quirks(struct siw_touch_chip *chip)
{
	siw_hal_chipset_quirk_reset(chip);

	siw_hal_chipset_quirk_fw(chip);
}

static void __siw_hal_do_remove(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_power(dev, POWER_OFF);

	siw_hal_power_free(dev);
	siw_hal_free_gpios(dev);

	siw_hal_free_works(chip);
	siw_hal_free_locks(chip);

	touch_set_dev_data(ts, NULL);

	touch_kfree(dev, chip);
}

static int siw_hal_probe(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_chip *chip = NULL;
	char log_str[64] = {0, };
	int ret = 0;

	chip = touch_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		t_dev_err(dev, "failed to allocate %s data\n",
				touch_chip_name(ts));
		return -ENOMEM;
	}

	chip->dev = dev;
	chip->reg = siw_ops_reg(ts);
	chip->ts = ts;

	touch_set_dev_data(ts, chip);

	siw_hal_chipset_option(chip);

	siw_hal_chipset_quirks(chip);

	siw_hal_init_locks(chip);
	siw_hal_init_works(chip);

	siw_hal_init_gpios(dev);
	siw_hal_power_init(dev);

	siw_hal_power(dev, POWER_ON);
	siw_hal_trigger_gpio_reset(dev, 0);

	siw_hal_show_chipset_option(chip);

	if (ts->is_charger) {
		ret = siw_hal_init_charger(dev);
		goto out;
	}

	siw_hal_get_tci_info(dev);
	siw_hal_get_swipe_info(dev);

	chip->driving_mode = LCD_MODE_U3;
	chip->lcd_mode = LCD_MODE_U3;
	chip->tci_debug_type = 1;

out:
	snprintf(log_str, sizeof(log_str), "%s hal probe %s%s",
		touch_chip_name(ts),
		(ret < 0) ? "failed" : "done",
		(ts->is_charger) ? " (charger)" : "");

	t_dev_dbg_base(dev, "%s\n", log_str);

	siwmon_submit_ops_step_chip_wh_name(dev, "%s", log_str, 0);

	if (ret < 0) {
		__siw_hal_do_remove(dev);
	}

	return ret;
}

static int siw_hal_remove(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	/* Send cancel message for quirk */
	siw_hal_send_abnormal_notifier(dev, -1);

	siw_hal_sysfs_post(dev, DRIVER_FREE);

	__siw_hal_do_remove(dev);

	t_dev_dbg_base(dev, "%s remove done\n",
				touch_chip_name(ts));

	return 0;
}

#if defined(__SIW_CONFIG_SYSTEM_PM)
static int siw_hal_do_suspend(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (ts->is_charger)
		return -EPERM;

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	siw_hal_power(dev, POWER_OFF);

	return 0;
}

static int siw_hal_do_resume(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);

	siw_hal_power(dev, POWER_ON);

	siw_hal_trigger_gpio_reset(dev, 0);	//Double check for reset

	if (ts->is_charger) {
		siw_hal_init_charger(dev);
		return -EPERM;
	}

	return 0;
}
#else	/* __SIW_CONFIG_SYSTEM_PM */
static int siw_hal_do_suspend_pre(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (chip->opt.t_oled && ts->role.use_lpwg) {
		struct lpwg_info *lpwg = &ts->lpwg;
		int mode = lpwg->mode;

		lpwg->mode = (mode) ? mode : 1;
		lpwg->screen = 0;
		siw_hal_lcd_mode(dev, LCD_MODE_U0);
	}

	return 0;
}

static int siw_hal_do_suspend_post(struct device *dev)
{
	return 0;
}

static int siw_hal_do_resume_pre(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (chip->opt.t_oled) {
		siw_touch_irq_control(dev, INTERRUPT_DISABLE);

		siw_hal_power(dev, POWER_OFF);
		siw_hal_power(dev, POWER_ON);

		touch_msleep(ts->caps.hw_reset_delay);
	}

	return 0;
}

static int siw_hal_do_resume_post(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (chip->opt.t_oled) {
		struct siw_ts *ts = chip->ts;
		struct lpwg_info *lpwg = &ts->lpwg;

		lpwg->mode = 0;
		lpwg->screen = 1;
		siw_hal_lcd_mode(dev, LCD_MODE_U3);
	}

	return 0;
}

static int siw_hal_do_suspend(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int mfst_mode = 0;
	int ret = 0;

	if (ts->is_charger)
		return -EPERM;

	siw_hal_do_suspend_pre(dev);

	mfst_mode = siw_touch_boot_mode_check(dev);
	if ((mfst_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		t_dev_info(dev, "touch_suspend - MFTS\n");
		siw_touch_irq_control(dev, INTERRUPT_DISABLE);
		siw_hal_power(dev, POWER_OFF);
		return -EPERM;
	}

	if ((chip->lcd_mode == LCD_MODE_U2) &&
		siw_hal_watch_is_disp_waton(dev) &&
		siw_hal_watch_is_rtc_run(dev)) {
		siw_hal_watch_get_curr_time(dev, NULL, NULL);
	}

	if (atomic_read(&chip->init) == IC_INIT_DONE) {
		siw_hal_lpwg_mode(dev);
	} else {	/* need init */
		ret = 1;
	}

	siw_hal_do_suspend_post(dev);

	return ret;
}

static int siw_hal_do_resume(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int mfst_mode = 0;
	int ret = 0;

	siw_hal_do_resume_pre(dev);

	mfst_mode = siw_touch_boot_mode_check(dev);
	if ((mfst_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		siw_hal_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		ret = siw_hal_ic_info(dev);
		if (ret < 0) {
			t_dev_err(dev, "ic info err, %d\n", ret);
		}
		if (siw_hal_upgrade(dev) == 0) {
			siw_hal_power(dev, POWER_OFF);
			siw_hal_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
	}

	siw_hal_do_resume_post(dev);

	if (ts->is_charger) {
		siw_hal_init_charger(dev);
		return -EPERM;
	}

	return 0;
}
#endif	/* __SIW_CONFIG_SYSTEM_PM */

static int siw_hal_suspend(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		return -EPERM;
	}

	ret = siw_hal_do_suspend(dev);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_pm(dev, "%s suspend done\n",
			touch_chip_name(ts));

	return ret;
}

static int siw_hal_resume(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		return -EPERM;
	}

	ret = siw_hal_do_resume(dev);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_pm(dev, "%s resume done\n",
			touch_chip_name(ts));

	return ret;
}

static const struct siw_hal_reg siw_touch_default_reg = {
	.spr_chip_test				= SPR_CHIP_TEST,
	.spr_chip_id				= SPR_CHIP_ID,
	.spr_rst_ctl				= SPR_RST_CTL,
	.spr_boot_ctl				= SPR_BOOT_CTL,
	.spr_sram_ctl				= SPR_SRAM_CTL,
	.spr_boot_status			= SPR_BOOT_STS,
	.spr_subdisp_status			= SPR_SUBDISP_STS,
	.spr_code_offset			= SPR_CODE_OFFSET,
	.tc_ic_status				= TC_IC_STATUS,
	.tc_status					= TC_STS,
	.tc_version					= TC_VERSION,
	.tc_product_id1				= TC_PRODUCT_ID1,
	.tc_product_id2				= TC_PRODUCT_ID2,
	.tc_version_ext				= TC_VERSION_EXT,
	.info_fpc_type				= INFO_FPC_TYPE,
	.info_wfr_type				= INFO_WFR_TYPE,
	.info_chip_version			= INFO_CHIP_VERSION,
	.info_cg_type				= INFO_CG_TYPE,
	.info_lot_num				= INFO_LOT_NUM,
	.info_serial_num			= INFO_SERIAL_NUM,
	.info_date					= INFO_DATE,
	.info_time					= INFO_TIME,
	.cmd_abt_loc_x_start_read	= CMD_ABT_LOC_X_START_READ,
	.cmd_abt_loc_x_end_read		= CMD_ABT_LOC_X_END_READ,
	.cmd_abt_loc_y_start_read	= CMD_ABT_LOC_Y_START_READ,
	.cmd_abt_loc_y_end_read		= CMD_ABT_LOC_Y_END_READ,
	.code_access_addr			= CODE_ACCESS_ADDR,
	.data_i2cbase_addr			= DATA_I2CBASE_ADDR,
	.prd_tcm_base_addr			= PRD_TCM_BASE_ADDR,
	.tc_device_ctl				= TC_DEVICE_CTL,
	.tc_interrupt_ctl			= TC_INTERRUPT_CTL,
	.tc_interrupt_status		= TC_INTERRUPT_STS,
	.tc_drive_ctl				= TC_DRIVE_CTL,
	.tci_enable_w				= TCI_ENABLE_W,
	.tap_count_w				= TAP_COUNT_W,
	.min_intertap_w				= MIN_INTERTAP_W,
	.max_intertap_w				= MAX_INTERTAP_W,
	.touch_slop_w				= TOUCH_SLOP_W,
	.tap_distance_w				= TAP_DISTANCE_W,
	.int_delay_w				= INT_DELAY_W,
	.act_area_x1_w				= ACT_AREA_X1_W,
	.act_area_y1_w				= ACT_AREA_Y1_W,
	.act_area_x2_w				= ACT_AREA_X2_W,
	.act_area_y2_w				= ACT_AREA_Y2_W,
	.tci_debug_fail_ctrl		= TCI_DEBUG_FAIL_CTRL,
	.tci_debug_fail_buffer		= TCI_DEBUG_FAIL_BUFFER,
	.tci_debug_fail_status		= TCI_DEBUG_FAIL_STATUS,
	.swipe_enable_w				= SWIPE_ENABLE_W,
	.swipe_dist_w				= SWIPE_DIST_W,
	.swipe_ratio_thr_w			= SWIPE_RATIO_THR_W,
	.swipe_ratio_period_w		= SWIPE_RATIO_PERIOD_W,
	.swipe_ratio_dist_w			= SWIPE_RATIO_DIST_W,
	.swipe_time_min_w			= SWIPE_TIME_MIN_W,
	.swipe_time_max_w			= SWIPE_TIME_MAX_W,
	.swipe_act_area_x1_w		= SWIPE_ACT_AREA_X1_W,
	.swipe_act_area_y1_w		= SWIPE_ACT_AREA_Y1_W,
	.swipe_act_area_x2_w		= SWIPE_ACT_AREA_X2_W,
	.swipe_act_area_y2_w		= SWIPE_ACT_AREA_Y2_W,
	.swipe_fail_debug_r			= SWIPE_FAIL_DEBUG_R,
	.swipe_debug_r				= SWIPE_DEBUG_R,
	.cmd_raw_data_report_mode_read	= CMD_RAW_DATA_REPORT_MODE_READ,
	.cmd_raw_data_compress_write	= CMD_RAW_DATA_COMPRESS_WRITE,
	.cmd_raw_data_report_mode_write	= CMD_RAW_DATA_REPORT_MODE_WRITE,
	.spr_charger_status				= SPR_CHARGER_STS,
	.ime_state					= IME_STATE,
	.max_delta					= MAX_DELTA,
	.touch_max_w				= TOUCH_MAX_W,
	.touch_max_r				= TOUCH_MAX_R,
	.call_state					= CALL_STATE,
	.tc_tsp_test_ctl			= TC_TSP_TEST_CTL,
	.tc_tsp_test_status			= TC_TSP_TEST_STS,
	.tc_tsp_test_pf_result		= TC_TSP_TEST_PF_RESULT,
	.tc_tsp_test_off_info		= TC_TSP_TEST_OFF_INFO,
	.tc_flash_dn_status			= TC_FLASH_DN_STS,
	.tc_confdn_base_addr		= TC_CONFDN_BASE_ADDR,
	.tc_flash_dn_ctl			= TC_FLASH_DN_CTL,
	.raw_data_ctl_read			= RAW_DATA_CTL_READ,
	.raw_data_ctl_write			= RAW_DATA_CTL_WRITE,
	.serial_data_offset			= SERIAL_DATA_OFFSET,
	/* __SIW_SUPPORT_WATCH */
	.ext_watch_font_offset		= EXT_WATCH_FONT_OFFSET,
	.ext_watch_font_addr		= EXT_WATCH_FONT_ADDR,
	.ext_watch_font_dn_addr_info = EXT_WATCH_FONT_DN_ADDR_INFO,
	.ext_watch_font_crc			= EXT_WATCH_FONT_CRC,
	.ext_watch_dcs_ctrl			= EXT_WATCH_DCS_CTRL,
	.ext_watch_mem_ctrl			= EXT_WATCH_MEM_CTRL,
	.ext_watch_ctrl				= EXT_WATCH_CTRL,
	.ext_watch_area_x			= EXT_WATCH_AREA_X,
	.ext_watch_area_y			= EXT_WATCH_AREA_Y,
	.ext_watch_blink_area		= EXT_WATCH_BLINK_AREA,
	.ext_watch_lut				= EXT_WATCH_LUT,
	.ext_watch_display_on		= EXT_WATCH_DISPLAY_ON,
	.ext_watch_display_status	= EXT_WATCH_DISPLAY_STATUS,
	.ext_watch_rtc_sct			= EXT_WATCH_RTC_SCT,
	.ext_watch_rtc_sctcnt		= EXT_WATCH_RTC_SCTCNT,
	.ext_watch_rtc_capture		= EXT_WATCH_RTC_CAPTURE,
	.ext_watch_rtc_ctst			= EXT_WATCH_RTC_CTST,
	.ext_watch_rtc_ecnt			= EXT_WATCH_RTC_ECNT,
	.ext_watch_hour_disp		= EXT_WATCH_HOUR_DISP,
	.ext_watch_blink_prd		= EXT_WATCH_BLINK_PRD,
	.ext_watch_rtc_run			= EXT_WATCH_RTC_RUN,
	.ext_watch_position			= EXT_WATCH_POSITION,
	.ext_watch_position_r		= EXT_WATCH_POSITION_R,
	.ext_watch_state			= EXT_WATCH_STATE,
	.sys_dispmode_status		= SYS_DISPMODE_STATUS,
	/* __SIW_SUPPORT_PRD */
	.prd_serial_tcm_offset		= PRD_SERIAL_TCM_OFFSET,
	.prd_tc_mem_sel				= PRD_TC_MEM_SEL,
	.prd_tc_test_mode_ctl		= PRD_TC_TEST_MODE_CTL,
	.prd_m1_m2_raw_offset		= PRD_M1_M2_RAW_OFFSET,
	.prd_tune_result_offset		= PRD_TUNE_RESULT_OFFSET,
	.prd_open3_short_offset		= PRD_OPEN3_SHORT_OFFSET,
	.prd_ic_ait_start_reg		= PRD_IC_AIT_START_REG,
	.prd_ic_ait_data_readystatus= PRD_IC_AIT_DATA_READYSTATUS,
	/* */
	.glove_en					= GLOVE_EN,
	.grab_en					= GRAB_EN,
};

enum {
	HAL_MON_INTERVAL_DEFAULT = MON_INTERVAL_DEFAULT,
};

static const struct siw_touch_operations siw_touch_default_ops = {
	/* Register Map */
	.reg				= (void *)&siw_touch_default_reg,
	/* Functions */
	.early_probe		= siw_hal_early_probe,
	.probe				= siw_hal_probe,
	.remove				= siw_hal_remove,
	.suspend			= siw_hal_suspend,
	.resume				= siw_hal_resume,
	.init				= siw_hal_init,
	.reset				= siw_hal_reset_ctrl,
	.ic_info			= siw_hal_ic_info,
	.tc_con				= siw_hal_tc_con,
	.tc_driving			= siw_hal_tc_driving,
	.chk_status			= siw_hal_check_status,
	.irq_handler		= siw_hal_irq_handler,
	.irq_abs			= siw_hal_irq_abs,
	.irq_lpwg			= siw_hal_irq_lpwg,
	.power				= siw_hal_power,
	.upgrade			= siw_hal_upgrade,
	.lpwg				= siw_hal_lpwg,
	.notify				= siw_hal_notify,
	.set				= siw_hal_set,
	.get				= siw_hal_get,
	/* */
	.sysfs				= siw_hal_sysfs,
	/* */
	.mon_handler		= siw_hal_mon_handler,
	.mon_interval		= HAL_MON_INTERVAL_DEFAULT,
	/* */
	.abt_sysfs			= siw_hal_abt_sysfs,
	.prd_sysfs			= siw_hal_prd_sysfs,
	.watch_sysfs		= siw_hal_watch_sysfs,
};

struct siw_hal_reg *siw_hal_get_default_reg(int opt)
{
	return (struct siw_hal_reg *)&siw_touch_default_reg;
}

struct siw_touch_operations *siw_hal_get_default_ops(int opt)
{
	return (struct siw_touch_operations *)&siw_touch_default_ops;
}


