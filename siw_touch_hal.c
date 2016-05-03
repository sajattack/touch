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


//#define __FW_VERIFY_TEST


extern int siw_hal_sysfs(struct device *dev, int on_off);

#if defined(__SIW_SUPPORT_ABT)	//See siw_touch_cfg.h
extern int siw_hal_abt_init(struct device *dev);
extern int siw_hal_abt_sysfs(struct device *dev, int on_off);
#else
static int siw_hal_abt_init(struct device *dev){ return 0; }
static int siw_hal_abt_sysfs(struct device *dev, int on_off){ return 0; }
#endif	/* __SIW_SUPPORT_ABT */

#if defined(__SIW_SUPPORT_PRD)	//See siw_touch_cfg.h
extern int siw_hal_prd_sysfs(struct device *dev, int on_off);
#else
static int siw_hal_prd_sysfs(struct device *dev, int on_off){ return 0; }
#endif	/* __SIW_SUPPORT_PRD */

#if defined(__SIW_SUPPORT_WATCH)	//See siw_touch_cfg.h
extern int siw_hal_watch_sysfs(struct device *dev, int on_off);
extern int siw_hal_watch_init(struct device *dev);
extern int siw_hal_watch_chk_font_status(struct device *dev);
extern int siw_hal_watch_get_curr_time(struct device *dev, char *buf, int *len);
extern int siw_hal_watch_display_off(struct device *dev);
extern int siw_hal_watch_is_disp_waton(struct device *dev);
extern int siw_hal_watch_is_rtc_run(struct device *dev);
extern void siw_hal_watch_set_rtc_run(struct device *dev);
extern void siw_hal_watch_set_rtc_clear(struct device *dev);
extern void siw_hal_watch_set_font_empty(struct device *dev);
#else	/* __SIW_SUPPORT_WATCH */
static int __used siw_hal_watch_sysfs(struct device *dev, int on_off){ return 0; }
static int __used siw_hal_watch_init(struct device *dev){ return 0; }
static int __used siw_hal_watch_chk_font_status(struct device *dev){ return 0; }
static int __used siw_hal_watch_get_curr_time(struct device *dev, char *buf, int *len){ return 0; }
static int __used siw_hal_watch_display_off(struct device *dev){ return 0; }
static int __used siw_hal_watch_is_disp_waton(struct device *dev){ return 0; }
static int __used siw_hal_watch_is_rtc_run(struct device *dev){ return 0; }
extern void __used siw_hal_watch_set_rtc_run(struct device *dev){ }
extern void __used siw_hal_watch_set_rtc_clear(struct device *dev){ }
extern void __used siw_hal_watch_set_font_empty(struct device *dev){ }
#endif	/* __SIW_SUPPORT_WATCH */


static int siw_hal_reset_ctrl(struct device *dev, int ctrl);

static int siw_hal_tc_driving(struct device *dev, int mode);


#define t_hal_bus_info(_dev, fmt, args...)	\
		__t_dev_info(_dev, "hal(bus) : " fmt, ##args)

#define t_hal_bus_err(_dev, fmt, args...)	\
		__t_dev_err(_dev, "hal(bus) : " fmt, ##args)

#define t_hal_bus_warn(_abt, fmt, args...)	\
		__t_dev_warn(_dev, "hal(bus) : " fmt, ##args)


#define TCI_FAIL_NUM 11
static const char const *siw_hal_tci_debug_str[TCI_FAIL_NUM] = {
	"NONE",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP_LONG",
	"MULTI_FINGER",
	"DELAY_TIME",/* It means Over Tap */
	"TIMEOUT_INTER_TAP_SHORT",
	"PALM_STATE",
	"TAP_TIMEOVER",
	"DEBUG9",
	"DEBUG10"
};

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

static void siw_hal_deep_sleep(struct device *dev);

static int siw_hal_lpwg_mode(struct device *dev);

static void siw_hal_power_init(struct device *dev)
{
	siw_touch_power_init(dev);
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

static void siw_hal_set_gpio_reset(struct device *dev, int val)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);

	siw_touch_gpio_direction_output(dev,
			reset_pin, !!(val));
	t_dev_dbg_gpio(dev, "set %s(%d) : %d\n",
			SIW_HAL_GPIO_RST,
			reset_pin, !!(val));
}

static void siw_hal_init_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);
	int ret = 0;

	ret = siw_touch_gpio_init(dev,
			reset_pin,
			SIW_HAL_GPIO_RST);
	if (ret)
		return;

	siw_touch_gpio_direction_output(dev,
			reset_pin, GPIO_OUT_ONE);
	t_dev_dbg_gpio(dev, "set %s(%d) as output\n",
			SIW_HAL_GPIO_RST, reset_pin);
}

static void siw_hal_free_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);

	siw_touch_gpio_free(dev, reset_pin);
}

static void siw_hal_init_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int irq_pin = touch_irq_pin(ts);
	int ret = 0;

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

static void siw_hal_free_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int irq_pin = touch_irq_pin(ts);

	siw_touch_gpio_free(dev, irq_pin);
}

static void siw_hal_init_gpio_maker_id(struct device *dev)
{
#if 0
	struct siw_ts *ts = to_touch_core(dev);
	int maker_id_pin = touch_maker_id_pin(ts);

	int ret = 0;

	ret = siw_touch_gpio_init(dev,
			maker_id_pin,
			SIW_HAL_GPIO_MAKER, ts->addr);
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

static int __used __siw_hal_do_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int bus_tx_hdr_size = touch_tx_hdr_size(ts);
	int bus_rx_hdr_size = touch_rx_hdr_size(ts);
//	int bus_tx_dummy_size = touch_tx_dummy_size(ts);
	int bus_rx_dummy_size = touch_rx_dummy_size(ts);
	struct touch_bus_msg _msg;
	struct touch_bus_msg *msg = &_msg;
	int tx_size = bus_tx_hdr_size;
	int ret = 0;

#if 0
	if (!addr) {
		t_dev_err(dev, "NULL addr\n");
		return -EFAULT;
	}
#endif
	if (!data) {
		t_dev_err(dev, "NULL data\n");
		return -EFAULT;
	}

//	t_dev_info(dev, "addr %04Xh, size %d\n", addr, size);

	ts->tx_buf[0] = ((size > 4) ? 0x20 : 0x00);
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr & 0xff);
//	while (bus_tx_dummy_size--) {
	while (bus_rx_dummy_size--) {
		ts->tx_buf[tx_size++] = 0;
	}

	msg->tx_buf = ts->tx_buf;
	msg->tx_size = tx_size;
	msg->rx_buf = ts->rx_buf;
	msg->rx_size = bus_rx_hdr_size + size;
	msg->bits_per_word = 8;
	msg->priv = 0;

	ret = siw_touch_bus_read(dev, msg);
	if (ret < 0) {
		t_dev_err(dev, "touch bus read error(0x%04X, 0x%04X), %d\n",
				(u32)addr, (u32)size, ret);
		return ret;
	}

	memcpy(data, &ts->rx_buf[bus_rx_hdr_size], size);

	return size;
}

static int __used __siw_hal_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

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
	struct touch_bus_msg _msg;
	struct touch_bus_msg *msg = &_msg;
	int ret = 0;

#if 0
	if (!addr) {
		t_dev_err(dev, "NULL addr\n");
		return -EFAULT;
	}
#endif
	if (!data) {
		t_dev_err(dev, "NULL data\n");
		return -EFAULT;
	}

	ts->tx_buf[0] = (touch_bus_type(ts) == BUS_IF_SPI)? 0x60 :
					((size > 4) ? 0x60 : 0x40);
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr  & 0xff);

	msg->tx_buf = ts->tx_buf;
	msg->tx_size = bus_tx_hdr_size + size;
	msg->rx_buf = NULL;
	msg->rx_size = 0;
	msg->bits_per_word = 8;
	msg->priv = 0;

	memcpy(&ts->tx_buf[bus_tx_hdr_size], data, size);

	ret = siw_touch_bus_write(dev, msg);
	if (ret < 0) {
		t_dev_err(dev, "touch bus write error(0x%04X, 0x%04X), %d\n",
				(u32)addr, (u32)size, ret);
		return ret;
	}

	return size;
}

static int __used __siw_hal_reg_write(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	mutex_lock(&chip->bus_lock);
	ret = __siw_hal_do_reg_write(dev, addr, data, size);
	mutex_unlock(&chip->bus_lock);

	return ret;
}

static void __used __siw_hal_do_xfer_dbg(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	int i;

	for (i = 0; i < xfer->msg_count; i++) {
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;

		t_dev_err(dev, "[%d] rx(0x%04X, 0x%04X) tx(0x%04X, 0x%04X)\n",
				i,
				(u32)rx->addr, (u32)rx->size,
				(u32)tx->addr, (u32)tx->size);
	}
}

static void __siw_hal_do_xfer_to_single_interval(struct device *dev)
{
#if defined(CONFIG_TOUCHSCREEN_SIWMON)
	if (siw_mon_ops && siw_mon_ops->submit_bus) {
		usleep_range(100, 100);
	}
#endif
}

static int __used __siw_hal_do_xfer_to_single(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	int i = 0;
	int ret = 0;

	for (i = 0; i < xfer->msg_count; i++) {
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;

		if (rx->size) {
			ret = __siw_hal_do_reg_read(dev, rx->addr, rx->buf, rx->size);
			t_dev_dbg_trace(dev, "xfer single [%d/%d] - read(%04Xh, %d), %d\n",
				i, xfer->msg_count, rx->addr, rx->size, ret);
			if (ret < 0) {
				return ret;
			}
		} else if (tx->size) {
			ret = __siw_hal_do_reg_write(dev, tx->addr, tx->buf, tx->size);
			t_dev_dbg_trace(dev, "xfer single [%d/%d] - wr(%04Xh, %d), %d\n",
				i, xfer->msg_count, rx->addr, rx->size, ret);
			if (ret < 0) {
				return ret;
			}
		}

		__siw_hal_do_xfer_to_single_interval(dev);
	}

	return 0;
}

static int __used __siw_hal_do_xfer_msg(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	int bus_tx_hdr_size = touch_tx_hdr_size(ts);
	int bus_rx_hdr_size = touch_rx_hdr_size(ts);
//	int bus_tx_dummy_size = touch_tx_dummy_size(ts);
	int bus_rx_dummy_size = touch_rx_dummy_size(ts);
	int bus_dummy;
	int buf_size = touch_buf_size(ts);
	int tx_size;
	int i = 0;
	int ret = 0;

	if (!touch_xfer_allowed(ts)) {
		return __siw_hal_do_xfer_to_single(dev, xfer);
	}

	if (!buf_size)
		buf_size = SIW_TOUCH_MAX_XFER_BUF_SIZE;

	t_dev_dbg_base(dev, "xfer: start\n");

	for (i = 0; i < xfer->msg_count; i++) {
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;

		if (rx->size) {
			t_dev_dbg_base(dev, "xfer: rd set(%d)\n", i);
		#if 0
			if (!rx->addr) {
				t_dev_err(dev, "NULL xfer rx->addr(%i)\n", i);
				__siw_hal_do_xfer_dbg(dev, xfer);
				return -EFAULT;
			}
		#endif
			tx_size = bus_tx_hdr_size;
			bus_dummy = bus_rx_dummy_size;

			tx->data[0] = (rx->size > 4) ? 0x20 : 0x00;
			tx->data[0] |= ((rx->addr >> 8) & 0x0f);
			tx->data[1] = (rx->addr & 0xff);
			while (bus_dummy--) {
				tx->data[tx_size++] = 0;
			}
			tx->size = tx_size;
			rx->size += bus_rx_hdr_size;
			continue;
		}

		t_dev_dbg_base(dev, "xfer: wr set(%d)\n", i);

	#if 0
		if (!tx->addr) {
			t_dev_err(dev, "NULL xfer tx->addr(%i)\n", i);
			__siw_hal_do_xfer_dbg(dev, xfer);
			return -EFAULT;
		}
	#endif

		if (tx->size > (buf_size - bus_tx_hdr_size)) {
			t_dev_err(dev, "buffer overflow\n");
			return -EOVERFLOW;
		}

	//	tx->data[0] = ((tx->size == 1) ? 0x60 : 0x40);
		tx->data[0] = 0x60;
		tx->data[0] |= ((tx->addr >> 8) & 0x0f);
		tx->data[1] = (tx->addr  & 0xff);
		memcpy(&tx->data[bus_tx_hdr_size], tx->buf, tx->size);
		tx->size += bus_tx_hdr_size;
	}

	t_dev_dbg_base(dev, "xfer: call bus xfer\n");

	ret = siw_touch_bus_xfer(dev, xfer);
	if (ret < 0) {
		t_dev_err(dev, "touch bus xfer error, %d\n", ret);
		__siw_hal_do_xfer_dbg(dev, xfer);
		return ret;
	}

	ret = 0;
	for (i = 0; i < xfer->msg_count; i++) {
		rx = &xfer->data[i].rx;

		if (rx->size) {
			if (!rx->buf) {
				t_dev_err(dev, "NULL xfer->data[%d].rx.buf\n", i);
				return -EFAULT;
			}
			memcpy(rx->buf, rx->data + bus_rx_hdr_size,
				(rx->size - bus_rx_hdr_size));
		}
		ret += rx->size;
	}

	return ret;
}

static int __used __siw_hal_xfer_msg(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	mutex_lock(&chip->bus_lock);
	ret = __siw_hal_do_xfer_msg(dev, xfer);
	mutex_unlock(&chip->bus_lock);

	return ret;
}


int siw_hal_read_value(struct device *dev, u32 addr, u32 *value)
{
	int ret = __siw_hal_reg_read(dev, addr, value, sizeof(u32));
	if (ret < 0)
		t_hal_bus_err(dev, "read val err[%03Xh, 0x%X], %d",
				addr, *value, ret);
	return ret;
}

int siw_hal_write_value(struct device *dev, u32 addr, u32 value)
{
	int ret = __siw_hal_reg_write(dev, addr, &value, sizeof(u32));
	if (ret < 0)
		t_hal_bus_err(dev, "write val err[%03Xh, 0x%X], %d",
				addr, value, ret);
	return ret;
}

int siw_hal_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	int ret = __siw_hal_reg_read(dev, addr, data, size);
	if (ret < 0)
		t_hal_bus_err(dev, "read reg err[%03Xh, 0x%X], %d",
				addr, ((u32 *)data)[0], ret);
	return ret;
}

int siw_hal_reg_write(struct device *dev, u32 addr, void *data, int size)
{
	int ret = __siw_hal_reg_write(dev, addr, data, size);
	if (ret < 0)
		t_hal_bus_err(dev, "write reg err[%03Xh, 0x%X], %d",
				addr, ((u32 *)data)[0], ret);
	return ret;
}

void siw_hal_xfer_init(struct device *dev, void *xfer_data)
{
	struct touch_xfer_msg *xfer = xfer_data;
	struct siw_touch_chip *chip = to_touch_chip(dev);

	mutex_lock(&chip->bus_lock);
	xfer->bits_per_word = 8;
	xfer->msg_count = 0;
	mutex_unlock(&chip->bus_lock);
}

void siw_hal_xfer_add_rx(void *xfer_data, u32 reg, void *buf, u32 size)
{
	struct touch_xfer_msg *xfer = xfer_data;
	struct touch_xfer_data_t *rx = &xfer->data[xfer->msg_count].rx;

	if (xfer->msg_count >= SIW_TOUCH_MAX_XFER_COUNT) {
		t_pr_err("msg_count overflow\n");
		return;
	}

	rx->addr = reg;
	rx->buf = buf;
	rx->size = size;

	xfer->msg_count++;
}

void siw_hal_xfer_add_rx_seq(void *xfer_data, u32 reg, u32 *data, int cnt)
{
	struct touch_xfer_msg *xfer = xfer_data;
	int i;

	for (i=0 ; i<cnt ; i++) {
		siw_hal_xfer_add_rx(xfer,
				reg + i,
				(u8 *)&(data[i]), sizeof(u32));
	}
}

void siw_hal_xfer_add_tx(void *xfer_data, u32 reg, void *buf, u32 size)
{
	struct touch_xfer_msg *xfer = xfer_data;
	struct touch_xfer_data_t *tx = &xfer->data[xfer->msg_count].tx;

	if (xfer->msg_count >= SIW_TOUCH_MAX_XFER_COUNT) {
		t_pr_err("msg_count overflow\n");
		return;
	}

	tx->addr = reg;
	tx->buf = buf;
	tx->size = size;

	xfer->msg_count++;
}

void siw_hal_xfer_add_tx_seq(void *xfer_data, u32 reg, u32 *data, int cnt)
{
	struct touch_xfer_msg *xfer = xfer_data;
	int i;

	for (i=0 ; i<cnt ; i++) {
		siw_hal_xfer_add_tx(xfer,
				reg + i,
				(u8 *)&(data[i]), sizeof(u32));
	}
}

int siw_hal_xfer_msg(struct device *dev, struct touch_xfer_msg *xfer)
{
	return __siw_hal_xfer_msg(dev, xfer);
}

int siw_hal_xfer_rx_seq(struct device *dev, u32 reg, u32 *data, int cnt)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct touch_xfer_msg *xfer = ts->xfer;

	siw_hal_xfer_init(dev, xfer);
	siw_hal_xfer_add_rx_seq(xfer, reg, data, cnt);
	return siw_hal_xfer_msg(dev, xfer);
}

int siw_hal_xfer_tx_seq(struct device *dev, u32 reg, u32 *data, int cnt)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct touch_xfer_msg *xfer = ts->xfer;

	siw_hal_xfer_init(dev, xfer);
	siw_hal_xfer_add_tx_seq(xfer, reg, data, cnt);
	return siw_hal_xfer_msg(dev, xfer);
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

static void siw_hal_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct siw_touch_chip *chip =
			container_of(to_delayed_work(fb_notify_work),
				struct siw_touch_chip, fb_notify_work);
	int type = 0;

	type = (chip->lcd_mode == LCD_MODE_U3)? FB_RESUME : FB_SUSPEND;

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


const struct tci_info siw_hal_tci_info_default[2] = {
	[TCI_1] = {
		.tap_count		= 2,
		.min_intertap	= 6,
		.max_intertap	= 70,
		.touch_slop		= 100,
		.tap_distance	= 10,
		.intr_delay		= 0,
	},
	[TCI_2] = {
		.tap_count		= 0,
		.min_intertap	= 6,
		.max_intertap	= 70,
		.touch_slop		= 100,
		.tap_distance	= 255,
		.intr_delay		= 20,
	},
};

static const struct reset_area siw_hal_tci_reset_area_default = {
	.x1	= ((65<<16) | 65),
	.y1 = ((1374<<16) | 1374),
	.x2 = ((65<<16) | 65),
	.y2 = ((2494<<16) | 2494),
};

static void siw_hal_get_tci_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	void *tci_src;
	void *tci_reset_area;
	struct reset_area *tci_qcover;

	tci_src = pdata_tci_info(ts->pdata);
	if (!tci_src)
		tci_src = (void *)siw_hal_tci_info_default;

	tci_reset_area = pdata_tci_reset_area(ts->pdata);
	if (!tci_reset_area)
		tci_reset_area = (void *)&siw_hal_tci_reset_area_default;

	memcpy(ts->tci.info, tci_src, sizeof(siw_hal_tci_info_default));
	memcpy(&ts->tci.rst_area, tci_reset_area, sizeof(siw_hal_tci_reset_area_default));

	tci_qcover = pdata_tci_qcover_open(ts->pdata);
	if (!tci_qcover) {
		memset(&ts->tci.qcover_open, ~0, sizeof(struct reset_area));
	} else {
		memcpy(&ts->tci.qcover_open, tci_qcover, sizeof(struct reset_area));
	}
	tci_qcover = &ts->tci.qcover_open;
	t_dev_dbg_base(dev, "qcover_open : %Xh %Xh %Xh %Xh\n",
			tci_qcover->x1, tci_qcover->y1, tci_qcover->x2, tci_qcover->y2);

	tci_qcover = pdata_tci_qcover_close(ts->pdata);
	if (!tci_qcover) {
		memset((void *)&ts->tci.qcover_close, ~0, sizeof(struct reset_area));
	} else {
		memcpy(&ts->tci.qcover_close, tci_qcover, sizeof(struct reset_area));
	}
	tci_qcover = &ts->tci.qcover_close;
	t_dev_dbg_base(dev, "qcover_close: %Xh %Xh %Xh %Xh\n",
			tci_qcover->x1, tci_qcover->y1, tci_qcover->x2, tci_qcover->y2);
}

const struct siw_hal_swipe_ctrl siw_hal_swipe_info_default = {
	.mode	= SWIPE_LEFT_BIT | SWIPE_RIGHT_BIT,
	.info = {
		[SWIPE_R] = {
			.distance		= 5,
			.ratio_thres	= 100,
			.ratio_distance	= 2,
			.ratio_period	= 5,
			.min_time		= 0,
			.max_time		= 150,
			.area.x1		= 401,
			.area.y1		= 0,
			.area.x2		= 1439,
			.area.y2		= 159,
		},
		[SWIPE_L] = {
			.distance		= 5,
			.ratio_thres	= 100,
			.ratio_distance	= 2,
			.ratio_period	= 5,
			.min_time		= 0,
			.max_time		= 150,
			.area.x1		= 401,	/* 0 */
			.area.y1		= 0,	/* 2060 */
			.area.x2		= 1439,
			.area.y2		= 159,	/* 2559 */
		},
	},
};

static void siw_hal_get_swipe_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	void *swipe_src;

	swipe_src = pdata_swipe_ctrl(ts->pdata);
	if (!swipe_src)
		swipe_src = (void *)&siw_hal_swipe_info_default;

	memcpy(&chip->swipe, swipe_src, sizeof(struct siw_hal_swipe_ctrl));
}

static const char *siw_hal_pwr_name[] = {
	[POWER_OFF]		= "Power off",
	[POWER_SLEEP]	= "Power sleep",
	[POWER_WAKE]	= "Power wake",
	[POWER_ON]		= "Power on",
};

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
		siw_hal_power_vio(dev, 0);
		siw_hal_power_vdd(dev, 0);
		touch_msleep(1);

		siw_hal_watch_set_font_empty(dev);
		break;

	case POWER_ON:
		t_dev_dbg_pm(dev, "power ctrl: power on\n");
		siw_hal_power_vdd(dev, 1);
		siw_hal_power_vio(dev, 1);
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
	}

	return 0;
}

static void siw_hal_ic_info_abnormal(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	t_dev_info(dev, "[%s] FW is in abnormal state\n",
			touch_chip_name(ts));
#if 1
	siw_hal_reset_ctrl(dev, HW_RESET_ASYNC);
#else
	siw_hal_power(dev, POWER_OFF);
	siw_hal_power(dev, POWER_ON);
	touch_msleep(ts->caps.hw_reset_delay);
#endif
}

static int siw_hal_ic_info_ver_check(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u32 version = chip->fw.version_raw;
	int ret = 0;

	switch (touch_chip_type(ts)) {
		case CHIP_LG4894 :
			if ((((version >> 16) & 0xFF) != 4) ||
				(((version >> 24) & 0xFF) != 4)) {
				siw_hal_ic_info_abnormal(dev);
				ret = -EFAULT;
			}
			break;
		case CHIP_LG4895 :
			break;
		case CHIP_LG4946 :
			break;
		case CHIP_SW1828 :
			if ((((version >> 16) & 0xFF) != 9) ||
				(((version >> 24) & 0xFF) != 4)) {
				siw_hal_ic_info_abnormal(dev);
				ret = -EFAULT;
			}
			break;
		default :
			t_dev_info(dev, "[%s] abnormal chip type\n",
				touch_chip_name(ts));
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int siw_hal_ic_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 product[2] = {0};
	u32 chip_id = 0;
	u32 version = 0;
	u32 revision = 0;
	u32 bootmode = 0;
	int ret = 0;

	{
		struct touch_xfer_msg *xfer = ts->xfer;

		siw_hal_xfer_init(dev, xfer);

		siw_hal_xfer_add_rx(xfer,
				reg->spr_chip_id,
				(u8 *)&chip_id, sizeof(chip_id));
		siw_hal_xfer_add_rx(xfer,
				reg->tc_version,
				(u8 *)&version, sizeof(version));
		siw_hal_xfer_add_rx(xfer,
				reg->info_chip_version,
				(u8 *)&revision, sizeof(revision));
		siw_hal_xfer_add_rx(xfer,
				reg->tc_product_id1,
				(u8 *)&product[0], sizeof(product));
		siw_hal_xfer_add_rx(xfer,
				reg->spr_boot_status,
				(u8 *)&bootmode, sizeof(bootmode));

		switch (touch_chip_type(ts)) {
		case CHIP_LG4946:
			siw_hal_xfer_add_rx(xfer,
					reg->info_fpc_type,
					(u8 *)&fw->fpc, sizeof(fw->fpc));
			siw_hal_xfer_add_rx(xfer,
					reg->info_wfr_type,
					(u8 *)&fw->wfr, sizeof(fw->wfr));
			siw_hal_xfer_add_rx(xfer,
					reg->info_cg_type,
					(u8 *)&fw->cg, sizeof(fw->cg));
			siw_hal_xfer_add_rx(xfer,
					reg->info_lot_num,
					(u8 *)&fw->lot, sizeof(fw->lot));
			siw_hal_xfer_add_rx(xfer,
					reg->info_serial_num,
					(u8 *)&fw->sn, sizeof(fw->sn));
			siw_hal_xfer_add_rx(xfer,
					reg->info_date,
					(u8 *)&fw->date, sizeof(fw->date));
			siw_hal_xfer_add_rx(xfer,
					reg->info_time,
					(u8 *)&fw->time, sizeof(fw->time));
			break;
		}
	}
	ret = siw_hal_xfer_msg(dev, ts->xfer);
	if (ret < 0) {
		t_dev_err(dev, "ic_info : xfer failed, %d\n", ret);
		return ret;
	}

	siw_hal_fw_set_chip_id(fw, chip_id);
	siw_hal_fw_set_version(fw, version);
	siw_hal_fw_set_revision(fw, revision);
	siw_hal_fw_set_prod_id(fw, (u8 *)product, sizeof(product));

	fw->wfr &= WAFER_TYPE_MASK;

	t_dev_info(dev, "[T] chip id    : %s\n", chip->fw.chip_id);
	t_dev_info(dev, "[T] version    : v%u.%02u (0x%08X, 0x%02X)\n",
			(u32)(fw->version[0]), (u32)(fw->version[1]),
			version, fw->revision);
	t_dev_info(dev, "[T] product id : %s\n", fw->product_id);
	t_dev_info(dev, "[T] flash boot : %s(%s), crc : %s (0x%08X)\n",
			(bootmode >> 1 & 0x1) ? "BUSY" : "idle",
			(bootmode >> 2 & 0x1) ? "done" : "booting",
			(bootmode >> 3 & 0x1) ? "ERROR" : "ok",
			bootmode);

	switch (touch_chip_type(ts)) {
	case CHIP_LG4946:
		t_dev_info(dev, "[T] lot        : %d\n", fw->lot);
		t_dev_info(dev, "[T] sn         : %Xh\n", fw->sn);
		t_dev_info(dev, "[T] date       : %04d.%02d.%02d\n",
			fw->date & 0xFFFF, ((fw->date>>16) & 0xFF), ((fw->date>>24) & 0xFF));
		t_dev_info(dev, "[T] time       : %02d:%02d:%02d Site%d\n",
			fw->time & 0xFF, ((fw->time>>8) & 0xFF), ((fw->time>>16) & 0xFF),
			((fw->time>>24) & 0xFF));
		break;
	}

	if (strcmp(fw->chip_id, ts->pdata->chip_id)) {
		t_dev_err(dev, "Invalid chip id, shall be %s\n", ts->pdata->chip_id);
		return -EINVAL;
	}

	siw_hal_ic_info_ver_check(dev);

	return 0;
}

static int siw_hal_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct siw_ts *ts =
			container_of(self, struct siw_ts, fb_notif);
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			t_dev_info(ts->dev, "FB_UNBLANK\n");
		else if (*blank == FB_BLANK_POWERDOWN)
			t_dev_info(ts->dev, "FB_BLANK\n");
	}

	return 0;
}

static int siw_hal_init_reg_set(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data = 1;
	int ret = 0;

	ret = siw_hal_write_value(dev,
				reg->tc_device_ctl,
				1);

	ret = siw_hal_write_value(dev,
				reg->tc_interrupt_ctl,
				1);

#if 0
	ret = siw_hal_write_value(dev,
				reg->spr_charger_status,
				chip->charger);
#endif

	data = atomic_read(&ts->state.ime);
	ret = siw_hal_write_value(dev,
				reg->ime_state,
				data);

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

static int siw_hal_check_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	if (chip->lcd_mode == LCD_MODE_U3) {
		goto out;
	}

	if (chip->lcd_mode == LCD_MODE_U2) {
		if (chip->driving_mode == LCD_MODE_U2_UNBLANK) {
			t_dev_info(dev, "U1 -> U2 : watch on\n");
			siw_hal_watch_init(dev);
			// knockon mode change + swipe enable
			siw_hal_tc_driving(dev, LCD_MODE_U2);
			ret = 1;
		} else {
			t_dev_info(dev, "U2 mode change\n");
		}
		goto out;
	}

	if (chip->lcd_mode == LCD_MODE_U2_UNBLANK) {
		switch (chip->driving_mode) {
		case LCD_MODE_STOP:
			t_dev_info(dev, "Skip mode change : LCD_MODE_STOP -> U1\n");
			siw_hal_watch_display_off(dev);
			ret = 1;
			break;
		case LCD_MODE_U2:
			t_dev_info(dev, "U2 -> U1 : watch off\n");
			siw_hal_watch_display_off(dev);
			// abs mode change + swipe disable
			siw_hal_tc_driving(dev, LCD_MODE_U2_UNBLANK);
			ret = 1;
			break;
		case LCD_MODE_U0:
			t_dev_info(dev, "U0 -> U1 mode change\n");
			break;
		default:
			t_dev_info(dev, "Not Defined Mode, %d\n", chip->driving_mode);
			break;
		}
		goto out;
	}

	if (chip->lcd_mode == LCD_MODE_U0) {
		t_dev_info(dev, "U0 mode change\n");
		goto out;
	}

	t_dev_info(dev, "Not defined mode, %d\n", chip->lcd_mode);

out:
	return ret;
}

static void siw_hal_lcd_event_read_reg(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 rdata[5] = {0, 0};
	int ret = 0;

	{
		struct touch_xfer_msg *xfer = ts->xfer;

		siw_hal_xfer_init(dev, xfer);

		siw_hal_xfer_add_rx(xfer,
				reg->tc_ic_status,
				(u8 *)&rdata[0], sizeof(rdata[0]));
		siw_hal_xfer_add_rx(xfer,
				reg->tc_status,
				(u8 *)&rdata[1], sizeof(rdata[1]));
		siw_hal_xfer_add_rx(xfer,
				reg->spr_subdisp_status,
				(u8 *)&rdata[2], sizeof(rdata[2]));
		siw_hal_xfer_add_rx(xfer,
				reg->tc_version,
				(u8 *)&rdata[3], sizeof(rdata[3]));
		siw_hal_xfer_add_rx(xfer,
				reg->spr_chip_id,
				(u8 *)&rdata[4], sizeof(rdata[4]));
	}
	ret = siw_hal_xfer_msg(dev, ts->xfer);
	if (ret < 0) {
		t_dev_err(dev, "xfer failed, %d\n", ret);
		return;
	}

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


static int siw_hal_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int i;
	int ret = 0;

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		t_dev_dbg_base(dev, "fb_notif change\n");

		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = siw_hal_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
	}

	t_dev_dbg_base(dev, "charger_state = 0x%02X\n", chip->charger);

	if (atomic_read(&ts->state.debug_tool) == DEBUG_TOOL_ENABLE) {
		siw_ops_abt_init(ts);
	}

	for (i=0 ; i<2 ; i++) {
		ret = siw_hal_ic_info(dev);
		if (ret >= 0) {
			break;
		}

		t_dev_dbg_base(dev, "retry getting ic info\n");

		siw_touch_irq_control(dev, INTERRUPT_DISABLE);
		siw_hal_power(dev, POWER_OFF);
		siw_hal_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
	}
	if (ret < 0) {
		goto out;
	}

	siw_hal_init_reg_set(dev);

	atomic_set(&chip->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);

	ret = siw_hal_lpwg_mode(dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to lpwg_control, %d\n", ret);
		goto out;
	}

	ret = siw_hal_check_watch(dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to check watch, %d\n", ret);
		goto out;
	}

out:
	if (ret) {
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

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	if (pwr_con) {
		siw_hal_power(dev, POWER_OFF);
		siw_hal_power(dev, POWER_ON);
	} else {
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);
		touch_msleep(1);
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);
	}
	atomic_set(&chip->init, IC_INIT_NEED);

	touch_msleep(delay);

	if (do_call)
		do_call(dev);

	if (irq_enable)
		siw_touch_irq_control(dev, INTERRUPT_ENABLE);

	return 0;
}


static int siw_hal_sw_reset_wh_cmd(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_cmd_write(dev, CMD_ENA);
	siw_hal_cmd_write(dev, CMD_RESET_LOW);

	touch_msleep(1);

	siw_hal_cmd_write(dev, CMD_RESET_HIGH);
	siw_hal_cmd_write(dev, CMD_DIS);

	touch_msleep(ts->caps.sw_reset_delay);

	return 0;
}

static int siw_hal_sw_reset_default(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 chk_resp;
	u32 data;
	int ret = 0;

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	/******************************************************
	* Siliconworks does not recommend to use SW reset    *
	* due to ist limitation in stability in LG4894.      *
	******************************************************/
	t_dev_info(dev, "SW Reset\n");
	ret = siw_hal_write_value(dev,
				reg->spr_rst_ctl,
				7);
	ret = siw_hal_write_value(dev,
				reg->spr_rst_ctl,
				0);

	/* Boot Start */
	ret = siw_hal_write_value(dev,
				reg->spr_boot_ctl,
				1);

	/* firmware boot done check */
	chk_resp = FLASH_BOOTCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, ~0, 10, 200);
	if (ret < 0) {
		t_dev_err(dev, "failed : boot check(%Xh), %Xh\n",
			chk_resp, data);
		goto out;
	}
	siw_touch_qd_init_work_sw(ts);

out:
	return ret;
}

static int siw_hal_sw_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_LG4895:
		/* fall through */
	case CHIP_LG4946:
		ret = siw_hal_sw_reset_wh_cmd(dev);
		atomic_set(&chip->init, IC_INIT_NEED);
		break;

	case CHIP_LG4894:
		/* fall through */
	case CHIP_SW1828:
		/* fall through */
	default:
		ret = siw_hal_sw_reset_default(dev);
		atomic_set(&chip->init, IC_INIT_NEED);
		break;
	}

	return ret;
}

static int siw_hal_hw_reset(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	t_dev_info(dev, "HW Reset\n");

	if (ctrl == HW_RESET_ASYNC) {
		siw_hal_reinit(dev, 0, 0, 0, NULL);
		siw_touch_qd_init_work_hw(ts);
		return 0;
	}

	siw_hal_reinit(dev, 0, ts->caps.hw_reset_delay, 1, siw_hal_init);

	return 0;
}

static int siw_hal_reset_ctrl(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	t_dev_info(dev, "%s reset control(%d)\n",
			touch_chip_name(ts), ctrl);

	switch (ctrl) {
	default :
	case SW_RESET:
		siw_hal_sw_reset(dev);
		break;

	case HW_RESET_ASYNC:
	case HW_RESET_SYNC:
		siw_hal_hw_reset(dev, ctrl);
		break;
	}

	siw_hal_watch_set_rtc_clear(dev);

	return 0;
}

enum {
	BIN_VER_OFFSET_POS = 0xE8,
	BIN_PID_OFFSET_POS = 0xF0,
};

static int siw_hal_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_max_size = touch_fw_size(ts);
	u32 bin_ver_offset = *((u32 *)&fw->data[BIN_VER_OFFSET_POS]);
	u32 bin_pid_offset = *((u32 *)&fw->data[BIN_PID_OFFSET_POS]);
	u8 dev_major = chip->fw.version[0];
	u8 dev_minor = chip->fw.version[1];
	char pid[12] = {0, };
	u8 bin_major;
	u8 bin_minor;
	int update = 0;

	t_dev_dbg_base(dev, "bin_ver_offset 0x%06Xh, bin_pid_offset 0x%06Xh\n",
			bin_ver_offset, bin_pid_offset);

	if ((bin_ver_offset > fw_max_size) ||
		(bin_pid_offset > fw_max_size)) {
		t_dev_info(dev, "FW compare: invalid offset - ver %08Xh, pid %08Xh, max %08Xh\n",
			bin_ver_offset, bin_pid_offset, fw_max_size);
		return -EINVAL;
	}

	bin_major = fw->data[bin_ver_offset];
	bin_minor = fw->data[bin_ver_offset + 1];
	memcpy(pid, &fw->data[bin_pid_offset], 8);

	t_dev_dbg_base(dev, "dev_major %02Xh, dev_minor %02Xh\n",
			dev_major, dev_minor);

	t_dev_dbg_base(dev, "bin_major %02Xh, bin_minor %02Xh\n",
			bin_major, bin_minor);

	t_dev_dbg_base(dev, "pid %s\n", pid);

	if (dev_major > bin_major) {
		ts->force_fwup = 1;
	}

	if (ts->force_fwup) {
		update = 1;
	} else if (bin_major && dev_major) {
		update = !!(bin_minor != dev_minor);
	} else if (bin_major ^ dev_major) {
	//	update = 1;
		update = 0;
	} else if (!bin_major && !dev_major) {
		update = !!(bin_minor > dev_minor);
	}

	if(!dev_major && !dev_minor){
		t_dev_err(dev, "fw can not be 0.0!! Check your panel connection!!\n");
		update = 0;
	}

	t_dev_info(dev,
		"FW compare: bin-ver: %d.%02d (%s), dev-ver: %d.%02d - update %d, force_fwup %d\n",
		bin_major, bin_minor, pid, dev_major, dev_minor,
		update, ts->force_fwup);

	return update;
}

static int siw_hal_fw_up_rd_value(struct device *dev,
				u32 addr, u32 *value)
{
	u32 data;
	int ret;

	ret = siw_hal_read_value(dev, addr, &data);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_base(dev, "FW upgrade: reg rd: addr[%04Xh], value[%08Xh], %d\n",
			addr, data, ret);

	if (value)
		*value = data;

	return 0;
}

static int siw_hal_fw_up_wr_value(struct device *dev,
				u32 addr, u32 value)
{
	int ret;

	ret = siw_hal_write_value(dev, addr, value);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_base(dev, "FW upgrade: reg wr: addr[%04Xh], value[%08Xh], %d\n",
			addr, value, ret);

	return 0;
}

static int siw_hal_fw_up_wr_seq(struct device *dev,
				u32 addr, u8 *data, int size)
{
	int ret;

	ret = siw_hal_reg_write(dev, addr, (void *)data,size);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_base(dev, "FW upgrade: reg wr: addr[%04Xh], data[%02X ...], %d\n",
			addr, data[0], ret);

	return 0;
}

static int siw_hal_fw_up_sram_wr_enable(struct device *dev, int onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data;
	int ret;

#if 1
	ret = siw_hal_fw_up_rd_value(dev, reg->spr_sram_ctl, &data);
	if (ret < 0) {
		goto out;
	}

	if (onoff)
		data |= 0x01;
	else
		data &= ~0x01;

	ret = siw_hal_fw_up_wr_value(dev, reg->spr_sram_ctl, data);
	if (ret < 0) {
		goto out;
	}
#else
	ret = siw_hal_fw_up_wr_value(dev, reg->spr_sram_ctl, !!onoff);
	if (ret < 0) {
		goto out;
	}
#endif

out:
	return ret;
}

#if defined(__FW_VERIFY_TEST)
static int __siw_hal_fw_up_verify(struct device *dev, u8 *chk_buf, int chk_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u8 *fw_rd_data, *fw_data;
	u8 *r_data, *w_data;
	int fw_size;
	int fw_pos, curr_size;
	int i;
	int ret = 0;

	fw_size = chk_size;

	fw_rd_data = kmalloc(fw_size, GFP_KERNEL);
	if (!fw_rd_data) {
		t_dev_err(dev, "FW upgrade: failed to allocate verifying memory\n");
		ret = -ENOMEM;
		goto out;
	}

	fw_data = fw_rd_data;
	fw_pos = 0;
	while (fw_size && fw_data) {
		curr_size = min(fw_size, MAX_RW_SIZE);

		/* code sram base address write */
		ret = siw_hal_write_value(dev, reg->spr_code_offset, fw_pos>>2);
		if (ret < 0) {
			goto out_free;
		}

		ret = siw_hal_reg_read(dev, reg->code_access_addr,
					(void *)fw_data, curr_size);
		if (ret < 0) {
			goto out_free;
		}

		fw_data += curr_size;
		fw_pos += curr_size;
		fw_size -= curr_size;
	}

	r_data = fw_rd_data;
	w_data = chk_buf;
	fw_size = chk_size;
	for (i=0 ; i<fw_size ; i++) {
		if ((*r_data) != (*w_data)) {
			t_dev_err(dev, "* Err [%06X] rd(%02X) != wr(%02X)\n",
				i, (*r_data), (*w_data));
			ret = -EFAULT;
		} else {
		#if 0
			t_dev_err(dev, "  OK! [%06X] rd(%02X) == wr(%02X)\n",
				i, (*r_data), (*w_data));
		#endif
		}

		r_data++;
		w_data++;
	}

out_free:
	kfree(fw_rd_data);

out:
	return ret;
}
#else	/* __FW_VERIFY_TEST */
static int __siw_hal_fw_up_verify(struct device *dev, u8 *chk_buf, int chk_size)
{
	return 0;
}
#endif	/* __FW_VERIFY_TEST */

static int siw_hal_fw_up_pre_fw_dn(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ret;

	/* Reset CM3 core */
	ret = siw_hal_fw_up_wr_value(dev, reg->spr_rst_ctl, 2);
	if (ret < 0) {
		goto out;
	}

	/* Disable SRAM write protection */
	ret = siw_hal_fw_up_sram_wr_enable(dev, 1);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_up_post_fw_dn(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dn_cmd, chk_resp, data;
	int ret;

	/* Enable SRAM write protection */
	ret = siw_hal_fw_up_sram_wr_enable(dev, 0);
	if (ret < 0) {
		goto out;
	}

	/* Release CM3 core */
	ret = siw_hal_fw_up_wr_value(dev, reg->spr_rst_ctl, 0);
	if (ret < 0) {
		goto out;
	}

	/* Set Serial Dump Done */
	ret = siw_hal_fw_up_wr_value(dev, reg->spr_boot_ctl, 1);
	if (ret < 0) {
		goto out;
	}

	/* firmware boot done check */
	chk_resp = FLASH_BOOTCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, ~0, 10, 200);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - boot check(%Xh), %Xh\n",
			chk_resp, data);
		return ret;
	}
	t_dev_info(dev, "FW upgrade: boot check done\n");

	/* Firmware Download Start */
	dn_cmd = (FLASH_KEY_CODE_CMD << 16) | 1;
	ret = siw_hal_fw_up_wr_value(dev, reg->tc_flash_dn_ctl, dn_cmd);
	if (ret < 0) {
		goto out;
	}

	touch_msleep(ts->caps.hw_reset_delay);

	/* download check */
	chk_resp = FLASH_CODE_DNCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, 0xFFFF, 30, 600);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - code check(%Xh), %Xh\n",
			chk_resp, data);
		goto out;
	}
	t_dev_info(dev, "FW upgrade: code check done\n");

out:
	return ret;
}

static int siw_hal_fw_up_do_fw_dn(struct device *dev, u8 *dn_buf, int dn_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u8 *fw_data;
	int fw_size;
	int fw_pos, curr_size;
	int ret = 0;

	fw_data = dn_buf;
	fw_size = dn_size;
	fw_pos = 0;
	while (fw_size) {
		t_dev_dbg_base(dev, "FW upgrade: fw_pos[%06Xh ...] = %02X %02X %02X %02X ...\n",
				fw_pos,
				fw_data[0], fw_data[1], fw_data[2], fw_data[3]);

		curr_size = min(fw_size, MAX_RW_SIZE);

		/* code sram base address write */
		ret = siw_hal_fw_up_wr_value(dev, reg->spr_code_offset, fw_pos>>2);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_fw_up_wr_seq(dev, reg->code_access_addr,
					(void *)fw_data, curr_size);
		if (ret < 0) {
			goto out;
		}

		fw_data += curr_size;
		fw_pos += curr_size;
		fw_size -= curr_size;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade_fw(struct device *dev,
				const struct firmware *fw)
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

	ret = siw_hal_fw_up_pre_fw_dn(dev);
	if (ret < 0) {
		goto out;
	}

	/*
	 * [Caution]
	 * The size for F/W upgrade is fw_size_max, not fw->size
	 * because the fw file can have config area.
	 */
	fw_data = (u8 *)fw->data;
	ret = siw_hal_fw_up_do_fw_dn(dev, fw_data, fw_size_max);
	if (ret < 0) {
		goto out;
	}

	ret = __siw_hal_fw_up_verify(dev, fw_data, fw_size_max);
	if (ret < 0) {
		goto out;
	}

	/*
	 * Stage 1-2: upgrade code data
	 */
	ret = siw_hal_fw_up_post_fw_dn(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}


static int siw_hal_fw_up_pre_conf_dn(struct device *dev, u32 *value)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int data = 0;
	int ret;

	ret = siw_hal_fw_up_rd_value(dev, reg->tc_confdn_base_addr, &data);
	if (ret < 0) {
		goto out;
	}

out:
	if (value)
		*value = data;

	return ret;
}

static int siw_hal_fw_up_post_conf_dn(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dn_cmd, chk_resp, data;
	int ret;

	/* Conf Download Start */
	dn_cmd = (FLASH_KEY_CONF_CMD << 16) | 2;
	ret = siw_hal_fw_up_wr_value(dev, reg->tc_flash_dn_ctl, dn_cmd);
	if (ret < 0) {
		goto out;
	}

	/* Conf check */
	chk_resp = FLASH_CONF_DNCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, 0xFFFF, 30, 600);
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


static int siw_hal_fw_up_do_conf_dn(struct device *dev,
				u32 addr, u8 *dn_buf, int dn_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ret;

	/* conf sram base address write */
	ret = siw_hal_fw_up_wr_value(dev, reg->spr_data_offset, addr);
	if (ret < 0) {
		goto out;
	}

	/* Conf data download to conf sram */
	ret = siw_hal_fw_up_wr_seq(dev, reg->data_access_addr,
				(void *)dn_buf, dn_size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf(struct device *dev,
			     const struct firmware *fw)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u8 *fw_data;
	int fw_size, fw_size_max;
	u32 conf_dn_addr;
	u32 data;
	int ret;

	fw_size = (int)fw->size;
	fw_size_max = touch_fw_size(ts);

	/*
	 * Stage 2-1: download config data
	 */
	ret = siw_hal_fw_up_pre_conf_dn(dev, &data);
	if (ret < 0) {
		goto out;
	}

	conf_dn_addr = ((data >> 16) & 0xFFFF);
	t_dev_dbg_base(dev, "FW upgrade: conf_dn_addr %04Xh (%08Xh)\n",
			conf_dn_addr, data);
	if (conf_dn_addr >= (0x1200) || conf_dn_addr < (0x8C0)) {
		t_dev_err(dev, "FW upgrade: failed - conf base invalid\n");
		ret = -EPERM;
		goto out;
	}

	fw_data = (u8 *)fw->data;
	ret = siw_hal_fw_up_do_conf_dn(dev, conf_dn_addr,
				(u8 *)&fw_data[fw_size_max], FLASH_CONF_SIZE);
	if (ret < 0) {
		goto out;
	}

	/*
	 * Stage 2-2: upgrade config data
	 */
	ret = siw_hal_fw_up_post_conf_dn(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade(struct device *dev,
			     const struct firmware *fw, int retry)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size, fw_size_max;
	u32 include_conf;
	int ret = 0;

	t_dev_info(dev, "===== FW upgrade: start (%d) =====\n", retry);

	fw_size = (int)fw->size;
	fw_size_max = touch_fw_size(ts);
	if ((fw_size != fw_size_max) &&
		(fw_size != (fw_size_max + FLASH_CONF_SIZE)))
	{
		t_dev_err(dev, "FW upgrade: wrong file size - %Xh,\n",
			fw_size);
		t_dev_err(dev, "            shall be '%Xh' or '%Xh + %Xh'\n",
			fw_size_max, fw_size_max, FLASH_CONF_SIZE);
		ret = -EFAULT;
		goto out;
	}

	include_conf = !!(fw_size == (fw_size_max + FLASH_CONF_SIZE));
	t_dev_info(dev, "FW upgrade:%s include conf data\n",
			(include_conf)?"":" not");

	t_dev_dbg_base(dev, "FW upgrade: fw size %08Xh, fw_size_max %08Xh\n",
			fw_size, fw_size_max);

	ret = siw_hal_fw_upgrade_fw(dev, fw);
	if (ret < 0) {
		goto out;
	}

	if (include_conf) {
		ret = siw_hal_fw_upgrade_conf(dev, fw);
		if (ret < 0) {
			goto out;
		}
	}

	t_dev_info(dev, "===== FW upgrade: done (%d) =====\n", retry);

out:
	return ret;
}

static int siw_hal_upgrade(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	const struct firmware *fw = NULL;
	char fwpath[DEFAULT_NAME_SZ] = {0, };
	int ret = 0;
	int ret_val = 0;
	int i = 0;

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		t_dev_warn(dev, "state.fb is not FB_RESUME\n");
		ret = -EPERM;
		goto out;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		t_dev_info(dev, "get fwpath from test_fwpath:%s\n",
				&ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		t_dev_info(dev, "get fwpath from def_fwpath : %s\n", fwpath);
	} else {
		t_dev_err(dev, "no firmware file\n");
		ret = -ENOENT;
		goto out;
	}

	t_dev_info(dev, "fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to request_firmware fwpath: %s (ret:%d)\n",
				fwpath, ret);
		goto out;
	}

	t_dev_info(dev, "fw size:%zu\n", fw->size);
//	ret = -EINVAL;
	ret = -EPERM;
	ret_val = siw_hal_fw_compare(dev, fw);
	if (ret_val < 0) {
		ret = ret_val;
	} else if (ret_val) {
		touch_msleep(200);
		for (i = 0; i < 2 && ret; i++) {
			ret = siw_hal_fw_upgrade(dev, fw, i);
		}
	}

	release_firmware(fw);

out:
	if (ret) {
		siwmon_submit_ops_step_chip_wh_name(dev, "%s - FW upgrade halted",
				touch_chip_name(ts), ret);
	} else {
		siwmon_submit_ops_step_chip_wh_name(dev, "%s - FW upgrade done",
				touch_chip_name(ts), ret);
	}
	return ret;
}

static void siw_hal_set_debug_reason(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 wdata[2] = {0, };
//	int ret = 0;

	wdata[0] = (u32)type;
	wdata[0] |= (chip->tci_debug_type == 1) ? 0x01 << 2 : 0x01 << 3;
	wdata[1] = TCI_DEBUG_ALL;
	t_dev_info(dev, "TCI%d-type:%d\n", type + 1, wdata[0]);

#if 1
	siw_hal_xfer_tx_seq(dev,
			reg->tci_fail_debug_w,
			(u32 *)wdata, ARRAY_SIZE(wdata));
#else
	siw_hal_reg_write(dev,
			reg->tci_fail_debug_w,
			(void *)wdata, sizeof(wdata));
#endif
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

	if (chip->tci_debug_type != 0)
		siw_hal_set_debug_reason(dev, TCI_1);

	lpwg_data[0] = ts->tci.mode;
	lpwg_data[1] = info1->tap_count | (info2->tap_count << 16);
	lpwg_data[2] = info1->min_intertap | (info2->min_intertap << 16);
	lpwg_data[3] = info1->max_intertap | (info2->max_intertap << 16);
	lpwg_data[4] = info1->touch_slop | (info2->touch_slop << 16);
	lpwg_data[5] = info1->tap_distance | (info2->tap_distance << 16);
	lpwg_data[6] = info1->intr_delay | (info2->intr_delay << 16);

#if 1
	ret = siw_hal_xfer_tx_seq(dev,
				reg->tci_enable_w,
				(u32 *)lpwg_data, ARRAY_SIZE(lpwg_data));
#else
	ret = siw_hal_reg_write(dev,
				reg->tci_enable_w,
				(void *)lpwg_data, sizeof(lpwg_data));
#endif

	return ret;
}

static int siw_hal_tci_password(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (chip->tci_debug_type != 0)
		siw_hal_set_debug_reason(dev, TCI_2);

	return siw_hal_tci_knock(dev);
}

static int siw_hal_tci_active_area(struct device *dev,
		u32 x1, u32 y1, u32 x2, u32 y2)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ret = 0;

	ret = siw_hal_write_value(dev,
				reg->act_area_x1_w,
				x1);
	if (ret < 0) {
		goto out;
	}
	ret = siw_hal_write_value(dev,
				reg->act_area_y1_w,
				y1);
	if (ret < 0) {
		goto out;
	}
	ret = siw_hal_write_value(dev,
				reg->act_area_x2_w,
				x2);
	if (ret < 0) {
		goto out;
	}
	ret = siw_hal_write_value(dev,
				reg->act_area_y2_w,
				y2);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static void siw_hal_tci_area_set(struct device *dev, int cover_status)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct reset_area *qcover;
	const char *msg;

	if (touch_mode_not_allowed(ts, LCD_MODE_U3_QUICKCOVER)) {
		return;
	}

	qcover = (cover_status == QUICKCOVER_CLOSE)?
			&ts->tci.qcover_close : &ts->tci.qcover_open;
	msg = (cover_status == QUICKCOVER_CLOSE)?
			"qcover close" : "normal";

	if (qcover->x1 != ~0) {
		siw_hal_tci_active_area(dev, 179, 144, 1261, 662);
		t_dev_info(dev, "lpwg active area - %s\n", msg);
	}
}

static int siw_hal_tci_control(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct tci_ctrl *tci = &ts->tci;
	struct active_area *area = &tci->area;
	struct reset_area *rst_area = &tci->rst_area;
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
					area->x2, area->y2);
		break;

	case ACTIVE_AREA_RESET_CTRL:
		ret = siw_hal_tci_active_area(dev,
					rst_area->x1, rst_area->y1,
					rst_area->x2, rst_area->y2);
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

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		ret = siw_hal_tci_knock(dev);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x01 | (0x01 << 16);
		info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
		info1->tap_distance = 7;

		ret = siw_hal_tci_password(dev);
		break;

	default:
		ts->tci.mode = 0;
		ret = siw_hal_tci_control(dev, ENABLE_CTRL);
		break;
	}

	t_dev_dbg_base(dev, "siw_hal_lpwg_control mode = %d\n", mode);

	return ret;
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

	t_dev_info(dev, "siw_hal_clock -> %s\n",
		(onoff) ? "ON" : (chip->lcd_mode) == 0 ? "OFF" : "SKIP");

	return 0;
}

static int siw_hal_clock(struct device *dev, bool onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	siw_touch_sys_osc(dev, onoff);

	switch(touch_chip_type(ts)) {
//	case CHIP_LG4895:
	case CHIP_LG4946:
		ret = siw_hal_clock_type_1(dev, onoff);
		break;
	default:
		atomic_set(&ts->state.sleep,
			(onoff)? IC_NORMAL : IC_DEEP_SLEEP);
		break;
	}

	return ret;
}

static int siw_hal_swipe_active_area(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_swipe_info *left = &chip->swipe.info[SWIPE_L];
	struct siw_hal_swipe_info *right = &chip->swipe.info[SWIPE_R];
	u32 active_area[4] = {0x0, };
	int ret = 0;

	active_area[0] = (right->area.x1) | (left->area.x1 << 16);
	active_area[1] = (right->area.y1) | (left->area.y1 << 16);
	active_area[2] = (right->area.x2) | (left->area.x2 << 16);
	active_area[3] = (right->area.y2) | (left->area.y2 << 16);

#if 1
	ret = siw_hal_xfer_tx_seq(dev,
				reg->swipe_act_area_x1_w,
				(u32 *)active_area, ARRAY_SIZE(active_area));
#else
	ret = siw_hal_reg_write(dev,
				reg->swipe_act_area_x1_w,
				(void *)active_area, sizeof(active_area));
#endif

	return ret;
}

static int siw_hal_swipe_control(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_swipe_info *left = &chip->swipe.info[SWIPE_L];
	struct siw_hal_swipe_info *right = &chip->swipe.info[SWIPE_R];
	u32 reg_w = ~0;
	u32 data = 0;
	int ret = 0;

	switch (type) {
	case SWIPE_ENABLE_CTRL:
	case SWIPE_DISABLE_CTRL:
		reg_w = reg->swipe_enable_w;
		data = (type == SWIPE_ENABLE_CTRL)?
					chip->swipe.mode : 0;
	case SWIPE_DIST_CTRL:
		reg_w = reg->swipe_dist_w;
		data = (right->distance) | (left->distance << 16);
		break;
	case SWIPE_RATIO_THR_CTRL:
		reg_w = reg->swipe_ratio_thr_w;
		data = (right->ratio_thres) | (left->ratio_thres << 16);
		break;
	case SWIPE_RATIO_PERIOD_CTRL:
		reg_w = reg->swipe_ratio_period_w;
		data = (right->ratio_period) | (left->ratio_period << 16);
		break;
	case SWIPE_RATIO_DIST_CTRL:
		reg_w = reg->swipe_ratio_dist_w;
		data = (right->ratio_distance) |
				(left->ratio_distance << 16);
		break;
	case SWIPE_TIME_MIN_CTRL:
		reg_w = reg->swipe_time_min_w;
		data = (right->min_time) | (left->min_time << 16);
		break;
	case SWIPE_TIME_MAX_CTRL:
		reg_w = reg->swipe_time_max_w;
		data = (right->max_time) | (left->max_time << 16);
		break;
	case SWIPE_AREA_CTRL:
		ret = siw_hal_swipe_active_area(dev);
		break;
	default:
		break;
	}

	if (reg_w != ~0) {
		ret = siw_hal_write_value(dev, reg_w, data);
	}

	return ret;
}

static int siw_hal_swipe_mode(struct device *dev, u8 lcd_mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_swipe_info *left = &chip->swipe.info[SWIPE_L];
	struct siw_hal_swipe_info *right = &chip->swipe.info[SWIPE_R];
	u32 swipe_data[11] = {0x0, };
	int ret = 0;

	if (!chip->swipe.mode)
		return ret;

	if (lcd_mode != LCD_MODE_U2) {
		ret = siw_hal_swipe_control(dev, SWIPE_DISABLE_CTRL);
		t_dev_dbg_base(dev, "swipe disabled\n");
		goto out;
	}

	swipe_data[0] = chip->swipe.mode;
	swipe_data[1] = (right->distance) | (left->distance << 16);
	swipe_data[2] = (right->ratio_thres) | (left->ratio_thres << 16);
	swipe_data[3] = (right->ratio_distance) | (left->ratio_distance << 16);
	swipe_data[4] = (right->ratio_period) | (left->ratio_period << 16);
	swipe_data[5] = (right->min_time) | (left->min_time << 16);
	swipe_data[6] = (right->max_time) | (left->max_time << 16);
	swipe_data[7] = (right->area.x1) | (left->area.x1 << 16);
	swipe_data[8] = (right->area.y1) | (left->area.y1 << 16);
	swipe_data[9] = (right->area.x2) | (left->area.x2 << 16);
	swipe_data[10] = (right->area.y2) | (left->area.y2 << 16);

#if 1
	ret = siw_hal_xfer_tx_seq(dev,
				reg->swipe_enable_w,
				(u32 *)swipe_data, ARRAY_SIZE(swipe_data));
#else
	ret = siw_hal_reg_write(dev,
				reg->swipe_enable_w,
				(void *)swipe_data, sizeof(swipe_data));
#endif
	if (ret >= 0) {
		t_dev_info(dev, "swipe enabled\n");
	}

out:
	return ret;
}


#define HAL_TC_DRIVING_DELAY	20

static inline int __used siw_hal_tc_driving_u0(struct device *dev)
{
	return TC_DRIVE_CTL_START;
}

static inline int __used siw_hal_tc_driving_u2(struct device *dev)
{
	return (TC_DRIVE_CTL_DISP_U2 | TC_DRIVE_CTL_START);	//0x101
}

static inline int __used siw_hal_tc_driving_u3(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ctrl = (TC_DRIVE_CTL_DISP_U3 | TC_DRIVE_CTL_MODE_6LHB | TC_DRIVE_CTL_START);

	if (atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_1)
		ctrl &= ~TC_DRIVE_CTL_MODE_6LHB;

	return ctrl;
}

static inline int __used siw_hal_tc_driving_u3_partial(struct device *dev)
{
	return (TC_DRIVE_CTL_PARTIAL | siw_hal_tc_driving_u3(dev));
}

static inline int __used siw_hal_tc_driving_u3_qcover(struct device *dev)
{
	return (TC_DRIVE_CTL_QCOVER | siw_hal_tc_driving_u3(dev));
}

static inline int siw_hal_tc_driving_stop(struct device *dev)
{
	return TC_DRIVE_CTL_STOP;
}

#define SIW_HAL_SET_LCD_DRIVING_MODE_STR(_mode)	\
		[LCD_MODE_IDX_##_mode] = #_mode

static const char *siw_hal_lcd_driving_mode_str[] = {
	SIW_HAL_SET_LCD_DRIVING_MODE_STR(U0),
	SIW_HAL_SET_LCD_DRIVING_MODE_STR(U2_UNBLANK),
	SIW_HAL_SET_LCD_DRIVING_MODE_STR(U2),
	SIW_HAL_SET_LCD_DRIVING_MODE_STR(U3),
	SIW_HAL_SET_LCD_DRIVING_MODE_STR(U3_PARTIAL),
	SIW_HAL_SET_LCD_DRIVING_MODE_STR(U3_QUICKCOVER),
	SIW_HAL_SET_LCD_DRIVING_MODE_STR(STOP),
};

static inline const char *siw_hal_lcd_driving_mode_name(int mode_bit)
{
	int i;

	for (i=0 ; i<LCD_MODE_IDX_MAX ; i++) {
		if (BIT(i) == mode_bit) {
			return siw_hal_lcd_driving_mode_str[i];
		}
	}
	return "(invalid)";
}

static int siw_hal_tc_driving(struct device *dev, int mode_bit)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 running_status = 0;
	u32 ctrl = 0;
	u32 rdata;
	int re_init = 0;
	int ret = 0;

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		t_dev_warn(dev, "can not control tc driving - deep sleep state\n");
		return 0;
	}

	chip->driving_mode = mode_bit;

	if (touch_mode_not_allowed(ts, mode_bit)) {
		return 0;
	}

	switch (mode_bit) {
	case LCD_MODE_U0:
		ctrl = siw_hal_tc_driving_u0(dev);
		break;

	case LCD_MODE_U2:
		ctrl = siw_hal_tc_driving_u2(dev);
		break;

	case LCD_MODE_U3:
		ctrl = siw_hal_tc_driving_u3(dev);
		break;

	case LCD_MODE_U3_PARTIAL:
		ctrl = siw_hal_tc_driving_u3_partial(dev);
		break;

	case LCD_MODE_U3_QUICKCOVER:
		ctrl = siw_hal_tc_driving_u3_qcover(dev);
		break;

	case LCD_MODE_STOP:
		ctrl = siw_hal_tc_driving_stop(dev);
		break;

	default:
		t_dev_err(dev, "mode(%d) not supported\n", mode_bit);
		return -ESRCH;
	}

	/* swipe set */
	ret = siw_hal_swipe_mode(dev, mode_bit);
	if (ret < 0) {
		t_dev_warn(dev, "swipe mode err, %d", ret);
	}

#if 0
	if ((chip->fw.wfr == REV1) &&
		((mode_bit == LCD_MODE_U0) || (mode_bit == LCD_MODE_U2))) {
		touch_msleep(200);
	}
#endif

	t_dev_info(dev, "current driving mode is %s\n",
			siw_hal_lcd_driving_mode_name(mode_bit));

	ret = siw_hal_read_value(dev,
				reg->spr_subdisp_status,
				&rdata);
	t_dev_info(dev, "DDI Display Mode = 0x%08X\n", rdata);

	ret = siw_hal_write_value(dev,
				reg->tc_drive_ctl,
				ctrl);
	t_dev_dbg_base(dev, "write 0x%X on tc_drive_ctl[%04Xh]\n",
			ctrl, reg->tc_drive_ctl);

	touch_msleep(HAL_TC_DRIVING_DELAY);

	t_dev_dbg_base(dev, "waiting %d msecs\n", HAL_TC_DRIVING_DELAY);

	if (mode_bit == LCD_MODE_U3_PARTIAL) {
		atomic_set(&ts->recur_chk, 0);
		return 0;
	}

	if (atomic_read(&ts->recur_chk)) {
		t_dev_info(dev, "running status is already checked\n");
		atomic_set(&ts->recur_chk, 0);
		return 0;
	}

	ret = siw_hal_read_value(dev,
				reg->tc_status,
				&running_status);
	if (ret < 0) {
		t_dev_err(dev, "check module\n");
		atomic_set(&ts->recur_chk, 0);
		return ret;
	}

	t_dev_dbg_base(dev, "running_status : %Xh\n", running_status);
	running_status &= 0x1F;

	re_init = 0;
	if (mode_bit != LCD_MODE_STOP) {
		if (!running_status ||
			(running_status == 0x10) ||
			(running_status == 0x0F)){
			re_init = 1;
		}
	} else {
		re_init = !!running_status;
	}

	if (re_init) {
		t_dev_err(dev, "command missed: mode %d, status %Xh\n",
			mode_bit, running_status);

		atomic_set(&ts->recur_chk, 1);

		siw_hal_reinit(dev, 1, 100, 1, siw_hal_init);
	} else {
		t_dev_dbg_base(dev, "command done: mode %d, status %Xh\n",
			mode_bit, running_status);
	}

	atomic_set(&ts->recur_chk, 0);

	return 0;
}

static void siw_hal_deep_sleep(struct device *dev)
{
	siw_hal_tc_driving(dev, LCD_MODE_STOP);
	siw_hal_clock(dev, 0);
}

static void siw_hal_debug_tci(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u8 debug_reason_buf[TCI_MAX_NUM][TCI_DEBUG_MAX_NUM];
	u32 rdata[9] = {0, };
	u8 count[2] = {0, };
	u8 count_max = 0;
	u32 i, j = 0;
	u8 buf = 0;
	int ret = 0;

	if (!chip->tci_debug_type)
		return;

	ret = siw_hal_reg_read(dev,
				reg->tci_debug_r,
				(void *)&rdata, sizeof(rdata));

	count[TCI_1] = (rdata[0] & 0xFFFF);
	count[TCI_2] = ((rdata[0] >> 16) & 0xFFFF);
	count_max = (count[TCI_1] > count[TCI_2]) ? count[TCI_1] : count[TCI_2];

	if (count_max == 0)
		return;

	if (count_max > TCI_DEBUG_MAX_NUM) {
		count_max = TCI_DEBUG_MAX_NUM;
		if (count[TCI_1] > TCI_DEBUG_MAX_NUM)
			count[TCI_1] = TCI_DEBUG_MAX_NUM;
		if (count[TCI_2] > TCI_DEBUG_MAX_NUM)
			count[TCI_2] = TCI_DEBUG_MAX_NUM;
	}

	for (i = 0; i < ((count_max-1)>>2)+1; i++) {
		memcpy(&debug_reason_buf[TCI_1][i<<2], &rdata[i+1], sizeof(u32));
		memcpy(&debug_reason_buf[TCI_2][i<<2], &rdata[i+5], sizeof(u32));
	}

	t_dev_info(dev, "TCI count_max = %d\n", count_max);
	for (i = 0; i < TCI_MAX_NUM; i++) {
		t_dev_info(dev, "TCI count[%d] = %d\n", i, count[i]);
		for (j = 0; j < count[i]; j++) {
			buf = debug_reason_buf[i][j];
			t_dev_info(dev, "TCI_%d - DBG[%d/%d]: %s\n",
						i + 1, j + 1, count[i],
						(buf > 0 && buf < TCI_FAIL_NUM) ?
						siw_hal_tci_debug_str[buf] :
						siw_hal_tci_debug_str[0]);
		}
	}
}

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

	if (!chip->swipe_debug_type)
		return;

	ret = siw_hal_reg_read(dev,
				reg->swipe_debug_r,
				(void *)&rdata, sizeof(rdata));

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


static int siw_hal_lpwg_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (atomic_read(&chip->init) == IC_INIT_NEED) {
		t_dev_info(dev, "Not Ready, Need IC init\n");
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			siw_hal_lpwg_control(dev, LPWG_DOUBLE_TAP);
			siw_hal_tc_driving(dev, chip->lcd_mode);
			return 0;
		}

		if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			t_dev_dbg_lpwg(dev, "suspend sensor == PROX_NEAR\n");

			if (ts->lpwg.screen) {
				siw_hal_clock(dev, 1);
			} else {
				siw_hal_deep_sleep(dev);
			}
		} else if (ts->lpwg.screen) {
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				siw_hal_clock(dev, 1);

			t_dev_dbg_lpwg(dev, "skip lpwg_mode\n");

			siw_hal_debug_tci(dev);
			siw_hal_debug_swipe(dev);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* knock on/code disable */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				siw_hal_clock(dev, 1);

			siw_hal_tci_area_set(dev, QUICKCOVER_CLOSE);
			siw_hal_lpwg_control(dev, LPWG_NONE);
			siw_hal_tc_driving(dev, chip->lcd_mode);
		} else {
			/* knock on/code */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				siw_hal_clock(dev, 1);

			siw_hal_tci_area_set(dev, QUICKCOVER_OPEN);
			siw_hal_lpwg_control(dev, ts->lpwg.mode);
			siw_hal_tc_driving(dev, chip->lcd_mode);
		}
		return 0;
	}

	/* resume */
	siw_touch_report_all_event(ts);		//clear (?)
	if (ts->lpwg.screen) {
		int mode;
		/* normal */
		t_dev_dbg_lpwg(dev, "resume ts->lpwg.screen\n");

		siw_hal_lpwg_control(dev, LPWG_NONE);

		mode = (ts->lpwg.qcover == HOLE_NEAR)?
				LCD_MODE_U3_QUICKCOVER :
				chip->lcd_mode;
		siw_hal_tc_driving(dev, mode);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* wake up */
		t_dev_dbg_lpwg(dev, "resume ts->lpwg.mode == LPWG_NONE\n");

	//	siw_hal_deep_sleep(dev);
		siw_hal_tc_driving(dev, LCD_MODE_STOP);
	} else {
		/* partial */
		if (touch_mode_allowed(ts, LCD_MODE_U3_QUICKCOVER)) {
			int qcover_mode;

			qcover_mode = (ts->lpwg.qcover == HOLE_NEAR)?
						QUICKCOVER_CLOSE : QUICKCOVER_OPEN;
			siw_hal_tci_area_set(dev, qcover_mode);
			siw_hal_lpwg_control(dev, ts->lpwg.mode);
			siw_hal_tc_driving(dev, LCD_MODE_U3_PARTIAL);
		} else {
			t_dev_dbg_lpwg(dev, "resume Partial-Do not set\n");
			/*
			if (ts->lpwg.qcover == HOLE_NEAR) {
				siw_hal_lpwg_control(dev, LPWG_NONE);
			} else {
				siw_hal_lpwg_control(dev, ts->lpwg.mode);
			}
			siw_hal_tc_driving(dev, LCD_MODE_U3_PARTIAL);
			*/
		}
	}

	return 0;
}

static int siw_hal_lpwg(struct device *dev, u32 code, void *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct tci_ctrl *tci = &ts->tci;
	struct active_area *area = &tci->area;
	struct lpwg_info *lpwg = &ts->lpwg;
	int *value = (int *)param;

//	if (!touch_test_quirks(ts, CHIP_QUIRK_SUPPORT_LPWG)) {
	if (!ts->role.use_lpwg) {
		t_dev_warn(dev, "LPWG control not supported in %s\n",
				touch_chip_name(ts));
		return 0;
	}

	switch (code) {
	case LPWG_ACTIVE_AREA:
		area->x1 = value[0];
		area->x2 = value[1];
		area->y1 = value[2];
		area->y2 = value[3];
		t_dev_dbg_lpwg(dev, "LPWG_ACTIVE_AREA: x1[%d], y1[%d], x2[%d], y2[%d]\n",
				area->x1, area->y1, area->x2, area->y2);
		break;

	case LPWG_TAP_COUNT:
		tci->info[TCI_2].tap_count = value[0];
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		tci->double_tap_check = value[0];
		break;

	case LPWG_UPDATE_ALL:
		lpwg->mode = value[0];
		lpwg->screen = value[1];
		lpwg->sensor = value[2];
		lpwg->qcover = value[3];

		t_dev_dbg_lpwg(dev,
				"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
				lpwg->mode,
				lpwg->screen ? "ON" : "OFF",
				lpwg->sensor ? "FAR" : "NEAR",
				lpwg->qcover ? "CLOSE" : "OPEN");

		siw_hal_lpwg_mode(dev);

		break;

	case LPWG_REPLY:
		break;

	}

	return 0;
}

#if defined(__SIW_SUPPORT_ASC)
static int siw_hal_asc(struct device *dev, u32 code, u32 value)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_asc_info *asc = &chip->asc;
	u32 rdata = 0;
	u32 wdata = 0;
	u32 asc_ret = 0;
	int mon_data[6] = { 0, };
	int mon_cnt = 0;
	int ret = 0;

	mon_data[mon_cnt++] = code;

	switch (code) {
	case ASC_READ_MAX_DELTA:
		ret = siw_hal_reg_read(dev,
					reg->max_delta,
					(void *)&rdata, sizeof(rdata));
		mon_data[mon_cnt++] = reg->max_delta;
		mon_data[mon_cnt++] = rdata;
		if (ret < 0) {
			break;
		}

		asc_ret = rdata;

		break;

	case ASC_GET_FW_SENSITIVITY:
		/* fall through */
	case ASC_WRITE_SENSITIVITY:
		ret = siw_hal_reg_read(dev,
					reg->touch_max_r,
					(void *)&rdata, sizeof(rdata));
		mon_data[mon_cnt++] = reg->touch_max_r;
		mon_data[mon_cnt++] = rdata;
		if (ret < 0) {
			break;
		}

		asc->normal_s = rdata;
		asc->acute_s = (rdata / 10) * 6;
		asc->obtuse_s = rdata;

		if (code == ASC_GET_FW_SENSITIVITY) {
			t_dev_info(dev,
					"max_r(%04Xh) = %d, n_s %d, a_s = %d, o_s = %d\n",
					reg->touch_max_r,
					rdata,
					asc->normal_s,
					asc->acute_s,
					asc->obtuse_s);
			break;
		}

		switch (value) {
		case NORMAL_SENSITIVITY :
			wdata = asc->normal_s;
			break;
		case ACUTE_SENSITIVITY :
			wdata = asc->acute_s;
			break;
		case OBTUSE_SENSITIVITY :
			wdata = asc->obtuse_s;
			break;
		default:
			wdata = rdata;
			break;
		}

		ret = siw_hal_write_value(dev,
					reg->touch_max_w,
					wdata);
		mon_data[mon_cnt++] = reg->touch_max_w;
		mon_data[mon_cnt++] = wdata;
		if (ret < 0) {
			break;
		}

		t_dev_info(dev, "max_w(%04Xh) changed (%d -> %d)\n",
				reg->touch_max_w,
				rdata, wdata);
		break;
	default:
		break;
	}

	siwmon_submit_ops_wh_name(dev, "%s asc done",
			touch_chip_name(ts),
			mon_data, mon_cnt, asc_ret);

	return asc_ret;
}
#else	/* __SIW_SUPPORT_ASC */
static int siw_hal_asc(struct device *dev, u32 code, u32 value)
{
	return 0;
}
#endif	/* __SIW_SUPPORT_ASC */

enum {
	INT_RESET_CLR_BIT	= ((1<<10)|(1<<9)|(1<<5)),	// 0x620
	INT_LOGGING_CLR_BIT	= ((1<<22)|(1<<20)|(1<<15)|(1<<13)|(1<<7)|(1<<6)),	//0x50A0C0
	INT_NORMAL_MASK		= ((1<<22)|(1<<20)|(1<<15)|(1<<7)|(1<<6)|(1<<5)),	//0x5080E0
	//
	IC_DEBUG_SIZE		= 16,	// byte
	//
	IC_CHK_LOG_MAX		= (1<<9),
	//
	INT_IC_ABNORMAL_STATUS	= ((1<<3) | (1<<0)),	//0x09
	INT_DEV_ABNORMAL_STATUS = ((1<<10) | (1<<9)),	//0x600
};

#define siw_chk_sts_snprintf(_dev, _buf, _max, _size, _fmt, _args...) \
		({	\
			int _n_size = 0;	\
			if (_size < _max)	\
				_n_size = snprintf(_buf + _size, _max - _size,\
								(const char *)_fmt, ##_args);	\
			t_dev_dbg_trace(_dev, (const char *)_fmt, ##_args);\
			_n_size;	\
		})

static int siw_hal_check_status_type_1(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	u32 status = chip->info.device_status;
	u32 ic_status = chip->info.ic_status;
	u32 dbg_mask = 0;
	int log_flag = 0;
	int log_max = IC_CHK_LOG_MAX;
	char log[IC_CHK_LOG_MAX] = {0, };
	int len = 0;
	int ret = 0;

	if (!(status & (1<<5))) {
		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b5] device ctl not Set ");
	}
	if (!(status & (1<<6))) {
		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b6] code crc invalid ");
	}
	if (!(status & (1<<7))) {
		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b7] cfg crc invalid ");
	}
	if (status & (1<<9)) {
		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b9] abnormal status detected ");
	}
	if (status & (1<<10)) {
		t_dev_err(dev, "h/w:%Xh, f/w:%Xh\n", (ic_status&(1<<1)), (status&(1<<10)));

		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b10] system error detected\n");

		if (chip->lcd_mode == LCD_MODE_U0) {
			ret = -ERESTART;
		} else {
		#if 1
			int esd = 1;
			ret = siw_touch_atomic_notifier_call(LCD_EVENT_TOUCH_ESD_DETECTED, (void *)&esd);
			if (ret) {
				t_dev_err(dev, "check the value, %d\n", ret);
			}
		#endif
		}
	}

	if (status & (1<<13)) {
		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b13] display mode mismatch ");
	}
	if (!(status & (1<<15))) {
		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b15] irq pin invalid ");
	}
	if (!(status & (1<<20))) {
		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b20] irq status invalid ");
	}
	if (!(status & (1<<22))) {
		log_flag = 1;
		len += siw_chk_sts_snprintf(dev, log, log_max, len,
					"[b22] driving invalid ");
	}

	if (log_flag) {
		t_dev_err(dev, "status %Xh, ic_status %Xh : %s\n",
			status, ic_status, log);
	}

	if ((ic_status&1) || (ic_status & (1<<3))) {
		t_dev_err(dev, "watchdog exception - status %Xh, ic_status %Xh\n",
					status, ic_status);
		if (chip->lcd_mode == LCD_MODE_U0) {
			ret = -ERESTART;
		} else {
		#if 1
			int esd = 1;
			ret = siw_touch_atomic_notifier_call(LCD_EVENT_TOUCH_ESD_DETECTED, (void*)&esd);
			if (ret)
				t_dev_err(dev, "check the value, %d\n", ret);
		#endif
		}
	}

	if (ret == -ERESTART) {
		return ret;
	}

	dbg_mask = ((status>>16) & 0xF);
	switch (dbg_mask) {
		case 0x2 :
		//	t_dev_dbg_irq(dev, "TC_Driving OK\n");
			/* fall through */
		case 0x3 :
			/* fall through */
		case 0x4 :
			t_dev_dbg_trace(dev, "dbg_mask %Xh\n", dbg_mask);
			ret = -ERANGE;
			break;
	}

	return ret;
}

static int siw_hal_check_status_default(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	u32 status = chip->info.device_status;
	u32 ic_status = chip->info.ic_status;
	int ret = 0;

	if (!(status & (1<<5))) {
		t_dev_err(dev, "abnormal device status %08Xh\n", status);
		ret = -ERESTART;
	} else if (status & INT_DEV_ABNORMAL_STATUS) {
		t_dev_err(dev, "abnormal device status %08Xh\n", status);
		ret = -ERESTART;
	}

	if (ic_status & INT_IC_ABNORMAL_STATUS) {
		t_dev_err(dev, "abnormal ic status %08Xh\n", status);
		ret = -ERESTART;
	}

	return ret;
}

static int siw_hal_check_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u32 status = chip->info.device_status;
	u32 ic_status = chip->info.ic_status;
	u32 status_mask = 0;
	int ret = 0;

	status_mask = status ^ INT_NORMAL_MASK;

	/*
	 * (normal state)
	 *                               [bit] 31   27   23   19   15   11   7    4
	 * status              = 0x06D5_80E7 = 0000 0110 1101 0101 1000 0000 1110 0111
	 *
	 * INT_NORMAL_MASK     = 0x0050_80E0 = 0000 0000 0101 0000 1000 0000 1110 0000
	 * status_mask         = 0x0685_0007 = 0000 0110 1000 0101 0000 0000 0000 0111
	 * INT_RESET_CLR_BIT   = 0x0000_0620 = 0000 0000 0000 0000 0000 0110 0010 0000
	 * INT_LOGGING_CLR_BIT = 0x0050_A0C0 = 0000 0000 0101 0000 1010 0000 1100 0000
	 */
	t_dev_dbg_trace(dev, "h/w:%Xh, f/w:%Xh(%Xh)\n", ic_status, status, status_mask);

	if (status_mask & INT_RESET_CLR_BIT) {
		t_dev_err(dev, "need reset : status %08Xh, ic_status %08Xh, chk %08Xh\n",
			status, ic_status, status_mask & INT_RESET_CLR_BIT);
		ret = -ERESTART;
	} else if (status_mask & INT_LOGGING_CLR_BIT) {
		t_dev_err(dev, "need logging : status %08Xh, ic_status %08Xh, chk %08Xh\n",
			status, ic_status, status_mask & INT_LOGGING_CLR_BIT);
		ret = -ERANGE;
	}
	if (ret < 0) {
		goto out;
	}

	switch(touch_chip_type(ts)) {
	case CHIP_LG4895:
	case CHIP_LG4946:
		ret = siw_hal_check_status_type_1(dev);
		break;
	default:
		ret = siw_hal_check_status_default(dev);
		break;
	}

out:
	return ret;
}

static int siw_hal_irq_abs_data(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_touch_data *data = chip->info.data;
	struct siw_hal_touch_data *data_curr;
	struct touch_data *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;

	touch_count = chip->info.touch_cnt;
	ts->new_mask = 0;

	/* check if palm detected */
	if (data[0].track_id == PALM_ID) {
		if (data[0].event == TOUCHSTS_DOWN) {
			ts->is_palm = 1;
			t_dev_info(dev, "Palm Detected\n");
		} else if (data[0].event == TOUCHSTS_UP) {
			ts->is_palm = 0;
			t_dev_info(dev, "Palm Released\n");
		}
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < touch_count; i++) {
		if (data[i].track_id >= MAX_FINGER)
			continue;

		data_curr = &data[i];
		if ((data_curr->event == TOUCHSTS_DOWN) ||
			(data_curr->event == TOUCHSTS_MOVE)) {
			ts->new_mask |= (1 << data_curr->track_id);
			tdata = ts->tdata + data_curr->track_id;

			tdata->id = data_curr->track_id;
			tdata->type = data_curr->tool_type;
			tdata->event = data_curr->event;
			tdata->x = data_curr->x;
			tdata->y = data_curr->y;
			tdata->pressure = data_curr->pressure;
			tdata->width_major = data_curr->width_major;
			tdata->width_minor = data_curr->width_minor;

			if (data_curr->width_major == data_curr->width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = data_curr->angle;

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

static int siw_hal_irq_abs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	/* check if touch cnt is valid */
	if (chip->info.touch_cnt == 0 || chip->info.touch_cnt > ts->caps.max_id) {
		t_dev_dbg_abs(dev, "Invalid touch count, %d(%d)\n",
				chip->info.touch_cnt, ts->caps.max_id);
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
			"Swipe Gesture: start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
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

static int siw_hal_irq_lpwg(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	if (chip->info.wakeup_type == KNOCK_1) {
		if (ts->lpwg.mode != LPWG_NONE) {
			siw_hal_get_tci_data(dev,
				ts->tci.info[TCI_1].tap_count);
			ts->intr_status = TOUCH_IRQ_KNOCK;
		}
	} else if (chip->info.wakeup_type == KNOCK_2) {
		if (ts->lpwg.mode == LPWG_PASSWORD) {
			siw_hal_get_tci_data(dev,
				ts->tci.info[TCI_2].tap_count);
			ts->intr_status = TOUCH_IRQ_PASSWD;
		}
	} else if (chip->info.wakeup_type == SWIPE_LEFT) {
		t_dev_info(dev, "SWIPE_LEFT\n");
		siw_hal_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
	} else if (chip->info.wakeup_type == SWIPE_RIGHT) {
		t_dev_info(dev, "SWIPE_RIGHT\n");
		siw_hal_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
	} else if (chip->info.wakeup_type == KNOCK_OVERTAP) {
		t_dev_info(dev, "LPWG wakeup_type is Overtap\n");
	//	siw_hal_get_tci_data(dev, 1);
		siw_hal_get_tci_data(dev, ts->tci.info[TCI_2].tap_count + 1);
		ts->intr_status = TOUCH_IRQ_PASSWD;
	} else if (chip->info.wakeup_type == CUSTOM_DEBUG) {
		t_dev_info(dev, "LPWG wakeup_type is CUSTOM_DEBUG\n");
		siw_hal_debug_tci(dev);
		siw_hal_debug_swipe(dev);
	} else {
		t_dev_info(dev, "LPWG wakeup_type is not support type![%d]\n",
			chip->info.wakeup_type);
	}

	return ret;
}

static int siw_hal_irq_handler(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ret = 0;

	pm_qos_update_request(&chip->pm_qos_req, 10);
	ret = siw_hal_reg_read(dev,
				reg->tc_ic_status,
				(void *)&chip->info, sizeof(chip->info));
	pm_qos_update_request(&chip->pm_qos_req, PM_QOS_DEFAULT_VALUE);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_check_status(dev);
	if (ret < 0) {
		goto out;
	}

	if (chip->info.wakeup_type == ABS_MODE) {
		ret = siw_hal_irq_abs(dev);
		if (ret) {
			t_dev_err(dev, "siw_hal_irq_abs failed, %d/n", ret);
			goto out;
		}
	} else {
		ret = siw_hal_irq_lpwg(dev);
		if (ret) {
			t_dev_err(dev, "siw_hal_irq_lpwg failed, %d/n", ret);
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
	chip->charger |= (wireless_state)? CONNECT_WIRELESS : 0;

	t_dev_info(dev, "write charger_state = 0x%02X\n", chip->charger);
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		t_dev_info(dev, "DEV_PM_SUSPEND - Don't try SPI\n");
		return;
	}

	siw_hal_write_value(dev,
			reg->spr_charger_status,
			chip->charger);
}

static void siw_hal_lcd_mode(struct device *dev, u32 mode_bit)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (touch_mode_not_allowed(ts, mode_bit)) {
		return;
	}

	t_dev_info(dev, "lcd_mode: %d (prev: %d)\n", mode_bit, chip->lcd_mode);

	if ((chip->lcd_mode == LCD_MODE_U2) &&
		siw_hal_watch_is_disp_waton(dev)) {
		siw_hal_watch_get_curr_time(dev, NULL, NULL);
	}

	if (mode_bit == LCD_MODE_U2_UNBLANK)
		mode_bit = LCD_MODE_U2;

	chip->prev_lcd_mode = chip->lcd_mode;
	chip->lcd_mode = mode_bit;
}

static void siw_hal_lcd_mode_idx(struct device *dev, u32 mode)
{
	if (mode >= LCD_MODE_IDX_MAX) {
		t_dev_err(dev, "invalid mode index, %d\n", mode);
		return;
	}

	siw_hal_lcd_mode(dev, BIT(mode));
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
extern int siw_hal_switch_to_abt_irq_handler(struct siw_ts *ts);

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
	int ret = 0;

	t_dev_dbg_noti(dev, "notify event(%d)\n", (u32)event);

	switch (event) {
	case NOTIFY_TOUCH_RESET:
		ret = !!(atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_1);
		t_dev_info(dev, "notify: reset, %d\n", ret);
	//	atomic_set(&chip->watch.state.font_status, FONT_EMPTY);
	//	atomic_set(&chip->block_watch_cfg, BLOCKED);
		break;
	case LCD_EVENT_TOUCH_RESET_START:
		atomic_set(&ts->state.hw_reset, event);

		t_dev_info(dev, "notify: lcd_event: touch reset start\n");
		siw_touch_irq_control(ts->dev, INTERRUPT_DISABLE);
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);
		break;
	case LCD_EVENT_TOUCH_RESET_END:
		atomic_set(&ts->state.hw_reset, event);

		t_dev_info(dev, "notify: lcd_event: touch reset end\n");
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);

		siw_touch_qd_init_work_hw(ts);
		break;
	case LCD_EVENT_LCD_MODE:
		t_dev_info(dev, "notify: lcd_event: lcd mode\n");
		siw_hal_lcd_mode_idx(dev, *(u32 *)data);
		ret = siw_hal_check_mode(dev);
		if (!ret) {
			queue_delayed_work(ts->wq, &chip->fb_notify_work, 0);
		}
		ret = 0;
		break;

	case LCD_EVENT_READ_REG:
		t_dev_info(dev, "notify: lcd event: read reg\n");
		siw_hal_lcd_event_read_reg(dev);
		break;

	case NOTIFY_CONNECTION:
		t_dev_info(dev, "notify: connection\n");
		ret = siw_hal_usb_status(dev, *(u32 *)data);
		break;
	case NOTIFY_WIRELEES:
		t_dev_info(dev, "notify: wireless\n");
		ret = siw_hal_wireless_status(dev, *(u32 *)data);
		break;
	case NOTIFY_EARJACK:
		t_dev_info(dev, "notify: earjack\n");
		ret = siw_hal_earjack_status(dev, *(u32 *)data);
		break;
	case NOTIFY_IME_STATE:
#if 0
		t_dev_info(dev, "notify: ime state\n");
		ret = siw_hal_reg_write(dev,
					reg->ime_state,
					(void *)data, sizeof(data));
#else
		t_dev_info(dev, "notify: do nothing for ime\n");
#endif
		break;
	case NOTIFY_DEBUG_TOOL:
		ret = siw_hal_debug_tool(dev, *(u32 *)data);
		t_dev_info(dev, "notify: debug tool\n");
		break;
	case NOTIFY_CALL_STATE:
		t_dev_info(dev, "notify: call state\n");
		ret = siw_hal_reg_write(dev,
					reg->call_state,
					(void *)data, sizeof(data));
		break;
	case LCD_EVENT_TOUCH_DRIVER_REGISTERED:
	case LCD_EVENT_TOUCH_DRIVER_UNREGISTERED:
		if (0) {
			/* from siw_touch_probe */
			t_dev_info(dev, "notify: driver %s\n",
					(event == LCD_EVENT_TOUCH_DRIVER_REGISTERED)?
					"registered" : "unregistered");
		}
		break;
	case LCD_EVENT_TOUCH_WATCH_LUT_UPDATE:
	case LCD_EVENT_TOUCH_WATCH_POS_UPDATE:
	case LCD_EVENT_TOUCH_PROXY_STATUS:
	case LCD_EVENT_TOUCH_ESD_DETECTED:
		t_dev_info(dev, "notify: %lu called\n", event);
		break;
	default:
		t_dev_err(dev, "notify: %lu is not supported\n", event);
		break;
	}

	return ret;
}

static int siw_hal_get_cmd_version(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 rdata[4] = {0};
	int offset = 0;
	int ret = 0;

	ret = siw_hal_ic_info(dev);
	if (ret < 0) {
		offset += siw_snprintf(buf, offset, "-1\n");
		offset += siw_snprintf(buf, offset, "Read Fail Touch IC Info\n");
		return offset;
	}

	offset += siw_snprintf(buf, offset,
				"version    : v%d.%02d\n",
				chip->fw.version[0], chip->fw.version[1]);

	if (chip->fw.revision == 0xFF) {
		offset += siw_snprintf(buf, offset,
					"revision   : Flash Erased(0xFF)\n");
	} else {
		offset += siw_snprintf(buf, offset,
					"revision   : %d\n", chip->fw.revision);
	}

	offset += siw_snprintf(buf, offset,
				"product id : [%s]\n", chip->fw.product_id);

	ret = siw_hal_reg_read(dev,
				reg->info_lot_num,
				(void *)&rdata, sizeof(rdata));
	offset += siw_snprintf(buf, offset, "lot    : %d\n", rdata[0]);
	offset += siw_snprintf(buf, offset, "serial : 0x%X\n", rdata[1]);
#if 0
	offset += siw_snprintf(buf, offset, "date   : 0x%X 0x%X\n",
					rdata[2], rdata[3]);
#endif
	offset += siw_snprintf(buf, offset, "date   : %04d.%02d.%02d " \
					"%02d:%02d:%02d Site%d\n",
					rdata[2] & 0xFFFF, (rdata[2] >> 16 & 0xFF), (rdata[2] >> 24 & 0xFF),
					rdata[3] & 0xFF, (rdata[3] >> 8 & 0xFF), (rdata[3] >> 16 & 0xFF),
					(rdata[3] >> 24 & 0xFF));

	return offset;
}

static int siw_hal_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int offset = 0;
	int ret = 0;

	ret = siw_hal_ic_info(dev);
	if (ret < 0) {
		offset += siw_snprintf(buf, offset, "-1\n");
		offset += siw_snprintf(buf, offset, "Read Fail Touch IC Info\n");
		return offset;
	}

	offset += siw_snprintf(buf, offset, "v%d.%02d\n",
				chip->fw.version[0], chip->fw.version[1]);

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
		ret = siw_hal_get_cmd_version(dev, (char *)buf);
		break;

	case CMD_ATCMD_VERSION:
		ret = siw_hal_get_cmd_atcmd_version(dev, (char *)buf);
		break;

	default:
		break;
	}

	return ret;
}

static void siw_hal_mon_handler_self_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 chip_id;
	int ret = 0;

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		return;
	}

	mutex_lock(&ts->lock);

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
	if (ret < 0) {
		t_dev_err(dev, "mon self-reset : recovery begins(hw reset)\n");

		siw_hal_reset_ctrl(dev, HW_RESET_SYNC);
	} else {
		t_dev_dbg_trace(dev, "mon self-reset : chip id check ok\n");
	}

	mutex_unlock(&ts->lock);
}

static int siw_hal_mon_handler(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		return 0;
	}

	t_dev_dbg_trace(dev, "mon handler begins\n");

	siw_hal_mon_handler_self_reset(dev);

	t_dev_dbg_trace(dev, "mon handler ends\n");
	return 0;
}

static int siw_hal_early_probe(struct device *dev)
{
	return 0;
}

static int siw_hal_probe(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_chip *chip = NULL;
	int ret = 0;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		t_dev_err(dev, "failed to allocate %s data\n",
				touch_chip_name(ts));
		return -ENOMEM;
	}

	chip->dev = dev;
	chip->reg = siw_ops_reg(ts);
	chip->ts = ts;

	touch_set_dev_data(ts, chip);

	siw_hal_init_gpios(dev);
	siw_hal_power_init(dev);

	siw_hal_init_locks(chip);
	siw_hal_init_works(chip);

	if (siw_touch_get_boot_mode() == SIW_TOUCH_CHARGER_MODE) {
		if (touch_mode_allowed(ts, LCD_MODE_U3_PARTIAL)) {
			/* U3P driving and maintain 100ms before Deep sleep */
			siw_hal_tc_driving(dev, LCD_MODE_U3_PARTIAL);
			touch_msleep(80);
		}

		/* Deep Sleep */
		siw_hal_deep_sleep(dev);

		siwmon_submit_ops_step_chip_wh_name(dev, "%s probe done(charger mode)",
				touch_chip_name(ts), 0);
		return 0;
	}

	siw_hal_get_tci_info(dev);
	siw_hal_get_swipe_info(dev);

	pm_qos_add_request(&chip->pm_qos_req,
				PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	chip->lcd_mode = LCD_MODE_U3;
	chip->tci_debug_type = 1;

	t_dev_dbg_base(dev, "%s probe done\n",
				touch_chip_name(ts));

	siwmon_submit_ops_step_chip_wh_name(dev, "%s probe done",
			touch_chip_name(ts), 0);

	return ret;
}

static int siw_hal_remove(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	pm_qos_remove_request(&chip->pm_qos_req);

	siw_hal_free_works(chip);
	siw_hal_free_locks(chip);

	siw_hal_free_gpios(dev);

	touch_set_dev_data(ts, NULL);

	devm_kfree(dev, chip);

	t_dev_dbg_base(dev, "%s remove done\n",
				touch_chip_name(ts));

	return 0;
}

static int siw_hal_suspend(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int mfst_mode = 0;
	int ret = 0;

	if (siw_touch_get_boot_mode() == SIW_TOUCH_CHARGER_MODE)
		return -EPERM;

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

	if (atomic_read(&chip->init) == IC_INIT_DONE)
		siw_hal_lpwg_mode(dev);
	else /* need init */
		ret = 1;

	t_dev_dbg_pm(dev, "%s suspend done\n",
			touch_chip_name(ts));

	return ret;
}

static int siw_hal_resume(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int mfst_mode = 0;
	int ret = 0;

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
	if (siw_touch_get_boot_mode() == SIW_TOUCH_CHARGER_MODE) {
		if (touch_mode_allowed(ts, LCD_MODE_U3_PARTIAL)) {
			/* U3P driving and maintain 100ms at Resume */
			siw_hal_tc_driving(dev, LCD_MODE_U3_PARTIAL);
			touch_msleep(80);
		}

		siw_hal_deep_sleep(dev);
		return -EPERM;
	}

	t_dev_dbg_pm(dev, "%s resume done\n",
			touch_chip_name(ts));

	return 0;
}

static const struct siw_hal_reg siw_touch_default_reg = {
	.spr_chip_id				= SPR_CHIP_ID,
	.spr_rst_ctl				= SPR_RST_CTL,
	.spr_boot_ctl				= SPR_BOOT_CTL,
	.spr_sram_ctl				= SPR_SRAM_CTL,
	.spr_boot_status			= SPR_BOOT_STS,
	.spr_subdisp_status			= SPR_SUBDISP_STS,
	.spr_code_offset			= SPR_CODE_OFFSET,
	.spr_data_offset			= SPR_DATA_OFFSET,
	.tc_ic_status				= TC_IC_STATUS,
	.tc_status					= TC_STS,
	.tc_version					= TC_VERSION,
	.tc_product_id1				= TC_PRODUCT_ID1,
	.tc_product_id2				= TC_PRODUCT_ID2,
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
	.data_access_addr			= DATA_ACCESS_ADDR,
	.tc_device_ctl				= TC_DEVICE_CTL,
	.tc_interrupt_ctl			= TC_INTERRUPT_CTL,
	.tc_interrupt_status		= TC_INTERRUPT_STS,
	.tc_drive_ctl				= TC_DRIVE_CTL,
	.tci_fail_debug_r			= TCI_FAIL_DEBUG_R,
	.tic_fail_bit_r				= TCI_FAIL_BIT_R,
	.tci_debug_r				= TCI_DEBUG_R,
	.tci_enable_w				= TCI_ENABLE_W,
	.tci_fail_debug_w			= TCI_FAIL_DEBUG_W,
	.tci_fail_bit_w				= TCI_FAIL_BIT_W,
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
	.swipe_fail_debug_w			= SWIPE_FAIL_DEBUG_W,
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
	.tc_tsp_test_data_offset	= TC_TSP_TEST_DATA_OFFSET,
	.tc_tsp_data_access_addr	= TC_TSP_DATA_ACCESS_ADDR,
	.tc_flash_dn_status			= TC_FLASH_DN_STS,
	.tc_confdn_base_addr		= TC_CONFDN_BASE_ADDR,
	.tc_flash_dn_ctl			= TC_FLASH_DN_CTL,
	.raw_data_ctl_read			= RAW_DATA_CTL_READ,
	.raw_data_ctl_write			= RAW_DATA_CTL_WRITE,
	.data_i2cbase_addr			= DATA_I2CBASE_ADDR,
	.serial_data_offset			= SERIAL_DATA_OFFSET,
#if defined(__SIW_SUPPORT_WATCH)
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
#endif	/* __SIW_SUPPORT_WATCH */
};

enum {
	HAL_MON_INTERVAL_DEFAULT = 5,
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
	.tc_driving			= siw_hal_tc_driving,
	.chk_status			= siw_hal_check_status,
	.irq_handler		= siw_hal_irq_handler,
	.irq_abs			= siw_hal_irq_abs,
	.irq_lpwg			= siw_hal_irq_lpwg,
	.power				= siw_hal_power,
	.upgrade			= siw_hal_upgrade,
	.lpwg				= siw_hal_lpwg,
	.asc				= siw_hal_asc,
	.notify				= siw_hal_notify,
	.set				= siw_hal_set,
	.get				= siw_hal_get,
	/* */
	.sysfs				= siw_hal_sysfs,
	/* */
	.mon_handler		= siw_hal_mon_handler,
	.mon_interval		= HAL_MON_INTERVAL_DEFAULT,
	/* */
	.abt_init			= siw_hal_abt_init,
	.abt_sysfs			= siw_hal_abt_sysfs,
	.prd_sysfs			= siw_hal_prd_sysfs,
	.watch_sysfs		= siw_hal_watch_sysfs,
};

struct siw_touch_operations *siw_hal_get_default_ops(int opt)
{
	return (struct siw_touch_operations *)&siw_touch_default_ops;
}


