/*
 * siw_touch_bus.c - SiW touch bus core driver
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
#include <linux/dma-mapping.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include <linux/spi/spi.h>

#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"
#include "siw_touch_bus_i2c.h"
#include "siw_touch_bus_spi.h"
#include "siw_touch_sys.h"


int siw_touch_bus_tr_data_init(struct siw_ts *ts)
{
	struct siw_touch_pdata *pdata = ts->pdata;

	ts->bus_tx_hdr_size = pdata_tx_hdr_size(pdata);
	ts->bus_rx_hdr_size = pdata_rx_hdr_size(pdata);
	ts->bus_tx_dummy_size = pdata_tx_dummy_size(pdata);
	ts->bus_rx_dummy_size = pdata_rx_dummy_size(pdata);

	return 0;
}

void siw_touch_bus_tr_data_free(struct siw_ts *ts)
{

}


#define TOUCH_PINCTRL_ACTIVE	"touch_pin_active"
#define TOUCH_PINCTRL_SLEEP		"touch_pin_sleep"

#if defined(__SIW_SUPPORT_PINCTRL)	//See siw_touch_cfg.h
int siw_touch_bus_pin_get(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct touch_pinctrl *pinctrl = &ts->pinctrl;
	struct pinctrl *pin_ctrl = NULL;
	struct pinctrl_state *pin_active = NULL;
	struct pinctrl_state *pin_suspend = NULL;
	int ret = 0;

	t_dev_dbg_base(dev, "get pinctrl\n");

	pin_ctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pin_ctrl)) {
		if (PTR_ERR(pin_ctrl) == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			return ret;
		}

		t_dev_info(dev, "pinctrl not used\n");
		goto out;
	}

	pin_active = pinctrl_lookup_state(pin_ctrl,
								TOUCH_PINCTRL_ACTIVE);
	if (IS_ERR_OR_NULL(pin_active)) {
		t_dev_dbg_gpio(dev, "cannot get pinctrl active\n");
		goto out;
	}

	pin_suspend = pinctrl_lookup_state(pin_ctrl,
								TOUCH_PINCTRL_SLEEP);
	if (IS_ERR_OR_NULL(pin_suspend)) {
		t_dev_dbg_gpio(dev, "cannot get pinctrl suspend\n");
		goto out;
	}

	ret = pinctrl_select_state(pin_ctrl, pin_active);
	if (ret) {
		t_dev_dbg_gpio(dev, "cannot set pinctrl active\n");
		goto out;
	}

	t_dev_info(dev, "pinctrl set active\n");

	pinctrl->ctrl = pin_ctrl;
	pinctrl->active = pin_active;
	pinctrl->suspend = pin_suspend;

out:
	return ret;
}

int siw_touch_bus_pin_put(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct touch_pinctrl *pinctrl = &ts->pinctrl;

	t_dev_dbg_base(dev, "put pinctrl\n");

	if (pinctrl->ctrl && !IS_ERR_OR_NULL(pinctrl->ctrl)) {
		devm_pinctrl_put(ts->pinctrl.ctrl);
		memset((void *)pinctrl, 0, sizeof(struct touch_pinctrl));
	}

	return 0;
}
#else	/* __SIW_SUPPORT_PINCTRL */
int siw_touch_bus_pin_get(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	t_dev_dbg_base(dev, "get pinctrl, nop ...\n");
	return 0;
}

int siw_touch_bus_pin_put(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	t_dev_dbg_base(dev, "put pinctrl, nop ...\n");
	return 0;
}
#endif	/* __SIW_SUPPORT_PINCTRL */

void *siw_touch_bus_create_bus_drv(int bus_type)
{
	struct siw_touch_bus_drv *bus_drv;

	bus_drv = kzalloc(sizeof(*bus_drv), GFP_KERNEL);
	if (!bus_drv) {
		t_pr_err("faied to allocate bus_drv(%d)\n", bus_type);
	}
	return bus_drv;
}

void *siw_touch_bus_create_bus_pdata(int bus_type)
{
	struct siw_touch_pdata *pdata;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		t_pr_err("faied to allocate pdata(%d)\n", bus_type);
	}
	return pdata;
}

static void *__buffer_alloc(struct device *dev, size_t size,
				dma_addr_t *dma_handle, gfp_t gfp,
				const char *name)
{
	void *buf;

	if (siw_touch_sys_bus_use_dma(dev) && dma_handle) {
		buf = dma_alloc_coherent(NULL, size, dma_handle, gfp);
		if (buf) {
			t_dev_dbg_base(dev, "alloc %s: buf %08Xh, phy %Xh, size %Xh\n",
					name, (u32)buf, (*dma_handle), size);
		}
	} else {
		buf = devm_kzalloc(dev, size, gfp);
		if (buf) {
			t_dev_dbg_base(dev, "alloc %s: buf %08Xh, size %Xh\n",
					name, (u32)buf, size);
			if (dma_handle)
				*dma_handle = 0;
		}
	}
	if (!buf) {
		t_dev_err(dev, "failed to allocate %s\n", name);
	}

	return buf;
}

static void __buffer_free(struct device *dev, size_t size,
				void *buf, dma_addr_t dma_handle,
				const char *name)
{
	if (siw_touch_sys_bus_use_dma(dev) && dma_handle) {
		t_dev_dbg_base(dev, "free %s: buf %08Xh, phy %Xh, size %Xh\n",
					name, (u32)buf, dma_handle, size);
		dma_free_coherent(NULL, size, buf, dma_handle);
		return;
	}

	t_dev_dbg_base(dev, "free %s: buf %08Xh, size %Xh\n",
					name, (u32)buf, size);
	devm_kfree(dev, buf);
}

int siw_touch_bus_alloc_buffer(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int buf_size = touch_buf_size(ts);
	u8 *tx_buf = NULL;
	u8 *rx_buf = NULL;
	dma_addr_t tx_pa = 0;
	dma_addr_t rx_pa = 0;
	struct touch_xfer_msg *xfer = NULL;
	int ret = 0;

	if (!buf_size)
		buf_size = SIW_TOUCH_MAX_XFER_BUF_SIZE;

	t_dev_dbg_base(dev, "allocate touch bus buffer\n");

	tx_buf = __buffer_alloc(dev, buf_size,
					&tx_pa, GFP_KERNEL | GFP_DMA, "tx_buf");
	if (!tx_buf) {
		ret = -ENOMEM;
		goto out_tx_buf;
	}

	rx_buf = __buffer_alloc(dev, buf_size,
					&rx_pa, GFP_KERNEL | GFP_DMA, "rx_buf");
	if (!rx_buf) {
		ret = -ENOMEM;
		goto out_rx_buf;
	}

	xfer = __buffer_alloc(dev, sizeof(struct touch_xfer_msg),
					NULL, GFP_KERNEL, "xfer");
	if (!xfer) {
		ret = -ENOMEM;
		goto out_xfer;
	}

	ts->xfer = xfer;
	ts->tx_buf = tx_buf;
	ts->rx_buf = rx_buf;
	ts->tx_pa = tx_pa;
	ts->rx_pa = rx_pa;

	return 0;

out_xfer:
	if (rx_buf) {
		__buffer_free(dev, buf_size, rx_buf, rx_pa, "rx_buf");
	}

out_rx_buf:
	if (tx_buf) {
		__buffer_free(dev, buf_size, tx_buf, tx_pa, "tx_buf");
	}

out_tx_buf:

	return ret;
}

int siw_touch_bus_free_buffer(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int buf_size = touch_buf_size(ts);

	t_dev_dbg_base(dev, "release touch bus buffer\n");

	if (ts->xfer) {
		__buffer_free(dev, sizeof(struct touch_xfer_msg),
				ts->xfer, 0, "xfer");
		ts->xfer = NULL;
	}

	if (ts->rx_buf) {
		__buffer_free(dev, buf_size,
				ts->rx_buf, ts->rx_pa, "rx_buf");
		ts->rx_buf= NULL;
	}

	if (ts->tx_buf) {
		__buffer_free(dev, buf_size,
				ts->tx_buf, ts->tx_pa, "tx_buf");
		ts->tx_buf = NULL;
	}

	return 0;
}

int siw_touch_bus_init(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (!ts->bus_init) {
		t_dev_err(dev, "no bus_init %s(%d)\n",
			dev_name(dev), touch_bus_type(ts));
		return -EINVAL;
	}

	return ts->bus_init(dev);
}

int siw_touch_bus_read(struct device *dev,
					struct touch_bus_msg *msg)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (!ts->bus_read) {
		t_dev_err(dev, "no bus_read %s(%d)\n",
			dev_name(dev), touch_bus_type(ts));
		return -EINVAL;
	}

	return ts->bus_read(dev, msg);
}

int siw_touch_bus_write(struct device *dev, struct touch_bus_msg *msg)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (!ts->bus_write) {
		t_dev_err(dev, "no bus_write for %s(%d)\n",
			dev_name(dev), touch_bus_type(ts));
		return -EINVAL;
	}

	return ts->bus_write(dev, msg);
}

int siw_touch_bus_xfer(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (!ts->bus_xfer) {
		t_dev_err(dev, "No bus_xfer for %s(%d)\n",
			dev_name(dev), touch_bus_type(ts));
		return -EINVAL;
	}

	return ts->bus_xfer(dev, xfer);
}

enum {
	SIW_BUS_ERR_PRT_BOUNDARY	= 16,
	SIW_BUS_ERR_PRT_BUF_SZ		= 128,
};
void siw_touch_bus_err_dump_data(struct device *dev,
							u8 *buf, int len,
							int idx, char *name)
{
	char __prt_buf[SIW_BUS_ERR_PRT_BUF_SZ + 1] = {0, };
	char *prt_buf;
	int prt_len;
	int prt_pos = 0;
	int prt_idx;
	int cnt, total;
	int i, k;

	if (!buf || !len)
		return;

	cnt = (len + SIW_BUS_ERR_PRT_BOUNDARY - 1)/SIW_BUS_ERR_PRT_BOUNDARY;
	total = cnt;

	prt_idx = 0;
	for (i=0 ; i<cnt ; i++) {
		prt_len = min(len, SIW_BUS_ERR_PRT_BOUNDARY);
		prt_buf = __prt_buf;
		prt_pos = 0;
		for (k=0 ; k<prt_len ; k++) {
			prt_pos += snprintf(prt_buf + prt_pos,
							SIW_BUS_ERR_PRT_BUF_SZ - prt_pos,
							"%02X ",
							buf[prt_idx+k]);
			if (prt_pos >= SIW_BUS_ERR_PRT_BUF_SZ) {
				cnt = 0;
				break;
			}
		}
		t_dev_err(dev,
				" - %s[%d] buf[%3d~%3d] %s\n",
				name, idx,
				prt_idx,
				prt_idx + prt_len - 1,
				__prt_buf);

		len -= prt_len;
		prt_idx += prt_len;
	}
}


static const struct siw_op_dbg siw_bus_init_ops[2][2] = {
	[BUS_IF_I2C] = {
		_SIW_OP_DBG(DRIVER_FREE, siw_touch_i2c_del_driver, NULL, 0),
		_SIW_OP_DBG(DRIVER_INIT, siw_touch_i2c_add_driver, NULL, 0),
	},
	[BUS_IF_SPI] = {
		_SIW_OP_DBG(DRIVER_FREE, siw_touch_spi_del_driver, NULL, 0),
		_SIW_OP_DBG(DRIVER_INIT, siw_touch_spi_add_driver, NULL, 0),
	},
};

#define SIW_BUS_MAX		sizeof(siw_bus_init_ops) / sizeof(siw_bus_init_ops[0])

static int __siw_touch_bus_add_chk(struct siw_touch_chip_data *chip_data)
{
	int bus_type;

	if (chip_data == NULL) {
		t_pr_err("NULL touch chip data\n");
		return -ENODEV;
	}

	if (chip_data->pdata == NULL) {
		t_pr_err("NULL touch pdata\n");
		return -ENODEV;
	}

	bus_type = pdata_bus_type((struct siw_touch_pdata *)chip_data->pdata);
	if (bus_type >= SIW_BUS_MAX) {
		t_pr_err("Unknown touch interface : %d\n", bus_type);
		return -EINVAL;
	}

	return 0;
}

static int __siw_touch_bus_add_op(
					struct siw_touch_chip_data *chip_data,
					void *op_func)
{
	struct siw_op_dbg *op = op_func;
	int ret = 0;

	ret = __siw_touch_bus_add_chk(chip_data);
	if (ret)
		goto out;

//	t_pr_info("%s\n", op->name);
	ret = __siw_touch_op_dbg(op, chip_data);
	if (ret) {
		t_pr_err("%s failed, %d\n", op->name, ret);
		goto out;
	}

out:
	return ret;
}

static int __siw_touch_bus_add_driver(struct siw_touch_chip_data *chip_data, int on_off)
{
	return __siw_touch_bus_add_op(chip_data,
			(void *)&siw_bus_init_ops[chip_data->pdata->bus_info.bus_type][on_off]);
}

int siw_touch_bus_add_driver(struct siw_touch_chip_data *chip_data)
{
	return __siw_touch_bus_add_driver(chip_data, DRIVER_INIT);
}

int siw_touch_bus_del_driver(struct siw_touch_chip_data *chip_data)
{
	return __siw_touch_bus_add_driver(chip_data, DRIVER_FREE);
}


