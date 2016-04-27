/*
 * siw_touch_bus_spi.c - SiW touch bus spi driver
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

#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include "siw_touch.h"
#include "siw_touch_bus.h"
#include "siw_touch_irq.h"

#if defined(CONFIG_SPI_MASTER)

#define siwmon_submit_bus_spi_read(_spi, _data, _ret)	\
		siwmon_submit_bus(&_spi->dev, "SPI_R", _data, _ret)

#define siwmon_submit_bus_spi_write(_spi, _data, _ret)	\
		siwmon_submit_bus(&_spi->dev, "SPI_W", _data, _ret)

#define siwmon_submit_bus_spi_xfer(_spi, _data, _ret)	\
		siwmon_submit_bus(&_spi->dev, "SPI_X", _data, _ret)


static void siw_touch_spi_err_dump(struct spi_device *spi,
							struct spi_transfer *xs, int num,
							int _read)
{
	struct spi_transfer *x = xs;
	int i;

	t_dev_err(&spi->dev, "spi transfer err :\n");
	for (i=0 ; i<num ; i++) {
		t_dev_err(&spi->dev,
				" x[%d] - len %d, cs %d, bpw %d\n",
				i,
				x->len, x->cs_change, spi->bits_per_word);
		siw_touch_bus_err_dump_data(&spi->dev,
					(char *)x->tx_buf, x->len, i, "x");
		if (_read) {
			siw_touch_bus_err_dump_data(&spi->dev,
						(char *)x->rx_buf, x->len, i, "x");
		}
		x++;
	}
}

static int siw_touch_spi_do_read(struct spi_device *spi,
							struct touch_bus_msg *msg)
{
	struct siw_ts *ts = spi_get_drvdata(spi);
	int bus_tx_dummy_size = touch_tx_dummy_size(ts);
	struct spi_transfer x = { 0, };
	struct spi_message m;
	int ret = 0;

	/*
	 * Bus control can need to be modifyed up to main chipset sepc.
	 */

	if ((msg->rx_size > SIW_TOUCH_MAX_BUF_SIZE) ||
		(msg->tx_size > SIW_TOUCH_MAX_BUF_SIZE)) {
		t_dev_err(&spi->dev, "spi read: buffer overflow - rx %Xh, tx %Xh\n",
			msg->rx_size, msg->tx_size);
		return -EOVERFLOW;
	}

	spi_message_init(&m);

	//Add dummy packet
	while (bus_tx_dummy_size--) {
		msg->tx_buf[msg->tx_size++] = 0;
		msg->tx_buf[msg->tx_size++] = 0;
	}

	x.tx_buf = msg->tx_buf;
	x.rx_buf = msg->rx_buf;
	x.len = msg->rx_size;
	x.cs_change = 0;
	x.bits_per_word = spi->bits_per_word;

	spi_message_add_tail(&x, &m);

	ret = spi_sync(spi, &m);
	siwmon_submit_bus_spi_read(spi, msg, ret);
	if (ret < 0)
		siw_touch_spi_err_dump(spi, &x, 1, 1);

	return ret;
}

static int siw_touch_spi_read(struct device *dev, void *msg)
{
	return siw_touch_spi_do_read(to_spi_device(dev), (struct touch_bus_msg *)msg);
}

int siw_touch_spi_do_write(struct spi_device *spi,
						struct touch_bus_msg *msg)
{
	struct siw_ts *ts = spi_get_drvdata(spi);
	int bus_tx_dummy_size = touch_tx_dummy_size(ts);
	struct spi_transfer x = { 0, };
	struct spi_message m;
	int ret = 0;

	/*
	 * Bus control can need to be modifyed up to main chipset sepc.
	 */

	if (msg->tx_size > SIW_TOUCH_MAX_BUF_SIZE) {
		t_dev_err(&spi->dev, "spi write: buffer overflow - tx %Xh\n",
			msg->tx_size);
		return -EOVERFLOW;
	}

	spi_message_init(&m);

	//Add dummy packet
	while (bus_tx_dummy_size--) {
		msg->tx_buf[msg->tx_size++] = 0;
		msg->tx_buf[msg->tx_size++] = 0;
	}

	x.tx_buf = msg->tx_buf;
	x.rx_buf = msg->rx_buf;
	x.len = msg->tx_size;
	x.cs_change = 0;
	x.bits_per_word = spi->bits_per_word;

	spi_message_add_tail(&x, &m);

	ret = spi_sync(spi, &m);
	siwmon_submit_bus_spi_write(spi, msg, ret);
	if (ret < 0)
		siw_touch_spi_err_dump(spi, &x, 1, 0);

	return ret;
}

static int siw_touch_spi_write(struct device *dev, void *msg)
{
	return siw_touch_spi_do_write(to_spi_device(dev), (struct touch_bus_msg *)msg);
}

#if defined(CONFIG_TOUCHSCREEN_SIWMON)
static void __siw_touch_spi_xfer_mon(struct spi_device *spi,
				struct touch_xfer_msg *xfer,
				int ret)
{
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	struct touch_bus_msg msg;
	int cnt = xfer->msg_count;
	int i;

	for (i = 0; i < cnt; i++) {
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;

		msg.tx_buf = tx->data;
		msg.tx_size = tx->size;
		msg.rx_buf = rx->data;
		msg.rx_size = rx->size;
		msg.bits_per_word = spi->bits_per_word;
		msg.priv = (i<<8) | cnt;

		//For xfer mon,
		//the last character of dir string shall be 'X'
		siwmon_submit_bus_spi_xfer(spi, &msg, ret);
	}
}
#else	/* CONFIG_TOUCHSCREEN_SIWMON */
static inline void __siw_touch_spi_xfer_mon(struct spi_device *spi,
				struct touch_xfer_msg *xfer,
				int ret){ }
#endif	/* CONFIG_TOUCHSCREEN_SIWMON */

static int siw_touch_spi_do_xfer(struct spi_device *spi, struct touch_xfer_msg *xfer)
{
//	struct siw_ts *ts = spi_get_drvdata(spi);
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	struct spi_transfer x[SIW_TOUCH_MAX_XFER_COUNT];
	struct spi_message m;
	int cnt = xfer->msg_count;
	int i = 0;
	int ret = 0;

	/*
	 * Bus control can need to be modifyed up to main chipset sepc.
	 */

	if (cnt > SIW_TOUCH_MAX_XFER_COUNT) {
		t_dev_err(&spi->dev, "cout exceed, %d\n", cnt);
		return -EINVAL;
	}

	spi_message_init(&m);
	memset(x, 0, sizeof(x));

	for (i = 0; i < cnt; i++) {
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;

		x[i].cs_change = !!(i < (xfer->msg_count - 1));
		x[i].bits_per_word = spi->bits_per_word;

		if (rx->size) {
			x[i].tx_buf = tx->data;
			x[i].rx_buf = rx->data;
			x[i].len = rx->size;
		} else {
			x[i].tx_buf = tx->data;
			x[i].rx_buf = NULL;
			x[i].len = tx->size;
		}
		spi_message_add_tail(&x[i], &m);
	}

	ret = spi_sync(spi, &m);
	__siw_touch_spi_xfer_mon(spi, xfer, ret);
	if (ret < 0)
		siw_touch_spi_err_dump(spi, &x[0], cnt, 0);

	return ret;
}

static int siw_touch_spi_xfer(struct device *dev, void *xfer)
{
	return siw_touch_spi_do_xfer(to_spi_device(dev), (struct touch_xfer_msg *)xfer);
}

static struct siw_ts *siw_touch_spi_alloc(
			struct spi_device *spi,
			struct siw_touch_bus_drv *bus_drv)
{
	struct device *dev = &spi->dev;
	struct siw_ts *ts = NULL;
	struct siw_touch_pdata *pdata = NULL;
	u32 tmp;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		t_dev_err(dev,
				"spi alloc: failed to allocate memory for touch data\n");
		goto out;
	}

	ts->bus_dev = spi;
	ts->addr = (u32)spi;
	ts->dev = dev;
	ts->irq = spi->irq;
	pdata = bus_drv->pdata;
	if (!pdata) {
		t_dev_err(dev, "spi alloc: NULL pdata\n");
		goto out_pdata;
	}
	ts->pdata = pdata;

	siw_setup_operations(ts, bus_drv->pdata->ops);

	ts->bus_read = siw_touch_spi_read;
	ts->bus_write = siw_touch_spi_write;
	ts->bus_xfer = siw_touch_spi_xfer;
	ts->bus_tx_hdr_size = pdata_tx_hdr_size(pdata);
	ts->bus_rx_hdr_size = pdata_rx_hdr_size(pdata);
	ts->bus_tx_dummy_size = pdata_tx_dummy_size(pdata);
	ts->bus_rx_dummy_size = pdata_rx_dummy_size(pdata);

	spi->chip_select = 0;

	tmp = pdata_bits_per_word(pdata);
	if (tmp == ~0) {
		t_dev_err(dev, "spi alloc: wrong spi setup: bits_per_word, %d\n", tmp);
		goto out_spi;
	}
	spi->bits_per_word = (u8)tmp;

	tmp = pdata_spi_mode(pdata);
	if (tmp == ~0) {
		t_dev_err(dev, "spi alloc: wrong spi setup: spi_mode, %d\n", tmp);
		goto out_spi;
	}
	spi->mode = tmp;

	tmp = pdata_max_freq(pdata);
	if ((tmp == ~0) || (tmp < 1000000)) {
		t_dev_err(dev, "spi alloc: wrong spi setup: max_freq, %d\n", tmp);
		goto out_spi;
	}
	spi->max_speed_hz = tmp;
	t_dev_info(dev, "spi alloc: %d Mhz\n", tmp/1000000);

	spi_set_drvdata(spi, ts);

	return ts;

out_spi:

out_pdata:
	devm_kfree(dev, ts);

out:
	return NULL;
}

static void siw_touch_spi_free(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct siw_ts *ts = to_touch_core(dev);

	spi_set_drvdata(spi, NULL);
	devm_kfree(dev, ts);
}

static int siw_touch_spi_probe(struct spi_device *spi)
{
	struct siw_touch_bus_drv *bus_drv = NULL;
	struct siw_ts *ts = NULL;
	struct device *dev = &spi->dev;
	int ret = 0;

	t_dev_info_bus_parent(dev);

	bus_drv = container_of(to_spi_driver(dev->driver),
					struct siw_touch_bus_drv, bus.spi_drv);
	if (bus_drv == NULL) {
		t_dev_err(dev, "NULL bus_drv\n");
		return -EINVAL;
	}

	ts = siw_touch_spi_alloc(spi, bus_drv);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	ret = siw_touch_probe(ts);
	if (ret)
		goto out_plat;

	return 0;

out_plat:

out:
	return ret;
}

static int siw_touch_spi_remove(struct spi_device *spi)
{
	struct siw_ts *ts = to_touch_core(&spi->dev);

	siw_touch_remove(ts);

	siw_touch_spi_free(spi);

	return 0;
}

static int siw_touch_spi_pm_suspend(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);

	siw_touch_suspend_call(dev);

	atomic_set(&ts->state.pm, DEV_PM_SUSPEND);

	t_dev_dbg_pm(dev, "dev pm suspend\n");

	return 0;
}

static int siw_touch_spi_pm_resume(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);

	siw_touch_resume_call(dev);

	if (atomic_read(&ts->state.pm) == DEV_PM_SUSPEND_IRQ) {
		atomic_set(&ts->state.pm, DEV_PM_RESUME);

		siw_touch_resume_irq(dev);
		goto out;
	}

	atomic_set(&ts->state.pm, DEV_PM_RESUME);

out:
	t_dev_dbg_pm(dev, "dev pm resume\n");

	return 0;
}


static const struct dev_pm_ops siw_touch_spi_pm_ops = {
	.suspend = siw_touch_spi_pm_suspend,
	.resume = siw_touch_spi_pm_resume,
};

static struct spi_device_id siw_touch_spi_id[] = {
	{ SIW_TOUCH_NAME, 0 },
	{ }
};

int siw_touch_spi_add_driver(void *data)
{
	struct siw_touch_pdata *pdata = data;
	struct siw_touch_bus_drv *bus_drv = NULL;
	struct spi_driver *spi_drv = NULL;
	char *drv_name = NULL;
	int ret = 0;

	if (pdata == NULL) {
		t_pr_err("NULL touch driver\n");
		return -EINVAL;
	}

	bus_drv = (struct siw_touch_bus_drv *)kzalloc(sizeof(*bus_drv), GFP_KERNEL);
	if (!bus_drv) {
		t_pr_err("faied to allocate bus_drv(%d)\n", pdata_bus_type(pdata));
		ret = -ENOMEM;
		goto out;
	}

	drv_name = pdata_drv_name(pdata);

	bus_drv->pdata = pdata;

	spi_drv = &bus_drv->bus.spi_drv;
	spi_drv->driver.name = drv_name;
	spi_drv->driver.owner = pdata->owner;
	spi_drv->driver.of_match_table = pdata->of_match_table;
	spi_drv->driver.pm = &siw_touch_spi_pm_ops;

	spi_drv->probe = siw_touch_spi_probe;
	spi_drv->remove = siw_touch_spi_remove;
	spi_drv->id_table = siw_touch_spi_id;
	if (drv_name) {
		memset((void *)siw_touch_spi_id[0].name, 0, SPI_NAME_SIZE);
		snprintf((char *)siw_touch_spi_id[0].name,
				SPI_NAME_SIZE,
				"%s",
				drv_name);
	}

	ret = spi_register_driver(spi_drv);
	if (ret) {
		t_pr_err("spi_register_driver[%s] failed, %d\n",
				drv_name, ret);
		goto out_spi;
	}

	pdata_set_bus_drv(pdata, bus_drv);

	return 0;

out_spi:
	kfree(bus_drv);

out:
	return ret;
}

int siw_touch_spi_del_driver(void *data)
{
	struct siw_touch_pdata *pdata = data;
	void *bus_drv;

	if (pdata == NULL) {
		t_pr_err("NULL touch driver\n");
		return -ENODEV;
	}

	bus_drv = (void *)pdata_get_bus_drv(pdata);
	if (bus_drv) {
		spi_unregister_driver(&((struct siw_touch_bus_drv *)bus_drv)->bus.spi_drv);

		kfree(bus_drv);
		pdata_set_bus_drv(pdata, NULL);
	}

	return 0;
}

#else	/* CONFIG_SPI_MASTER */
int siw_touch_spi_add_driver(void *data)
{
	struct siw_touch_pdata *pdata = data;

	t_pr_err("SPI : not supported in this system\n");
	return -ENODEV;
}

int siw_touch_spi_del_driver(void *data)
{
	struct siw_touch_pdata *pdata = data;

	t_pr_err("SPI : not supported in this system\n");
	return -ENODEV;
}

#endif	/* CONFIG_SPI_MASTER */

