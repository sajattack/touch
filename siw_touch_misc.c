/*
 * siw_touch_misc.c - SiW touch misc driver
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "siw_touch_cfg.h"

#if defined(__SIW_SUPPORT_MISC)	//See siw_touch_cfg.h

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
#include <linux/miscdevice.h>

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

/*
 * Helper layer to support direct access via device node
 * This misc layer makes a dedicated device node(/dev/{misc name}) for touch device
 * and user app can access the bus regardless the interface type.
 *
 * [Background]
 * I2C:
 *  I2C supports /dev/i2c-x (i2c-dev.c) node and user can access this bus
 *  during the touch driver is working state.
 * SPI:
 *  You can't use spidev(CONFIG_SPI_SPIDEV) like this way
 *  because it's different from the I2C driver.
 *
 * [Caution]
 * If user app wants to access /dev/i2c-x directly without loading touch driver,
 * the GPIO control for TCH_RSTn pin is necessary
 * because the touch device can't work properly when the TCH_RSTn is low state.
 * This can be controlled via '/dev/class/gpio/export' in user-space
 * if the kernel of target system supports this 'export' attr.
 *
 */

#define SIW_MISC_NAME	"siw_touch_misc"

enum {
	SIW_MISC_BUF_SZ = (4<<10),
	/* */
	SIW_MISC_NAME_SZ = 128,
};

struct siw_misc_data {
	struct miscdevice misc;
	struct device *dev;
	/* */
	char name[SIW_MISC_NAME_SZ];
//	spinlock_t bus_lock;
	int users;
	u8* buf;
};

struct siw_misc_data *__siw_misc_data = NULL;

DEFINE_MUTEX(siw_misc_lock);

#if defined(CONFIG_SPI_MASTER)
struct sb_bus_msg_user {
	unsigned char *tx_buf;
	int tx_size;
	unsigned char *rx_buf;
	int rx_size;
//	int reg;
//	int fd;
//	int slave;
//	char *fname;
};

static ssize_t siw_misc_spi_transfer(struct siw_misc_data *misc_data,
					char __user *buf, size_t len)
{
	struct sb_bus_msg_user msg_user;
	struct device *dev = misc_data->dev;
	struct spi_device *spi = to_touch_core(dev)->bus_dev;
	struct spi_transfer x = {
		.tx_dma = 0,
		.rx_dma = 0,
		.cs_change = 0,
		.bits_per_word = spi->bits_per_word,
		.delay_usecs = 0,
		.speed_hz = spi->max_speed_hz,
	};
	struct spi_message	m;
	int tx_size = 0;
	int rx_size = 0;
	int ret = 0;

	ret = copy_from_user(&msg_user, buf, sizeof(msg_user));
	if (ret) {
		t_dev_info(dev, "can't get msg_user data\n");
		return ret;
	}

	tx_size = msg_user.tx_size;
	rx_size = msg_user.rx_size;

	if (!tx_size && !rx_size) {
		t_dev_info(dev, "no data size\n");
		return -EFAULT;
	}
	if (!msg_user.tx_buf || !tx_size) {
		t_dev_info(dev, "no tx data\n");
		return -EFAULT;
	}

	x.tx_buf = misc_data->buf + SIW_MISC_BUF_SZ - 128;
	x.rx_buf = (msg_user.rx_buf) ? misc_data->buf : NULL;
	x.len = (msg_user.rx_buf) ? rx_size : tx_size;

	ret = copy_from_user((void *)x.tx_buf, (const void __user *)msg_user.tx_buf, tx_size);
	if (ret) {
		t_dev_info(dev, "can't get tx_buf(%d) data, %d\n",
			tx_size, ret);
		return ret;
	}

	spi_message_init(&m);
	spi_message_add_tail(&x, &m);
	ret = spi_sync(spi, &m);
	if (ret < 0) {
		t_dev_info(dev, "spi %s failed, %d",
			(msg_user.rx_buf) ? "recv" : "send",
			ret);
		return ret;
	}

	if (msg_user.rx_buf) {
		if (rx_size <= 0) {
			t_dev_info(dev, "invalid rx_size, %d\n",
				rx_size);
			return -EFAULT;
		}

		if (!access_ok(VERIFY_WRITE,
				(void __user *)msg_user.rx_buf,
				rx_size)) {
			return -EFAULT;
		}

		ret = copy_to_user((void __user *)msg_user.rx_buf,
					(const void *)(((u8 *)x.rx_buf) + SPI_BUS_RX_HDR_SZ), rx_size);
		if (ret) {
			t_dev_info(dev, "can't set rx_buf(%d) data, %d\n",
				rx_size, ret);
			return ret;
		}
	}

	return ret;
}

static ssize_t siw_misc_spi_recv(struct siw_misc_data *misc_data,
					char __user *buf, size_t len)
{
	return siw_misc_spi_transfer(misc_data, buf, len);
}

static ssize_t siw_misc_spi_send(struct siw_misc_data *misc_data,
					const char __user *buf, size_t len)
{
	return siw_misc_spi_transfer(misc_data, (char __user *)buf, len);
}
#else
static ssize_t siw_misc_spi_recv(struct siw_misc_data *misc_data,
					char __user *buf, size_t len)
{
	t_pr_err("SPI: not supported in this system\n");
	return -EFAULT;
}

static ssize_t siw_misc_spi_send(struct siw_misc_data *misc_data,
					const char __user *buf, size_t len)
{
	t_pr_err("SPI: not supported in this system\n");
	return -EFAULT;
}

#endif

#if defined(CONFIG_I2C)
static ssize_t siw_misc_i2c_recv(struct siw_misc_data *misc_data,
					char __user *buf, size_t len)
{
	struct device *dev = misc_data->dev;
	struct i2c_client *i2c = to_touch_core(dev)->bus_dev;
	unsigned long missing;
	int ret = 0;

	ret = i2c_master_recv(i2c, misc_data->buf, len);
	if (ret > 0) {
		missing = copy_to_user(buf, misc_data->buf, ret);
		if (missing == ret)
			ret = -EFAULT;
		else
			ret -= missing;
	}

	return ret;
}

static ssize_t siw_misc_i2c_send(struct siw_misc_data *misc_data,
					const char __user *buf, size_t len)
{
	struct device *dev = misc_data->dev;
	struct i2c_client *i2c = to_touch_core(dev)->bus_dev;
	unsigned long missing;
	int ret = -EFAULT;

	missing = copy_from_user(misc_data->buf, buf, len);
	if (missing == 0) {
		ret = i2c_master_send(i2c, misc_data->buf, len);
	}

	return ret;
}
#else
static ssize_t siw_misc_i2c_recv(struct siw_misc_data *misc_data,
					char __user *buf, size_t len)
{
	t_pr_err("I2C: not supported in this system\n");
	return -EFAULT;
}

static ssize_t siw_misc_i2c_send(struct siw_misc_data *misc_data,
					const char __user *buf, size_t len)
{
	t_pr_err("I2C: not supported in this system\n");
	return -EFAULT;
}
#endif

static ssize_t siw_misc_read(struct file *filp,
					char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct siw_misc_data *misc_data = __siw_misc_data;
	struct device *dev = NULL;
	struct siw_touch_chip *chip = NULL;
	struct siw_ts *ts = NULL;
	ssize_t ret = -EFAULT;

	if (count > SIW_MISC_BUF_SZ) {
		return -EMSGSIZE;
	}

	mutex_lock(&siw_misc_lock);

	dev = misc_data->dev;
	chip = to_touch_chip(dev);
	ts = chip->ts;

	mutex_lock(&chip->bus_lock);
	switch (touch_bus_type(ts)) {
	case BUS_IF_I2C:
		ret = siw_misc_i2c_recv(misc_data, buf, count);
		break;
	case BUS_IF_SPI:
		ret = siw_misc_spi_recv(misc_data, buf, count);
		break;
	}
	mutex_unlock(&chip->bus_lock);

	mutex_unlock(&siw_misc_lock);

	return ret;
}

static ssize_t siw_misc_write(struct file *filp,
					const char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct siw_misc_data *misc_data = __siw_misc_data;
	struct device *dev = NULL;
	struct siw_touch_chip *chip = NULL;
	struct siw_ts *ts = NULL;
	ssize_t ret = -EFAULT;

	if (count > SIW_MISC_BUF_SZ) {
		return -EMSGSIZE;
	}

	mutex_lock(&siw_misc_lock);

	dev = misc_data->dev;
	chip = to_touch_chip(dev);
	ts = chip->ts;

	mutex_lock(&chip->bus_lock);
	switch (touch_bus_type(ts)) {
	case BUS_IF_I2C:
		ret = siw_misc_i2c_send(misc_data, buf, count);
		break;
	case BUS_IF_SPI:
		ret = siw_misc_spi_send(misc_data, buf, count);
		break;
	}
	mutex_unlock(&chip->bus_lock);

	mutex_unlock(&siw_misc_lock);

	return ret;
}

static int siw_misc_open(struct inode *inode, struct file *filp)
{
	struct siw_misc_data *misc_data = __siw_misc_data;
	struct device *dev = NULL;
	int ret = 0;

	if (misc_data == NULL) {
		return -ENOENT;
	}

	mutex_lock(&siw_misc_lock);

	dev = misc_data->dev;

	if (!misc_data->buf) {
		misc_data->buf = kmalloc(SIW_MISC_BUF_SZ, GFP_KERNEL);
		if (!misc_data->buf) {
			t_dev_err(dev, "open ENOMEM\n");
			ret = -ENOMEM;
			goto out;
		}
	}

	misc_data->users++;

out:
	mutex_unlock(&siw_misc_lock);
	return ret;
}

static int siw_misc_release(struct inode *inode, struct file *filp)
{
	struct siw_misc_data *misc_data = __siw_misc_data;
	int ret = 0;

	mutex_lock(&siw_misc_lock);

	misc_data->users--;
	if (!misc_data->users) {
		kfree(misc_data->buf);
		misc_data->buf = NULL;
	}

	mutex_unlock(&siw_misc_lock);

	return ret;
}

static const struct file_operations siw_misc_fops = {
	.owner			= THIS_MODULE,
	.read			= siw_misc_read,
	.write			= siw_misc_write,
//	.unlocked_ioctl	= siw_misc_ioctl,
	.open 			= siw_misc_open,
	.release		= siw_misc_release,
	.llseek			= no_llseek,
};

int siw_touch_misc_init(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_misc_data *misc_data = NULL;
	struct miscdevice *misc = NULL;
	char *name;
	int ret = 0;

	misc_data = kzalloc(sizeof(*misc_data), GFP_KERNEL);
	if (misc_data == NULL) {
		t_dev_err(dev, "can't alloacte misc data\n");
		ret = -ENOMEM;
		goto out;
	}

	misc = &misc_data->misc;

	name = touch_drv_name(ts);
	if (!name) {
		name = SIW_MISC_NAME;
	}
	snprintf(misc_data->name, SIW_MISC_NAME_SZ,
		"%s", name);

	misc->minor	= MISC_DYNAMIC_MINOR;
	misc->name = misc_data->name;
	misc->fops = &siw_misc_fops;

	/* register misc device */
	ret = misc_register(misc);
	if (ret < 0) {
		t_dev_err(dev, "siw misc_register failed\n");
		goto out_register;
	}

	misc_data->dev = dev;

//	spin_lock_init(&misc_data->bus_lock);

	__siw_misc_data = misc_data;

	t_dev_info(dev, "siw misc register done (%d)\n", misc->minor);

	return 0;

out_register:
	kfree(misc_data);

out:

	return ret;
}

void siw_touch_misc_free(struct device *dev)
{
	struct siw_misc_data *misc_data = __siw_misc_data;

	if (misc_data) {
		misc_deregister(&misc_data->misc);

		kfree(misc_data);
	}

	__siw_misc_data = NULL;
}

#endif	/* __SIW_SUPPORT_MISC */

