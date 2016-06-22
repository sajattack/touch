/*
 * siw_touch_hal_sysfs.c - SiW touch hal driver
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


#define siw_hal_sysfs_err_invalid_param(_dev)	\
		t_dev_err(_dev, "Invalid param\n");


static const char *siw_hal_debug_type_str[] = {
	"Disable Type",
	"Buffer Type",
	"Always Report Type"
};

#define _reg_snprintf(_buf, _size, _reg, _element)	\
		siw_snprintf(_buf, _size, "# 0x%04X [%s]\n", _reg->_element, #_element)

static int __show_reg_list(struct device *dev, char *buf, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;

	size += siw_snprintf(buf, size, "# Reg. Map List\n");

	size += _reg_snprintf(buf, size, reg, spr_chip_id);
	size += _reg_snprintf(buf, size, reg, spr_rst_ctl);
	size += _reg_snprintf(buf, size, reg, spr_boot_ctl);
	size += _reg_snprintf(buf, size, reg, spr_sram_ctl);
	size += _reg_snprintf(buf, size, reg, spr_boot_status);
	size += _reg_snprintf(buf, size, reg, spr_subdisp_status);
	size += _reg_snprintf(buf, size, reg, spr_code_offset);
	size += _reg_snprintf(buf, size, reg, tc_ic_status);
	size += _reg_snprintf(buf, size, reg, tc_status);
	size += _reg_snprintf(buf, size, reg, tc_version);
	size += _reg_snprintf(buf, size, reg, tc_product_id1);
	size += _reg_snprintf(buf, size, reg, tc_product_id2);
	size += _reg_snprintf(buf, size, reg, info_chip_version);
	size += _reg_snprintf(buf, size, reg, info_lot_num);
	size += _reg_snprintf(buf, size, reg, info_serial_num);
	size += _reg_snprintf(buf, size, reg, info_date);
	size += _reg_snprintf(buf, size, reg, info_time);
	size += _reg_snprintf(buf, size, reg, cmd_abt_loc_x_start_read);
	size += _reg_snprintf(buf, size, reg, cmd_abt_loc_x_end_read);
	size += _reg_snprintf(buf, size, reg, cmd_abt_loc_y_start_read);
	size += _reg_snprintf(buf, size, reg, cmd_abt_loc_y_end_read);
	size += _reg_snprintf(buf, size, reg, code_access_addr);
	size += _reg_snprintf(buf, size, reg, data_i2cbase_addr);
	size += _reg_snprintf(buf, size, reg, prd_tcm_base_addr);
	size += _reg_snprintf(buf, size, reg, tc_device_ctl);
	size += _reg_snprintf(buf, size, reg, tc_interrupt_ctl);
	size += _reg_snprintf(buf, size, reg, tc_interrupt_status);
	size += _reg_snprintf(buf, size, reg, tc_drive_ctl);
	size += _reg_snprintf(buf, size, reg, tci_fail_debug_r);
	size += _reg_snprintf(buf, size, reg, tic_fail_bit_r);
	size += _reg_snprintf(buf, size, reg, tci_debug_r);
	size += _reg_snprintf(buf, size, reg, tci_enable_w);
	size += _reg_snprintf(buf, size, reg, tci_fail_debug_w);
	size += _reg_snprintf(buf, size, reg, tci_fail_bit_w);
	size += _reg_snprintf(buf, size, reg, tap_count_w);
	size += _reg_snprintf(buf, size, reg, min_intertap_w);
	size += _reg_snprintf(buf, size, reg, max_intertap_w);
	size += _reg_snprintf(buf, size, reg, touch_slop_w);
	size += _reg_snprintf(buf, size, reg, tap_distance_w);
	size += _reg_snprintf(buf, size, reg, int_delay_w);
	size += _reg_snprintf(buf, size, reg, act_area_x1_w);
	size += _reg_snprintf(buf, size, reg, act_area_y1_w);
	size += _reg_snprintf(buf, size, reg, act_area_x2_w);
	size += _reg_snprintf(buf, size, reg, act_area_y2_w);
	size += _reg_snprintf(buf, size, reg, swipe_enable_w);
	size += _reg_snprintf(buf, size, reg, swipe_dist_w);
	size += _reg_snprintf(buf, size, reg, swipe_ratio_thr_w);
	size += _reg_snprintf(buf, size, reg, swipe_ratio_period_w);
	size += _reg_snprintf(buf, size, reg, swipe_ratio_dist_w);
	size += _reg_snprintf(buf, size, reg, swipe_time_min_w);
	size += _reg_snprintf(buf, size, reg, swipe_time_max_w);
	size += _reg_snprintf(buf, size, reg, swipe_act_area_x1_w);
	size += _reg_snprintf(buf, size, reg, swipe_act_area_y1_w);
	size += _reg_snprintf(buf, size, reg, swipe_act_area_x2_w);
	size += _reg_snprintf(buf, size, reg, swipe_act_area_y2_w);
	size += _reg_snprintf(buf, size, reg, swipe_fail_debug_w);
	size += _reg_snprintf(buf, size, reg, swipe_fail_debug_r);
	size += _reg_snprintf(buf, size, reg, swipe_debug_r);
	size += _reg_snprintf(buf, size, reg, cmd_raw_data_report_mode_read);
	size += _reg_snprintf(buf, size, reg, cmd_raw_data_compress_write);
	size += _reg_snprintf(buf, size, reg, cmd_raw_data_report_mode_write);
	size += _reg_snprintf(buf, size, reg, spr_charger_status);
	size += _reg_snprintf(buf, size, reg, ime_state);
	size += _reg_snprintf(buf, size, reg, max_delta);
	size += _reg_snprintf(buf, size, reg, touch_max_w);
	size += _reg_snprintf(buf, size, reg, touch_max_r);
	size += _reg_snprintf(buf, size, reg, call_state);
	size += _reg_snprintf(buf, size, reg, tc_tsp_test_ctl);
	size += _reg_snprintf(buf, size, reg, tc_tsp_test_status);
	size += _reg_snprintf(buf, size, reg, tc_tsp_test_pf_result);
	size += _reg_snprintf(buf, size, reg, tc_tsp_test_off_info);
	size += _reg_snprintf(buf, size, reg, tc_flash_dn_status);
	size += _reg_snprintf(buf, size, reg, tc_confdn_base_addr);
	size += _reg_snprintf(buf, size, reg, tc_flash_dn_ctl);
	size += _reg_snprintf(buf, size, reg, raw_data_ctl_read);
	size += _reg_snprintf(buf, size, reg, raw_data_ctl_write);
	size += _reg_snprintf(buf, size, reg, serial_data_offset);
	/* __SIW_SUPPORT_WATCH */
	if (!touch_test_quirks(ts, CHIP_QUIRK_NOT_SUPPORT_WATCH)) {
		size += _reg_snprintf(buf, size, reg, ext_watch_font_offset);
		size += _reg_snprintf(buf, size, reg, ext_watch_font_addr);
		size += _reg_snprintf(buf, size, reg, ext_watch_font_dn_addr_info);
		size += _reg_snprintf(buf, size, reg, ext_watch_font_crc);
		size += _reg_snprintf(buf, size, reg, ext_watch_dcs_ctrl);
		size += _reg_snprintf(buf, size, reg, ext_watch_mem_ctrl);
		size += _reg_snprintf(buf, size, reg, ext_watch_ctrl);
		size += _reg_snprintf(buf, size, reg, ext_watch_area_x);
		size += _reg_snprintf(buf, size, reg, ext_watch_area_y);
		size += _reg_snprintf(buf, size, reg, ext_watch_blink_area);

		switch (touch_chip_type(ts)) {
		case CHIP_LG4895:
			/* SKIP : reg->ext_watch_lut not available */
			break;
		default:
			size += _reg_snprintf(buf, size, reg, ext_watch_lut);
			break;
		}

		size += _reg_snprintf(buf, size, reg, ext_watch_display_on);
		size += _reg_snprintf(buf, size, reg, ext_watch_display_status);
		size += _reg_snprintf(buf, size, reg, ext_watch_rtc_sct);
		size += _reg_snprintf(buf, size, reg, ext_watch_rtc_sctcnt);
		size += _reg_snprintf(buf, size, reg, ext_watch_rtc_capture);
		size += _reg_snprintf(buf, size, reg, ext_watch_rtc_ctst);
		size += _reg_snprintf(buf, size, reg, ext_watch_rtc_ecnt);
		size += _reg_snprintf(buf, size, reg, ext_watch_hour_disp);
		size += _reg_snprintf(buf, size, reg, ext_watch_blink_prd);
		size += _reg_snprintf(buf, size, reg, ext_watch_rtc_run);
		size += _reg_snprintf(buf, size, reg, ext_watch_position);
		size += _reg_snprintf(buf, size, reg, ext_watch_position_r);
		size += _reg_snprintf(buf, size, reg, ext_watch_state);
		size += _reg_snprintf(buf, size, reg, sys_dispmode_status);
	}
	/* __SIW_SUPPORT_PRD */
	size += _reg_snprintf(buf, size, reg, prd_serial_tcm_offset);
	size += _reg_snprintf(buf, size, reg, prd_tc_mem_sel);
	size += _reg_snprintf(buf, size, reg, prd_tc_test_mode_ctl);
	size += _reg_snprintf(buf, size, reg, prd_m1_m2_raw_offset);
	size += _reg_snprintf(buf, size, reg, prd_tune_result_offset);
	size += _reg_snprintf(buf, size, reg, prd_open3_short_offset);
	size += _reg_snprintf(buf, size, reg, prd_ic_ait_start_reg);
	size += _reg_snprintf(buf, size, reg, prd_ic_ait_data_readystatus);

	return size;
}

static ssize_t _show_reg_ctrl(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 product[2] = {0};
	u32 version = 0;
	u32 revision = 0;
	u32 bootmode = 0;
	int size = 0;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->tc_version,
				&version);
	if (ret < 0) {
		return (ssize_t)ret;
	}

	ret = siw_hal_read_value(dev,
				reg->info_chip_version,
				&revision);
	if (ret < 0) {
		return (ssize_t)ret;
	}

	ret = siw_hal_reg_read(dev,
				reg->tc_product_id1,
				(void *)product, sizeof(product));
	if (ret < 0) {
		return (ssize_t)ret;
	}

	ret = siw_hal_read_value(dev,
				reg->spr_boot_status,
				&bootmode);
	if (ret < 0) {
		return (ssize_t)ret;
	}

	size += siw_snprintf(buf, size,
				"[Version] 	: 0x%08X\n",
				version);

	size += siw_snprintf(buf, size,
				"[Revision]	: 0x%08X\n",
				revision);

	size += siw_snprintf(buf, size,
				"[Product ID1] : 0x%08X, 0x%08X\n",
				product[0], product[1]);

	size += siw_snprintf(buf, size,
				"[Boot mode]	: 0x%08X\n",
				bootmode);

	size += siw_snprintf(buf, size,
				">> version	: v%u.%02u, chip : %u, protocol : %u\n",
				(u32)(chip->fw.version[0]),
				(u32)(chip->fw.version[1]),
				(u32)((version >> 16) & 0xFF),
				(u32)((version >> 24) & 0xFF));

	size += siw_snprintf(buf, size,
					">> revision	: %d\n", chip->fw.revision);

	size += siw_snprintf(buf, size,
				">> product id : %s\n",
				chip->fw.product_id);

	size += siw_snprintf(buf, size,
				">> flash boot : %s, %s, crc : %s\n",
				(bootmode >> 1 & 0x1) ? "BUSY" : "idle",
				(bootmode >> 2 & 0x1) ? "done" : "booting",
				(bootmode >> 3 & 0x1) ? "ERROR" : "ok");

	size += __show_reg_list(dev, buf, size);

	size += siw_snprintf(buf, size, "\n");

	return (ssize_t)size;
}

static ssize_t _store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
//	struct siw_ts *ts = to_touch_core(dev);
	char command[6] = {0};
	u32 reg = 0;
	u32 data = 1;
	u16 reg_addr;
	int value = 0;
	int ret = 0;

	if (sscanf(buf, "%5s %X %X", command, &reg, &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	reg_addr = reg;
	if (!strcmp(command, "write")) {
		data = value;
		ret = siw_hal_write_value(dev,
					reg_addr,
					data);
		if (ret >= 0) {
			t_dev_info(dev, "reg[0x%04X] = 0x%08X\n", reg_addr, data);
		}
		goto out;
	}

	if (!strcmp(command, "read")) {
		ret = siw_hal_read_value(dev,
					reg_addr,
					&data);
		if (ret >= 0) {
			t_dev_info(dev, "reg[0x%04X] = 0x%08X\n", reg_addr, data);
		}
		goto out;
	}

	t_dev_dbg_base(dev, "[Usage]\n");
	t_dev_dbg_base(dev,
		" echo write 0x1234 {value} > /sys/devices/virtual/input/siw_touch_input/reg_ctrl\n");
	t_dev_dbg_base(dev,
		" echo read 0x1234 > /sys/devices/virtual/input/siw_touch_input/reg_ctrl\n");

out:
	return count;
}

static ssize_t _show_tci_debug(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 rdata = -1;
	int size = 0;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->tci_fail_debug_r,
				&rdata);
	if (ret < 0) {
		t_dev_err(dev, "Fail to Read TCI Debug Reason type\n");
		return (ssize_t)ret;
	}

	size += siw_snprintf(buf, size,
				"Read TCI Debug Reason type[IC] = %s\n",
				siw_hal_debug_type_str[(rdata & 0x8) ? 2 :
				(rdata & 0x4 ? 1 : 0)]);

	size += siw_snprintf(buf, size,
				"Read TCI Debug Reason type[Driver] = %s\n",
				siw_hal_debug_type_str[chip->tci_debug_type]);

	return (ssize_t)size;
}

static ssize_t _store_tci_debug(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	if ((value > 2) || (value < 0)) {
		t_dev_info(dev, "SET TCI debug reason wrong, 0, 1, 2 only\n");
		return count;
	}

	chip->tci_debug_type = (u8)value;
	t_dev_info(dev, "SET TCI Debug reason type = %s\n",
				siw_hal_debug_type_str[value]);

	return count;
}

static ssize_t _show_swipe_debug(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 rdata = -1;
	int size = 0;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->swipe_fail_debug_r,
				&rdata);
	if (ret < 0) {
		t_dev_info(dev, "Fail to Read SWIPE Debug reason type\n");
		return (ssize_t)ret;
	}

	size += siw_snprintf(buf, size,
				"Read SWIPE Debug reason type[IC] = %s\n",
				siw_hal_debug_type_str[rdata]);
	size += siw_snprintf(buf, size,
				"Read SWIPE Debug reason type[Driver] = %s\n",
				siw_hal_debug_type_str[chip->swipe_debug_type]);

	return (ssize_t)size;
}

static ssize_t _store_swipe_debug(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	if (value > 2 || value < 0) {
		t_dev_err(dev, "SET SWIPE debug reason wrong, 0, 1, 2 only\n");
		return count;
	}

	chip->swipe_debug_type = (u8)value;
	t_dev_info(dev, "Write SWIPE Debug reason type = %s\n",
			siw_hal_debug_type_str[value]);

	return count;
}

static ssize_t _show_reset_ctrl(struct device *dev, char *buf)
{
	int size = 0;

	size += siw_snprintf(buf, size, "%s\n", "Reset Control Usage");
	size += siw_snprintf(buf, size,
				" SW Reset        : echo %d > hal_reset_ctrl\n",
				SW_RESET);
	size += siw_snprintf(buf, size,
				" HW Reset(Async) : echo %d > hal_reset_ctrl\n",
				HW_RESET_ASYNC);
	size += siw_snprintf(buf, size,
				" HW Reset(Sync)  : echo %d > hal_reset_ctrl\n",
				HW_RESET_SYNC);

	return size;
}

static ssize_t _store_reset_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

//	mutex_lock(&ts->lock);
	siw_ops_reset(ts, value);
//	mutex_lock(&ts->lock);

//	siw_hal_init(dev);

	return count;
}

static ssize_t _show_lcd_mode(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int size = 0;

	size += siw_snprintf(buf, size, "current driving mode is %s\n",
				siw_lcd_driving_mode_str(chip->lcd_mode));

	return size;
}

static ssize_t _store_lcd_mode(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	t_dev_info(dev, "try to change driving mode: %s -> %s\n",
			siw_lcd_driving_mode_str(chip->lcd_mode),
			siw_lcd_driving_mode_str(value));
	siw_ops_notify(ts, LCD_EVENT_LCD_MODE, &value);

	return count;
}


#define SIW_TOUCH_HAL_ATTR(_name, _show, _store)	\
		TOUCH_ATTR(_name, _show, _store)

#define _SIW_TOUCH_HAL_ATTR_T(_name)	\
		touch_attr_##_name

static SIW_TOUCH_HAL_ATTR(reg_ctrl, _show_reg_ctrl, _store_reg_ctrl);
static SIW_TOUCH_HAL_ATTR(tci_debug, _show_tci_debug, _store_tci_debug);
static SIW_TOUCH_HAL_ATTR(swipe_debug, _show_swipe_debug, _store_swipe_debug);
static SIW_TOUCH_HAL_ATTR(reset_ctrl, _show_reset_ctrl, _store_reset_ctrl);
static SIW_TOUCH_HAL_ATTR(lcd_mode, _show_lcd_mode, _store_lcd_mode);

static struct attribute *siw_hal_attribute_list[] = {
	&_SIW_TOUCH_HAL_ATTR_T(reg_ctrl).attr,
	&_SIW_TOUCH_HAL_ATTR_T(tci_debug).attr,
	&_SIW_TOUCH_HAL_ATTR_T(swipe_debug).attr,
	&_SIW_TOUCH_HAL_ATTR_T(reset_ctrl).attr,
	&_SIW_TOUCH_HAL_ATTR_T(lcd_mode).attr,
	NULL,
};

static const struct attribute_group siw_hal_attribute_group = {
	.attrs = siw_hal_attribute_list,
};

static int __siw_hal_sysfs_add_abt(struct device *dev, int on_off)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	ret = siw_ops_abt_sysfs(ts, on_off);
	if ((on_off == DRIVER_INIT) && (ret < 0)) {
		t_dev_err(dev, "%s abt sysfs register failed\n",
			touch_chip_name(ts));
	}

	return ret;
}

static int __siw_hal_sysfs_add_prd(struct device *dev, int on_off)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	ret = siw_ops_prd_sysfs(ts, on_off);
	if ((on_off == DRIVER_INIT) && (ret < 0)) {
		t_dev_err(dev, "%s prd sysfs register failed\n",
			touch_chip_name(ts));
	}

	return ret;
}

static int __siw_hal_sysfs_add_watch(struct device *dev, int on_off)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	ret = siw_ops_watch_sysfs(ts, on_off);
	if ((on_off == DRIVER_INIT) && (ret < 0)) {
		t_dev_err(dev, "%s watch sysfs register failed\n",
			touch_chip_name(ts));
	}

	return ret;
}


static int siw_hal_sysfs_add(struct device *dev, int on_off)
{
	int ret = 0;

	if (on_off == DRIVER_INIT) {
		ret = __siw_hal_sysfs_add_abt(dev, DRIVER_INIT);
		if (ret < 0) {
			goto out;
		}

		ret = __siw_hal_sysfs_add_prd(dev, DRIVER_INIT);
		if (ret < 0) {
			goto out_prd;
		}

		ret = __siw_hal_sysfs_add_watch(dev, DRIVER_INIT);
		if (ret < 0) {
			goto out_watch;
		}
		return 0;
	}

	__siw_hal_sysfs_add_watch(dev, DRIVER_FREE);

out_watch:
	__siw_hal_sysfs_add_prd(dev, DRIVER_FREE);

out_prd:
	__siw_hal_sysfs_add_abt(dev, DRIVER_FREE);

out:
	return ret;
}

static int siw_hal_create_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct kobject *kobj = &ts->kobj;
	int ret = 0;

	ret = sysfs_create_group(kobj, &siw_hal_attribute_group);
	if (ret < 0) {
		t_dev_err(dev, "%s sysfs register failed\n",
				touch_chip_name(ts));
		goto out;
	}

	ret = siw_hal_sysfs_add(dev, DRIVER_INIT);
	if (ret < 0) {
		goto out_add;
	}

	t_dev_dbg_base(dev, "%s sysfs registered\n",
			touch_chip_name(ts));

	return 0;

out_add:
	sysfs_remove_group(kobj, &siw_hal_attribute_group);

out:

	return ret;
}

static void siw_hal_remove_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_sysfs_add(dev, DRIVER_FREE);

	sysfs_remove_group(&ts->kobj, &siw_hal_attribute_group);

	t_dev_dbg_base(dev, "%s sysfs unregistered\n",
			touch_chip_name(ts));
}

int siw_hal_sysfs(struct device *dev, int on_off)
{
	if (on_off == DRIVER_INIT) {
		return siw_hal_create_sysfs(dev);
	}

	siw_hal_remove_sysfs(dev);
	return 0;
}


