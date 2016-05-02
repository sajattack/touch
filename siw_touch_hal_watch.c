/*
 * siw_touch_hal_watch.c - SiW touch hal driver for ABT
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/syscalls.h>
#include <linux/file.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"
#include "siw_touch_event.h"
#include "siw_touch_gpio.h"
#include "siw_touch_irq.h"
#include "siw_touch_sys.h"

#if defined(__SIW_SUPPORT_WATCH)	//See siw_touch_cfg.h

#define SIW_EXT_WATCH_NAME			"siw_ext_watch"
#define SIW_MAX_FONT_SIZE			(6<<10)	//6KB used
#define SIW_FONT_MAGIC_CODE_SIZE	(4)

enum {
	EXT_WATCH_LUT_MAX	= 7,
	EXT_WATCH_LUT_NUM	= 7,
};

enum {
	NOT_SUPPORT = 0,
	SUPPORT,
};

enum {
	UNBLOCKED = 0,
	BLOCKED,
};

enum {
	EXT_WATCH_OFF = 0,
	EXT_WATCH_ON,
};

enum {
	EXT_WATCH_RTC_START = 1,
	EXT_WATCH_RTC_STOP = 2,
};

enum {
	EXT_WATCH_SET_RTC_COUNT	= 305,
	EXT_WATCH_SET_RTC_ECNT	= 32764,
};

enum {
	FONT_MEM_CRC = 0x80,
};

enum {
	FONT_EMPTY = 0,
	FONT_DOWNLOADING,
	FONT_READY,
};

enum {
	RTC_CLEAR = 0,
	RTC_RUN,
};

__packed struct ext_watch_query_font_data {
	bool Font_supported;	/* 0:not supported, 1:supported */
	u8 max_font_x_size;		/* 1~9 number X max size. */
	u8 max_font_y_size;		/* 1~9 number Y max size. */
	u8 max_cln_x_size;		/* ":" X max size. (ex. 23:47) */
	u8 max_cln_y_size;		/* ":" Y max size. (ex. 23:47) */
};

struct ext_watch_query_font_position {	/* 0:not supported, 1:supported */
	bool vertical_position_supported;
	bool horizontal_position_supported;
};

struct ext_watch_query_font_time {	/* 0:not supported, 1:supported */
	bool h24_supported;
	bool ampm_supported;
};

__packed struct ext_watch_query_font_color {	/* 0:not supported, 1:supported */
	u8 max_num;		/* The number of LUT */
	bool lut_supported;
	bool alpha_supported;
	bool gradation_supported;
};

__packed struct ext_watch_query_font_effect {
	bool zero_supported;	/* 0:display off, 1:display on */
	u8 blink_type;			/* 0:blink disable, 1:500ms, 2:1sec, 3:2sec. */
};

struct ext_watch_config_font_effect_blink {
	u32	blink_type; /* 0:blink disable, 1:500ms, 2:1sec, 3:2sec. */
	u32	bstartx; /* blink startx.watstartx<=bstartx, bstartx<=bendx */
	u32	bendx; /* blink end position. bendx <= watendx */
};

struct ext_watch_config_font_effect {
	u32	len;
	u32	watchon;				/* 0:watch off, 1:watch on */
	u32	h24_en;					/* 0:12 hour display, 1:24 hour display */
	u32	zero_disp;				/* 0:display off, 1:display on */
	u32	clock_disp_type;		/* 0:hour and min, 1:min and sec */
	u32	midnight_hour_zero_en;	/* 0: 12:00 mode, 1: 00:00 */
	struct ext_watch_config_font_effect_blink blink;	/*for blink effect */
};

struct ext_watch_config_font_lut {
	/* LUT */
	u32	rgb_blue;
	u32	rgb_green;
	u32	rgb_red;
};

struct ext_watch_config_font_property {
	u32	len;
	u32	max_num;		/* The number of LUT */
	struct ext_watch_config_font_lut lut[EXT_WATCH_LUT_MAX];
};

struct ext_watch_config_font_position {
	u32 len;
	u32	watstartx;	/* 520 <= watstartx, watstartx <= watendx */
	u32	watendx;	/* watch end positon. watendx <= 720 */
	u32	watstarty;
	u32	watendy;	/* watch end positon. watendy <= 80 */
	u32	h1x_pos;	/* 1 ~ 9hour position */
	u32	h10x_pos;	/* 10, 20 hour position */
	u32	m1x_pos;	/* 1 ~ 9min position */
	u32	m10x_pos;	/* 10 ~ 50 min position */
	u32	clx_pos;	/* 1 ~ 60 second position */
};

struct ext_watch_config_time_sync {	/* to sync with AP's current time */
	u32	len;
	u32	rtc_cwhour;	/* for hour */
	u32	rtc_cwmin;	/* for min */
	u32	rtc_cwsec;	/* for sec */
	u32	rtc_cwmilli;	/* for millisecond */
};
struct ext_watch_config_font_data {
	u8	*data;		/* Font Data (53,120 bytes) */
};

__packed struct ext_watch_bits_ctrl {	/* 0x2D2 */
	u32	reserved0:1;
	u32	dispmode:1;	/* 0:U2, 1:AnyMode */
	u32	reserved1:5;
	u32	alpha:9;
};

__packed struct ext_watch_bits_area { 	/* 0x2D3, 0x2D4 */
	u32	watstart:12;
	u32	watend:12;
};

__packed struct ext_watch_bits_blink_area { /* 0x2D5 */
	u32 bstartx:12;
	u32 bendx:12;
};

__packed struct ext_watch_bits_lut {	/* 0x2D6 */
	u8 b;
	u8 g;
	u8 r;
	u8 rssvd0;
};

__packed struct ext_watch_bits_time {
	u32 hour:5;
	u32 min:6;
	u32 sec:6;
	u32 rsvd0:15;
};

__packed struct ext_watch_cfg_mode {	/* 36 bytes */
	struct ext_watch_bits_ctrl watch_ctrl;				/* 2 bytes */
	struct ext_watch_bits_area watch_area_x;			/* 3 bytes */
	struct ext_watch_bits_area watch_area_y;			/* 3 bytes */
	struct ext_watch_bits_blink_area blink_area;		/* 3 bytes */
	struct ext_watch_bits_lut lut[EXT_WATCH_LUT_NUM];	/* 21 bytes */
};

__packed struct ext_watch_cfg_time {	/* 36 bytes */
	u32 disp_waton;						/* 0xC10 watch display off:0, on:1*/
	struct ext_watch_bits_time rtc_sct;	/* 0x081 */
	u32 rtc_sctcnt;						/* 0x082 */
	u32 rtc_mit;						/* 0x083 */
	u32 rtc_capture;					/* 0x084 */
	struct ext_watch_bits_time rtc_ctst;	/* 0x087 */
	u32 rtc_ecnt;						/* 0x088 */
};

__packed struct ext_watch_cfg_position {	/* 0xC11 */
	u32 h10x_pos:9;
	u32 h1x_pos:9;
	u32 reserved0:14;
	u32 m10x_pos:9;
	u32 m1x_pos:9;
	u32 reserved1:14;
	u32 clx_pos:9;
	u32 reserved2:23;
	u32 zero_disp:1;
	u32 h24_en:1;
	u32 clock_disp_mode:1;
	u32 midnight_hour_zero_en:1;
	u32 reserved3:28;
	u32 bhprd:3;
	u32 reserved4:29;
	u32 num_width:8;
	u32 colon_width:8;
	u32 height:8;
	u32 font_id:8;
};

__packed struct ext_watch_cfg_status {	/* 0x270*/
	u32 step:3;
	u32 en:1;
	u32 en_24:1;
	u32 zero_en:1;
	u32 disp_mode:1;
	u32 bhprd:3;
	u32 cur_hour:5;
	u32 cur_min:6;
	u32 cur_sec:6;
	u32 midnight_hour_zero_en:1;
	u32 reserved0:4;
};

struct ext_watch_font_header {
	u8 width_num;
	u8 width_colon;
	u8 height;
	u8 font_id;
	u32 size;
};

struct ext_watch_cfg {
	u8 *font_data;
	u32 magic_code;
	u32 font_crc;
	struct ext_watch_cfg_mode mode;
	struct ext_watch_cfg_time time;
	struct ext_watch_cfg_position position;
};

struct watch_state_info {
	atomic_t font_status;
	atomic_t rtc_status;
};

struct watch_data {
	struct watch_state_info state;
	struct bin_attribute fontdata_attr;
	u32 font_written_size;
	struct ext_watch_cfg ext_wdata;
};


#define SIW_WATCH_TAG 	"watch: "

#define t_watch_info(_dev, fmt, args...)	\
		__t_dev_info(_dev, SIW_WATCH_TAG fmt, ##args)

#define t_watch_err(_dev, fmt, args...)	\
		__t_dev_err(_dev, SIW_WATCH_TAG fmt, ##args)

#define t_watch_warn(_dev, fmt, args...)	\
		__t_dev_warn(_dev, SIW_WATCH_TAG fmt, ##args)

#define t_watch_dbg(_dev, fmt, args...)	\
		t_dev_dbg_watch(_dev, SIW_WATCH_TAG fmt, ##args)

#define siw_watch_snprintf(_buf, _max, _size, _fmt, _args...) \
		({	\
			int _n_size = 0;	\
			if (_size < _max)	\
				_n_size = snprintf(_buf + _size, _max - _size,\
								(const char *)_fmt, ##_args);	\
			_n_size;	\
		})

#define TS_MODULE "[watch]"

enum {
	FONT_TEMP_LOG_SZ = 256,
};

enum {
	WATCH_MAX_RW_SIZE = (60<<10),
};


// LOOK UP TABLE for CRC16 generation
// Polynomial X^16+X^15+X^2+1
static const u16 ext_watch_crc16lut[] = {
	0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
	0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
	0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
	0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
	0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
	0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
	0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
	0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
	0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
	0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
	0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
	0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
	0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
	0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
	0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
	0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
	0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
	0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
	0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
	0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
	0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
	0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
	0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
	0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
	0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
	0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
	0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

#define __ret_val_blocked(_val)		(-EPERM)
//#define	__ret_val_blocked(_val)		(_val)

static int ext_quirk_check(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	unsigned long test_bit;

	test_bit = touch_test_quirks(ts, CHIP_QUIRK_NOT_SUPPORT_WATCH);
	if (test_bit)
		t_dev_dbg_base(dev, "%s watch control not supported\n",
			touch_chip_name(ts));

	return !!test_bit;
}

static int ext_watch_rtc_start(struct device *dev, u8 start)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 rtc_ctrl;
	int ret = 0;

	rtc_ctrl = (start == EXT_WATCH_RTC_START)?
				EXT_WATCH_RTC_START :
				EXT_WATCH_RTC_STOP;

	ret = siw_hal_write_value(dev,
				reg->ext_watch_rtc_run,
				rtc_ctrl);
	if (ret < 0) {
		t_watch_err(dev, "failed to run rtc, %d\n", ret);
		goto out;
	}

	t_watch_info(dev, "rtc %s\n",
		(rtc_ctrl == EXT_WATCH_RTC_START)? "on" : "off");

out:
	return ret;
}

static int ext_watch_mem_ctrl(struct device *dev, int on_off, const char *msg)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
//	u32 value = !!on_off;
	int ret;

	ret = siw_hal_write_value(dev,
				reg->ext_watch_mem_ctrl,
		 		!!on_off);
	if (ret < 0) {
		return ret;
	}
	t_watch_info(dev, "%s, watch memory clk %s\n",
		msg, (on_off)? "on" : "off");
	return 0;
}

static int ext_watch_get_position(struct device *dev, char *buf, int *len)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct ext_watch_cfg_position position = {0, };
	struct ext_watch_cfg_status status = {0, };
	char log[FONT_TEMP_LOG_SZ] = {0, 0};
	int loglen = 0;
	int buflen = 0;
	int ret = 0;

#if 0
	if (!buf) {
		t_watch_err(dev, "get position: NULL buffer");
		ret = -EINVAL;
		goto out;
	}
#endif

	if (len)
		buflen = *len;

	{
		struct touch_xfer_msg *xfer = ts->xfer;

		siw_hal_bus_xfer_init(dev, xfer);

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_position_r,
				(u8 *)&position, sizeof(u32)*3);

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_state,
				(u8 *)&status, sizeof(u32));
	}
	ret = siw_hal_xfer_msg(dev, ts->xfer);
	if (ret < 0) {
		goto out;
	}

	loglen = siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, 0,
					"position %d %d %d %d %d ",
					position.h10x_pos, position.h1x_pos,
		 			position.clx_pos, position.m10x_pos,
		 			position.m1x_pos);

	if (buf) {
		memcpy(&buf[buflen], log, loglen);
		buflen += loglen;
	} else {
		t_watch_warn(dev, "get position: NULL buffer");
	}
	t_watch_info(dev, "%s", log);

	position.zero_disp = status.zero_en;
	position.h24_en = status.en_24;
	position.clock_disp_mode = status.disp_mode;
	position.midnight_hour_zero_en = status.midnight_hour_zero_en;
	position.bhprd = status.bhprd;

	loglen = siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, 0,
					"zero display %d, 24H mode %d, clock mode %d\n",
					position.zero_disp, position.h24_en,
					position.clock_disp_mode);

	if (buf) {
		memcpy(&buf[buflen], log, loglen);
		buflen += loglen;
	}
	t_watch_info(dev, "%s", log);

	loglen = siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, 0,
					"midnight mode %d, blink period %d\n",
					position.midnight_hour_zero_en,
					position.bhprd);
	if (buf) {
		memcpy(&buf[buflen], log, loglen);
		buflen += loglen;
	}
	t_watch_info(dev, "%s", log);

	loglen = siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, 0,
					"step %d, enable %d\n",
		 			status.step, status.en);
	if (buf) {
		memcpy(&buf[buflen], log, loglen);
		buflen += loglen;
	}
	t_watch_info(dev, "%s", log);

	if (len)
		*len = buflen;

	return 0;

out:
	t_watch_err(dev, "get position: failed, %d\n", ret);
	return ret;
}

static int ext_watch_get_curr_time(struct device *dev, char *buf, int *len)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_bits_time *rtc_ctst = &watch->ext_wdata.time.rtc_ctst;
	struct ext_watch_cfg_status status = {0, 0};
	char log[FONT_TEMP_LOG_SZ] = {0, 0};
	int loglen = 0;
	int buflen = 0;
	int ret = 0;

#if 0
	if (!buf) {
		t_watch_err(dev, "get curr time: NULL buffer");
		ret = -EINVAL;
		goto out;
	}
#endif

	if (len)
		buflen = *len;

	{
		struct touch_xfer_msg *xfer = ts->xfer;

		siw_hal_bus_xfer_init(dev, xfer);

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_state,
				(u8 *)&status, sizeof(u32));

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_rtc_ctst,
				(u8 *)rtc_ctst, sizeof(u32));
	}
	ret = siw_hal_xfer_msg(dev, ts->xfer);
	if (ret < 0) {
		goto out;
	}

	loglen = siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, 0,
					"display %02d:%02d:%02d, rtc %02d:%02d:%02d\n",
					status.cur_hour, status.cur_min, status.cur_sec,
	 				rtc_ctst->hour, rtc_ctst->min, rtc_ctst->sec);

	if (buf) {
		memcpy(&buf[buflen], log, loglen);
		buflen += loglen;
	} else {
		t_watch_warn(dev, "get curr time: NULL buffer");
	}
	t_watch_info(dev, "%s", log);

	if (len)
		*len = buflen;

	return 0;

out:
	t_watch_err(dev, "get curr time: failed, %d\n", ret);
	return ret;
}

#if 0
static int ext_watch_set_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg_mode *mode = &watch->ext_wdata.mode;
	int ret;

	{
		struct touch_xfer_msg *xfer = ts->xfer;

		siw_hal_bus_xfer_init(dev, xfer);

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_ctrl,
				(u8 *)&mode->watch_ctrl, sizeof(u32));

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_area_x,
				(u8 *)&mode->watch_area_x, sizeof(u32));

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_area_y,
				(u8 *)&mode->watch_area_y, sizeof(u32));

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_blink_area,
				(u8 *)&mode->blink_area, sizeof(u32));

		siw_hal_bus_xfer_add_rx(xfer,
				reg->ext_watch_lut,
				(u8 *)&mode->lut, sizeof(u32) * EXT_WATCH_LUT_NUM);
	}
	ret = siw_hal_xfer_msg(dev, ts->xfer);
	if (ret < 0) {
		goto out;
	}

	return 0;

out:
	t_watch_err(dev, "set mode: failed, %d\n", ret);
	return ret;
}
#endif

static int ext_watch_set_curr_time(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg_time *time = &watch->ext_wdata.time;
//	struct ext_watch_cfg_status status = {0, 0};
	u32 rtc_ctrl = 0;
	u16 rtc_count = EXT_WATCH_SET_RTC_COUNT;		 /* 30.5 us */
	int ret = 0;

	time->rtc_ecnt = EXT_WATCH_SET_RTC_ECNT;
	time->rtc_sctcnt = (int)((time->rtc_sctcnt * rtc_count) / 10);
	rtc_ctrl = time->rtc_ecnt & 0xFFFF;

	ret = ext_watch_rtc_start(dev, EXT_WATCH_RTC_STOP);
	if (ret < 0) {
		goto out;
	}

	{
		struct touch_xfer_msg *xfer = ts->xfer;

		siw_hal_bus_xfer_init(dev, xfer);

		siw_hal_bus_xfer_add_tx(xfer,
				reg->ext_watch_rtc_sct,
				(u8 *)&time->rtc_sct, sizeof(u32));

		siw_hal_bus_xfer_add_tx(xfer,
				reg->ext_watch_rtc_sctcnt,
				(u8 *)&time->rtc_sctcnt, sizeof(u32));

		siw_hal_bus_xfer_add_tx(xfer,
				reg->ext_watch_rtc_ecnt,
				(u8 *)&rtc_ctrl, sizeof(u32));
	}
	ret = siw_hal_xfer_msg(dev, ts->xfer);
	if (ret < 0) {
		goto out;
	}

	ret = ext_watch_rtc_start(dev, EXT_WATCH_RTC_START);
	if (ret < 0) {
		goto out;
	}

	t_watch_info(dev, "set time : %02d:%02d:%02d, clk[%d Hz]\n",
		 time->rtc_sct.hour,
		 time->rtc_sct.min,
		 time->rtc_sct.sec,
		 time->rtc_ecnt);

	atomic_set(&watch->state.rtc_status, RTC_RUN);

	return 0;

out:
	t_watch_err(dev, "set curr time, failed, %d\n", ret);
	return ret;
}

static int ext_watch_set_position(struct device *dev, char log)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg_position *position = &watch->ext_wdata.position;
//	u8 *ptr = (u8 *)position;
	int ret = 0;

	ret = siw_hal_reg_write(dev,
				reg->ext_watch_position,
				(void *)position, sizeof(u32)*5);
	if (ret < 0) {
		goto out;
	}

	t_watch_info(dev, "set position: done\n");

	return 0;

out:
	t_watch_err(dev, "set position: failed, %d\n", ret);
	return ret;
}

static int __ext_watch_display_onoff(struct device *dev, u32 data, int log)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg_time *time = &watch->ext_wdata.time;
	int ret = 0;

	if (data == ~0) {
		data = time->disp_waton;
	}

	ret = siw_hal_write_value(dev,
				reg->ext_watch_display_on,
				data);
	if (ret < 0) {
		goto out;
	}

	if (log) {
		t_watch_info(dev, "display[%04Xh] : %s\n",
			reg->ext_watch_display_on,
			time->disp_waton? "on" : "off");
	}
	return 0;

out:
	t_watch_err(dev, "failed to control display on/off(%d), %d\n",
	 	time->disp_waton, ret);
	return ret;
}

static int ext_watch_display_onoff(struct device *dev, char log)
{
	return __ext_watch_display_onoff(dev, ~0, log);
}

static void ext_watch_display_off(struct device *dev)
{
	__ext_watch_display_onoff(dev, 0, 0);
}

static u16 ext_watch_cal_crc16(const u16 *data, u32 size, u16 init_val)
{
	const u16 *crc16lut = ext_watch_crc16lut;
	u16 crc_sum = 0;
	u8 temp = 0;
	int i;

	crc_sum = init_val;
	for (i = 0; i < size; i += 2) {
		temp = (data[i]>>8) & 0xFF;
		crc_sum = (crc_sum<<8) ^ crc16lut[((crc_sum>>8)&0xFF)^temp];

		temp = data[i]&0xFF;
		crc_sum = (crc_sum<<8) ^ crc16lut[((crc_sum>>8)&0xFF)^temp];
	}

	return crc_sum;
}

static u32 ext_watch_font_crc_cal(char *data, u32 size)
{
	u32 crc_value = 0;

	/*CRC Calculation*/
	crc_value = ext_watch_cal_crc16((const u16*)&data[0], size>>1, 0);
	crc_value |= (ext_watch_cal_crc16((const u16*)&data[2], size>>1, 0) << 16);

	return crc_value & 0x3FFFFFFF;
}

static int ext_watch_chk_font_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	u32 status = 1;
	int ret = 0;

	if (siw_touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER) {
		return -EBUSY;
	}

	switch (atomic_read(&watch->state.font_status)) {
	case FONT_EMPTY :
		return -EBUSY;

	case FONT_DOWNLOADING :
		mod_delayed_work(ts->wq, &chip->font_download_work, 0);
		return 0;
	}

	ret = siw_hal_read_value(dev,
				reg->tc_status,
				&status);
	if (ret < 0) {
		/* */
	}
	if (!(status & FONT_MEM_CRC)) {
		t_watch_err(dev, "crc fail [tc_status %08Xh]\n", status);
		mod_delayed_work(ts->wq, &chip->font_download_work, 0);
		goto out;
	}

	t_watch_info(dev, "crc ok\n");

out:
	return ret;
}


static void ext_watch_font_download(struct work_struct *font_download_work)
{
	struct siw_touch_chip *chip =
			container_of(to_delayed_work(font_download_work),
				 struct siw_touch_chip, font_download_work);
	struct siw_hal_reg *reg = chip->reg;
	struct siw_ts *ts = chip->ts;
	struct device *dev = chip->dev;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg *ext_wdata = &watch->ext_wdata;
	struct ext_watch_font_header *font_hdr = NULL;
	u8 *font_data;
	int font_size;
	int curr_size;
	u32 offset = 0;
	u32 crc_addr = 0;
	u32 font_crc_check = 0;
	u32 magic_addr = 0;
	u32 font_magic_check = 0;
	int ret = 0;

	if (atomic_read(&watch->state.font_status) == FONT_EMPTY) {
		t_watch_err(dev, "font dn work: data not downloaded\n");
		return;
	}

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		atomic_set(&watch->state.font_status, FONT_DOWNLOADING);

		t_watch_info(dev, "font dn work: IC deep sleep\n");
		return;
	}

	mutex_lock(&ts->lock);

	t_watch_info(dev, "font dn work: begins\n");

	if (chip->lcd_mode == LCD_MODE_U2) {
		ext_wdata->time.disp_waton = 0;
		ret = ext_watch_display_onoff(dev, 1);
		if (ret < 0) {
			return;
		}

		ret = ext_watch_mem_ctrl(dev, 1, "font dn work: U2 mode");
		if (ret < 0) {
			return;
		}
	}

	font_hdr = (struct ext_watch_font_header *)watch->ext_wdata.font_data;

	t_watch_info(dev, "font dn work: id %d, size %d\n",
		 font_hdr->font_id, font_hdr->size);
	t_watch_info(dev, "font dn work: width_num %d, width_colon %d, height %d\n",
		 font_hdr->width_num, font_hdr->width_colon, font_hdr->height);

	magic_addr = (font_hdr->size)>>2;

	memcpy((void *)&ext_wdata->magic_code,
		(void *)&ext_wdata->font_data[font_hdr->size], sizeof(u32));
	t_watch_info(dev, "font dn work: input magic code %08Xh[addr %08Xh]\n",
			ext_wdata->magic_code, magic_addr);

	crc_addr = (font_hdr->size+SIW_FONT_MAGIC_CODE_SIZE)>>2;

	memcpy((void *)&ext_wdata->font_crc,
		(void *)&ext_wdata->font_data[font_hdr->size+SIW_FONT_MAGIC_CODE_SIZE],
		sizeof(u32));
	t_watch_info(dev, "font dn work: input crc %08Xh[addr %08Xh]\n",
			ext_wdata->font_crc, crc_addr);

	ext_wdata->font_crc =
		 ext_watch_font_crc_cal(ext_wdata->font_data, font_hdr->size);
	t_watch_info(dev, "font dn work: result crc %08Xh\n", ext_wdata->font_crc);

	memcpy((void *)&ext_wdata->font_data[font_hdr->size+SIW_FONT_MAGIC_CODE_SIZE],
		 (void *)&ext_wdata->font_crc, sizeof(u32));

	offset = 0;
	font_data = ext_wdata->font_data;
	font_size = watch->font_written_size;
	while (font_size) {
		curr_size = min(font_size, WATCH_MAX_RW_SIZE);

		/* write font */
		ret = siw_hal_write_value(dev,
					reg->ext_watch_font_offset,
			 		offset);
		if (ret < 0) {
			 goto out;
		}

		ret = siw_hal_reg_write(dev,
					reg->ext_watch_font_addr,
			 		(void *)font_data, curr_size);
		if (ret < 0) {
			goto out;
		}

		/* dump font */
		ret = siw_hal_write_value(dev,
					reg->ext_watch_font_offset,
			 		magic_addr);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_read_value(dev,
					reg->ext_watch_font_addr,
			 		&font_magic_check);
		if (ret < 0) {
			goto out;
		}

		t_watch_info(dev, "fotn dn work: font magic return check : %Xh\n",
				font_magic_check);

		ret = siw_hal_write_value(dev,
					reg->ext_watch_font_offset,
			 		crc_addr);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_read_value(dev,
					reg->ext_watch_font_addr,
			 		&font_crc_check);
		if (ret < 0) {
			goto out;
		}

		t_watch_info(dev, "fotn dn work: font crc return check : %Xh\n",
				font_crc_check);


		font_size -= curr_size;
		font_data += curr_size;
		offset += (curr_size>>2);
	}

	ret = siw_hal_write_value(dev,
				reg->ext_watch_font_crc, 1);
	if (ret < 0) {
		goto out;
	}

	atomic_set(&watch->state.font_status, FONT_READY);

	if (chip->lcd_mode == LCD_MODE_U2) {
		ret = ext_watch_mem_ctrl(dev, 0, "font dn work: U2 mode");
		if (ret < 0) {
			goto out;
		}
	}

	t_watch_info(dev, "font dn work: done(%d)\n",
			watch->font_written_size);

	mutex_unlock(&ts->lock);
	return;

out:
	atomic_set(&watch->state.font_status, FONT_EMPTY);

	t_watch_err(dev, "font dn work: failed, %d\n", ret);

	mutex_unlock(&ts->lock);
	return;
}

static int ext_watch_font_dump(struct device *dev, u8 *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	u8 *font_data;
	int font_size;
	int curr_size;
	u32 offset = 0;
	int ret = 0;

	mutex_lock(&ts->lock);

	t_watch_info(dev, "font dump: begins\n");

	if (chip->lcd_mode == LCD_MODE_U2) {
		ret = ext_watch_mem_ctrl(dev, 1, "font dump: U2 mode");
		if (ret < 0) {
			goto out;
		}
	}

	font_data = buf;
	font_size = watch->font_written_size;
	while (font_size) {
		curr_size = min(font_size, WATCH_MAX_RW_SIZE);

		ret = siw_hal_write_value(dev,
					reg->ext_watch_font_offset,
					offset);
		if (ret < 0) {
			 goto out;
		}

		ret = siw_hal_reg_write(dev,
					reg->ext_watch_font_addr,
			 		(void *)font_data, curr_size);
		if (ret < 0) {
			goto out;
		}

		font_size -= curr_size;
		font_data += curr_size;
		offset += (curr_size>>2);
	}

	if (chip->lcd_mode == LCD_MODE_U2) {
		ret = ext_watch_mem_ctrl(dev, 0, "font dump: U2 mode");
		if (ret < 0) {
			goto out;
		}
	}

	t_watch_info(dev, "font dump: done(%d)\n",
			watch->font_written_size);

	mutex_unlock(&ts->lock);

	return 0;

out:
	t_watch_err(dev, "font dump: failed, %d\n", ret);

	mutex_unlock(&ts->lock);

	return ret;
}

static int ext_watch_set_cfg(struct device *dev, char log)
{
	int ret = 0;

	ret = ext_watch_set_position(dev, log);

	return ret;
}

static int ext_watch_get_dic_st(struct device *dev, char *buf, int *len)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dic_status = 0;
	u32 watch_status = 0;
	char log[FONT_TEMP_LOG_SZ] = {0, };
	int loglen = 0;
	int buflen = 0;
	int ret = 0;

	if (len)
		buflen = *len;

	ret = siw_hal_read_value(dev,
				reg->sys_dispmode_status,
				&dic_status);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_read_value(dev,
				reg->ext_watch_display_status,
				&watch_status);
	if (ret < 0) {
		goto out;
	}

	if (dic_status == 2) {
		loglen = siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, 0,
						"display mode U2, watch display %s\n",
						(watch_status & 1) ? "on" : "off");
	} else {
		loglen = siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, 0,
						"display mode U%d\n",
						dic_status);
	}
	if (buf) {
		memcpy(&buf[buflen], log, loglen);
		buflen += loglen;
	} else {
		t_watch_warn(dev, "get dic st: NULL buffer");
	}
	t_watch_info(dev, "%s", log);

	if (len)
		*len = buflen;

	t_watch_info(dev, "get dic st: done\n");

	return 0;

out:
	t_watch_err(dev, "get dic st: failed, %d\n", ret);

	return ret;
}

static ssize_t store_ext_watch_rtc_onoff(struct device *dev,
		 			const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	u8 buf_val, value = 0;
	u8 zero = '0';
	int ret = 0;

	buf_val = buf[0];
	value = !(!value || (value == zero));

	mutex_lock(&ts->lock);
	ret = ext_watch_rtc_start(dev, value);
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_ext_watch_block_cfg(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "block_watch_cfg : %d\n",
				atomic_read(&chip->block_watch_cfg));

	return ret;
}

static ssize_t store_ext_watch_block_cfg(struct device *dev,
		 const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	u8 buf_val, value;
	u8 zero = '0';

	buf_val = buf[0];
	value = !((buf_val == UNBLOCKED) || (buf_val == zero));

	atomic_set(&chip->block_watch_cfg, value);

	t_watch_info(dev, "%s\n", (value)? "BLOCKED" : "UNBLOCKED");

	return count;
}

static ssize_t store_ext_watch_font_onoff(struct device *dev,
					const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	u8 buf_val, value;
	u8 zero = '0';
	int ret = 0;

	if (atomic_read(&chip->block_watch_cfg) == BLOCKED) {
		t_watch_err(dev, "store font onoff blocked\n");
		return __ret_val_blocked(count);
	}

	mutex_lock(&ts->lock);

	buf_val = buf[0];
	value = !((buf_val == POWER_OFF) || (buf_val == zero));

	ret = ext_watch_set_cfg(dev, 1);
	if (ret < 0) {
		t_watch_err(dev, "failed to set watch cfg, %d\n", ret);
		goto out;
	}

	watch->ext_wdata.time.disp_waton = value;

	ret = ext_watch_display_onoff(dev, 1);
	if (ret < 0) {
		t_watch_err(dev, "failed to set watch display control, %d\n", ret);
		goto out;
	}

	ret = count;

out:
	mutex_unlock(&ts->lock);
	return ret;
}

static ssize_t show_ext_watch_query_font_data(struct device *dev, char *buf)
{
	struct ext_watch_query_font_data query;
	ssize_t size = sizeof(struct ext_watch_query_font_data);

	query.Font_supported = SUPPORT;
	query.max_font_x_size = 255;
	query.max_font_y_size = 184;
	query.max_cln_x_size = 255;
	query.max_cln_y_size = 48;

	memcpy(buf, (void *)&query, size);

	return size;
}

static ssize_t show_ext_watch_query_font_position(struct device *dev, char *buf)
{
	struct ext_watch_query_font_position query;
	ssize_t size = sizeof(struct ext_watch_query_font_position);

	query.vertical_position_supported = SUPPORT;
	query.horizontal_position_supported = SUPPORT;

	memcpy(buf, (void *)&query, size);

	return size;
}

static ssize_t show_ext_watch_query_font_time(struct device *dev, char *buf)
{
	struct ext_watch_query_font_time query;
	ssize_t size = sizeof(struct ext_watch_query_font_time);

	query.h24_supported = SUPPORT;
	query.ampm_supported = NOT_SUPPORT;

	t_watch_info(dev, "query font time: %2d %2d\n",
		query.h24_supported, query.ampm_supported);

	memcpy(buf, (void *)&query, size);

	return size;
}

static ssize_t show_ext_watch_query_font_color(struct device *dev, char *buf)
{
	struct ext_watch_query_font_color query;
	ssize_t size = sizeof(struct ext_watch_query_font_color);

	query.max_num = EXT_WATCH_LUT_MAX;
	query.lut_supported = SUPPORT;
	query.alpha_supported = SUPPORT;
	query.gradation_supported = SUPPORT;

	memcpy(buf, (void *)&query, size);

	return size;
}

static ssize_t show_ext_watch_query_font_effect(struct device *dev, char *buf)
{
	struct ext_watch_query_font_effect query;
	ssize_t size = sizeof(struct ext_watch_query_font_effect);

	query.zero_supported = SUPPORT;
	query.blink_type = 2;

	memcpy(buf, (void *)&query, size);

	return size;
}

static ssize_t show_ext_watch_curr_time(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int len = 0;

	if (chip == NULL)
		return 0;

	ext_watch_get_curr_time(dev, buf, &len);

	return len;
}

static ssize_t store_ext_watch_config_font_effect(struct device *dev,
		 			const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg_mode *mode = &watch->ext_wdata.mode;
	struct ext_watch_cfg_time *time = &watch->ext_wdata.time;
	struct ext_watch_cfg_position *position = &watch->ext_wdata.position;
	struct ext_watch_config_font_effect cfg;
	char *period;

	if (atomic_read(&chip->block_watch_cfg) == BLOCKED) {
		t_watch_err(dev, "store font effect blocked\n");
		return __ret_val_blocked(count);
	}

	memcpy((char *)&cfg, buf, sizeof(struct ext_watch_config_font_effect));

	position->h24_en = cfg.h24_en;
	position->zero_disp = cfg.zero_disp;
	position->clock_disp_mode = cfg.clock_disp_type;
	position->midnight_hour_zero_en = cfg.midnight_hour_zero_en;
	position->bhprd = cfg.blink.blink_type;

	mode->blink_area.bstartx = cfg.blink.bstartx;
	mode->blink_area.bendx = cfg.blink.bendx;

	time->disp_waton = cfg.watchon;

	switch (cfg.blink.blink_type) {
	case 3:
		period = "2 sec";
		break;
	case 2:
		period = "1 sec";
		break;
	case 1:
		period = "0.5 sec";
		break;
	case 0:
		/* fall through */
	default:
		period = "off";
		break;
	}

	t_watch_info(dev,
		"24h mode %s, Zero Dispaly %s,%s Type %s mode "
		"blink area [%d , %d] Period %s "
		"watch display on/off : %s\n",
		(cfg.h24_en) ? "on" : "off",
		(cfg.zero_disp) ? "on" : "off",
		(cfg.clock_disp_type) ? "MM:SS" : "HH:MM",
		(cfg.midnight_hour_zero_en) ? "00:00" : "12:00",
		cfg.blink.bstartx, cfg.blink.bendx, period,
		(time->disp_waton)? "on" : "off");

	return count;
}

static ssize_t store_ext_watch_config_font_property(struct device *dev,
 					const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg_mode *mode = &watch->ext_wdata.mode;
	struct ext_watch_bits_lut *lut;
	struct ext_watch_config_font_property cfg;
	struct ext_watch_config_font_lut *lut_src;
	char log[FONT_TEMP_LOG_SZ] = {0, };
	int loglen = 0;
	int idx = 0;
//	int ret = 0;

	if (atomic_read(&chip->block_watch_cfg) == BLOCKED) {
		t_watch_err(dev, "store font property blocked\n");
		return __ret_val_blocked(count);
	}

	memcpy((char *)&cfg, buf, sizeof(struct ext_watch_config_font_property));


	loglen += siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, loglen,
				"lut[%d] ", cfg.max_num);

	lut = mode->lut;
	lut_src = cfg.lut;
	for (idx = 0; idx < (int)cfg.max_num; idx++) {
		lut->b = lut_src->rgb_blue;
		lut->g = lut_src->rgb_green;
		lut->r = lut_src->rgb_red;

		loglen += siw_watch_snprintf(log, FONT_TEMP_LOG_SZ, loglen,
					"%d:%02X%02X%02X ",
					idx + 1,
					lut_src->rgb_blue,
					lut_src->rgb_green,
					lut_src->rgb_red);

		lut++;
		lut_src++;
	}

	t_watch_info(dev, "%s\n", log);

	siw_touch_blocking_notifier_call(LCD_EVENT_TOUCH_WATCH_LUT_UPDATE, (void*)(&cfg));

	return count;
}

static const struct reset_area watch_win_default = {
	.x1	= 520,
	.y1 = 0,
	.x2 = 720,
	.y2 = 80,
};

static ssize_t store_ext_watch_config_font_position(struct device *dev,
					const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg_mode *mode = &watch->ext_wdata.mode;
	struct ext_watch_cfg_position *position = &watch->ext_wdata.position;
	struct ext_watch_config_font_position cfg;
	struct reset_area *watch_win = pdata_watch_win(ts->pdata);
	int ret = 0;

	if (atomic_read(&chip->block_watch_cfg) == BLOCKED) {
		t_watch_err(dev, "store font position blocked\n");
		return __ret_val_blocked(count);
	}

	if (!watch_win) {
		watch_win = (struct reset_area *)&watch_win_default;
	}

	memcpy((char *)&cfg, buf, sizeof(struct ext_watch_config_font_position));

	mode->watch_area_x.watstart = cfg.watstartx;
	mode->watch_area_x.watend = cfg.watendx;
	mode->watch_area_y.watstart = cfg.watstarty;
	mode->watch_area_y.watend = cfg.watendy;

	position->h10x_pos = cfg.h10x_pos;
	position->h1x_pos = cfg.h1x_pos;
	position->m10x_pos = cfg.m10x_pos;
	position->m1x_pos = cfg.m1x_pos;
	position->clx_pos = cfg.clx_pos;

#if 1
	if ((cfg.watstartx < watch_win->x1) || (cfg.watendx > watch_win->x2) ||
		(cfg.watstarty <  watch_win->y1) || (cfg.watendy > watch_win->y2)) {
		t_watch_err(dev, "check the position. (invalid range)\n");
		ret = -EINVAL;
	}
#else
	if ((cfg.watstartx < 0x208) || (cfg.watendx > 0x2D0) ||
		(cfg.watstarty < 0x0) || (cfg.watendy > 0x50)) {
		t_watch_err(dev, "check the position. (invalid range)\n");
		ret = -EINVAL;
	}
#endif

	t_watch_info(dev,
			"watch area: sx %d, ex %d, sy %d, ey %d]\n",
			cfg.watstartx, cfg.watendx, cfg.watstarty, cfg.watendy);
	t_watch_info(dev,
			"watch position: h10x %d, h1x %d, c1x %d, m10x %d, m1x %d]\n",
		 	cfg.h10x_pos, cfg.h1x_pos, cfg.clx_pos, cfg.m10x_pos, cfg.m1x_pos);

	if (ret < 0)
		goto out;

	siw_touch_blocking_notifier_call(LCD_EVENT_TOUCH_WATCH_POS_UPDATE, (void*)(&cfg));

	ret = count;

out:
	return ret;
 }

static ssize_t store_ext_watch_config_time_sync(struct device *dev,
					const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct ext_watch_cfg_time *time = &watch->ext_wdata.time;
	struct ext_watch_config_time_sync cfg;
	int ret = 0;

	if (atomic_read(&chip->block_watch_cfg) == BLOCKED) {
		t_watch_err(dev, "store time sync blocked\n");
		return __ret_val_blocked(count);
	}

	mutex_lock(&ts->lock);

	t_watch_dbg(dev, "store config type sync\n");

	memcpy((char *)&cfg, buf, sizeof(struct ext_watch_config_time_sync));

	time->rtc_sct.hour = cfg.rtc_cwhour;
	time->rtc_sct.min = cfg.rtc_cwmin;
	time->rtc_sct.sec = cfg.rtc_cwsec;
	time->rtc_sctcnt = cfg.rtc_cwmilli;

	ret = ext_watch_set_curr_time(dev);
	if (ret < 0) {
		goto out;
	}

	ret = ext_watch_get_curr_time(dev, NULL, NULL);
	if (ret < 0) {
		goto out;
	}

	ret = ext_watch_display_onoff(dev, 1);
	if (ret < 0) {
		goto out;
	}

	ret = count;

out:
	mutex_unlock(&ts->lock);
	return ret;
}

static ssize_t show_ext_watch_setting(struct device *dev, char *buf)
{
	int len = 0;

	t_watch_dbg(dev, "show watch setting\n");

	//ext_watch_get_mode(dev, buf, &len);
	ext_watch_get_position(dev, buf, &len);
	ext_watch_get_curr_time(dev, buf, &len);
	ext_watch_get_dic_st(dev, buf, &len);

	return len;
}


#define SIW_WATCH_ATTR(_name, _show, _store)	\
		TOUCH_ATTR(_name, _show, _store)

#define _SIW_WATCH_ATTR_T(_name)	\
		touch_attr_##_name

static SIW_WATCH_ATTR(rtc_onoff, NULL, store_ext_watch_rtc_onoff);
static SIW_WATCH_ATTR(block_cfg, show_ext_watch_block_cfg,
					store_ext_watch_block_cfg);
static SIW_WATCH_ATTR(config_fontonoff, NULL,
					store_ext_watch_font_onoff);
static SIW_WATCH_ATTR(config_fonteffect, NULL,
					store_ext_watch_config_font_effect);
static SIW_WATCH_ATTR(config_fontproperty, NULL,
					store_ext_watch_config_font_property);
static SIW_WATCH_ATTR(config_fontposition, NULL,
					store_ext_watch_config_font_position);
static SIW_WATCH_ATTR(config_timesync, NULL,
					store_ext_watch_config_time_sync);
static SIW_WATCH_ATTR(current_time, show_ext_watch_curr_time, NULL);
static SIW_WATCH_ATTR(query_fontdata,
					show_ext_watch_query_font_data, NULL);
static SIW_WATCH_ATTR(query_fontposition,
					show_ext_watch_query_font_position, NULL);
static SIW_WATCH_ATTR(query_timesync,
					show_ext_watch_query_font_time, NULL);
static SIW_WATCH_ATTR(query_fontcolor,
					show_ext_watch_query_font_color, NULL);
static SIW_WATCH_ATTR(query_fonteffect,
					show_ext_watch_query_font_effect, NULL);
static SIW_WATCH_ATTR(get_cfg, show_ext_watch_setting, NULL);

 static struct attribute *ext_watch_attribute_list[] = {
	 &_SIW_WATCH_ATTR_T(rtc_onoff).attr,
	 &_SIW_WATCH_ATTR_T(block_cfg).attr,
	 &_SIW_WATCH_ATTR_T(config_fontonoff).attr,
	 &_SIW_WATCH_ATTR_T(config_fonteffect).attr,
	 &_SIW_WATCH_ATTR_T(config_fontproperty).attr,
	 &_SIW_WATCH_ATTR_T(config_fontposition).attr,
	 &_SIW_WATCH_ATTR_T(config_timesync).attr,
	 &_SIW_WATCH_ATTR_T(current_time).attr,
	 &_SIW_WATCH_ATTR_T(query_fontdata).attr,
	 &_SIW_WATCH_ATTR_T(query_fontposition).attr,
	 &_SIW_WATCH_ATTR_T(query_timesync).attr,
	 &_SIW_WATCH_ATTR_T(query_fontcolor).attr,
	 &_SIW_WATCH_ATTR_T(query_fonteffect).attr,
	 &_SIW_WATCH_ATTR_T(get_cfg).attr,
	 NULL,
 };

 static const struct attribute_group ext_watch_attribute_group = {
	 .attrs = ext_watch_attribute_list,
};

static ssize_t ext_watch_attr_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct siw_touch_chip *chip =
		container_of(kobj, struct siw_touch_chip, kobj);
	struct siw_touch_attribute *priv =
		container_of(attr, struct siw_touch_attribute, attr);
//	struct siw_ts *ts = chip->ts;
	ssize_t ret = 0;

	if (priv->show)
		ret = priv->show(chip->dev, buf);

	return ret;
}

static ssize_t ext_watch_attr_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct siw_touch_chip *chip =
		container_of(kobj, struct siw_touch_chip, kobj);
	struct siw_touch_attribute *priv =
		container_of(attr, struct siw_touch_attribute, attr);
//	struct siw_ts *ts = chip->ts;
	ssize_t ret = 0;

	if (priv->store)
		ret = priv->store(chip->dev, buf, count);

	return ret;
}

static const struct sysfs_ops ext_watch_sysfs_ops = {
	.show	= ext_watch_attr_show,
	.store	= ext_watch_attr_store,
};

static struct kobj_type ext_watch_kobj_type = {
	.sysfs_ops = &ext_watch_sysfs_ops,
};

static ssize_t ext_watch_access_read(struct file *filp, struct kobject *kobj,
					struct bin_attribute *bin_attr,
					char *buf, loff_t off, size_t count)
{
	struct siw_touch_chip *chip =
		container_of(kobj, struct siw_touch_chip, kobj);
	struct siw_ts *ts = chip->ts;
	struct device *dev = chip->dev;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	int ret = -EFAULT;

	if (off == 0) {
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
			t_watch_info(dev, "IC deep sleep. cannot read\n");
		} else {
			while (1) {
				if (atomic_read(&watch->state.font_status) != FONT_DOWNLOADING) {
					break;
				}

				t_watch_info(dev, "font downloading\n");
				touch_msleep(10);
			}
			ext_watch_font_dump(dev, watch->ext_wdata.font_data);
		 }
	 }

	if ((off + count) > SIW_MAX_FONT_SIZE) {
		t_watch_err(dev, "access_read: size overflow : offset[%d] size[%d]\n",
			 (int)off, (int)count);
		ret = -EOVERFLOW;
	} else {
		memcpy(buf, &watch->ext_wdata.font_data[off], count);
		ret = count;
	}

	return (ssize_t)ret;
}

static ssize_t ext_watch_access_write(struct file *filp, struct kobject *kobj,
					struct bin_attribute *bin_attr,
					char *buf, loff_t off, size_t count)
{
	struct siw_touch_chip *chip =
		container_of(kobj, struct siw_touch_chip, kobj);
	struct siw_ts *ts = chip->ts;
	struct device *dev = chip->dev;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	int ret = -EFAULT;

	if (atomic_read(&chip->block_watch_cfg) == BLOCKED) {
		t_watch_warn(dev, "access_write: blocked\n");
		return count;
	}

	if (buf == NULL) {
		t_watch_err(dev, "access_write: NULL buf\n");
		goto out;
	}

	if (watch->ext_wdata.font_data == NULL) {
		t_watch_err(dev, "access_write: NULL font_data\n");
		goto out;
	}

	if ((off + count) > SIW_MAX_FONT_SIZE)	 {
		t_watch_err(dev, "access_read: size overflow : offset[%d] size[%d]\n",
			 (int)off, (int)count);
		atomic_set(&watch->state.font_status, FONT_EMPTY);
		ret = -EOVERFLOW;
		goto out;
	}

	watch->font_written_size = off + count;

	memcpy(&watch->ext_wdata.font_data[off], buf, count);

	atomic_set(&watch->state.font_status, FONT_DOWNLOADING);

	mod_delayed_work(ts->wq, &chip->font_download_work, 20);

	ret = count;

 out:
	 return ret;
 }

static int ext_watch_fontdata_attr_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct bin_attribute *fontdata_attr = &watch->fontdata_attr;
	u8 *buf;
	int size = SIW_MAX_FONT_SIZE;
	int ret = 0;

	watch->font_written_size = 0;
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		t_watch_err(dev, "failed to allocate font buffer[%Xh]\n", size);
		ret = -ENOMEM;
		goto out;
	}
	watch->ext_wdata.font_data = buf;

	t_dev_dbg_base(dev, "font buffer[%Xh] allocated\n", size);

	sysfs_bin_attr_init(fontdata_attr);
	fontdata_attr->attr.name = "config_fontdata";
	fontdata_attr->attr.mode = S_IWUSR | S_IRUSR;
	fontdata_attr->read = ext_watch_access_read;
	fontdata_attr->write = ext_watch_access_write;
	fontdata_attr->size = size;

	ret = sysfs_create_bin_file(&chip->kobj, fontdata_attr);
	if (ret < 0) {
		t_watch_err(dev, "failed to create %s, %d\n",
		 		fontdata_attr->attr.name, ret);
		goto out_bin;
	}

	t_dev_dbg_base(dev, "file[%s] file created\n",
			fontdata_attr->attr.name);

	return 0;

out_bin:
	kfree(buf);

out:

	return ret;
}

static void ext_watch_fontdata_attr_free(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct watch_data *watch = (struct watch_data *)chip->watch;
	struct bin_attribute *fontdata_attr = &watch->fontdata_attr;
	int size = (int)fontdata_attr->size;

	t_dev_dbg_base(dev, "file[%s] file released\n",
			fontdata_attr->attr.name);

	sysfs_remove_bin_file(&chip->kobj, fontdata_attr);

	t_dev_dbg_base(dev, "font buffer[%Xh] released\n", size);

	kfree(watch->ext_wdata.font_data);
	watch->ext_wdata.font_data = NULL;
	watch->font_written_size = 0;
}

static int ext_watch_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct watch_data *watch = (struct watch_data *)chip->watch;
	int ret = 0;

	if (ext_quirk_check(dev)) {
		return 0;
	}

	if (siw_touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER){
		return -EBUSY;
	}

	if (!watch) {
		t_watch_err(dev, "NULL watch\n");
		return -ENOMEM;
	}

	if (atomic_read(&watch->state.font_status) == FONT_EMPTY){
		return -EBUSY;
	}

	ret = ext_watch_set_cfg(dev, 0);
	if (ret < 0) {
		t_watch_err(dev, "failed to set watch cfg, %d\n", ret);
		goto out;
	}

	ret = ext_watch_display_onoff(dev, 0);
	if (ret < 0) {
		t_watch_err(dev, "failed to set watch display control, %d\n", ret);
		goto out;
	}

out:
	 return ret;
}


static int ext_watch_create_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct device *idev = &ts->input->dev;
	//Caution : not ts->kobj
	struct kobject *kobj = &chip->kobj;
	struct watch_data *watch;
	char *name = NULL;
	int ret = 0;

	watch = kzalloc(sizeof(*watch), GFP_KERNEL);
	if (!watch) {
		t_dev_err(dev, "failed to allocate memory for watch data\n");
		ret = -ENOMEM;
		goto out;
	}
	chip->watch = watch;
	t_dev_dbg_base(dev, "watch_data allocated\n");

//	name = touch_chip_name(ts);
	name = SIW_EXT_WATCH_NAME;

	ret = kobject_init_and_add(kobj, &ext_watch_kobj_type,
			idev->kobj.parent, "%s", name);
	if (ret < 0) {
		t_watch_err(dev, "failed to create sysfs entry\n");
		goto out_kobj;
	}

	ret = sysfs_create_group(kobj, &ext_watch_attribute_group);
	if (ret < 0) {
		t_watch_err(dev, "failed to create sysfs\n");
		goto out_sys;
	}

	ret = ext_watch_fontdata_attr_init(dev);
	if (ret < 0) {
		t_watch_err(dev, "failed to create sysfs\n");
		goto out_init;
	}

	INIT_DELAYED_WORK(&chip->font_download_work, ext_watch_font_download);

	t_dev_dbg_base(dev, "%s watch sysfs registered\n",
			touch_chip_name(ts));

	return 0;

out_init:
	sysfs_remove_group(kobj, &ext_watch_attribute_group);

out_sys:
	kobject_del(kobj);

out_kobj:
	kfree(watch);
	chip->watch = NULL;

out:

	return ret;
}

static void ext_watch_remove_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct device *idev = &ts->input->dev;
	//Caution : not ts->kobj
	struct kobject *kobj = &chip->kobj;

	cancel_delayed_work(&chip->font_download_work);

	ext_watch_fontdata_attr_free(dev);

	sysfs_remove_group(kobj, &ext_watch_attribute_group);

	kobject_del(kobj);

	kfree(chip->watch);
	chip->watch = NULL;

	t_dev_dbg_base(dev, "%s watch sysfs unregistered\n",
			touch_chip_name(ts));
}

int siw_hal_watch_sysfs(struct device *dev, int on_off)
{
	if (ext_quirk_check(dev)) {
		return 0;
	}

	if (on_off == DRIVER_INIT) {
		return ext_watch_create_sysfs(dev);
	}

	ext_watch_remove_sysfs(dev);
	return 0;
}

int siw_hal_watch_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (ext_quirk_check(dev)) {
		return 0;
	}

	if (chip->watch == NULL) {
		t_watch_err(dev, "NULL watch_data\n");
		return -ENOMEM;
	}

	return ext_watch_init(dev);
}

int siw_hal_watch_get_curr_time(struct device *dev, char *buf, int *len)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (ext_quirk_check(dev)) {
		return 0;
	}

	if (chip->watch == NULL) {
		t_watch_err(dev, "NULL watch_data\n");
		return -ENOMEM;
	}

	return ext_watch_get_curr_time(dev, buf, len);
}

void siw_hal_watch_display_off(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (ext_quirk_check(dev)) {
		return;
	}

	if (chip->watch == NULL) {
		t_watch_err(dev, "NULL watch_data\n");
		return;
	}

	ext_watch_display_off(dev);
}

int siw_hal_watch_chk_font_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (ext_quirk_check(dev)) {
		return 0;
	}

	if (chip->watch == NULL) {
		t_watch_err(dev, "NULL watch_data\n");
		return -ENOMEM;
	}

	ext_watch_init(dev);

	return ext_watch_chk_font_status(dev);
}

int siw_hal_watch_is_disp_waton(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct watch_data *watch = (struct watch_data *)chip->watch;

	return (!watch)? 0 :
		watch->ext_wdata.time.disp_waton;
}

int siw_hal_watch_is_rtc_run(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct watch_data *watch = (struct watch_data *)chip->watch;

	return (!watch)? 0 :
		(atomic_read(&watch->state.rtc_status) == RTC_RUN);
}

void siw_hal_watch_set_rtc_run(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct watch_data *watch = (struct watch_data *)chip->watch;

	if (watch)
		atomic_set(&watch->state.rtc_status, RTC_RUN);
}

void siw_hal_watch_set_rtc_clear(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct watch_data *watch = (struct watch_data *)chip->watch;

	if (watch)
		atomic_set(&watch->state.rtc_status, RTC_CLEAR);
}

void siw_hal_watch_set_font_empty(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct watch_data *watch = (struct watch_data *)chip->watch;

	if (watch)
		atomic_set(&watch->state.font_status, FONT_EMPTY);
}

#endif	/* __SIW_SUPPORT_WATCH */


