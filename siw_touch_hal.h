/*
 * SiW touch hal driver
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef __SIW_TOUCH_HAL_H
#define __SIW_TOUCH_HAL_H

#include "siw_touch_hal_reg.h"

/* report packet */
struct siw_hal_touch_data {
	u8 tool_type:4;
	u8 event:4;
	s8 track_id;
	u16 x;
	u16 y;
	u8 pressure;
	u8 angle;
	u16 width_major;
	u16 width_minor;
} __packed;

struct siw_hal_touch_info {
	u32 ic_status;
	u32 device_status;
	//
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	u32 button_cnt:3;
	u32 palm_bit:16;
	//
	struct siw_hal_touch_data data[MAX_FINGER];
} __packed;

#define PALM_ID					15

enum {
	TC_DRIVE_CTL_START		= (0x1<<0),
	TC_DRIVE_CTL_STOP		= (0x1<<1),
	TC_DRIVE_CTL_DISP_U0	= (0x0<<7),
	TC_DRIVE_CTL_DISP_U2	= (0x2<<7),
	TC_DRIVE_CTL_DISP_U3	= (0x3<<7),
	/* */
	TC_DRIVE_CTL_QCOVER		= (0x1<<9),
	/* */
	TC_DRIVE_CTL_MODE_VB	= (0x0<<2),
	TC_DRIVE_CTL_MODE_6LHB	= (0x1<<2),
	TC_DRIVE_CTL_MODE_MIK	= (0x1<<3),
};

#define CMD_DIS					0xAA
#define CMD_ENA					0xAB
#define CMD_CLK_ON				0x83
#define CMD_CLK_OFF				0x82
#define CMD_OSC_ON				0x81
#define CMD_OSC_OFF				0x80
#define CMD_RESET_LOW			0x84
#define CMD_RESET_HIGH			0x85


#define CONNECT_NONE			(0x00)
#define CONNECT_USB				(0x01)
#define CONNECT_TA				(0x02)
#define CONNECT_OTG				(0x03)
#define CONNECT_WIRELESS		(0x10)

enum {
	SW_RESET = 0,
	HW_RESET,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	ABS_MODE = 0,
	KNOCK_1,
	KNOCK_2,
	SWIPE_RIGHT,
	SWIPE_LEFT,
	/* */
	CUSTOM_DEBUG = 200,
	KNOCK_OVERTAP,
};

enum {
	LCD_MODE_U0 			= (1L<<0),
	LCD_MODE_U2_UNBLANK		= (1L<<1),
	LCD_MODE_U2				= (1L<<2),
	LCD_MODE_U3				= (1L<<3),
	LCD_MODE_U3_PARTIAL		= (1L<<4),
	LCD_MODE_U3_QUICKCOVER	= (1L<<5),
	LCD_MODE_STOP			= (1L<<6),
	LCD_MODE_MAX			= (1L<<30),
};

enum {
	SWIPE_R = 0,
	SWIPE_L,
};

enum {
	SWIPE_RIGHT_BIT	= 1,
	SWIPE_LEFT_BIT = (1<<16),
};

/* swipe */
enum {
	SWIPE_ENABLE_CTRL = 0,
	SWIPE_DISABLE_CTRL,
	SWIPE_DIST_CTRL,
	SWIPE_RATIO_THR_CTRL,
	SWIPE_RATIO_PERIOD_CTRL,
	SWIPE_RATIO_DIST_CTRL,
	SWIPE_TIME_MIN_CTRL,
	SWIPE_TIME_MAX_CTRL,
	SWIPE_AREA_CTRL,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

#define MAX_RW_SIZE_POW			(10)
#define MAX_RW_SIZE				__SIZE_POW(MAX_RW_SIZE_POW)
#define FLASH_CONF_SIZE_POWER	(10)
#define FLASH_CONF_SIZE			(1 * __SIZE_POW(FLASH_CONF_SIZE_POWER))
#define FLASH_FW_SIZE			(69 * __SIZE_POW(FLASH_CONF_SIZE_POWER))

#define FLASH_KEY_CODE_CMD		0xDFC1
#define FLASH_KEY_CONF_CMD		0xE87B
#define FLASH_BOOTCHK_VALUE		0x0A0A0000
#define FLASH_CODE_DNCHK_VALUE	0x42
#define FLASH_CONF_DNCHK_VALUE	0x84


enum {
	WAFER_TYPE_MASK = (0x07),
};

struct siw_hal_fw_info {
	u32 chip_id_raw;
	u8 chip_id[8];
	u32 version_raw;
	u8 version[4];
	u8 product_id[8+4];
	u8 image_version[4];
	u8 image_product_id[8+4];
	u8 revision;
	u32 fpc;
	u32 wfr;
	u32 cg;
	u32 lot;
	u32 sn;
	u32 date;
	u32 time;
};

static inline void siw_hal_fw_set_chip_id(struct siw_hal_fw_info *fw, u32 chip_id)
{
	fw->chip_id_raw = chip_id;
	memset(fw->chip_id, 0, sizeof(fw->chip_id));
	fw->chip_id[0] = (chip_id>>24) & 0xFF;
	fw->chip_id[1] = (chip_id>>16) & 0xFF;
	fw->chip_id[2] = (chip_id>>8) & 0xFF;
	fw->chip_id[3] = chip_id & 0xFF;
}

static inline void siw_hal_fw_set_version(struct siw_hal_fw_info *fw, u32 version)
{
	fw->version_raw = version;
	fw->version[0] = ((version >> 8) & 0xFF);
	fw->version[1] = version & 0xFF;
}

static inline void siw_hal_fw_set_revision(struct siw_hal_fw_info *fw, u32 revision)
{
	fw->revision = revision & 0xFF;
}

static inline void siw_hal_fw_set_prod_id(struct siw_hal_fw_info *fw, u8 *prod, u32 size)
{
	int len = min(sizeof(fw->product_id), size);
	memset(fw->product_id, 0, sizeof(fw->product_id));
	memcpy(fw->product_id, prod, len);
}


struct siw_hal_asc_info {
	u16 normal_s;
	u16 acute_s;
	u16 obtuse_s;
};

struct siw_hal_swipe_info {
	u8	distance;
	u8	ratio_thres;
	u8	ratio_distance;
	u8	ratio_period;
	u16	min_time;
	u16	max_time;
	struct active_area area;
};

struct siw_hal_swipe_ctrl {
	u32 mode;
	struct siw_hal_swipe_info info[2]; /* down is 0, up 1 - LG4894 use up */
};

struct siw_touch_chip {
	void *ts;			//struct siw_ts
	struct siw_hal_reg *reg;
	struct device *dev;
	struct kobject kobj;
	struct siw_hal_touch_info info;
	struct siw_hal_fw_info fw;
	struct siw_hal_asc_info asc;
	struct siw_hal_swipe_ctrl swipe;
	u8 lcd_mode;
	u8 driving_mode;
	u8 u3fake;
	void *watch;
	struct mutex bus_lock;
	struct delayed_work font_download_work;
	struct delayed_work fb_notify_work;
	u32 charger;
	u32 earjack;
	u8 tci_debug_type;
	u8 swipe_debug_type;
	atomic_t block_watch_cfg;
	atomic_t init;
};

#define TCI_MAX_NUM					2
#define SWIPE_MAX_NUM				2
#define TCI_DEBUG_MAX_NUM			16
#define SWIPE_DEBUG_MAX_NUM			8
#define DISTANCE_INTER_TAP			__SIZE_POW(1) /* 2 */
#define DISTANCE_TOUCHSLOP			__SIZE_POW(2)	/* 4 */
#define TIMEOUT_INTER_TAP_LONG		__SIZE_POW(3)	/* 8 */
#define MULTI_FINGER				__SIZE_POW(4)	/* 16 */
#define DELAY_TIME					__SIZE_POW(5)	/* 32 */
#define TIMEOUT_INTER_TAP_SHORT		__SIZE_POW(6)	/* 64 */
#define PALM_STATE					__SIZE_POW(7)	/* 128 */
#define TAP_TIMEOVER				__SIZE_POW(8)	/* 256 */

#define TCI_DEBUG_ALL				(0 | \
									DISTANCE_INTER_TAP |	\
									DISTANCE_TOUCHSLOP | \
									TIMEOUT_INTER_TAP_LONG | \
									MULTI_FINGER | \
									DELAY_TIME | \
									TIMEOUT_INTER_TAP_SHORT | \
									PALM_STATE | \
									TAP_TIMEOVER | \
									0)


static inline struct siw_touch_chip *to_touch_chip(struct device *dev)
{
	return (struct siw_touch_chip *)touch_get_dev_data(to_touch_core(dev));
}

static inline struct siw_touch_chip *to_touch_chip_from_kobj(struct kobject *kobj)
{
	return (struct siw_touch_chip *)container_of(kobj,
								struct siw_touch_chip, kobj);
}

extern int siw_hal_reg_read(struct device *dev, u32 addr, void *data, int size);
extern int siw_hal_reg_write(struct device *dev, u32 addr, void *data, int size);
extern int siw_hal_read_value(struct device *dev, u32 addr, u32 *value);
extern int siw_hal_write_value(struct device *dev, u32 addr, u32 value);
extern int siw_hal_xfer_msg(struct device *dev, struct touch_xfer_msg *xfer);
extern int siw_hal_xfer_rx_seq(struct device *dev, u32 reg, u32 *data, int size);
extern int siw_hal_xfer_tx_seq(struct device *dev, u32 reg, u32 *data, int size);


extern struct siw_touch_operations *siw_hal_get_default_ops(int opt);

#endif	/* __SIW_TOUCH_HAL_H */


