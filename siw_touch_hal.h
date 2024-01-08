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

#include "siw_touch_cfg.h"

#include "siw_touch_hal_reg.h"

struct siw_hal_rw_multi {
	int wr;
	int addr;
	void *data;
	int size;
	char *name;
};

enum {
	NON_FAULT_INT 	= -1,
	NON_FAULT_U32	= ~0,
};

/* report packet - type 1 */
struct siw_hal_touch_data_type_1 {
	u32 track_id:5;
	u32 tool_type:3;
	u32 angle:8;
	u32 event:2;
	u32 x:14;

	u32 y:14;
	u32 pressure:8;
	u32 reserve1:10;

	u32 reserve2:4;
	u32 width_major:14;
	u32 width_minor:14;
} __packed;

/* report packet */
struct siw_hal_touch_data {
	u8 tool_type:4;
	u8 event:4;
	u8 track_id;
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

static inline u32 siw_tc_sts_irq_type(int status)
{
	return ((status >> 16) & 0x0F);
}

static inline u32 siw_tc_sts_running_sts(int status)
{
	return (status & 0x1F);
}

enum {
	TC_STS_IRQ_TYPE_INIT_DONE	= 2,
	TC_STS_IRQ_TYPE_ABNORMAL	= 3,
	TC_STS_IRQ_TYPE_DEBUG		= 4,
	TC_STS_IRQ_TYPE_REPORT		= 5,
};

#define PALM_ID					15

enum {
	TC_DRIVE_CTL_START		= (0x1<<0),
	TC_DRIVE_CTL_STOP		= (0x1<<1),
	TC_DRIVE_CTL_DISP_U0	= (0x0<<7),
	TC_DRIVE_CTL_DISP_U2	= (0x2<<7),
	TC_DRIVE_CTL_DISP_U3	= (0x3<<7),
	/* */
	TC_DRIVE_CTL_PARTIAL	= (0x1<<9),
	TC_DRIVE_CTL_QCOVER		= (0x1<<10),
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
#define CONNECT_DC				(0x02)
#define CONNECT_OTG				(0x03)
#define CONNECT_WIRELESS		(0x04)

enum {
	SW_RESET = 0,
	HW_RESET_ASYNC,
	HW_RESET_SYNC,
	//
	HW_RESET_COND = 0x5A,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	LCD_MODE_U0	= 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
	LCD_MODE_MAX,
};

#define SIW_LCD_DRIVING_MODE_STR(_mode)	\
		[LCD_MODE_##_mode] = #_mode

static const char *__siw_lcd_driving_mode_strs[] = {
	SIW_LCD_DRIVING_MODE_STR(U0),
	SIW_LCD_DRIVING_MODE_STR(U2_UNBLANK),
	SIW_LCD_DRIVING_MODE_STR(U2),
	SIW_LCD_DRIVING_MODE_STR(U3),
	SIW_LCD_DRIVING_MODE_STR(U3_PARTIAL),
	SIW_LCD_DRIVING_MODE_STR(U3_QUICKCOVER),
	SIW_LCD_DRIVING_MODE_STR(STOP),
};

static inline const char *siw_lcd_driving_mode_str(int mode)
{
	return (mode < LCD_MODE_MAX) ?
			__siw_lcd_driving_mode_strs[mode] : "(invalid)";
}

/*
 * for pdata->mode_allowed
 */
enum {
	LCD_MODE_BIT_U0 			= BIT(LCD_MODE_U0),
	LCD_MODE_BIT_U2_UNBLANK		= BIT(LCD_MODE_U2_UNBLANK),
	LCD_MODE_BIT_U2				= BIT(LCD_MODE_U2),
	LCD_MODE_BIT_U3				= BIT(LCD_MODE_U3),
	LCD_MODE_BIT_U3_PARTIAL		= BIT(LCD_MODE_U3_PARTIAL),
	LCD_MODE_BIT_U3_QUICKCOVER	= BIT(LCD_MODE_U3_QUICKCOVER),
	LCD_MODE_BIT_STOP			= BIT(LCD_MODE_STOP),
	LCD_MODE_BIT_MAX			= BIT(LCD_MODE_MAX),
};

enum {
	LPWG_T1_KNOCK_1 = 1,
	LPWG_T1_KNOCK_2,
	LPWG_T1_SWIPE_LEFT = 3,
	LPWG_T1_SWIPE_RIGHT,
	LPWG_T1_SWIPE_UP,
	LPWG_T1_SWIPE_DOWN,
};

#define LPWG_T1_SWIPE_LAST	LPWG_T1_SWIPE_DOWN

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

enum {
	IC_BOOT_DONE = 0,
	IC_BOOT_FAIL,
};

enum {
	ATTN_ESD_EN				= (1U<<0),
	ATTN_WDOG_EN			= (1U<<1),
	ATTN_ABNORMAL_SPI_EN	= (1U<<2),
	/* */
	ABNORMAL_IC_DETECTION	= (ATTN_ESD_EN | ATTN_WDOG_EN),
};

enum {
	WAFER_TYPE_MASK = (0x07),
};

#if defined(__SIW_PANEL_CLASS_MOBILE_OLED)
#define __SIW_FW_TYPE_OLED_BASE
#endif

#if defined(__SIW_FW_TYPE_OLED_BASE)
#define __SIW_FLASH_CRC_PASS

#define OLED_CFG_MAGIC_CODE			(0xCACACACA)
#define OLED_CFG_C_SIZE				(OLED_NUM_C_CONF<<OLED_POW_C_CONF)
#define OLED_CFG_S_SIZE				(1<<OLED_POW_S_CONF)

enum {
	OLED_E_FW_CODE_SIZE_ERR		= 1,
	OLED_E_FW_CODE_ONLY_VALID,
	OLED_E_FW_CODE_AND_CFG_VALID,
	OLED_E_FW_CODE_CFG_ERR,
};

#define GDMA_CTRL_READONLY		BIT(17)
#define GDMA_CTRL_EN			BIT(26)

#define __FC_CTRL_PAGE_ERASE	BIT(0)
#define __FC_CTRL_MASS_ERASE	BIT(1)
#define __FC_CTRL_WR_EN			BIT(2)

#define FC_CTRL_PAGE_ERASE		(__FC_CTRL_PAGE_ERASE | __FC_CTRL_WR_EN)
#define FC_CTRL_MASS_ERASE		(__FC_CTRL_MASS_ERASE | __FC_CTRL_WR_EN)
#define FC_CTRL_WR_EN			(__FC_CTRL_WR_EN)

#define BDMA_CTRL_EN			BIT(16)
#define BDMA_CTRL_BST			(0x001E0000)
#define BDMA_STS_TR_BUSY		BIT(6)

#if defined(__SIW_FLASH_CFG)
#define OLED_POW_C_CONF		10
#define	OLED_POW_S_CONF		10

#define OLED_NUM_C_CONF		0
#define OLED_MIN_S_CONF		1
#define OLED_MAX_S_CONF		10

enum {
	OLED_MIN_S_CONF_IDX = 1,
	OLED_MAX_S_CONF_IDX = (OLED_MAX_S_CONF + 1),
};

union oled_cfg_size {
	struct {
		u32 common_size:16;
		u32 specific_size:16;
	} b;
	u32 w;
};

union oled_s_cfg_info_1 {
	struct {
		u32 chip_rev:8;
		u32 model_id:8;
		u32 lcm_id:8;
		u32 fpc_id:8;
	} b;
	u32 w;
};

union oled_s_cfg_info_2 {
	struct {
		u32 lot_id:8;
		u32 rsvd:24;
	} b;
	u32 w;
};

struct oled_s_cfg_head {
	union oled_s_cfg_info_1	info_1;
	union oled_s_cfg_info_2	info_2;
	u32					version;
	u32					model_name;
};

struct oled_cfg_head {
	u32 magic_code;
	u32 chip_id;
	u32 s_index;
	union oled_cfg_size c_size;
	//
	struct oled_s_cfg_head s_cfg_head;
	//
	u32 rsvd1;
	u32 rsvd2;
	u32 rsvd3;
	u32 rsvd4;
};
#endif	/* __SIW_FLASH_CFG */
#endif	/* __SIW_FW_TYPE_OLED_BASE */

#define FW_DN_LOG_UNIT			(8<<10)

#define MAX_RW_SIZE_POW			(10)
#define MAX_RW_SIZE				__SIZE_POW(MAX_RW_SIZE_POW)		//1K
#define FLASH_CONF_SIZE_POWER	(10)
#define FLASH_CONF_SIZE			(1 * __SIZE_POW(FLASH_CONF_SIZE_POWER))		// 1K

#define FLASH_KEY_CODE_CMD		0xDFC1
#define FLASH_KEY_CONF_CMD		0xE87B
#define FLASH_BOOTCHK_VALUE		0x0A0A0000
#define FLASH_CODE_DNCHK_VALUE	0x42
#define FLASH_CONF_DNCHK_VALUE	0x84

#define FW_BOOT_LOADER_INIT		(0x74696E69)	//"init"
#define FW_BOOT_LOADER_CODE		(0x544F4F42)	//"BOOT"

#if defined(__SIW_FW_TYPE_1)
/*
 * Common CONF + Specific CONF(s)
 */
enum {
	POW_C_CONF = 9,
	POW_S_CONF = 10,
};

enum {
	NUM_C_CONF = 1,
	MIN_S_CONF = 1,
	MAX_S_CONF = 31,
};

enum {
	MIN_S_CONF_IDX = 1,
	MAX_S_CONF_IDX = (MAX_S_CONF + 1),
};

#define FW_BOOT_CODE_ADDR		(0x044)
#define FW_S_CONF_IDX_ADDR		(0x260)
#define FW_S_CONF_DN_ADDR		(0x267)

#define FW_TYPE_STR		"FW_TYPE_1"

#define FLASH_CONF_DNCHK_VALUE_TYPE_X	(FLASH_CONF_DNCHK_VALUE | 0x0C)
#define FLASH_CONF_SIZE_TYPE_X			(1<<POW_C_CONF)

#define S_CFG_DBG_IDX			0
#else	/* !__SIW_FW_TYPE_1 */
#define FW_TYPE_STR		"FW_TYPE_0"

#define FLASH_CONF_DNCHK_VALUE_TYPE_X	(FLASH_CONF_DNCHK_VALUE)
#define FLASH_CONF_SIZE_TYPE_X			FLASH_CONF_SIZE
#endif	/* __SIW_FW_TYPE_1 */

#define FW_POST_QUIRK_DELAY		20
#define FW_POST_QUIRK_COUNT		200

#define	FW_POST_DELAY			20
#define FW_POST_COUNT			200

#define	CONF_POST_DELAY			20
#define CONF_POST_COUNT			200

#if defined(__SIW_FW_TYPE_OLED_BASE)
#ifdef FW_TYPE_STR
#undef FW_TYPE_STR
#define FW_TYPE_STR		"FW_TYPE_OLED"
#endif
//
#ifdef FLASH_CONF_DNCHK_VALUE_TYPE_X
#undef FLASH_CONF_DNCHK_VALUE_TYPE_X
#define FLASH_CONF_DNCHK_VALUE_TYPE_X	0
#endif
//
#ifdef FLASH_CONF_SIZE_TYPE_X
#undef FLASH_CONF_SIZE_TYPE_X
#define FLASH_CONF_SIZE_TYPE_X			0
#endif
//
#endif	/*__SIW_FW_TYPE_OLED_BASE */

enum {
	BIN_CFG_OFFSET_POS = 0xE0,
	BIN_VER_OFFSET_POS = 0xE8,
	BIN_VER_EXT_OFFSET_POS = 0xDC,
	BIN_PID_OFFSET_POS = 0xF0,
};

struct siw_hal_tc_version_bin {
	u8 major:4;
	u8 build:4;
	u8 minor;
	u16 rsvd:12;
	u16 ext:4;
} __packed;

struct siw_hal_tc_version {
	u8 minor;
	u8 major:4;
	u8 build:4;
	u8 chip;
	u8 protocol:4;
	u8 ext:4;
} __packed;

struct siw_hal_tc_version_ext_date {
	u8 rr;
	u8 dd;
	u8 mm;
	u8 yy;
} __packed;

struct siw_hal_fw_info {
	u32 chip_id_raw;
	u8 chip_id[8];
	u32 sys_id_addr;
	u32 sys_id_raw;
	u8 sys_id[8];
	/* */
	union {
		struct siw_hal_tc_version version;
		u32 version_raw;
	} v;
	u32 version_ext;
	/* */
	int invalid_pid;
	u8 product_id[8+4];
	u8 revision;
	u32 fpc;
	u32 wfr;
	u32 cg;
	u32 lot;
	u32 sn;
	u32 date;
	u32 time;
	u32 conf_index;
	/* __SIW_FW_TYPE_1 */
	u32 conf_idx_addr;
	u32 conf_dn_addr;
	u32 boot_code_addr;
	int conf_skip;
	/* __SIW_FW_TYPE_OLED_BASE */
	int sizeof_flash;
	int flash_page_offset;
	int flash_page_size;
	int flash_max_rw_size;
	u32 gdma_saddr;
	u32 gdma_ctrl;
	u32 gdma_ctrl_en;
	u32 gdma_ctrl_ro;
	u32 gdma_start;
	u32 fc_ctrl;
	u32 fc_ctrl_page_erase;
	u32 fc_ctrl_mass_erase;
	u32 fc_ctrl_wr_en;
	u32 fc_start;
	u32 fc_addr;
	u32 flash_status;
	u32 fc_erase_wait_cnt;
	u32 fc_erase_wait_time;
	u32 bdma_saddr;
	u32 bdma_daddr;
	u32 bdma_cal_op;
	u32 bdma_cal_op_ctrl;
	u32 bdma_ctrl;
	u32 bdma_ctrl_en;
	u32 bdma_ctrl_bst;
	u32 bdma_start;
	u32 bdma_sts;
	u32 bdma_sts_tr_busy;
	u32 datasram_addr;
	u32 info_ptr;
	u32 gdma_crc_result;
	u32 gdma_crc_pass;
	u32 crc_fixed_value;
	u32 cfg_chip_id;
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

enum {
	VERSION_EXT_DATE = 1,
};

static inline int siw_hal_fw_chk_version_ext(u32 version_ext, u32 ext_flag)
{
	switch (ext_flag) {
	case VERSION_EXT_DATE:
		{
			struct siw_hal_tc_version_ext_date ext_date;

			memcpy(&ext_date, &version_ext, sizeof(ext_date));

			if (ext_date.yy &&
				ext_date.mm &&
				ext_date.dd) {

				if (ext_date.yy < 0x16) {
					break;
				}

				if (ext_date.mm > 0x12) {
					break;
				}

				if (ext_date.dd > 0x31) {
					break;
				}

				return 0;
			}
		}
		break;
	}

	return -EINVAL;
}

static inline void siw_hal_fw_set_version(struct siw_hal_fw_info *fw,
						u32 version, u32 version_ext)
{
	fw->v.version_raw = version;
	fw->version_ext = (fw->v.version.ext) ? version_ext : 0;
}

static inline void siw_hal_fw_set_revision(struct siw_hal_fw_info *fw, u32 revision)
{
	fw->revision = revision & 0xFF;
}

#define PID_LEN_MAX	8
#if defined(CONFIG_TOUCHSCREEN_SIW_SW42000)
#define PID_LEN_MIN	6
#else
#define PID_LEN_MIN	7
#endif

#define PID_IN_RANGE(_val, _min, _max)	(((_val) >= (_min)) && ((_val) <= (_max)))

static inline int siw_hal_fw_check_pid(char *pid)
{
	int len = strlen(pid);
	int invalid = 0;
	int i;
	char c;

	if (len > PID_LEN_MAX) {
		invalid |= BIT(9);
	} else if (len < PID_LEN_MIN) {
		invalid |= BIT(8);
	}

	for (i = 0; i < len; i++) {
		c = pid[i];

		if (PID_IN_RANGE(c, '0', '9')) {
			continue;
		}

		if (PID_IN_RANGE(c, 'A', 'Z')) {
			continue;
		}

		invalid |= BIT(i);
	}

	return invalid;
}

static inline void siw_hal_fw_set_prod_id(struct siw_hal_fw_info *fw, u8 *prod, u32 size)
{
	int len = min((int)sizeof(fw->product_id), (int)size);
	memset(fw->product_id, 0, sizeof(fw->product_id));
	memcpy(fw->product_id, prod, len);

	fw->invalid_pid = siw_hal_fw_check_pid(fw->product_id);
}


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

enum {
	CHIP_REPORT_NONE = 0,
	CHIP_REPORT_TYPE_0,
	CHIP_REPORT_TYPE_1,
};

enum {
	CHIP_STATUS_NONE = 0,
	CHIP_STATUS_TYPE_0,
	CHIP_STATUS_TYPE_1,
	CHIP_STATUS_TYPE_2,
};

enum {
	STS_ID_NONE = 0,
	STS_ID_VALID_DEV_CTL,
	STS_ID_VALID_CODE_CRC,
	STS_ID_VALID_CFG_CRC,
	STS_ID_VALID_FONT_CRC,
	STS_ID_ERROR_ABNORMAL,
	STS_ID_ERROR_SYSTEM,
	STS_ID_ERROR_MISMTACH,
	STS_ID_VALID_IRQ_PIN,
	STS_ID_VALID_IRQ_EN,
	STS_ID_ERROR_MEM,
	STS_ID_VALID_TC_DRV,
	STS_ID_ERROR_DISP,
};

enum {
	STS_POS_VALID_DEV_CTL			= 5,
	STS_POS_VALID_CODE_CRC			= 6,
	STS_POS_VALID_CFG_CRC			= 7,
	STS_POS_VALID_FONT_CRC			= 8,
	STS_POS_ERROR_ABNORMAL			= 9,
	STS_POS_ERROR_SYSTEM			= 10,
	STS_POS_ERROR_MISMTACH			= 13,
	STS_POS_VALID_IRQ_PIN			= 15,
	STS_POS_VALID_IRQ_EN			= 20,
	STS_POS_ERROR_MEM				= 21,
	STS_POS_VALID_TC_DRV			= 22,
	STS_POS_ERROR_DISP				= 31,
	/* */
	STS_POS_VALID_CODE_CRC_TYPE_0	= 22,
};

struct siw_hal_status_mask_bit {
	u32 valid_dev_ctl;
	u32 valid_code_crc;
	u32 valid_cfg_crc;
	u32 error_abnormal;
	u32 error_system;
	u32 error_mismtach;
	u32 valid_irq_pin;
	u32 valid_irq_en;
	u32 error_mem;
	u32 valid_tv_drv;
	u32 error_disp;
};

struct siw_hal_status_filter {
	int id;
	u32 width;
	u32 pos;
	u32 flag;
#define _STS_FILTER_FLAG_TYPE_ERROR		(1<<0)
#define _STS_FILTER_FLAG_ESD_SEND		(1<<16)
#define _STS_FILTER_FLAG_CHK_FAULT		(1<<17)
	const char *str;
};

#define _STS_FILTER(_id, _width, _pos, _flag, _str)	\
		{ .id = _id, .width = _width, .pos = _pos, .flag = _flag, .str = _str, }

enum {
	STS_FILTER_FLAG_TYPE_ERROR		= _STS_FILTER_FLAG_TYPE_ERROR,
	STS_FILTER_FLAG_ESD_SEND		= _STS_FILTER_FLAG_ESD_SEND,
	STS_FILTER_FLAG_CHK_FAULT		= _STS_FILTER_FLAG_CHK_FAULT,
};

#define REG_LOG_MAX		8
#define REG_DIR_NONE	0
#define REG_DIR_RD		1
#define REG_DIR_WR		2
#define REG_DIR_ERR		(1<<8)
#define REG_DIR_MASK	(REG_DIR_ERR-1)

struct siw_hal_reg_log {
	int dir;
	u32 addr;
	u32 data;
};

struct siw_touch_chip_opt {
	u32 f_info_more:1;
	u32 f_ver_ext:1;
	u32 f_attn_opt:1;
	u32 f_glove_en:1;
	u32 f_grab_en:1;
	u32 f_dbg_report:1;
	u32 f_u2_blank_chg:1;
	u32 f_rsvd00:1;
	/* */
	u32 f_flex_report:1;
	u32 f_rsvd01:7;
	/* */
	u32 f_rsvd02:8;
	/* */
	u32 f_rsvd03:8;
	/* */
	u32 t_boot_mode:4;
	u32 t_sts_mask:4;
	u32 t_chk_mode:4;
	u32 t_sw_rst:4;
	u32 t_clock:4;
	u32 t_chk_mipi:4;
	u32 t_chk_frame:4;
	u32 t_chk_tci_debug:4;
	/* */
	u32 t_chk_sys_error:4;
	u32 t_chk_sys_fault:4;
	u32 t_chk_fault:4;
	u32 rsvd21:4;
	u32 rsvd22:8;
	u32 rsvd23:4;
	u32 t_oled:4;
	/* */
	u32 t_tc_cmd:4;
	u32 t_tc_quirk:4;
	u32 rsvd31:8;
	u32 t_lpwg:4;
	u32 t_knock:4;
	u32 t_swipe:4;
	u32 rsvd33:4;
	/* */
	u32 t_bus_opt:4;
	u32 t_rw_opt:4;
	u32 t_i2c_opt:4;
	u32 t_spi_opt:4;
	u32 rsvd42:8;
	u32 rsvd43:8;
} __packed;

enum {
	HAL_DBG_GRP_0 = 0,
	HAL_DBG_GRP_MAX,
};

enum {
	HAL_DBG_DLY_TC_DRIVING_0 = 0,
	HAL_DBG_DLY_TC_DRIVING_1,
	HAL_DBG_DLY_FW_0,
	HAL_DBG_DLY_FW_1,
	HAL_DBG_DLY_FW_2,
	HAL_DBG_DLY_HW_RST_0,
	HAL_DBG_DLY_HW_RST_1,
	HAL_DBG_DLY_HW_RST_2,
	HAL_DBG_DLY_SW_RST_0,
	HAL_DBG_DLY_SW_RST_1,
	HAL_DBG_DLY_NOTIFY,
	HAL_DBG_DLY_LPWG,
	HAL_DBG_DLY_MAX,
};

struct siw_hal_debug {
	/* group 0 : delay */
	u32 delay[HAL_DBG_DLY_MAX];
	/* group 1 : rsvd */
	/* group 2 : rsvd */
	/* group 3 : rsvd */
};

struct siw_hal_ops_quirk {	//quirk operation
	int (*hw_reset)(struct device *dev, int pwr_con, int delay);
	/* */
	int (*fwup_check)(struct device *dev, u8 *fw_buf);
	int (*fwup_upgrade)(struct device *dev, u8 *fw_buf, int fw_size, int retry);
	int (*boot_status)(struct device *dev, u32 *boot_st);
};

struct siw_touch_chip {
	void *ts;			//struct siw_ts
	int tc_cmd_table[LCD_MODE_MAX];
	struct siw_touch_chip_opt opt;
	struct siw_hal_reg *reg;
	struct device *dev;
	struct kobject kobj;
	struct siw_hal_touch_info info;
	struct siw_hal_fw_info fw;
	struct siw_hal_swipe_ctrl swipe;
	/* */
	int report_type;
	int status_type;
	u32 status_mask;
	u32 status_mask_normal;
	u32 status_mask_logging;
	u32 status_mask_reset;
	u32 status_mask_ic_normal;
	u32 status_mask_ic_abnormal;
	u32 status_mask_ic_error;
	u32 status_mask_ic_valid;
	u32 status_mask_ic_disp_err;
	struct siw_hal_status_mask_bit status_mask_bit;
	struct siw_hal_status_filter *status_filter;
	/* */
	int mode_allowed_partial;
	int mode_allowed_qcover;
	int drv_reset_low;
	int drv_delay;
	int drv_opt_delay;
	int driving_ctrl;
	u8 prev_lcd_mode;
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
	atomic_t boot;
	atomic_t abnormal_noti_sent;
	int boot_fail_cnt;
	struct siw_hal_reg_log reg_log[REG_LOG_MAX];
	struct siw_hal_debug dbg;
	int fw_abs_path;
	int fwup_status;
	/* */
	int sysfs_done;
	/* */
	struct siw_hal_ops_quirk ops_quirk;
};

enum {
	FWUP_STATUS_OK			= 0,
	FWUP_STATUS_BUSY		= 1,
	FWUP_STATUS_NG_OP	 	= 2,
	FWUP_STATUS_NG_F_OPEN	= 3,
	FWUP_STATUS_NG_F_CHK	= 4,
	FWUP_STATUS_NG_CODE 	= 5,
	FWUP_STATUS_NG_CFG		= 6,
	FWUP_STATUS_NG_IO		= 9,
	FWUP_STATUS_MAX,
};

#define SIW_FWUP_STATUS_STR(_sts)	\
		[FWUP_STATUS_##_sts] = #_sts

static const char *__siw_fwup_status_strs[] = {
	SIW_FWUP_STATUS_STR(OK),
	SIW_FWUP_STATUS_STR(BUSY),
	SIW_FWUP_STATUS_STR(NG_OP),
	SIW_FWUP_STATUS_STR(NG_F_OPEN),
	SIW_FWUP_STATUS_STR(NG_F_CHK),
	SIW_FWUP_STATUS_STR(NG_CODE),
	SIW_FWUP_STATUS_STR(NG_CFG),
	SIW_FWUP_STATUS_STR(NG_IO),
};

static inline const char *siw_fwup_status_str(int status)
{
	const char *str = (status < FWUP_STATUS_MAX) ?
						__siw_fwup_status_strs[status] : NULL;

	return (str) ? str : "(invalid)";
}

static inline int siw_hal_get_fwup_status(struct siw_touch_chip *chip)
{
	return chip->fwup_status;
}

static inline void siw_hal_set_fwup_status(struct siw_touch_chip *chip, int status)
{
	const char *str = siw_fwup_status_str(status);

	t_dev_info(chip->dev, "FW upgrade: status %d(%s)\n", status, str);

	chip->fwup_status = status;
}

static inline int hal_dbg_delay(struct siw_touch_chip *chip, int index)
{
	return (index < HAL_DBG_DLY_MAX) ? chip->dbg.delay[index] : 0;
}

enum {
	BOOT_FAIL_RECOVERY_MAX = 3,	/* to avoid infinite repetition */
};

enum {
	CHIP_INIT_RETRY_PROBE = 2,
	CHIP_INIT_RETRY_MAX = 5,
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

enum {
	ADDR_SKIP_MASK	= 0xFFFF,
};

static inline int siw_addr_is_skip(u32 addr)
{
	return (addr >= ADDR_SKIP_MASK);
}

enum {
	BOOT_STS_POS_MODE = 0,
	BOOT_STS_POS_BUSY,
	BOOT_STS_POS_DUMP_DONE,
	BOOT_STS_POS_DUMP_ERR,
	BOOT_STS_POS_MAGIC_ERR,
};

static inline u32 siw_hal_boot_sts_pos_busy(struct siw_touch_chip *chip)
{
	u32 pos = BOOT_STS_POS_BUSY;

	switch (chip->opt.t_boot_mode) {
	case 2:
	case 1:
		pos = 0;
		break;
	}

	return pos;
}

static inline u32 siw_hal_boot_sts_pos_dump_err(struct siw_touch_chip *chip)
{
	u32 pos = BOOT_STS_POS_DUMP_ERR;

	switch (chip->opt.t_boot_mode) {
	case 2:
	case 1:
		pos = 2;
		break;
	}

	return pos;
}

static inline u32 siw_hal_boot_sts_mask_empty(struct siw_touch_chip *chip)
{
	u32 mask = 0;

	switch (chip->opt.t_boot_mode) {
	case 2:
		mask = BIT(6);
		break;
	}

	return mask;
}

#define HAL_ACCESS_CHK_SKIP_SLEEP	BIT(3)
#define HAL_ACCESS_CHK_SKIP_FB		BIT(2)
#define HAL_ACCESS_CHK_SKIP_PM		BIT(1)
#define HAL_ACCESS_CHK_SKIP_INIT	BIT(0)

extern int siw_hal_get_boot_status(struct device *dev, u32 *boot_st);

extern int siw_hal_read_value(struct device *dev, u32 addr, u32 *value);
extern int siw_hal_write_value(struct device *dev, u32 addr, u32 value);
extern int siw_hal_reg_read(struct device *dev, u32 addr, void *data, int size);
extern int siw_hal_reg_write(struct device *dev, u32 addr, void *data, int size);
extern int siw_hal_reg_read_single(struct device *dev, u32 addr, void *data, int size);
extern int siw_hal_reg_write_single(struct device *dev, u32 addr, void *data, int size);

extern int siw_hal_reg_rw_multi(struct device *dev,
			struct siw_hal_rw_multi *multi, char *title);

extern int siw_hal_enable_flash_wp(struct device *dev);
extern int siw_hal_disable_flash_wp(struct device *dev);

extern int siw_hal_access_not_allowed(struct device *dev, char *title, int skip_flag);

extern int siw_hal_ic_test_unit(struct device *dev, u32 data);

extern struct siw_hal_reg *siw_hal_get_default_reg(int opt);

extern struct siw_touch_operations *siw_hal_get_default_ops(int opt);

#endif	/* __SIW_TOUCH_HAL_H */


