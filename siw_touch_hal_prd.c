/*
 * siw_touch_hal_prd.c - SiW touch hal driver for PRD
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Sungyeal Park <parksy5@siliconworks.co.kr>
 *         Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "siw_touch_cfg.h"

#if defined(__SIW_SUPPORT_PRD)	//See siw_touch_cfg.h

#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/linkage.h>
#include <linux/syscalls.h>
#include <linux/namei.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/fs.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"
#include "siw_touch_event.h"
#include "siw_touch_gpio.h"
#include "siw_touch_irq.h"
#include "siw_touch_sys.h"

/*
 * [PRD]
 * Touch IC Production Test
 * This supports a method to verify the quality of Touch IC using input/output file test.
 *
 * 1. Input setup file(spec file)
 * 2. Self testing
 * 3. Output result file(self test result)
 * 4. Verify the result
 *
 */

#define __SIW_SUPPORT_PRD_SET_SD

#if defined(__SIW_SUPPORT_PRD_SET_SD)
//#define __SIW_SUPPORT_PRD_SET_SD_ONLY
#endif

#define __SIW_SUPPORT_PRD_RAW_CMP

enum {
	PRD_LINE_NUM		= (1<<10),
//	PRD_PATH_SIZE		= (1<<6),		//64
//	PRD_BURST_SIZE		= (1<<9),		//512
	/* */
	MAX_LOG_FILE_COUNT	= (4),
	MAX_LOG_FILE_SIZE	= (10 * (1<<20)),	/* 10M byte */
	/* */
	MAX_TEST_CNT			= 2,
};

enum {
	PRD_RAWDATA_SZ_POW	= 1,
	PRD_RAWDATA_SIZE	= (1<<PRD_RAWDATA_SZ_POW),
	/* */
	PRD_M1_COL_SIZE		= (1<<1),
	/* */
	PRD_LOG_BUF_SIZE	= (1<<10),	//1K
	PRD_BUF_SIZE		= (8<<10),	//8K fixed, PAGE_SIZE can be adjust by kernel
	/* */
	PRD_DEBUG_BUF_SIZE	= (336),
	/* */
	PRD_BUF_DUMMY		= 128,		//dummy for avoiding memory panic
	PRD_APP_INFO_SIZE	= 32,
	/* */
	PRD_SHORT_COL_SZ	= 4,
};

enum {
	IMG_OFFSET_IDX_NONE = 0,
	IMG_OFFSET_IDX_RAW,
	IMG_OFFSET_IDX_BASELINE_EVEN,
	IMG_OFFSET_IDX_BASELINE_ODD,
	IMG_OFFSET_IDX_DELTA,
	IMG_OFFSET_IDX_LABEL,
	IMG_OFFSET_IDX_F_DELTA,
	IMG_OFFSET_IDX_DEBUG,
	IMG_OFFSET_IDX_RAW_S,
	IMG_OFFSET_IDX_BASELINE_S,
	IMG_OFFSET_IDX_DELTA_S,
	IMG_OFFSET_IDX_LABEL_S,
	IMG_OFFSET_IDX_DEBUG_S,
	IMG_OFFSET_IDX_MAX,
};

enum {
	TIME_INFO_SKIP = 0,
	TIME_INFO_WRITE,
};

enum {
	/* need to tune test */
	U3_M2_RAWDATA_TEST = 0,
	U3_M1_RAWDATA_TEST,
	U0_M2_RAWDATA_TEST,
	U0_M1_RAWDATA_TEST,
	/* */
	OPEN_SHORT_ALL_TEST,
	OPEN_NODE_TEST,
	SHORT_NODE_TEST,
	U3_BLU_JITTER_TEST,
	/* */
	U3_JITTER_TEST,
	U3_M1_JITTER_TEST,
	U0_JITTER_TEST,
	U0_M1_JITTER_TEST,
	/* */
	SHORT_FULL_TEST,
	IRQ_TEST,
	/* */
	UX_INVALID,

	/* for sd_test_flag */
	OPEN_SHORT_RESULT_DATA_IDX = 24,
	OPEN_SHORT_RESULT_RAWDATA_IDX,
	OPEN_SHORT_RESULT_ALWAYS_IDX,
};

#define PRD_SET_TEST_STR(_name)		[_name] = #_name

static const char *__prd_test_str[UX_INVALID] = {
	PRD_SET_TEST_STR(U3_M2_RAWDATA_TEST),
	PRD_SET_TEST_STR(U3_M1_RAWDATA_TEST),
	PRD_SET_TEST_STR(U0_M2_RAWDATA_TEST),
	PRD_SET_TEST_STR(U0_M1_RAWDATA_TEST),
	/* */
	PRD_SET_TEST_STR(OPEN_SHORT_ALL_TEST),
	PRD_SET_TEST_STR(OPEN_NODE_TEST),
	PRD_SET_TEST_STR(SHORT_NODE_TEST),
	PRD_SET_TEST_STR(U3_BLU_JITTER_TEST),
	/* */
	PRD_SET_TEST_STR(U3_JITTER_TEST),
	PRD_SET_TEST_STR(U3_M1_JITTER_TEST),
	PRD_SET_TEST_STR(U0_JITTER_TEST),
	PRD_SET_TEST_STR(U0_M1_JITTER_TEST),
	/* */
	PRD_SET_TEST_STR(SHORT_FULL_TEST),
	PRD_SET_TEST_STR(IRQ_TEST),
};

static inline const char *prd_get_test_str(int type)
{
	int valid = (type < UX_INVALID) & (__prd_test_str[type] != NULL);
	return (valid) ? __prd_test_str[type] : "";
}

enum {
	U3_TEST_PRE_CMD = 0x3,
	U0_TEST_PRE_CMD = 0x0,
};

enum {
	NO_TEST_POST_CMD = 0,
	OPEN_SHORT_ALL_TEST_POST_CMD,
	OPEN_NODE_TEST_POST_CMD,
	SHORT_NODE_TEST_POST_CMD,
	M2_RAWDATA_TEST_POST_CMD = 5,
	M1_RAWDATA_TEST_POST_CMD,
	JITTER_TEST_POST_CMD = 12,
};

enum {
	LINE_FILTER_OPTION	= (0x40000),
};

enum {
	U3_M2_RAWDATA_TEST_FLAG		= (1<<U3_M2_RAWDATA_TEST),
	U3_M1_RAWDATA_TEST_FLAG		= (1<<U3_M1_RAWDATA_TEST),
	U0_M2_RAWDATA_TEST_FLAG		= (1<<U0_M2_RAWDATA_TEST),
	U0_M1_RAWDATA_TEST_FLAG 	= (1<<U0_M1_RAWDATA_TEST),
	/* */
	OPEN_NODE_TEST_FLAG			= (1<<OPEN_NODE_TEST),
	SHORT_NODE_TEST_FLAG		= (1<<SHORT_NODE_TEST),
	OPEN_SHORT_NODE_TEST_FLAG	= (OPEN_NODE_TEST_FLAG|SHORT_NODE_TEST_FLAG),
	U3_BLU_JITTER_TEST_FLAG 	= (1<<U3_BLU_JITTER_TEST),
	/* */
	U3_JITTER_TEST_FLAG			= (1<<U3_JITTER_TEST),
	U3_M1_JITTER_TEST_FLAG		= (1<<U3_M1_JITTER_TEST),
	U0_JITTER_TEST_FLAG			= (1<<U0_JITTER_TEST),
	U0_M1_JITTER_TEST_FLAG		= (1<<U0_M1_JITTER_TEST),
	/* */
	SHORT_FULL_TEST_FLAG		= (1<<SHORT_FULL_TEST),
	IRQ_TEST_FLAG				= (1<<IRQ_TEST),
	/* */
	OPEN_SHORT_RESULT_DATA_FLAG		= (1<<OPEN_SHORT_RESULT_DATA_IDX),
	OPEN_SHORT_RESULT_RAWDATA_FLAG	= (1<<OPEN_SHORT_RESULT_RAWDATA_IDX),
	OPEN_SHORT_RESULT_ALWAYS_FLAG	= (1<<OPEN_SHORT_RESULT_ALWAYS_IDX),
	/* */
	TEST_CLASS_FLAG_SD			= U3_M2_RAWDATA_TEST_FLAG |	\
								U3_M1_RAWDATA_TEST_FLAG |	\
								OPEN_SHORT_NODE_TEST_FLAG |	\
								U3_BLU_JITTER_TEST_FLAG |	\
								U3_JITTER_TEST_FLAG |		\
								U3_M1_JITTER_TEST_FLAG |	\
								SHORT_FULL_TEST_FLAG |		\
								IRQ_TEST_FLAG,
	TEST_CLASS_FLAG_LPWG_SD		= U0_M2_RAWDATA_TEST_FLAG |	\
								U0_M1_RAWDATA_TEST_FLAG |	\
								U0_JITTER_TEST_FLAG |		\
								U0_M1_JITTER_TEST_FLAG,
};

enum {
	CTRL_FLAG_RESULT_OFF			= (1<<0),
	/* */
	CTRL_FLAG_FILE_WR_OFF			= (1<<8),
	CTRL_FLAG_FILE_RD_OFF			= (1<<9),
};

enum {
	/*
	 * [Caution]
	 * Do not touch this ordering
	 */
	PRD_SYS_EN_IDX_SD = 0,
	PRD_SYS_EN_IDX_DELTA,
	PRD_SYS_EN_IDX_LABEL,
	PRD_SYS_EN_IDX_RAWDATA_PRD,
	//
	PRD_SYS_EN_IDX_RAWDATA_TCM,	//4
	PRD_SYS_EN_IDX_RAWDATA_AIT,
	PRD_SYS_EN_IDX_BASE,
	PRD_SYS_EN_IDX_DEBUG_BUF,
	//
	PRD_SYS_EN_IDX_LPWG_SD,		//8
	PRD_SYS_EN_IDX_FILE_TEST,
	PRD_SYS_EN_IDX_APP_RAW,
	PRD_SYS_EN_IDX_APP_BASE,
	//
	PRD_SYS_EN_IDX_APP_LABEL,	//12
	PRD_SYS_EN_IDX_APP_DELTA,
	PRD_SYS_EN_IDX_APP_DEBUG_BUF,
	PRD_SYS_EN_IDX_APP_END,
	//
	PRD_SYS_EN_IDX_APP_INFO,	//16
	//
	PRD_SYS_ATTR_MAX,
};

enum {
	PRD_SYS_EN_SD					= (1<<PRD_SYS_EN_IDX_SD),
	PRD_SYS_EN_DELTA				= (1<<PRD_SYS_EN_IDX_DELTA),
	PRD_SYS_EN_LABEL				= (1<<PRD_SYS_EN_IDX_LABEL),
	PRD_SYS_EN_RAWDATA_PRD			= (1<<PRD_SYS_EN_IDX_RAWDATA_PRD),
	//
	PRD_SYS_EN_RAWDATA_TCM			= (1<<PRD_SYS_EN_IDX_RAWDATA_TCM),
	PRD_SYS_EN_RAWDATA_AIT			= (1<<PRD_SYS_EN_IDX_RAWDATA_AIT),
	PRD_SYS_EN_BASE					= (1<<PRD_SYS_EN_IDX_BASE),
	PRD_SYS_EN_DEBUG_BUF			= (1<<PRD_SYS_EN_IDX_DEBUG_BUF),
	//
	PRD_SYS_EN_LPWG_SD				= (1<<PRD_SYS_EN_IDX_LPWG_SD),
	PRD_SYS_EN_FILE_TEST			= (1<<PRD_SYS_EN_IDX_FILE_TEST),
	PRD_SYS_EN_APP_RAW				= (1<<PRD_SYS_EN_IDX_APP_RAW),
	PRD_SYS_EN_APP_BASE				= (1<<PRD_SYS_EN_IDX_APP_BASE),
	//
	PRD_SYS_EN_APP_LABEL			= (1<<PRD_SYS_EN_IDX_APP_LABEL),
	PRD_SYS_EN_APP_DELTA			= (1<<PRD_SYS_EN_IDX_APP_DELTA),
	PRD_SYS_EN_APP_DEBUG_BUF		= (1<<PRD_SYS_EN_IDX_APP_DEBUG_BUF),
	PRD_SYS_EN_APP_END				= (1<<PRD_SYS_EN_IDX_APP_END),
	//
	PRD_SYS_EN_APP_INFO				= (1<<PRD_SYS_EN_IDX_APP_INFO),
};

#define PRD_SYS_ATTR_EN_FLAG 		(0 |	\
									PRD_SYS_EN_SD |	\
									PRD_SYS_EN_DELTA |	\
									PRD_SYS_EN_LABEL |	\
									PRD_SYS_EN_RAWDATA_PRD |	\
									PRD_SYS_EN_RAWDATA_TCM |	\
									PRD_SYS_EN_RAWDATA_AIT |	\
									PRD_SYS_EN_BASE |	\
									PRD_SYS_EN_DEBUG_BUF |	\
									PRD_SYS_EN_LPWG_SD |	\
									PRD_SYS_EN_FILE_TEST |	\
									PRD_SYS_EN_APP_RAW |	\
									PRD_SYS_EN_APP_BASE |	\
									PRD_SYS_EN_APP_LABEL |	\
									PRD_SYS_EN_APP_DELTA | 	\
									PRD_SYS_EN_APP_DEBUG_BUF |	\
									PRD_SYS_EN_APP_END |	\
									PRD_SYS_EN_APP_INFO |	\
									0)

struct siw_hal_prd_img_cmd {
	u32 raw;
	u32 baseline_even;
	u32 baseline_odd;
	u32 delta;
	u32 label;
	u32 f_delta;
	u32 debug;
};

struct siw_hal_prd_img_offset {
	u32 raw;
	u32 baseline_even;
	u32 baseline_odd;
	u32 delta;
	u32 label;
	u32 f_delta;
	u32 debug;
	u32 raw_s;
	u32 baseline_s;
	u32 delta_s;
	u32 label_s;
	u32 debug_s;
};

struct siw_hal_prd_param {
	int chip_type;
	const char **name;	/* to make group */
	//
	u32 cmd_type;
	u32 addr[IMG_OFFSET_IDX_MAX];
	//
	u32 row;
	u32 col;
	u32 col_add;
	u32 ch;
	u32 m1_col;
	u32 m1_cnt;
	u32 m2_cnt;
	//
	struct siw_touch_second_screen second_scr;
	//
	u32 sysfs_off_flag;
	//
	u32 sd_test_flag;
	u32 lpwg_sd_test_flag;
	//
	u32 ctrl_flag;
	//
	int buf_swap_flag;
};

enum {
	PRD_BUF_SWAP_RAW		= (1<<0),
	PRD_BUF_SWAP_BASE		= (1<<1),
	PRD_BUF_SWAP_DELTA		= (1<<2),
	PRD_BUF_SWAP_LABEL		= (1<<3),
	PRD_BUF_SWAP_DEBUG		= (1<<4),
};

struct siw_hal_prd_ctrl {
	u32 m2_row_col_size;
	u32 m2_row_col_buf_size;
	u32 m1_row_col_size;
	//
	u32 m2_frame_size;
	u32 m1_frame_size;
	//
	u32 delta_size;
	//
	u32 label_tmp_size;
	u32 debug_buf_size;
	//
	u32 open_result_size;
	u32 open_result_col;
	u32 open_rawdata_size;
	u32 open_rawdata_col;
	u32 short_result_size;
	u32 short_result_col;
	u32 short_rawdata_size;
	u32 short_rawdata_col;
	u32 short_full_result_size;
	u32 short_full_result_col;
	u32 short_full_rawdata_size;
	u32 short_full_rawdata_col;
	//
	u32 buf_self_size;
};

/* __SIW_SUPPORT_PRD_TUNE_FLEX */
/*
 * [OLED]
 * (TBD)
 */

/*
 * [AIT]
 * unit_base = sizeof(code_goft_tune_hdr) + ((ch)<<2);
 *                                          : u8 loft_tune_ux_m1[ch]
 *                                            u8 loft_tune_ux_m2_g1[ch]
 *                                            u8 loft_tune_ux_m2_g2[ch]
 *                                            u8 loft_tune_ux_m2_g3[ch]
 * unit_size = unit_base<<1;	//left + right
 * [u3 & u0 case]
 * total size = sizeof(u32) + (unit_size * 2);
 *              : u32 magic
 * grp_u3_offset = sizeof(u32)
 * grp_u0_offset = sizeof(u32) + unit_size
 *
 * [u3 & u2 & u0 case - SW49408/SW49409/SW49410]
 * total size = sizeof(u32) + (unit_size * 3);
 * grp_u3_offset = sizeof(u32)
 * grp_u2_offset = sizeof(u32) + unit_size
 * grp_u0_offset = sizeof(u32) + (unit_size * 2);
 *
 * [u3 & u0 & u2 case - later]
 * total size = sizeof(u32) + (unit_size * 3);
 * grp_u3_offset = sizeof(u32)
 * grp_u0_offset = sizeof(u32) + unit_size
 * grp_u2_offset = sizeof(u32) + (unit_size * 2);
 */

struct code_goft_tune_hdr {
	u32 goft_tune_m1:4;
	u32 goft_tune_m1_sign:1;
	u32 goft_tune_m2:4;
	u32 goft_tune_m2_sign:1;
	u32 goft_tune_nd:5;
	u32 rsvd:17;
} __packed;

struct siw_hal_prd_tune {
	int type;
	u32 magic_size;
	u32 hdr_size;
	u32 unit_base;
	u32 unit_size;
	u32 grp_u3_offset;
	u32 grp_u2_offset;
	u32 grp_u0_offset;
	//
	u32 ch;
	u32 code_size;
	//
	u32 code_l_goft_offset;
	u32 code_l_m1_goft_offset;
	u32 code_l_g1_goft_offset;
	u32 code_l_g2_goft_offset;
	u32 code_l_g3_goft_offset;
	u32 code_l_nd_goft_offset;
	//
	u32 code_r_goft_offset;
	u32 code_r_m1_goft_offset;
	u32 code_r_g1_goft_offset;
	u32 code_r_g2_goft_offset;
	u32 code_r_g3_goft_offset;
	u32 code_r_nd_goft_offset;
};

struct siw_hal_prd_sd_cmd {
	u32 cmd_open_node;
	u32 cmd_short_node;
	u32 cmd_m2_rawdata;
	u32 cmd_m1_rawdata;
	u32 cmd_jitter;
	u32 cmd_m1_jitter;
	u32 cmd_short_full;
};

enum _SIW_PRD_DBG_MASK_FLAG {
	PRD_DBG_NONE				= 0,
	PRD_DBG_OPEN_SHORT_DATA		= (1U<<0),
};

#if defined(__SIW_SUPPORT_PRD_SET_SD)
enum {
	SET_SD_CLASS_NONE = 0,
	SET_SD_CLASS_SD,
	SET_SD_CLASS_LPWG_SD,
};

enum {
	GET_SD_CLASS_NONE = 0,
	GET_SD_CLASS_SD,
	GET_SD_CLASS_LPWG_SD,
};

struct siw_hal_prd_sd_param {
	int get_sd_type;
	int get_sd_class;
	u32 set_sd_flag;
	int set_sd_class;
	int u3_m2[2];	/* [0] = lower, [1] = upper */
	int u3_m1[2];
	int u0_m2[2];
	int u0_m1[2];
	int u3_blu_jitter[2];
	int u3_jitter[2];
	int u3_m1_jitter[2];
	int u0_jitter[2];
	int u0_m1_jitter[2];
	int last_type;
	int last_val[2];
};
#endif	/* __SIW_SUPPORT_PRD_SET_SD */

struct siw_hal_prd_data {
	struct device *dev;
	int panel_type;
	int result_on;
	int file_wr_off;
	int file_rd_off;
	struct siw_hal_prd_sd_cmd sd_cmd;
	u32 dbg_mask;
	/* */
	struct siw_hal_prd_param param;
	struct siw_hal_prd_ctrl ctrl;
	struct siw_hal_prd_tune tune;
	/* */
	struct siw_hal_prd_img_cmd img_cmd;
	struct siw_hal_prd_img_offset img_offset;
	/* */
#if defined(__SIW_SUPPORT_PRD_SET_SD)
	struct siw_hal_prd_sd_param sd_param;
#endif
	/* */
	int prd_app_mode;
	/* */
	u8 *buf_src;
	int buf_size;
	int16_t	*m2_buf_even_rawdata;
	int16_t	*m2_buf_odd_rawdata;
	int16_t	*m2_buf_tmp;
#if defined(__SIW_SUPPORT_PRD_RAW_CMP)
	int16_t	*m2_buf_back;
#endif
	/* */
	int16_t	*m1_buf_even_rawdata;
	int16_t	*m1_buf_odd_rawdata;
	int16_t *m1_buf_tmp;
	/* */
	int16_t *open_buf_result_rawdata;
	int16_t	*short_buf_result_rawdata;
	int16_t	*open_buf_result_data;
	int16_t	*short_buf_result_data;
	/* */
	int open_result_type;
	int short_result_type;
	int image_lower;
	int image_upper;
	/* */
	int16_t	*buf_delta;
	int16_t	*buf_debug;
	u8	*buf_label_tmp;
	u8	*buf_label;
	/* */
	int16_t *buf_self;
#if defined(__SIW_SUPPORT_PRD_RAW_CMP)
	int16_t *buf_self_back;
#endif

	/* */
	int sysfs_flag;
	int sysfs_done;
	/* */
	int mon_flag;
	/* */
	char log_buf[PRD_LOG_BUF_SIZE + PRD_BUF_DUMMY];
	char line[PRD_LINE_NUM + PRD_BUF_DUMMY];
	char buf_write[PRD_BUF_SIZE + PRD_BUF_DUMMY];
	/* */
	int irq_test;
	int irq_cnt;
};

enum {
	PRD_CMD_TYPE_1 = 0,		//new type: base only
	PRD_CMD_TYPE_2 = 1,		//old type: base_even, base_odd
};

#define PRD_OFFSET_QUIRK_SET(_idx, _offset)		(((_idx)<<24) | ((_offset) & 0x00FFFFFF))
#define PRD_OFFSET_QUIRK_GET_IDX(_addr)			((_addr) >> 24)
#define PRD_OFFSET_QUIRK_GET_OFFSET(_addr)		((_addr) & 0x00FFFFFF)

#define __PRD_PARAM_DIMENSION(_row, _col, _col_add, _ch, _m1_col, _m1_cnt, _m2_cnt)	\
		.row = (_row), .col = (_col), .col_add = (_col_add), .ch = (_ch),	\
		.m1_col = (_m1_col), .m1_cnt = (_m1_cnt), .m2_cnt = (_m2_cnt)

#define __PRD_2ND_SCR(_bound_i, _bound_j)	\
		.second_scr = { _bound_i, _bound_j }

static const char *prd_param_name_lg4894_k[] = {
	"L0W53K6P", NULL
};

static const char *prd_param_name_lg4894_t[] = {
	"T0W53CV3", "TJW53CV3", NULL
};

static const char *prd_param_name_lg4894_cv[] = {
	"L0W53CV3", "L0W50CV1", "LJW53CV3", NULL
};

static const char *prd_param_name_lg4894_lv[] = {
	"L0W53LV5", "L0W50LV3", NULL
};

static const char *prd_param_name_lg4894_sf[] = {
	"L0W53SF3", NULL
};

static const char *prd_param_name_lg4895_k[] = {
	"LPW49K5", NULL
};

static const char *prd_param_name_lg4946_g[] = {
	"L0L53P1", "L0W53P1", NULL
};

static const char *prd_param_name_sw49106_type_2[] = {
	"L0W59HRT", NULL
};

static const char *prd_param_name_sw49106_type_1[] = {
	"CSOTDEMO", NULL
};

static const char *prd_param_name_sw49501_type_1[] = {
	"AURORA58", NULL
};

enum {
	SD_FLAG_LG4894 		=	(0	|	\
							U3_M2_RAWDATA_TEST_FLAG |	\
							OPEN_SHORT_NODE_TEST_FLAG |	\
							U3_JITTER_TEST_FLAG |	\
							U3_BLU_JITTER_TEST_FLAG |	\
							OPEN_SHORT_RESULT_DATA_FLAG |	\
							0),
	LPWG_SD_FLAG_LG4894 = 	(0 |	\
							U0_M2_RAWDATA_TEST_FLAG |	\
							U0_M1_RAWDATA_TEST_FLAG |	\
							U0_JITTER_TEST_FLAG |	\
							0),
};

enum {
	SD_FLAG_LG4895		=	(0	|	\
							U3_M2_RAWDATA_TEST_FLAG |	\
							OPEN_SHORT_NODE_TEST_FLAG |	\
							U3_BLU_JITTER_TEST_FLAG |	\
							OPEN_SHORT_RESULT_DATA_FLAG |	\
							0),
	LPWG_SD_FLAG_LG4895 =	(0 |	\
							U0_M2_RAWDATA_TEST_FLAG |	\
							U0_M1_RAWDATA_TEST_FLAG |	\
							0),
};

enum {
	SD_FLAG_LG4946		=	(0	|	\
							U3_M2_RAWDATA_TEST_FLAG |	\
							OPEN_SHORT_NODE_TEST_FLAG |	\
							OPEN_SHORT_RESULT_RAWDATA_FLAG |	\
							0),
	LPWG_SD_FLAG_LG4946 = 	(0 |	\
							U0_M2_RAWDATA_TEST_FLAG |	\
							U0_M1_RAWDATA_TEST_FLAG |	\
							0),
};

enum {
	__SD_FLAG_SW49XXX	=	(0	|	\
							U3_M2_RAWDATA_TEST_FLAG |	\
							OPEN_SHORT_NODE_TEST_FLAG |	\
							U3_JITTER_TEST_FLAG |	\
							OPEN_SHORT_RESULT_RAWDATA_FLAG |	\
							0),

	__LPWG_SD_FLAG_SW49XXX	=	(0 |	\
								U0_M2_RAWDATA_TEST_FLAG |	\
								U0_M1_RAWDATA_TEST_FLAG |	\
								0),

	SD_FLAG_SW49105			= __SD_FLAG_SW49XXX | OPEN_SHORT_RESULT_DATA_FLAG,
	LPWG_SD_FLAG_SW49105	= __LPWG_SD_FLAG_SW49XXX| U0_JITTER_TEST_FLAG,

	SD_FLAG_SW49106			= __SD_FLAG_SW49XXX | OPEN_SHORT_RESULT_DATA_FLAG,
	LPWG_SD_FLAG_SW49106	= __LPWG_SD_FLAG_SW49XXX | U0_JITTER_TEST_FLAG,

	SD_FLAG_SW49406			= __SD_FLAG_SW49XXX,
	LPWG_SD_FLAG_SW49406	= __LPWG_SD_FLAG_SW49XXX | U0_JITTER_TEST_FLAG,

	SD_FLAG_SW49407			= __SD_FLAG_SW49XXX,
	LPWG_SD_FLAG_SW49407	= __LPWG_SD_FLAG_SW49XXX | U0_M1_JITTER_TEST_FLAG,

	SD_FLAG_SW49408			= __SD_FLAG_SW49XXX,
	LPWG_SD_FLAG_SW49408	= __LPWG_SD_FLAG_SW49XXX | U0_JITTER_TEST_FLAG,

	SD_FLAG_SW49501			= __SD_FLAG_SW49XXX,
};

/* TBD */
enum {
	SD_FLAG_SW42000A		=	0,
	LPWG_SD_FLAG_SW42000A	=	0,

	SYS_OFF_FLAG_SW42000A	=	(0 |	\
								PRD_SYS_EN_RAWDATA_TCM |	\
								0),
	SWAP_FLAG_SW42000A		=	(0 |	\
								PRD_BUF_SWAP_RAW |	\
								PRD_BUF_SWAP_BASE |	\
								PRD_BUF_SWAP_DELTA |	\
								PRD_BUF_SWAP_LABEL |	\
								PRD_BUF_SWAP_DEBUG |	\
								0),
};

enum {
	SD_FLAG_SW1828			=	(0	|	\
								U3_M2_RAWDATA_TEST_FLAG |	\
								OPEN_SHORT_NODE_TEST_FLAG |	\
								U3_JITTER_TEST_FLAG |	\
								0),

	SYS_OFF_FLAG_SW1828		=	(0	|	\
								PRD_SYS_EN_RAWDATA_PRD	|	\
								PRD_SYS_EN_RAWDATA_TCM	|	\
								PRD_SYS_EN_DEBUG_BUF	|	\
								PRD_SYS_EN_APP_RAW	|	\
								PRD_SYS_EN_APP_BASE	|	\
								PRD_SYS_EN_APP_LABEL	|	\
								PRD_SYS_EN_APP_DELTA	|	\
								PRD_SYS_EN_APP_DEBUG_BUF	|	\
								PRD_SYS_EN_APP_END	|	\
								PRD_SYS_EN_APP_INFO	|	\
								PRD_SYS_EN_LPWG_SD	|	\
								0),
};

static const struct siw_hal_prd_param prd_params[] = {
	/*
	 * LG4894 group
	 */
	{	.chip_type = CHIP_LG4894,
		.name = prd_param_name_lg4894_k,
		.cmd_type = PRD_CMD_TYPE_2,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA8C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xC0F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_ODD, 0xCD2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xD95),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xE83),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xBCF),
			0,
		},
		__PRD_PARAM_DIMENSION(26, 15, 1, 32, PRD_M1_COL_SIZE, 2, 2),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4894,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	{	.chip_type = CHIP_LG4894,
		.name = prd_param_name_lg4894_lv,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA8C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xC0F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_ODD, 0xCD2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xD95),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xE83),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xBCF),
			0,
		},
		__PRD_PARAM_DIMENSION(26, 15, 1, 32, PRD_M1_COL_SIZE, 2, 2),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4894,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	{	.chip_type = CHIP_LG4894,
		.name = prd_param_name_lg4894_sf,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA02),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xB22),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xC42),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xD96),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = (PRD_SYS_EN_DEBUG_BUF|PRD_SYS_EN_APP_DEBUG_BUF),
		.sd_test_flag = SD_FLAG_LG4894,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	{	.chip_type = CHIP_LG4894,
		.name = prd_param_name_lg4894_cv,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xC0F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCD2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xD95),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xE83),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xA8C),
			0,
		},
		__PRD_PARAM_DIMENSION(26, 15, 1, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = (SD_FLAG_LG4894 & ~U3_BLU_JITTER_TEST_FLAG),
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	{	.chip_type = CHIP_LG4894,
		.name = prd_param_name_lg4894_t,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA02),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xB22),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xC42),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xD96),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0x9CE),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = (SD_FLAG_LG4894 & ~U3_BLU_JITTER_TEST_FLAG),
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	{	.chip_type = CHIP_LG4894,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA02),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xB22),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xC42),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xD96),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = (PRD_SYS_EN_DEBUG_BUF|PRD_SYS_EN_APP_DEBUG_BUF),
		.sd_test_flag = SD_FLAG_LG4894,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	/*
	 * LG4895 group
	 */
	{	.chip_type = CHIP_LG4895,
		.name = prd_param_name_lg4895_k,
		.cmd_type = PRD_CMD_TYPE_2,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xD1C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xE4E),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xF80),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x10E8),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_F_DELTA, 0x7FD),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(34, 18, 0, 32+2, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(1, 4),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4895,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4895,
	},
	{	.chip_type = CHIP_LG4895,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xD1C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xE4E),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xF80),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x10E8),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_F_DELTA, 0x7FD),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(34, 18, 0, 32+2, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(1, 4),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4895,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4895,
	},
	/*
	 * LG4946 group
	 */
	{	.chip_type = CHIP_LG4946,
		.name = prd_param_name_lg4946_g,
		.cmd_type = PRD_CMD_TYPE_2,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB82),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF16),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_F_DELTA, 0x7FD),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4946,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4946,
	},
	{	.chip_type = CHIP_LG4946,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB82),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF16),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4946,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4946,
	},
	/*
	 * SW46104 group (Not fixed)
	 */
	{	.chip_type = CHIP_SW46104,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0x1182),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0x12A2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0x13C2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x1516),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0x10FC),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = 0,			/* temorary disabled */
		.lpwg_sd_test_flag = 0,		/* temorary disabled */
	},
	/*
	 * SW49105 group (Not fixed)
	 */
	{	.chip_type = CHIP_SW49105,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB85),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF19),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xB0F),
			0,
		},
		__PRD_PARAM_DIMENSION(30, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49105,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49105,
	},
	/*
	 * SW49106 group (Not fixed)
	 */
	{	.chip_type = CHIP_SW49106,
		.name = prd_param_name_sw49106_type_2,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xD82),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xEA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xFC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x1116),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xC3F),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 16, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = (SD_FLAG_SW49106 & ~SHORT_NODE_TEST_FLAG) |	\
						SHORT_FULL_TEST_FLAG,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49106,
	},
	{	.chip_type = CHIP_SW49106,
		.name = prd_param_name_sw49106_type_1,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xD82),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xEA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xFC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x1116),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xC3F),
			0,
		},
		__PRD_PARAM_DIMENSION(30, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49106,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49106,
	},
	{	.chip_type = CHIP_SW49106,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xF88),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0x104C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0x1110),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x1200),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xC3F),
			0,
		},
		__PRD_PARAM_DIMENSION(28, 14, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49106,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49106,
	},
	/*
	 * SW49406 group (Not fixed)
	 */
	{	.chip_type = CHIP_SW49406,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB82),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF16),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49406,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49406,
	},
	/*
	 * SW49407 group
	 */
	{	.chip_type = CHIP_SW49407,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xF1C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0x104E),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0x1180),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x12E8),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xCCF),
			0,
		},
		__PRD_PARAM_DIMENSION(32+2, 18, 0, 32+2, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(1, 4),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49407,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49407,
	},
	/*
	 * SW49408 group (Not fixed)
	 */
	{	.chip_type = CHIP_SW49408,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB85),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF19),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xB0F),
			0,
		},
		__PRD_PARAM_DIMENSION(30, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49408,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49408,
	},
	/*
	 * SW49501 group
	 */
	{	.chip_type = CHIP_SW49501,
		.name = prd_param_name_sw49501_type_1,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0x1982),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0x1AA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0x1BC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x1D16),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0x14C8),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = PRD_SYS_EN_LPWG_SD,
		.sd_test_flag = SD_FLAG_SW49501 |	\
						U3_M1_RAWDATA_TEST_FLAG |	\
						U3_M1_JITTER_TEST_FLAG |	\
						SHORT_FULL_TEST_FLAG |	\
						IRQ_TEST_FLAG |	\
						OPEN_SHORT_RESULT_DATA_FLAG |	\
						OPEN_SHORT_RESULT_ALWAYS_FLAG,
		.lpwg_sd_test_flag = 0,
	},
	{	.chip_type = CHIP_SW49501,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0x1982),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0x1AA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0x1BC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x1D16),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0x14C8),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = PRD_SYS_EN_LPWG_SD,
		.sd_test_flag = SD_FLAG_SW49501,
		.lpwg_sd_test_flag = 0,
	},
	/*
	 * SW42000A group
	 */
	{	.chip_type = CHIP_SW42000A,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, (0x6C2C>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, (0x726C>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, (0x78AC>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, (0x7FE4>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, (0x65EC>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW_S, (0x634C>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_S, (0x63C4>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA_S, (0x643C>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL_S, (0x64BC>>2)),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG_S, (0x62D4>>2)),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 15, 0, 40, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = SYS_OFF_FLAG_SW42000A,
		.sd_test_flag = SD_FLAG_SW42000A,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW42000A,
		.buf_swap_flag = SWAP_FLAG_SW42000A,
	},
	/*
	 * SW1828 group
	 */
	{	.chip_type = CHIP_SW1828,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xACF),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xC0F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xD4F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xEC5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_F_DELTA, 0x7FD),
			0,
		},
		__PRD_PARAM_DIMENSION(20, 32, 0, 48, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = SYS_OFF_FLAG_SW1828,
		.sd_test_flag = SD_FLAG_SW1828,
		.lpwg_sd_test_flag = 0,
	},
	/*
	 * End
	 */
	{ 0, },
};

#define __RESULT_FLAG_CAL(_result_all, result, flag)	\
		((_result_all) | ((result) << (flag)))

#define RAW_OFFSET_EVEN(_addr)		(((_addr) >> 16) & 0xFFFF)
#define RAW_OFFSET_ODD(_addr)		((_addr) & 0xFFFF)

enum {
	CMD_TEST_EXIT = 0,
	CMD_TEST_ENTER,
};

enum {
	CMD_RAWDATA_PRD = 1,
	CMD_RAWDATA_TCM,
	CMD_RAWDATA_AIT,
	CMD_AIT_BASEDATA,
	CMD_FILTERED_DELTADATA,
	CMD_DELTADATA,
	CMD_LABELDATA,
	CMD_BLU_JITTER,
	CMD_DEBUG_BUF,
	CMD_MAX,
};

#define __PRD_GET_DATA_CMD_SET(_idx)	[_idx] = #_idx

static const char *prd_get_data_cmd_name[] = {
	[0] = "(none)",
	__PRD_GET_DATA_CMD_SET(CMD_RAWDATA_PRD),
	__PRD_GET_DATA_CMD_SET(CMD_RAWDATA_TCM),
	__PRD_GET_DATA_CMD_SET(CMD_RAWDATA_AIT),
	__PRD_GET_DATA_CMD_SET(CMD_AIT_BASEDATA),
	__PRD_GET_DATA_CMD_SET(CMD_FILTERED_DELTADATA),
	__PRD_GET_DATA_CMD_SET(CMD_DELTADATA),
	__PRD_GET_DATA_CMD_SET(CMD_LABELDATA),
	__PRD_GET_DATA_CMD_SET(CMD_BLU_JITTER),
	__PRD_GET_DATA_CMD_SET(CMD_DEBUG_BUF),
};

static const char *prd_cmp_tool_str[UX_INVALID][2] = {
	[U3_M2_RAWDATA_TEST]	= { "U3_M2_Lower", "U3_M2_Upper", },
	[U3_M1_RAWDATA_TEST]	= { "U3_M1_Lower", "U3_M1_Upper", },
	[U0_M2_RAWDATA_TEST]	= { "U0_M2_Lower", "U0_M2_Upper", },
	[U0_M1_RAWDATA_TEST]	= { "U0_M1_Lower", "U0_M1_Upper", },
	[U3_BLU_JITTER_TEST]	= { "U3_Blu_Jitter_Lower", "U3_Blu_Jitter_Upper", },
	[U3_JITTER_TEST]		= { "U3_Jitter_Lower", "U3_Jitter_Upper", },
	[U3_M1_JITTER_TEST]		= { "U3_M1_Jitter_Lower", "U3_M1_Jitter_Upper", },
	[U0_JITTER_TEST]		= { "U0_Jitter_Lower", "U0_Jitter_Upper", },
	[U0_M1_JITTER_TEST]		= { "U0_M1_Jitter_Lower", "U0_M1_Jitter_Upper", },
};

static inline const char *prd_get_cmp_tool_str(int type, int upper)
{
	return (type < UX_INVALID) ? prd_cmp_tool_str[type][!!upper] : NULL;
}

enum{
	M1_EVEN_DATA = 0,
	M1_ODD_DATA,
	M2_EVEN_DATA,
	M2_ODD_DATA ,
	LABEL_DATA,
	DEBUG_DATA,
	};

/* PRD RAWDATA test RESULT Save On/Off CMD */
enum {
	RESULT_OFF = 0,
	RESULT_ON,
};

/* TCM Memory Access Select */
//tc_mem_sel(0x0457) RAW : 0 , BASE1 : 1 , BASE2 : 2 , BASE3: 3
enum {
	TCM_MEM_RAW = 0,
	TCM_MEM_BASE1,
	TCM_MEM_BASE2,
	TCM_MEM_BASE3,
};

/* AIT Algorithm Engine HandShake CMD */
enum {
	IT_IMAGE_NONE = 0,
	IT_IMAGE_RAW,
	IT_IMAGE_BASELINE,
	IT_IMAGE_DELTA,
	IT_IMAGE_LABEL,
	IT_IMAGE_FILTERED_DELTA,
	IT_IMAGE_RESERVED,
	IT_IMAGE_DEBUG,
	IT_DONT_USE_CMD = 0xEE,
	IT_WAIT = 0xFF,
};

/* AIT Algorithm Engine HandShake Status */
enum {
	RS_READY    = 0xA0,
	RS_NONE     = 0x05,
	RS_LOG      = 0x77,
	RS_IMAGE	= 0xAA
};

enum {
	PRD_TIME_STR_SZ = 64,
	PRD_TMP_FILE_NAME_SZ = PATH_MAX,
};

enum {
	PRD_TUNE_DISPLAY_TYPE_1 = 1,
	PRD_TUNE_DISPLAY_TYPE_2,
};

static char __prd_in_file[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_in.txt";
static char __prd_in_file_m[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_in_mfts.txt";
static char __prd_out_file[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out.txt";
static char __prd_out_file_mo_aat[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out_mfts_aat.txt";
static char __prd_out_file_mo_mfo[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out_mfts_mfo.txt";
static char __prd_out_file_mo_mfl[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out_mfts_mfl.txt";
static char __prd_out_file_mo_mcv[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out_mfts_mcv.txt";

#if defined(MODULE)
#define __PRD_FILE_DEFAULT_PERM		(S_IRUGO | S_IWUSR | S_IWGRP)

/* use eg. prd_if=arc1 to change name */
module_param_string(prd_if, __prd_in_file, sizeof(__prd_in_file), __PRD_FILE_DEFAULT_PERM);

/* use eg. prd_if_m=arc1 to change name */
module_param_string(prd_if_m, __prd_in_file_m, sizeof(__prd_in_file_m), __PRD_FILE_DEFAULT_PERM);

/* use eg. prd_of=arc1 to change name */
module_param_string(prd_of, __prd_out_file, sizeof(__prd_out_file), __PRD_FILE_DEFAULT_PERM);

/* use eg. prd_of_mo_aat=arc1 to change name */
module_param_string(prd_of_mo_aat, __prd_out_file_mo_aat, sizeof(__prd_out_file_mo_aat), __PRD_FILE_DEFAULT_PERM);

/* use eg. prd_of_mo_mfo=arc1 to change name */
module_param_string(prd_of_mo_mfo, __prd_out_file_mo_mfo, sizeof(__prd_out_file_mo_mfo), __PRD_FILE_DEFAULT_PERM);

/* use eg. prd_of_mo_mfl=arc1 to change name */
module_param_string(prd_of_mo_mfl, __prd_out_file_mo_mfl, sizeof(__prd_out_file_mo_mfl), __PRD_FILE_DEFAULT_PERM);

/* use eg. prd_of_mo_mcv=arc1 to change name */
module_param_string(prd_of_mo_mcv, __prd_out_file_mo_mcv, sizeof(__prd_out_file_mo_mcv), __PRD_FILE_DEFAULT_PERM);
#endif


static u32 t_prd_dbg_flag = 0;

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/s_prd_dbg_flag
 * (2) insmod {Siw Touch Module Name}.ko s_prd_dbg_flag=<value>
 */
module_param_named(s_prd_dbg_flag, t_prd_dbg_flag, int, S_IRUGO|S_IWUSR|S_IWGRP);

enum {
	PRD_DBG_FLAG_RAW_CMP			= (1<<7),
	/* */
	PRD_DBG_FLAG_RAW_LOG_OFF		= (1<<16),
	/* */
	PRD_DBG_FLAG_DISABLE			= (1<<30),
};

static u32 t_prd_dbg_mask = 0;

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/s_prd_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko s_prd_dbg_mask=<value>
 */
module_param_named(s_prd_dbg_mask, t_prd_dbg_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);


enum {
	PRD_SHOW_FLAG_DISABLE_PRT_RAW	= (1<<0),
};

#define SIW_PRD_TAG 		"prd: "
#define SIW_PRD_TAG_ERR 	"prd(E): "
#define SIW_PRD_TAG_WARN	"prd(W): "
#define SIW_PRD_TAG_DBG		"prd(D): "

#define t_prd_info(_prd, fmt, args...)	\
		__t_dev_info(_prd->dev, SIW_PRD_TAG fmt, ##args)

#define t_prd_err(_prd, fmt, args...)	\
		__t_dev_err(_prd->dev, SIW_PRD_TAG_ERR fmt, ##args)

#define t_prd_warn(_prd, fmt, args...)	\
		__t_dev_warn(_prd->dev, SIW_PRD_TAG_WARN fmt, ##args)

#define t_prd_dbg(condition, _prd, fmt, args...)	\
		do {	\
			if (unlikely(t_prd_dbg_mask & (condition)))	\
				__t_dev_info(_prd->dev, SIW_PRD_TAG_DBG fmt, ##args);	\
		} while (0)

#define t_prd_dbg_base(_prd, fmt, args...)	\
		t_prd_dbg(DBG_BASE, _prd, fmt, ##args)

#define t_prd_dbg_trace(_prd, fmt, args...)	\
		t_prd_dbg(DBG_TRACE, _prd, fmt, ##args)

#define t_prd_info_flag(_prd, _flag, fmt, args...)	\
		do { \
			if (!(_flag & PRD_SHOW_FLAG_DISABLE_PRT_RAW)){ \
				t_prd_info(_prd, fmt, ##args); \
			} \
		} while(0)

#define t_prd_dbg_base_flag(_prd, _flag, fmt, args...)	\
		do { \
			if (!(_flag & PRD_SHOW_FLAG_DISABLE_PRT_RAW)){ \
				t_prd_dbg_base(_prd, fmt, ##args); \
			} \
		} while(0)

#define siw_prd_sysfs_err_invalid_param(_prd)	\
		t_prd_err(_prd, "Invalid param\n");

#define siw_prd_buf_snprintf(_buf, _size, _fmt, _args...) \
		__siw_snprintf(_buf, PRD_BUF_SIZE, _size, _fmt, ##_args);

#define siw_prd_log_buf_snprintf(_buf, _size, _fmt, _args...) \
		__siw_snprintf(_buf, PRD_LOG_BUF_SIZE, _size, _fmt, ##_args);

#define siw_prd_log_info(_prd, _buf, _size)	\
		do {	\
			if (!(t_prd_dbg_flag & PRD_DBG_FLAG_RAW_LOG_OFF)) {	\
				if (_size)	t_prd_info(_prd, "%s\n", _buf);	\
			}	\
		} while (0)

#define siw_snprintf_sd_result(_buf, _size, _item, _ret) \
		siw_snprintf(_buf, _size, "%s : %s\n", _item, (_ret) ? "Fail" : "Pass")

#define siw_prd_log_flush(_prd, _log_buf, _buf, _log_size, _size)	\
		({	int __size = 0;	\
			siw_prd_log_info(_prd, _log_buf, _log_size);	\
			__size = siw_prd_buf_snprintf(_buf, _size, "%s\n", _log_buf);	\
			__size;	\
		})

#define siw_prd_log_end(_prd, _log_buf, _log_size)	\
		({	int __size = 0;	\
			siw_prd_log_info(_prd, _log_buf, _log_size);	\
			__size = siw_prd_log_buf_snprintf(_log_buf, _log_size, "\n");	\
			__size;	\
		})

#define __PRD_LOG_VIA_SHELL

static int prd_drv_exception_check(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (atomic_read(&chip->boot) == IC_BOOT_FAIL) {
		t_prd_warn(prd, "boot failed\n");
		return 1;
	}

	if (atomic_read(&chip->init) != IC_INIT_DONE) {
		t_prd_warn(prd, "not ready, need IC init\n");
		return 1;
	}

	if (atomic_read(&ts->state.sleep) != IC_NORMAL) {
		t_prd_warn(prd, "not IC normal\n");
		return 1;
	}

	return 0;
}

static int prd_ic_exception_check(struct siw_hal_prd_data *prd, char *buf)
{
	struct device *dev = prd->dev;
	int boot_mode = 0;

	boot_mode = siw_touch_boot_mode_check(dev);

	return boot_mode;
}

static int prd_chip_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret;

//	mutex_lock(&ts->lock);
	ret = siw_ops_reset(ts, HW_RESET_SYNC);
//	mutex_unlock(&ts->lock);

	return ret;
}

static int prd_chip_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	return siw_ops_ic_info(ts);
}

static int prd_chip_driving(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	return siw_ops_tc_driving(ts, mode);
}

static ssize_t prd_kernel_write(struct file *file,
				const char *buf, size_t count,
			    loff_t *pos)
{
	/*mm_segment_t old_fs;*/
	ssize_t res;

	/*old_fs = get_fs();*/
	/*set_fs(get_ds());*/
	/* The cast to a user pointer is valid due to the set_fs() */
	res = kernel_write(file, (__force const char __user *)buf, count, pos);
	//set_fs(old_fs);

	return res;
}

static int __prd_boot_mode_is_err(struct device *dev, int boot_mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	if ((boot_mode > MINIOS_MFTS_CURVED) || (boot_mode < NORMAL_BOOT)) {
		t_prd_err(prd, "boot mode(%d) not support mode\n", boot_mode);
		return -EINVAL;
	}
	return 0;
}

static int __used prd_chk_file_mode(struct siw_hal_prd_data *prd,
						struct file *filp,
						char *fname)
{
	struct dentry *fentry = filp->f_path.dentry;
	unsigned int accmode = filp->f_flags & O_ACCMODE;

	if (accmode & (O_RDWR|O_WRONLY)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0))
		if (!(filp->f_mode & FMODE_CAN_WRITE)) {
			t_prd_err(prd, "file not writeable : %s\n", fname);
			return -EINVAL;
		}
#endif
	}

	if ((accmode == O_RDONLY) || (accmode & O_RDWR)){
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0))
		if (!(filp->f_mode & FMODE_CAN_READ)) {
			t_prd_err(prd, "file not readable : %s\n", fname);
			return -EINVAL;
		}
#endif

		if (S_ISDIR(fentry->d_inode->i_mode)) {
			t_prd_err(prd, "%s is folder\n", fname);
			return -EOPNOTSUPP;
		}

		if (S_ISLNK(fentry->d_inode->i_mode)) {
			t_prd_err(prd, "%s is link\n", fname);
			return -EOPNOTSUPP;
		}
	}
	return 0;
}

static struct file __used *prd_vfs_file_open(struct siw_hal_prd_data *prd,
				char *fname, int flags, umode_t mode)
{
	struct file *filp;
	unsigned int accmode = mode & O_ACCMODE;

	if (accmode & (O_WRONLY|O_RDWR)) {
		if (prd->file_wr_off) {
			t_prd_warn(prd, "file writing not permitted\n");
			return NULL;
		}
	}

	if ((accmode == O_RDONLY) || (accmode & O_RDWR)) {
		if (prd->file_rd_off) {
			t_prd_warn(prd, "file reading not permitted\n");
			return NULL;
		}
	}

	filp = filp_open((const char *)fname, flags, mode);
	if (IS_ERR(filp)) {
		int ret = (int)PTR_ERR(filp);
		t_prd_err(prd, "file[%s, 0x%X, 0x%X] open failed, %d\n",
				fname, flags, mode, ret);
		return NULL;
	}

	return filp;
}

static int __used prd_vfs_file_close(struct file *filp, fl_owner_t id)
{
	return filp_close(filp, id);
}

static int __used prd_vfs_file_chk(struct siw_hal_prd_data *prd,
				char *fname,
				int flags,
				umode_t mode,
				loff_t *size)
{
	struct file *filp;
	int ret = 0;

	filp = prd_vfs_file_open(prd, fname, flags, mode);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	if (size) {
		*size = vfs_llseek(filp, 0, SEEK_END);
	}

	ret = prd_chk_file_mode(prd, filp, fname);
	if (ret < 0) {
		goto out_close;
	}

out_close:
	prd_vfs_file_close(filp, NULL);

out:
	return ret;
}


/*
 * [Safety and Compatibility]
 * Using call_usermodehelper is recommended
 * because direct access to file system in kernel space is not good way.
 */
#define __PRD_USE_USERMODE_HELPER

static inline char *prd_find_name_pos(char *pathname)
{
	char *pos, *tpos;

	pos = strrchr(pathname, '/');
	tpos = (pos) ? pos + 1 : pathname;

	return tpos;
}

#if defined(__PRD_USE_USERMODE_HELPER)
#define t_prd_err_uh(_prd, _ret, fmt, args...)	\
		t_prd_err(_prd, "user mode helper" fmt " failed, %d\n",	\
				##args, _ret)

#define t_prd_dbg_uh(_prd, fmt, args...)	\
		t_prd_dbg_trace(_prd, "user mode helper" fmt " done\n", ##args)

static int prd_vfs_uh_remove(struct siw_hal_prd_data *prd,
				char *pathname)
{
	char *argv[4] = { "/bin/rm", NULL, NULL, NULL };
	char *envp[1] = { NULL };
	int ret = 0;

	argv[1] = (char *)pathname;

//	envp[0] = "HOME=/";
//	envp[1] = "PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin";

	ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
	if (ret < 0) {
		t_prd_err_uh(prd, ret, "[%s %s]", argv[0], argv[1]);
		return ret;
	}

	t_prd_dbg_uh(prd, "[%s %s]", argv[0], argv[1]);

	return ret;
}

static int prd_vfs_uh_rename(struct siw_hal_prd_data *prd,
				char *pathname,
				char *pathname_new)
{
	char *argv[4] = { "/bin/mv", NULL, NULL, NULL };
	char *envp[1] = { NULL };
	int ret = 0;

	argv[1] = (char *)pathname;
	argv[2] = (char *)pathname_new;

//	envp[0] = "HOME=/";
//	envp[1] = "PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin";

	ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
	if (ret < 0) {
		t_prd_err_uh(prd, ret, "[%s %s %s]", argv[0], argv[1], argv[2]);
		return ret;
	}

	t_prd_dbg_uh(prd, "[%s %s %s]", argv[0], argv[1], argv[2]);

	return ret;
}

static int prd_vfs_remove(struct siw_hal_prd_data *prd,
				char *fname)
{
	struct file *filp = NULL;
	char *tmp = NULL;
	char *pathname = NULL;
	int ret = 0;

	tmp = (char *)kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp) {
		return -ENOMEM;
	}

	/*
	 * Step 1. Get fentry data
	 */
	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	pathname = d_path(&filp->f_path, tmp, PAGE_SIZE);
	if (IS_ERR(pathname)) {
		ret = -ENOMEM;
		goto out;
	}

	prd_vfs_file_close(filp, NULL);

	/*
	 * Step 2. Remove target file
	 */
	ret = prd_vfs_uh_remove(prd, pathname);

out:
	kfree(tmp);
	return ret;
}

static int prd_vfs_rename(struct siw_hal_prd_data *prd,
					char *fname,
					char *fname_new)
{
	struct file *filp = NULL;
	char *tmp = NULL;
	char *tmp_new = NULL;
	char *pathname = NULL;
	char *pathname_new = NULL;
	char *wpos, *rpos;
	int ret = 0;

	tmp = (char *)kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp) {
		return -ENOMEM;
	}

	tmp_new = (char *)kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp_new) {
		kfree(tmp);
		return -ENOMEM;
	}

	/*
	 * Step 1. Get fentry data
	 */
	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	pathname = d_path(&filp->f_path, tmp, PAGE_SIZE);
	if (IS_ERR(pathname)) {
		ret = -ENOMEM;
		goto out;
	}

	prd_vfs_file_close(filp, NULL);

	/*
	 * Step 2. pathname_new
	 */
	pathname_new = tmp_new;
	strncpy(pathname_new, pathname, strlen(pathname));

	wpos = prd_find_name_pos(pathname_new);
	rpos = prd_find_name_pos(fname_new);

	memcpy(wpos, rpos, strlen(rpos));

	/*
	 * Step 3. Change name
	 */
	ret = prd_vfs_uh_rename(prd, pathname, pathname_new);

out:
	kfree(tmp_new);
	kfree(tmp);
	return ret;
}
#else	/* __PRD_USE_USERMODE_HELPER */
static int prd_vfs_do_remove(struct siw_hal_prd_data *prd,
				struct dentry *fentry,
				char *fname)
{
	struct dentry *dir = NULL;
	int ret = 0;

	dir = dget_parent(fentry);

	mutex_lock(&dir->d_inode->i_mutex);

	spin_lock(&fentry->d_lock);

	t_prd_dbg_trace(prd, "unhashed 0x%X, d_inode %p\n",
			d_unhashed(fentry), fentry->d_inode);

	if (d_unhashed(fentry) && fentry->d_inode) {
		spin_unlock(&fentry->d_lock);
		goto out;
	}

	dget_dlock(fentry);
	__d_drop(fentry);
	spin_unlock(&fentry->d_lock);

	vfs_unlink(dir->d_inode, fentry);

	t_prd_dbg_trace(prd, "vfs_unlink done\n");

out:
	mutex_unlock(&dir->d_inode->i_mutex);
	return ret;
}

static int prd_vfs_do_rename(struct siw_hal_prd_data *prd,
				struct dentry *fentry,
				char *fname_new)
{
	struct dentry *dir = NULL;
	struct inode *d_inode = NULL;
	struct dentry *fentry_new = NULL;
	char *base_name = NULL;
	int ret = 0;

	dir = dget_parent(fentry);
	d_inode = dir->d_inode;

	lock_rename(dir, dir);

	base_name = prd_find_name_pos(fname_new);
	t_prd_dbg_trace(prd, "base name : %s\n", base_name);

	fentry_new = lookup_one_len(base_name, dir, strlen(base_name));
	if (IS_ERR(fentry_new)) {
		ret = PTR_ERR(fentry_new);
		t_prd_err(prd, "lookup_one_len for fentry_new failed, %d\n", ret);
		goto out;
	}

	ret = vfs_rename(d_inode, fentry, d_inode, fentry_new);

out:
	unlock_rename(dir, dir);

	return ret;
}

static int prd_vfs_remove(struct siw_hal_prd_data *prd,
				char *fname)
{
	struct file *filp = NULL;
	struct dentry *fentry = NULL;
	int ret = 0;

	/*
	 * Step 1. Get fentry data
	 */
	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		return -ENOENT;
	}

	fentry = filp->f_path.dentry;

	prd_vfs_file_close(filp, NULL);

	/*
	 * Step 2. Remove target file
	 */
	ret = prd_vfs_do_remove(prd, fentry, fname);

	return ret;
}

static int prd_vfs_rename(struct siw_hal_prd_data *prd,
					char *fname,
					char *fname_new)
{
	struct file *filp = NULL;
	struct dentry *fentry = NULL;
	int ret = 0;

	/*
	 * Step 1. Get fentry data
	 */
	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		return -ENOENT;
	}

	fentry = filp->f_path.dentry;

	prd_vfs_file_close(filp, NULL);

	/*
	 * Step 2. Change name
	 */
	ret = prd_vfs_do_rename(prd, fentry, fname_new);

	return ret;
}

#endif	/* __PRD_USE_USERMODE_HELPER */


#define t_prd_dbg_file_modify(_prd, fmt, args...)	\
		t_prd_dbg_trace(_prd, "try to " fmt "\n", ##args)

#define t_prd_dbg_file_remove(_prd, fmt, args...)	\
		t_prd_dbg_file_modify(_prd, "remove " fmt, ##args)

#define t_prd_dbg_file_rename(_prd, fmt, args...)	\
		t_prd_dbg_file_modify(_prd, "rename " fmt, ##args)

static int __used prd_vfs_file_remove(struct siw_hal_prd_data *prd,
				char *fname)
{
	int ret = 0;

	t_prd_dbg_file_remove(prd, "%s", fname);

	ret = prd_vfs_remove(prd, fname);
	if (ret < 0) {
		if (ret != -ENOENT) {
			t_prd_err(prd, "failed to remove file : %s, %d\n", fname, ret);
		}
		goto out;
	}
	t_prd_info(prd, "file removed : %s\n", fname);

out:
	return 0;
}

static int __used prd_vfs_file_rename(struct siw_hal_prd_data *prd,
				char *fname, char *fname_new)
{
	int ret = 0;

	t_prd_dbg_file_rename(prd, "%s -> %s", fname, fname_new);

	ret = prd_vfs_rename(prd, fname, fname_new);
	if (ret < 0) {
		if (ret != -ENOENT) {
			t_prd_err(prd, "failed to rename file : %s -> %s, %d\n",
					fname, fname_new, ret);
		}
		goto out;
	}
	t_prd_info(prd, "file renamed : %s -> %s\n",
			fname, fname_new);

out:
	return 0;
}

static int prd_check_type(struct siw_hal_prd_data *prd,
			int type, int index)
{
	int ret = 0;

	switch (type) {
	case U3_M2_RAWDATA_TEST:
	case U0_M2_RAWDATA_TEST:
	case U3_BLU_JITTER_TEST:
	case U3_JITTER_TEST:
	case U0_JITTER_TEST:
		break;

	case U3_M1_RAWDATA_TEST:
	case U0_M1_RAWDATA_TEST:
	case U3_M1_JITTER_TEST:
	case U0_M1_JITTER_TEST:
		ret = 1;
		break;

	default:
		t_prd_err(prd, "[%d] unsupported test mode, %d\n",
			index, type);
		ret = -EINVAL;
	}

	return ret;
}

static int prd_log_file_size_check(struct siw_hal_prd_data *prd)
{
	char *prd_out_fname[] = {
		[NORMAL_BOOT]			= __prd_out_file,
		[MINIOS_AAT]			= __prd_out_file_mo_aat,
		[MINIOS_MFTS_FOLDER]	= __prd_out_file_mo_mfo,
		[MINIOS_MFTS_FLAT]		= __prd_out_file_mo_mfl,
		[MINIOS_MFTS_CURVED]	= __prd_out_file_mo_mcv,
	};
	struct device *dev = prd->dev;
	char *fname = NULL;
	loff_t file_size = 0;
	int i = 0;
	char *buf1 = NULL;
	char *buf2 = NULL;
	int boot_mode = 0;
	int ret = 0;

	boot_mode = siw_touch_boot_mode_check(dev);

	if (__prd_boot_mode_is_err(dev, boot_mode)) {
		return -EINVAL;
	}

	buf1 = touch_getname();
	if (buf1 == NULL) {
		t_prd_err(prd, "failed to allocate name buffer 1\n");
		return -ENOMEM;
	}

	buf2 = touch_getname();
	if (buf2 == NULL) {
		t_prd_err(prd, "failed to allocate name buffer 2\n");
		touch_putname(buf1);
		return -ENOMEM;
	}

	fname = (char *)prd_out_fname[boot_mode];

	ret = prd_vfs_file_chk(prd, fname, O_RDONLY, 0666, &file_size);
	if (ret < 0) {
		goto out;
	}

	if (file_size <= MAX_LOG_FILE_SIZE) {
		goto out;
	}

	t_prd_info(prd, "file size : %s, %lld > MAX_LOG_FILE_SIZE(%d)\n",
			fname, file_size, MAX_LOG_FILE_SIZE);

	for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
		if (i == 0) {
			snprintf(buf1, PATH_MAX, "%s", fname);
		} else {
			snprintf(buf1, PATH_MAX, "%s.%d", fname, i);
		}

		if (i == (MAX_LOG_FILE_COUNT - 1)) {
			ret = prd_vfs_file_remove(prd, buf1);
			if (ret < 0) {
				goto out;
			}
			t_prd_info(prd, "file removed : %s\n", buf1);
		} else {
			snprintf(buf2, PATH_MAX, "%s.%d", fname, (i + 1));

			ret = prd_vfs_file_rename(prd, buf1, buf2);
			if (ret < 0) {
				goto out;
			}
		}
	}

out:
	touch_putname(buf2);
	touch_putname(buf1);

	return ret;
}

static int prd_do_write_file(struct siw_hal_prd_data *prd,
				char *fname,
				char *data,
				int write_time)
{
//	struct device *dev = prd->dev;
	struct file *filp = NULL;
	loff_t offset = 0;
	char time_string[PRD_TIME_STR_SZ] = {0, };
	int twlen = 0;
	int nwlen = 0;
	int ret = 0;

	filp = prd_vfs_file_open(prd, fname,
				O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	ret = prd_chk_file_mode(prd, filp, fname);
	if (ret < 0) {
		goto out_close;
	}

	ret = prd_kernel_write(filp, (const char *)data,
				strlen(data), &offset);
	if (ret < 0) {
		t_prd_err(prd, "failed to write file : %s(%d), %d \n",
				fname, (int)offset, ret);
		goto out_close;
	}
	twlen += strlen(data);
	nwlen += ret;

out_close:
	prd_vfs_file_close(filp, NULL);
	if (ret >= 0) {
		t_prd_info(prd, "file write done : %s(%d, %d)\n",
				fname, twlen, nwlen);
	}

out:
	return ret;
}

static int prd_write_file(struct siw_hal_prd_data *prd, char *data, int write_time)
{
	char *prd_out_fname[] = {
		[NORMAL_BOOT]			= __prd_out_file,
		[MINIOS_AAT]			= __prd_out_file_mo_aat,
		[MINIOS_MFTS_FOLDER]	= __prd_out_file_mo_mfo,
		[MINIOS_MFTS_FLAT]		= __prd_out_file_mo_mfl,
		[MINIOS_MFTS_CURVED]	= __prd_out_file_mo_mcv,
	};
	struct device *dev = prd->dev;
	char *fname = NULL;
	int boot_mode = 0;
	int ret = 0;

	boot_mode = siw_touch_boot_mode_check(dev);
	if (__prd_boot_mode_is_err(dev, boot_mode)) {
		return -EINVAL;
	}

	fname = (char *)prd_out_fname[boot_mode];

	ret = prd_do_write_file(prd, fname, data, write_time);

	return ret;
}

/*
 * Conrtol LCD Backlightness
 */
static int prd_set_blu(struct device *dev)
{
	int backlightness;

//	LCD brightness ON 742ms -> LCD brightness OFF 278mms -> LCD brightness ON 278ms

	backlightness = siw_touch_sys_get_panel_bl(dev);

	touch_msleep(742);

	siw_touch_sys_set_panel_bl(dev, 0);

	touch_msleep(278);

	siw_touch_sys_set_panel_bl(dev, backlightness);

	touch_msleep(278);

	return 0;
}

#define __WRITE_CMD_PARAM(_t, _m, _w)	\
		{ .type = _t, .mode = _m, .waiting = _w }

static int prd_write_test_mode(struct siw_hal_prd_data *prd, int type)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_sd_cmd *sd_cmd = &prd->sd_cmd;
	struct write_cmd_list {
		int type;
		u32 mode;
		int waiting;
	} *table, write_cmd_tables[] = {
		__WRITE_CMD_PARAM(U3_M2_RAWDATA_TEST,
			(U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_m2_rawdata, 0),
		__WRITE_CMD_PARAM(U3_M1_RAWDATA_TEST,
			(U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_m1_rawdata, 0),
		__WRITE_CMD_PARAM(U0_M2_RAWDATA_TEST,
			(U0_TEST_PRE_CMD << 8) + sd_cmd->cmd_m2_rawdata, 0),
		__WRITE_CMD_PARAM(U0_M1_RAWDATA_TEST,
			(U0_TEST_PRE_CMD << 8) + sd_cmd->cmd_m1_rawdata, 0),
		__WRITE_CMD_PARAM(OPEN_NODE_TEST,
			(U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_open_node, 10),
		__WRITE_CMD_PARAM(SHORT_NODE_TEST,
			(U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_short_node, 1000),
		__WRITE_CMD_PARAM(SHORT_FULL_TEST,
			(U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_short_full, 1000),
		__WRITE_CMD_PARAM(U3_BLU_JITTER_TEST,
			((U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_jitter) | LINE_FILTER_OPTION, 10),
		__WRITE_CMD_PARAM(U3_JITTER_TEST,
			((U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_jitter) | LINE_FILTER_OPTION, 0),
		__WRITE_CMD_PARAM(U3_M1_JITTER_TEST,
			((U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_m1_jitter) | LINE_FILTER_OPTION, 0),
		__WRITE_CMD_PARAM(U0_JITTER_TEST,
			((U0_TEST_PRE_CMD << 8) + sd_cmd->cmd_jitter) | LINE_FILTER_OPTION, 0),
		__WRITE_CMD_PARAM(U0_M1_JITTER_TEST,
			((U0_TEST_PRE_CMD << 8) + sd_cmd->cmd_m1_jitter) | LINE_FILTER_OPTION, 0),
		/* */
		__WRITE_CMD_PARAM(-1, 0, 0),
	};
	u32 testmode = 0;
	int retry = 40;
	u32 rdata = 0x01;
	u32 line_filter_option = 1;
	int waiting_time = 200;
	int waiting_time_quirk = 0;
	int addr = reg->tc_tsp_test_ctl;
	int ret = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_SW1828:
	case CHIP_SW42103:
		line_filter_option = 0;
		switch (type) {
		case U3_JITTER_TEST:
			waiting_time_quirk = 6000;
			break;
		}
		break;
	case CHIP_SW49501:
	case CHIP_SW49407:
		line_filter_option = 0;
		break;
	case CHIP_SW49106:
		if (type == U0_JITTER_TEST) {
			line_filter_option = 0;
		}
		break;
	}

	table = write_cmd_tables;
	while (table->type != -1) {
		if (table->type == type) {
			testmode = table->mode;
			if (!line_filter_option) {
				testmode &= ~LINE_FILTER_OPTION;
			}
			if (table->waiting) {
				waiting_time = table->waiting;
			}
			if (waiting_time_quirk) {
				waiting_time = waiting_time_quirk;
			}
			break;
		}
		table++;
	}
	if (table->type == -1) {
		t_prd_err(prd, "unsupported test mode, %d\n", type);
		return -EINVAL;
	}

	/* TestType Set */
	ret = siw_hal_write_value(dev, addr, testmode);
	if (ret < 0) {
		return ret;
	}
	t_prd_info(prd, "write testmode[%04Xh] = %Xh\n",
		addr, testmode);
	touch_msleep(waiting_time);

	if (type == U3_BLU_JITTER_TEST) {
		prd_set_blu(dev);
	}

	/* Check Test Result - wait until 0 is written */
	addr = reg->tc_tsp_test_status;
	do {
		touch_msleep(50);
		ret = siw_hal_read_value(dev, addr, &rdata);
		if (ret < 0) {
			return ret;
		}
		if (ret >= 0) {
			t_prd_dbg_base(prd, "rdata[%04Xh] = 0x%x\n",
				addr, rdata);
		}
	} while ((rdata != 0xAA) && retry--);

	if (rdata != 0xAA) {
		t_prd_err(prd, "ProductionTest Type [%d] Time out, %08Xh\n",
			type, rdata);
		goto out;
	}

	return 1;

out:
	t_prd_err(prd, "Write test mode failed\n");
	return 0;
}

static int prd_spec_file_read_ext(struct siw_hal_prd_data *prd)
{
	char *prd_in_fname[] = {
		[0]	= __prd_in_file,
		[1]	= __prd_in_file_m,
	};
	struct device *dev = prd->dev;
	struct file *filp = NULL;
	loff_t offset = 0;
	char *fname;
	int path_idx = 0;
	int ret = 0;

	path_idx = !!(siw_touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER);
	fname = (char *)prd_in_fname[path_idx];

	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	kernel_read(filp, offset, (char *)prd->line, sizeof(prd->line));

	prd_vfs_file_close(filp, NULL);

	t_prd_info(prd, "file detected : %s\n", fname);

out:
	return ret;
}

static int prd_spec_file_read(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	const struct firmware *fwlimit = NULL;
	const char *path[2] = {
		ts->panel_spec,
		ts->panel_spec_mfts
	};
	char *fname;
	int path_idx = 0;
	int ret = 0;

	path_idx = !!(siw_touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER);
	fname = (char *)path[path_idx];

	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		t_prd_err(prd, "panel spec file name is null\n");
		siw_prd_buf_snprintf(prd->buf_write, 0,
			"[E] panel spec file not defined\n");
		ret = -EINVAL;
		goto out;
	}

	ret = request_firmware(&fwlimit, fname, dev);
	if (ret < 0) {
		t_prd_err(prd, "request file(%s) is failed in normal mode\n", fname);
		siw_prd_buf_snprintf(prd->buf_write, 0,
			"[E] request file(%s) is failed in normal mode\n", fname);
		goto out;
	}

	if (fwlimit->data == NULL) {
		ret = -EFAULT;
		t_prd_err(prd, "fwlimit->data is NULL\n");
		siw_prd_buf_snprintf(prd->buf_write, 0,
			"[E] fwlimit->data is NULL\n");
		goto out;
	}

	strlcpy(prd->line, fwlimit->data, sizeof(prd->line));

out:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

static int __used prd_get_limit(struct siw_hal_prd_data *prd,
		char *breakpoint, int *limit)
{
	struct device *dev = prd->dev;
	int p = 0;
	int q = 0;
	int q_limit;
	int cipher = 1;
	char *found;
	int boot_mode = 0;
	int value = ~0;
	int ret = 0;

	if (limit)
		*limit = ~0;

	if (breakpoint == NULL) {
		ret = -EINVAL;
		goto out;
	}

	boot_mode = siw_touch_boot_mode_check(dev);
	if ((boot_mode > MINIOS_MFTS_CURVED) ||
		(boot_mode < NORMAL_BOOT)) {
		ret = -EINVAL;
		goto out;
	}

	ret = prd_spec_file_read_ext(prd);
	if (ret < 0) {
		ret = prd_spec_file_read(prd);
		if (ret < 0)
			goto out;
	}

	found = strnstr(prd->line, breakpoint, sizeof(prd->line));
	if (found == NULL) {
		t_prd_err(prd,
			"failed to find %s, spec file is wrong\n",
			breakpoint);
		ret = -EFAULT;
		goto out;
	}
	q = found - prd->line;

	q_limit = ARRAY_SIZE(prd->line);
	while (q < q_limit) {
		if (prd->line[q] == ',') {
			value = 0;

			cipher = 1;
			for (p = 1; (prd->line[q - p] >= '0') &&
					(prd->line[q - p] <= '9'); p++) {
				value += ((prd->line[q - p] - '0') * cipher);
				cipher *= 10;
			}

			if (limit)
				*limit = value;

			t_prd_info(prd, "scanning spec file done, %s = %d\n",
					breakpoint, value);

			break;
		}
		q++;
	}

	if (value == ~0) {
		t_prd_err(prd, "%s found, but getting num failed\n", breakpoint);
		ret = -EFAULT;
		goto out;
	}

out:
	return ret;
}

#define PRD_IRQ_TEST_CNT	10

static int prd_irq_test_handler(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int size = 0;

	if (!prd->irq_test) {
		return -ESRCH;
	}

	size = 12 + (sizeof(struct siw_hal_touch_data) * touch_max_finger(ts));
	siw_hal_reg_read(dev, reg->tc_ic_status, (void *)&chip->info, size);

	++prd->irq_cnt;

	t_prd_info(prd, "irq_test: irq detected %d (h/w:%Xh, f/w:%Xh)\n",
		prd->irq_cnt, chip->info.ic_status, chip->info.device_status);

	return 0;
}

struct prd_irq_setup {
	int addr;
	u32 data;
};

static const struct prd_irq_setup prd_irq_test_setup_sw49501[] = {
	{ 0x34, 0 },
	{ 0xB8, 0 },
	{ -1, 0 }	/* end mark */
};

static const struct prd_irq_setup prd_irq_test_ctrl_sw49501 = { 0x02, 0 };

static int prd_do_irq_test_setup(struct siw_hal_prd_data *prd,
			struct prd_irq_setup *ctrl)
{
	struct device *dev = prd->dev;
	int addr;
	u32 data;
	int ret = 0;

	while (ctrl) {
		addr = ctrl->addr;
		data = ctrl->data;
		if (addr == -1) {
			break;
		}

		if (addr) {
			t_prd_info(prd, "irq_test: setup wr[%04Xh] = %08Xh\n",
				addr, data);
			ret = siw_hal_write_value(dev, addr, data);
			if (ret < 0) {
				return ret;
			}
		}

		ctrl++;
	}

	return 0;

}

static int prd_do_irq_test_trigger(struct siw_hal_prd_data *prd,
			struct prd_irq_setup *ctrl)
{
	struct device *dev = prd->dev;
	int addr;
	u32 data;
	int i;
	int ret;

	addr = ctrl->addr;
	data = ctrl->data;

	prd->irq_cnt = 0;

	for (i = 0; i < PRD_IRQ_TEST_CNT; i++) {
		if (prd->irq_cnt != i) {
			t_prd_warn(prd,
				"irq_test: ctrl: irq cnt mismatch, trigger(%d) != detection(%d)\n",
				i, prd->irq_cnt);
		}

		t_prd_info(prd, "irq_test: ctrl: irq trigger %d (wr[%04Xh] = %d)\n",
				i + 1, addr, data);
		ret = siw_hal_write_value(dev, addr, data);
		if (ret < 0) {
			return ret;
		}

		touch_msleep(20);
	}

	return 0;
}

static int prd_do_irq_test(struct siw_hal_prd_data *prd, char *buf)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct prd_irq_setup *irq_test_setup = NULL;
	struct prd_irq_setup *irq_test_ctrl = NULL;
	int irq_cnt;
	int ret = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_SW49501:
		irq_test_setup = (struct prd_irq_setup *)prd_irq_test_setup_sw49501;
		irq_test_ctrl = (struct prd_irq_setup *)&prd_irq_test_ctrl_sw49501;
		break;
	default:
		t_prd_info(prd, "IRQ Test not supported, skip\n");
		return 0;
	}

	ret = prd_do_irq_test_setup(prd, irq_test_setup);
	if (ret < 0) {
		return ret;
	}

	/* temporary setup */
	atomic_set(&chip->init, IC_INIT_NEED);

	ts->ops->irq_dbg_handler = prd_irq_test_handler;

	/* temporary enable */
	siw_touch_irq_control(dev, INTERRUPT_ENABLE);

	prd->irq_test = 1;

	ret = prd_do_irq_test_trigger(prd, irq_test_ctrl);

	prd->irq_test = 0;

	/* restore */
	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	ts->ops->irq_dbg_handler = NULL;

	/* restore */
	atomic_set(&chip->init, IC_INIT_DONE);

	if (ret < 0) {
		siw_snprintf(buf, 0, "IRQ : Fail(trigger io failed)\n");
		goto out;
	}

	irq_cnt = prd->irq_cnt;

	ret = (irq_cnt == PRD_IRQ_TEST_CNT) ? 0 : 1;

	if (ret) {
		siw_snprintf(buf, 0, "IRQ : Fail, trigger(%d) != detection(%d)\n",
			PRD_IRQ_TEST_CNT, irq_cnt);
	} else {
		siw_snprintf(buf, 0, "IRQ : Pass\n");
	}

out:
	if (ret) {
		t_prd_err(prd, "%s", buf);
	} else {
		t_prd_info(prd, "%s", buf);
	}

	return ret;
}

static int __used prd_irq_test(struct siw_hal_prd_data *prd, int result_on)
{
	const char *test_str = NULL;
	char test_type[32] = {0, };
	int ret;

	test_str = prd_get_test_str(IRQ_TEST);

	t_prd_info(prd, "========%s========\n", test_str);

	memset(prd->buf_write, 0, PRD_BUF_SIZE);

	ret = prd_do_irq_test(prd, prd->buf_write);

	if (result_on == RESULT_ON) {
		snprintf(test_type, sizeof(test_type), "\n\n[%s]\n", test_str);
		/* Test Type Write */
		prd_write_file(prd, test_type, TIME_INFO_SKIP);

		prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);
	}

	return ret;
}

static int prd_read_raw_memory(struct siw_hal_prd_data *prd,
			u32 offset, void *buf, u32 size)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	int ret = 0;

	if (!offset) {
		t_prd_err(prd, "raw memory failed: zero offset\n");
		ret = -EFAULT;
		goto out;
	}

	if (!size) {
		t_prd_err(prd, "raw memory failed: zero size\n");
		ret = -EFAULT;
		goto out;
	}

	if (buf == NULL) {
		t_prd_err(prd, "raw memory failed: NULL buf\n");
		ret = -EFAULT;
		goto out;
	}

	//offset write
	ret = siw_hal_write_value(dev, reg->serial_data_offset, offset);
	if (ret < 0) {
		goto out;
	}

	//read raw data
	ret = siw_hal_reg_read(dev, reg->data_i2cbase_addr, buf, size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_read_raw_memory_self_pre(struct siw_hal_prd_data *prd,
	u32 cmd, u32 *offset, int *size)
{
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;

	if (cmd == prd->img_cmd.raw) {
		*offset = prd->img_offset.raw_s;
		*size = ctrl->buf_self_size;
		return 0;
	}

	if (cmd == prd->img_cmd.baseline_even) {
		*offset = prd->img_offset.baseline_s;
		*size = ctrl->buf_self_size;
		return 0;
	}

	if (cmd == prd->img_cmd.delta) {
		*offset = prd->img_offset.delta_s;
		*size = ctrl->buf_self_size;
		return 0;
	}

	if (cmd == prd->img_cmd.label) {
		*offset = prd->img_offset.label_s;
		*size = param->row + param->col + 2;
		return 0;
	}

	if (cmd == prd->img_cmd.debug) {
		*offset = prd->img_offset.debug_s;
		*size = ctrl->buf_self_size;
		return 0;
	}

	return -EINVAL;
}

static int prd_read_raw_memory_self(struct siw_hal_prd_data *prd, u32 cmd, void *buf)
{
	u32 offset = 0;
	int size = 0;
	int ret = 0;

	if (!prd->panel_type) {
		return 0;
	}

	ret = prd_read_raw_memory_self_pre(prd, cmd, &offset, &size);
	if (ret < 0) {
		return 0;
	}

	if (!offset) {
		return 0;
	}

	ret = prd_read_raw_memory(prd, offset, prd->buf_self, size);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static void __buff_swap(void *dst, void *src, int row, int col, int is_8bit)
{
	int r, c;
	int i;

	for (i = 0; i < (row * col); i++){
		r = i / col;
		c = i % col;
		if (is_8bit) {
			((u8 *)dst)[i] = ((u8 *)src)[r + (row * c)];
		} else {
			((u16 *)dst)[i] = ((u16 *)src)[r + (row * c)];
		}
	}
}

static void prd_swap_raw_memory(struct siw_hal_prd_data *prd, int idx,
		void *buf, int size, int row_size, int col_size, int is_8bit)
{
	struct siw_hal_prd_param *param = &prd->param;

	if (!(param->buf_swap_flag & idx)) {
		return;
	}

	if (!size)
		return;

	if (!row_size)
		row_size = param->row;

	if (!col_size)
		col_size = param->col;

	memcpy(prd->m2_buf_tmp, buf, size);

	__buff_swap(buf, prd->m2_buf_tmp, row_size, col_size, is_8bit);
}

static int __prd_os_result_rawdata_get(struct siw_hal_prd_data *prd,
			u32 offset, int16_t *buf, u32 size, const char *test_str)
{
	char info_str[64] = {0, };
	int ret = 0;

	if (test_str) {
		snprintf(info_str, sizeof(info_str), "\n[%s Result Rawdata]\n", test_str);

		prd_write_file(prd, info_str, TIME_INFO_SKIP);

		t_prd_info(prd, "%s Rawdata Offset = %xh\n", test_str, offset);
		t_prd_info(prd, "%s", &info_str[1]);
	}

	ret = prd_read_raw_memory(prd, offset, buf, size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_os_result_rawdata_get(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u16 open_data_offset = 0;
	u16 short_data_offset = 0;
	u32 os_result_offset;
	u32 data_offset;
	u32 read_size;
	int16_t *buf_result_data = NULL;
	const char *test_str = NULL;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->prd_open3_short_offset,
				&os_result_offset);
	if (ret < 0) {
		goto out;
	}
	open_data_offset = os_result_offset & 0xFFFF;
	short_data_offset = (os_result_offset >> 16) & 0xFFFF;

	switch (type) {
	case OPEN_NODE_TEST:
		data_offset = open_data_offset;

		buf_result_data = prd->open_buf_result_rawdata;

		read_size = ctrl->open_rawdata_size;
		break;
	case SHORT_NODE_TEST:
		data_offset = short_data_offset;

		buf_result_data = prd->short_buf_result_rawdata;

		read_size = ctrl->short_rawdata_size;
		break;
	case SHORT_FULL_TEST:
		data_offset = short_data_offset;

		buf_result_data = prd->short_buf_result_rawdata;

		read_size = ctrl->short_full_rawdata_size;
		break;
	default:
		t_prd_err(prd, "os test: [result_rawdata_get] unknown type, %d\n", type);
		goto out;
	}

	test_str = prd_get_test_str(type);

	ret = __prd_os_result_rawdata_get(prd,
			data_offset, buf_result_data, read_size, test_str);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_os_result_rawdata_get_sf_dual(struct siw_hal_prd_data *prd)
{
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
//	u32 open_data_offset = 0;
	u32 short_data_offset = 0;
	u32 os_result_offset = 0;
	u32 data_offset = 0;
	u32 read_size = 0;
	int16_t *buf_result_data = NULL;
	const char *test_str = NULL;
	int offset_reg = 0;
	int offset_pos = 0;
	int ret = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_SW49106:
		if (param->name == prd_param_name_sw49106_type_2) {
			if (!fw->v.version.major && (fw->v.version.minor <= 3)) {
				break;
			}

			offset_reg = reg->prd_open3_short_offset;
		}
		break;
	}

	if (!offset_reg) {
		return -EINVAL;
	}

	offset_pos = offset_reg>>16;
	offset_reg &= 0xFFFF;

	test_str = "SHORT_FULL_TEST (2nd)";

	ret = siw_hal_read_value(dev, offset_reg, &os_result_offset);
	if (ret < 0) {
		goto out;
	}

	short_data_offset = (os_result_offset >> offset_pos) & 0xFFFF;

	data_offset = short_data_offset;

	buf_result_data = prd->short_buf_result_rawdata;

	read_size = ctrl->short_full_rawdata_size;

	ret = __prd_os_result_rawdata_get(prd,
			data_offset, buf_result_data, read_size, test_str);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_os_result_data_get(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u16 open_short_data_offset = 0;
	u32 os_result_offset;
	u32 read_size;
	int16_t *buf_result_data = NULL;
	const char *test_str = NULL;
	char info_str[64] = { 0, };
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->prd_tune_result_offset,
				&os_result_offset);
	if (ret < 0) {
		goto out;
	}
	open_short_data_offset = os_result_offset & 0xFFFF;

	switch (type) {
	case OPEN_NODE_TEST:
		buf_result_data = prd->open_buf_result_data;

		read_size = ctrl->open_result_size;
		break;
	case SHORT_NODE_TEST:
		buf_result_data = prd->short_buf_result_data;

		read_size = ctrl->short_result_size;
		break;
	case SHORT_FULL_TEST:
		buf_result_data = prd->short_buf_result_data;

		read_size = ctrl->short_full_result_size;
		break;
	default:
		t_prd_err(prd, "os test: [result_data_get] unknown type, %d\n", type);
		goto out;
	}

	test_str = prd_get_test_str(type);

	snprintf(info_str, sizeof(info_str), "\n[%s Result Data]\n", test_str);

	prd_write_file(prd, info_str, TIME_INFO_SKIP);

	t_prd_info(prd, "%s Data Offset = %xh\n", test_str, open_short_data_offset);
	t_prd_info(prd, "%s", &info_str[1]);

	ret = prd_read_raw_memory(prd, open_short_data_offset, buf_result_data, read_size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

/* Flexible digit width */
#define DIGIT_RANGE_BASE		4

#define PRD_RAWDATA_MAX			(32767)
#define PRD_RAWDATA_MIN			(-32768)

#define RAWDATA_MAX_DIGIT6		(999999)
#define RAWDATA_MIN_DIGIT6		(-99999)

#define RAWDATA_MAX_DIGIT5		(99999)
#define RAWDATA_MIN_DIGIT5		(-9999)

#define RAWDATA_MAX_DIGIT4		(9999)
#define RAWDATA_MIN_DIGIT4		(-999)

static int __get_digit_range(int16_t *buf, int size)
{
	int min = RAWDATA_MAX_DIGIT6;
	int max = RAWDATA_MIN_DIGIT6;
	int num;

	while (size--) {
		num = *buf++;

		if (num < min) {
			min = num;
		}
		if (num > max) {
			max = num;
		}
	}

	if ((max > RAWDATA_MAX_DIGIT5) || (min < RAWDATA_MIN_DIGIT5)) {
		return 6;
	}

	if ((max > RAWDATA_MAX_DIGIT4) || (min < RAWDATA_MIN_DIGIT4)) {
		return 5;
	}

	return DIGIT_RANGE_BASE;
}

static int prd_print_os_data(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	const char *raw_fmt_row_no = NULL;
	const char *raw_fmt_col_pre = NULL;
	const char *raw_fmt_col_no = NULL;
	const char *raw_fmt = NULL;
	int size = 0;
	int log_size = 0;
	u16 *os_buf = NULL;
	int row_size = param->row;
	int col_size = param->col;
	char *log_buf = prd->log_buf;
	char *buf = prd->buf_write;
	int digit_range = DIGIT_RANGE_BASE;
	int data_size;
	int i = 0, j = 0;

	memset(buf, 0, PRD_BUF_SIZE);

	switch (type) {
	case OPEN_NODE_TEST:
		col_size = ctrl->open_result_col;
		os_buf = prd->open_buf_result_data;
		data_size = ctrl->open_result_size;
		break;
	case SHORT_NODE_TEST:
		col_size = ctrl->short_result_col;
		os_buf = prd->short_buf_result_data;
		data_size = ctrl->short_result_size;
		break;
	case SHORT_FULL_TEST:
		col_size = ctrl->short_full_result_col;
		os_buf = prd->short_buf_result_data;
		data_size = ctrl->short_full_result_size;
		break;
	default:
		siw_prd_log_buf_snprintf(log_buf, log_size,
			"os test: [print_data] unknown type, %d\n", type);
		t_prd_err(prd, "%s", log_buf);
		size = siw_prd_buf_snprintf(buf, size, "%s", log_buf);
		goto out;
	}

	if (!data_size) {
		siw_prd_log_buf_snprintf(log_buf, log_size,
			"os test: [print_data] %s size zero\n", prd_get_test_str(type));
		t_prd_err(prd, "%s", log_buf);
		size = siw_prd_buf_snprintf(buf, size, "%s", log_buf);
		goto out;
	}

	digit_range = __get_digit_range(os_buf, row_size * col_size);

	raw_fmt_row_no = "[%2d] ";

	switch (digit_range) {
	case 6:
		raw_fmt_col_pre = "   :   ";
		raw_fmt_col_no = "[%2d]   ";
		raw_fmt = "%6d ";
		break;
	case 5:
		raw_fmt_col_pre = "   :  ";
		raw_fmt_col_no = "[%2d]  ";
		raw_fmt = "%5d ";
		break;
	default:
		raw_fmt_col_pre = "   : ";
		raw_fmt_col_no = "[%2d] ";
		raw_fmt = "%4d ";
		break;
	}

	size += siw_prd_buf_snprintf(buf, size, "\n");

	log_size = siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_col_pre);

	for (i = 0; i < col_size; i++) {
		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_col_no, i);
	}

	size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

	for (i = 0; i < row_size; i++) {
		log_size = 0;

		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_row_no, i);
		for (j = 0; j < col_size; j++) {
			//if (j == param->col)
			//	continue;
			log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt, os_buf[i * col_size + j]);
		}

		size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);
	}

out:
	prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);
	memset(prd->buf_write, 0, size);

	return size;
}

static int prd_print_os_rawdata(struct siw_hal_prd_data *prd, u8 type)
{
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	const char *raw_fmt_row_no = NULL;
	const char *raw_fmt_col_pre = NULL;
	const char *raw_fmt_col_no = NULL;
	const char *raw_fmt = NULL;
	int size = 0;
	int log_size = 0;
	u16 *os_buf = NULL;
	int row_size = param->row;
	int col_size = param->col;
	char *log_buf = prd->log_buf;
	char *buf = prd->buf_write;
	int digit_range = DIGIT_RANGE_BASE;
	int data_size = 0;
	int i = 0, j = 0;

	memset(buf, 0, PRD_BUF_SIZE);

	switch (type) {
	case OPEN_NODE_TEST:
		col_size = ctrl->open_rawdata_col;
		os_buf = prd->open_buf_result_rawdata;
		data_size = ctrl->open_rawdata_size;
		break;
	case SHORT_NODE_TEST:
		col_size = ctrl->short_rawdata_col;
		os_buf = prd->short_buf_result_rawdata;
		data_size = ctrl->short_rawdata_size;
		break;
	case SHORT_FULL_TEST:
		col_size = ctrl->short_full_rawdata_col;
		os_buf = prd->short_buf_result_rawdata;
		data_size = ctrl->short_full_rawdata_size;
		break;
	default:
		siw_prd_log_buf_snprintf(log_buf, log_size,
			"os test: [print_rawdata] unknown type, %d\n", type);
		t_prd_err(prd, "%s", log_buf);
		size = siw_prd_buf_snprintf(buf, size, "%s", log_buf);
		goto out;
	}

	if (!data_size) {
		siw_prd_log_buf_snprintf(log_buf, log_size,
			"os test: [print_rawdata] %s size zero\n", prd_get_test_str(type));
		t_prd_err(prd, "%s", log_buf);
		size = siw_prd_buf_snprintf(buf, size, "%s", log_buf);
		goto out;
	}

	digit_range = __get_digit_range(os_buf, row_size * col_size);

	raw_fmt_row_no = "[%2d] ";

	switch (digit_range) {
	case 6:
		raw_fmt_col_pre = "   :   ";
		raw_fmt_col_no = "[%2d]   ";
		raw_fmt = "%6d ";
		break;
	case 5:
		raw_fmt_col_pre = "   :  ";
		raw_fmt_col_no = "[%2d]  ";
		raw_fmt = "%5d ";
		break;
	default:
		raw_fmt_col_pre = "   : ";
		raw_fmt_col_no = "[%2d] ";
		raw_fmt = "%4d ";
		break;
	}

	size += siw_prd_buf_snprintf(buf, size, "\n");

	log_size = siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_col_pre);

	for (i = 0; i < col_size; i++) {
		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_col_no, i);
	}

	size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

	/*
	 * short data replace algorithm
	 */
	if (type == SHORT_NODE_TEST) {
		u16 *short_temp_buf = prd->m2_buf_tmp;
		int short_buf_size = ctrl->short_rawdata_size;

		memcpy(short_temp_buf, os_buf, short_buf_size);
		memset(os_buf, 0, short_buf_size);

		for (i = 0; i < col_size; i++) {
			for (j = 0; j < row_size; j++) {
				os_buf[j * col_size + i] = short_temp_buf[i*row_size + j];
			}
		}
	}

	for (i = 0; i < row_size; i++) {
		log_size = 0;

		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_row_no, i);

		for (j = 0; j < col_size; j++) {
			if (j == param->col)
				continue;

			log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt, os_buf[i * col_size + j]);
		}

		size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);
	}

out:
	prd_write_file(prd, buf, TIME_INFO_SKIP);
	memset(buf, 0, size);

	return size;
}

static int prd_os_xline_result_read(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	int ret = 0;


	if (param->sd_test_flag & OPEN_SHORT_RESULT_DATA_FLAG) {
		//open short fail reason "result data"
		ret = prd_os_result_data_get(prd, type);
		if (ret < 0) {
			return ret;
		}

		prd_print_os_data(prd, type);
	}

	if (param->sd_test_flag & OPEN_SHORT_RESULT_RAWDATA_FLAG) {
		//open short fail reason "result raw data"
		ret = prd_os_result_rawdata_get(prd, type);
		if (ret < 0) {
			return ret;
		}

		prd_print_os_rawdata(prd, type);

		if (type == SHORT_FULL_TEST) {
			int ret_sf = 0;
			ret_sf = prd_os_result_rawdata_get_sf_dual(prd);
			if (ret_sf >= 0) {
				prd_print_os_rawdata(prd, type);
			}
		}
	}

	return 0;
}

static int prd_control_open_short_result(struct siw_hal_prd_data *prd,
				int type, int result_on)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	char *str = "";
	int result = 0;
	int ret = 0;

	switch (type) {
	case OPEN_NODE_TEST:
		str = "open";
		break;
	case SHORT_NODE_TEST:
		str = "short";
		break;
	case SHORT_FULL_TEST:
		str = "short full";
		break;
	}

	ret = siw_hal_read_value(dev,
				reg->tc_tsp_test_pf_result,
				&result);
	if (ret < 0) {
		return ret;
	}
	t_prd_info(prd, "%s result = %d\n", str, result);

	if (result | (prd->dbg_mask & PRD_DBG_OPEN_SHORT_DATA)) {
		ret = prd_os_xline_result_read(prd, type);
		if (ret < 0) {
			return ret;
		}
	}

	return result;
}

static int prd_print_pre(struct siw_hal_prd_data *prd, char *buf,
				int size, int row_size, int col_size,
				const char *name, int digit_range)
{
	char *log_buf = prd->log_buf;
	const char *raw_fmt_col_pre = NULL;
	const char *raw_fmt_col_no = NULL;
	int log_size = 0;
	int i;

	switch (digit_range) {
	case 6:
		raw_fmt_col_pre = "   :   ";
		raw_fmt_col_no = "[%2d]   ";
		break;
	case 5:
		raw_fmt_col_pre = "   :  ";
		raw_fmt_col_no = "[%2d]  ";
		break;
	default:
		raw_fmt_col_pre = "   : ";
		raw_fmt_col_no = "[%2d] ";
		break;
	}

	log_size += siw_prd_log_buf_snprintf(log_buf, log_size,
					"-------- %s(%d %d) --------",
					name, row_size, col_size);

	size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

	log_size = 0;
	log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_col_pre);

	for (i = 0; i < col_size; i++) {
		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_col_no, i);
	}

	size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

	return size;
}

static int prd_print_post(struct siw_hal_prd_data *prd, char *buf,
				int size, int min, int max)
{
	size += siw_prd_buf_snprintf(buf, size, "\n");

	size += siw_prd_buf_snprintf(buf, size,
				"Rawdata min : %d , max : %d\n",
				min, max);

	t_prd_info(prd, "Rawdata min : %d , max : %d\n",
				min, max);

	return size;
}

enum {
	PRD_PRT_TYPE_U8 = 0,
	PRD_PRT_TYPE_S16,
	PRD_PRT_TYPE_MAX,
};

static int prd_print_xxx(struct siw_hal_prd_data *prd, char *buf,
				int size, void *rawdata_buf,
				int row_size, int col_size,
				const char *name, int opt, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	char *log_buf = prd->log_buf;
	const char *raw_fmt_row_no = NULL;
	const char *raw_fmt = NULL;
	u8 *rawdata_u8 = rawdata_buf;
	int16_t *rawdata_s16 = rawdata_buf;
	int curr_raw;
	int i, j;
	int col_i = 0;
	int col_add = (opt) ? param->col_add : 0;
	int log_size = 0;
	int min_raw = PRD_RAWDATA_MAX;
	int max_raw = PRD_RAWDATA_MIN;
	int is_16bit = !!(type == PRD_PRT_TYPE_S16);
	int digit_range = DIGIT_RANGE_BASE;

	if (type >= PRD_PRT_TYPE_MAX) {
		t_prd_err(prd, "invalid print type, %d\n", type);
		return -EINVAL;
	}

	if (rawdata_buf == NULL) {
		t_prd_err(prd, "print failed: NULL buf\n");
		return -EFAULT;
	}

	if (is_16bit) {
		rawdata_s16 = &((int16_t *)rawdata_buf)[0];
		digit_range = __get_digit_range(rawdata_s16, row_size * (col_size + col_add));
	}

	raw_fmt_row_no = "[%2d] ";

	switch (digit_range) {
	case 6:
		raw_fmt = "%6d ";
		break;
	case 5:
		raw_fmt = "%5d ";
		break;
	default:
		raw_fmt = "%4d ";
		break;
	}

	size = prd_print_pre(prd, buf, size, row_size, col_size, name, digit_range);

	col_i = 0;
	for (i = 0; i < row_size; i++) {
		log_size = 0;
		memset(log_buf, 0, sizeof(prd->log_buf));
		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_row_no, i);

		if (is_16bit) {
			rawdata_s16 = &((int16_t *)rawdata_buf)[col_i];
		} else {
			rawdata_u8 = &((u8 *)rawdata_buf)[col_i];
		}
		for (j = 0; j < col_size; j++) {
			if (is_16bit) {
				curr_raw = *rawdata_s16++;
			} else {
				curr_raw = *rawdata_u8++;
			}

			log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt, curr_raw);

			if (curr_raw < min_raw) {
				min_raw = curr_raw;
			}
			if (curr_raw > max_raw) {
				max_raw = curr_raw;
			}
		}

		size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

		col_i += (col_size + col_add);
	}

	size = prd_print_post(prd, buf, size, min_raw, max_raw);

	return size;
}

static int prd_print_u8(struct siw_hal_prd_data *prd, char *buf,
				int size, u8 *rawdata_buf_u8,
				int row_size, int col_size,
				const char *name, int opt)
{
	return prd_print_xxx(prd, buf, size, (void *)rawdata_buf_u8,
				row_size, col_size, name, opt, PRD_PRT_TYPE_U8);
}

static int prd_print_s16(struct siw_hal_prd_data *prd, char *buf,
				int size, int16_t *rawdata_buf_u16,
				int row_size, int col_size,
				const char *name, int opt)
{
	return prd_print_xxx(prd, buf, size, (void *)rawdata_buf_u16,
				row_size, col_size, name, opt, PRD_PRT_TYPE_S16);
}

static int prd_print_rawdata(struct siw_hal_prd_data *prd,
			char *buf, int type, int size, int opt)
{
	struct siw_hal_prd_param *param = &prd->param;
	int16_t *rawdata_buf_16 = NULL;
	u8 *rawdata_buf_u8 = NULL;
	const char *name = NULL;
	int row_size = param->row;
	int col_size = param->col;

	switch (type) {
	case M1_EVEN_DATA:
		col_size = param->m1_col;
		rawdata_buf_16 = prd->m1_buf_even_rawdata;
		name = "EVEN Data";
		break;
	case M1_ODD_DATA:
		col_size = param->m1_col;
		rawdata_buf_16 = prd->m1_buf_odd_rawdata;
		name = "ODD Data";
		break;
	case M2_EVEN_DATA:
		rawdata_buf_16 = prd->m2_buf_even_rawdata;
		name = "EVEN Data";
		break;
	case M2_ODD_DATA:
		rawdata_buf_16 = prd->m2_buf_odd_rawdata;
		name = "ODD Data";
		break;
	case LABEL_DATA:
		rawdata_buf_u8 = prd->buf_label;
		name = "LABEL Data";
		break;
	case DEBUG_DATA:
		rawdata_buf_16 = prd->buf_debug;
		name = "DEBUG Data";
		break;
	default:
		t_prd_warn(prd, "prd_print_rawdata: invalud type, %d\n", type);
		break;
	}

	if (rawdata_buf_16) {
		size = prd_print_s16(prd, buf, size, rawdata_buf_16,
				row_size, col_size, name, opt);
	} else if (rawdata_buf_u8) {
		size = prd_print_u8(prd, buf, size, rawdata_buf_u8,
				row_size, col_size, name, opt);
	}

	return size;
}

static int prd_print_pre_self(struct siw_hal_prd_data *prd, char *buf,
				int size, int row_size, int col_size,
				const char *name, int digit_range)
{
	char *log_buf = prd->log_buf;
	const char *raw_fmt_col_pre = NULL;
	const char *raw_fmt_col_no = NULL;
	int log_size = 0;
	int i;

	size &= ~BIT(31);

	switch (digit_range) {
	case 6:
		raw_fmt_col_pre = "   :   [SF]   ";
		raw_fmt_col_no = "[%2d]   ";
		break;
	case 5:
		raw_fmt_col_pre = "   :  [SF]  ";
		raw_fmt_col_no = "[%2d]  ";
		break;
	default:
		raw_fmt_col_pre = "   : [SF] ";
		raw_fmt_col_no = "[%2d] ";
		break;
	}

	log_size += siw_prd_log_buf_snprintf(log_buf, log_size,
					"-------- %s(%d %d) --------",
					name, row_size, col_size);

	size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

	log_size = 0;
	log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_col_pre);

	for (i = 0; i < col_size; i++) {
		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_col_no, i);
	}

	size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

	return size;
}

static int prd_print_xxx_self(struct siw_hal_prd_data *prd,
				u32 cmd, char *buf,
				int size, void *rawdata_buf,
				int row_size, int col_size,
				const char *name, int opt, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	char *log_buf = prd->log_buf;
	const char *raw_fmt_row_no = NULL;
	const char *raw_fmt_self = NULL;
	const char *raw_fmt_empty = NULL;
	const char *raw_fmt = NULL;
	u8 *rawdata_u8 = rawdata_buf;
	int16_t *rawdata_s16 = rawdata_buf;
	int curr_raw;
	int i, j;
	int col_i = 0;
	int col_add = (opt) ? param->col_add : 0;
	int log_size = 0;
	int min_raw = PRD_RAWDATA_MAX;
	int max_raw = PRD_RAWDATA_MIN;
	int is_16bit = !!(type == PRD_PRT_TYPE_S16);
	int digit_range = DIGIT_RANGE_BASE;
	int digit_width_self = DIGIT_RANGE_BASE;
	int offset_horiz = 0;
	int offset_verti = col_size;

	if (type >= PRD_PRT_TYPE_MAX) {
		t_prd_err(prd, "invalid print type, %d\n", type);
		return -EINVAL;
	}

	if (rawdata_buf == NULL) {
		t_prd_err(prd, "print failed: NULL buf\n");
		return -EFAULT;
	}

	if (is_16bit) {
		rawdata_s16 = &((int16_t *)rawdata_buf)[0];
		digit_range = __get_digit_range(rawdata_s16, row_size * (col_size + col_add));
	}

	digit_width_self = __get_digit_range(prd->buf_self, row_size + col_size + col_add);
	if (digit_width_self > digit_range) {
		digit_range = digit_width_self;
	}

	raw_fmt_row_no = "[%2d] ";
	raw_fmt_self = "[SF] ";

	switch (digit_range) {
	case 6:
		raw_fmt_empty = "       ";
		raw_fmt = "%6d ";
		break;
	case 5:
		raw_fmt_empty = "      ";
		raw_fmt = "%5d ";
		break;
	default:
		raw_fmt_empty = "     ";
		raw_fmt = "%4d ";
		break;
	}

	size = prd_print_pre_self(prd, buf, size, row_size, col_size, name, digit_range);

	/* self data - horizental line */
	log_size = 0;
	memset(log_buf, 0, sizeof(prd->log_buf));

	log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_self);

	log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_empty);

	for (i = 0; i < col_size; i++) {
		if (is_16bit) {
			curr_raw = ((int16_t *)prd->buf_self)[offset_horiz];
		} else {
			curr_raw = ((u8 *)prd->buf_self)[offset_horiz];
		}
		offset_horiz++;

		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt, curr_raw);
	}

	size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

	col_i = 0;
	for (i = 0; i < row_size; i++) {
		log_size = 0;
		memset(log_buf, 0, sizeof(prd->log_buf));

		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt_row_no, i);

		/* self data : vertical line */
		if (is_16bit) {
			curr_raw = ((int16_t *)prd->buf_self)[offset_verti];
		} else {
			curr_raw = ((u8 *)prd->buf_self)[offset_verti];
		}
		offset_verti++;

		log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt, curr_raw);

		if (is_16bit) {
			rawdata_s16 = &((int16_t *)rawdata_buf)[col_i];
		} else {
			rawdata_u8 = &((u8 *)rawdata_buf)[col_i];
		}
		for (j = 0; j < col_size; j++) {
			/* rawdata */
			if (is_16bit) {
				curr_raw = *rawdata_s16++;
			} else {
				curr_raw = *rawdata_u8++;
			}

			log_size += siw_prd_log_buf_snprintf(log_buf, log_size, raw_fmt, curr_raw);

			if (curr_raw < min_raw) {
				min_raw = curr_raw;
			}
			if (curr_raw > max_raw) {
				max_raw = curr_raw;
			}
		}

		size += siw_prd_log_flush(prd, log_buf, buf, log_size, size);

		col_i += (col_size + col_add);
	}

	size = prd_print_post(prd, buf, size, min_raw, max_raw);

	return size;
}

static int prd_print_u8_self(struct siw_hal_prd_data *prd,
				u32 cmd, char *buf,
				int size, u8 *rawdata_buf_u8,
				int row_size, int col_size,
				const char *name, int opt)
{
	return prd_print_xxx_self(prd, cmd, buf, size, (void *)rawdata_buf_u8,
				row_size, col_size, name, opt, PRD_PRT_TYPE_U8);
}

static int prd_print_s16_self(struct siw_hal_prd_data *prd,
				u32 cmd, char *buf,
				int size, int16_t *rawdata_buf_u16,
				int row_size, int col_size,
				const char *name, int opt)
{
	return prd_print_xxx_self(prd, cmd, buf, size, (void *)rawdata_buf_u16,
				row_size, col_size, name, opt, PRD_PRT_TYPE_S16);
}

static int prd_print_rawdata_self_valid(struct siw_hal_prd_data *prd, u32 cmd)
{
	if (cmd == prd->img_cmd.raw) {
		if (prd->img_offset.raw_s) {
			return 1;
		}
	}

	if (cmd == prd->img_cmd.baseline_even) {
		if (prd->img_offset.baseline_s) {
			return 1;
		}
	}

	if (cmd == prd->img_cmd.delta) {
		if (prd->img_offset.delta_s) {
			return 1;
		}
	}

	if (cmd == prd->img_cmd.label) {
		if (prd->img_offset.label_s) {
			return 1;
		}
	}

	if (cmd == prd->img_cmd.debug) {
		if (prd->img_offset.debug_s) {
			return 1;
		}
	}

	return 0;
}

static int prd_print_rawdata_self(struct siw_hal_prd_data *prd,
			u32 cmd, char *buf, int type, int size, int opt)
{
	struct siw_hal_prd_param *param = &prd->param;
	int16_t *rawdata_buf_16 = NULL;
	u8 *rawdata_buf_u8 = NULL;
	const char *name = NULL;
	int row_size = param->row;
	int col_size = param->col;

	if (!prd_print_rawdata_self_valid(prd, cmd)) {
		return prd_print_rawdata(prd, buf, type, size, opt);
	}

	switch (type) {
	case M2_EVEN_DATA:
		rawdata_buf_16 = prd->m2_buf_even_rawdata;
		name = "EVEN Data";
		break;
	case LABEL_DATA:
		rawdata_buf_u8 = prd->buf_label;
		name = "LABEL Data";
		break;
	case DEBUG_DATA:
		rawdata_buf_16 = prd->buf_debug;
		name = "DEBUG Data";
		break;
	default:
		t_prd_warn(prd, "prd_print_rawdata_self: invalud type, %d\n", type);
		break;
	}

	if (rawdata_buf_16) {
		size = prd_print_s16_self(prd, cmd, buf, size, rawdata_buf_16,
				row_size, col_size, name, opt);
	} else if (rawdata_buf_u8) {
		size = prd_print_u8_self(prd, cmd, buf, size, rawdata_buf_u8,
				row_size, col_size, name, opt);
	}

	return size;
}

#define STR_SPEC_BY_PARAM	"(p)"

/*
 *	return "result Pass:0 , Fail:1"
 */
static int prd_compare_tool(struct siw_hal_prd_data *prd,
				int test_cnt, int16_t **buf,
				int row, int col, int type, int opt)
{
	struct siw_hal_prd_param *param = &prd->param;
//	struct device *dev = prd->dev;
//	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_second_screen *second_screen = NULL;
	int16_t *raw_buf;
	int16_t *raw_curr;
	int i, j ,k;
	int col_i;
	int col_add = (opt) ? param->col_add : 0;
	int size = 0;
	int curr_raw;
	int curr_lower, curr_upper;
	int raw_err = 0;
	int result = 0;
	int spec_by_param = 0;
	int cnt = 0;
	int bound_i1 = 0;
	int bound_i2 = 0;
	int bound_j1 = 0;
	int bound_j2 = 0;
	int second_scr_on = 0;

	spec_by_param = !!(opt & (1<<16));
	opt &= 0xFFFF;

	col_add = (opt) ? param->col_add : 0;

	if (!test_cnt) {
		t_prd_err(prd, "zero test count\n");
		result = 1;
		size += siw_prd_buf_snprintf(prd->buf_write,
					size,
					"zero test count\n");
		goto out;
	}

	curr_lower = prd->image_lower;
	curr_upper = prd->image_upper;

	t_prd_info(prd, "lower %d, upper %d %s\n",
		curr_lower, curr_upper,
		(spec_by_param) ? STR_SPEC_BY_PARAM : "");
	size += siw_prd_buf_snprintf(prd->buf_write,
				size,
				"lower %d, upper %d %s\n",
				curr_lower, curr_upper,
				(spec_by_param) ? STR_SPEC_BY_PARAM : "");

	switch (type) {
	case U3_M1_RAWDATA_TEST:
	case U0_M1_RAWDATA_TEST:
	case U3_M1_JITTER_TEST:
	case U0_M1_JITTER_TEST:
		break;

	default:
		second_screen = &param->second_scr;

		bound_i1 = (second_screen->bound_i>>8) & 0xFF;
		bound_i2 = second_screen->bound_i & 0xFF;
		bound_j1 = (second_screen->bound_j>>8) & 0xFF;
		bound_j2 = second_screen->bound_j & 0xFF;

		second_scr_on = !!(bound_i2 && bound_j2);
		break;
	}

	for (k = 0; k < test_cnt; k++) {
		t_prd_info(prd,
			"-------- Compare[%d/%d] --------\n",
			k, test_cnt);
		size += siw_prd_buf_snprintf(prd->buf_write,
					size,
					"-------- Compare[%d/%d] --------\n",
					k, test_cnt);
		raw_buf = buf[k];
		if (raw_buf == NULL) {
			t_prd_err(prd, "compare failed: NULL buf\n");
			result = 1;
			size += siw_prd_buf_snprintf(prd->buf_write,
					size,
					"compare failed: NULL buf\n");
			goto out;
		}
		col_i = 0;

		for (i = 0; i < row; i++) {
			raw_curr = &raw_buf[col_i];

			for (j = 0; j < col; j++) {
				curr_raw = *raw_curr++;

				if ((curr_raw >= curr_lower) &&
					(curr_raw <= curr_upper)) {
					t_prd_dbg_trace(prd,
						"cnt %d, %d(lower) <= %d <= %d(upper) \n",
						cnt, curr_lower, curr_raw, curr_upper);
					cnt++;
					continue;
				}

				raw_err = 1;
				if (second_scr_on &&
					((i >= bound_i1) && (i <= bound_i2)) &&
					((j >= bound_j1) && (j <= bound_j2))) {
					raw_err = !!(curr_raw)<<1;	/* 2 or 0 */
				}

				if (raw_err) {
					result = 1;
					t_prd_info(prd,
						"F [%d][%d] = %d(%d)\n",
						i, j, curr_raw, raw_err);
					size += siw_prd_buf_snprintf(prd->buf_write,
								size,
								"F [%d][%d] = %d(%d)\n",
								i, j, curr_raw, raw_err);
				} else {
				#if 0
					t_prd_info(prd,
						"G [%d][%d] = %d\n",
						i, j, curr_raw);
				#endif
				}
			}
			col_i += (col + col_add);
		}

		if (!result) {
			t_prd_info(prd,
				"(none)\n");
			size += siw_prd_buf_snprintf(prd->buf_write,
						size,
						"(none)\n");
		}
	}

//	t_prd_info(prd, "type %d, result %d\n", type, result);

out:
	return result;
}

/*
 * Rawdata compare result
 * Pass : reurn 0
 * Fail : return 1
 */
#define __CMP_RAW_PARAM(_t, _v)	\
		{ .type = _t, .value = _v, }

static int prd_compare_rawdata(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
#if defined(__SIW_SUPPORT_PRD_SET_SD)
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
	struct cmp_raw_list {
		int type;
		int *value;
	} *table, cmp_raw_tables[] = {
		__CMP_RAW_PARAM(U3_M2_RAWDATA_TEST, sd_param->u3_m2),
		__CMP_RAW_PARAM(U3_M1_RAWDATA_TEST, sd_param->u3_m1),
		__CMP_RAW_PARAM(U0_M2_RAWDATA_TEST, sd_param->u0_m2),
		__CMP_RAW_PARAM(U0_M1_RAWDATA_TEST, sd_param->u0_m1),
		__CMP_RAW_PARAM(U3_BLU_JITTER_TEST, sd_param->u3_blu_jitter),
		__CMP_RAW_PARAM(U3_JITTER_TEST, sd_param->u3_jitter),
		__CMP_RAW_PARAM(U3_M1_JITTER_TEST, sd_param->u3_m1_jitter),
		__CMP_RAW_PARAM(U0_JITTER_TEST, sd_param->u0_jitter),
		__CMP_RAW_PARAM(U0_M1_JITTER_TEST, sd_param->u0_m1_jitter),
		/* */
		__CMP_RAW_PARAM(-1, NULL),
	};
#endif
	int spec_by_param = 0;
//	struct device *dev = prd->dev;
	/* spec reading */
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	int16_t *rawdata_buf[MAX_TEST_CNT] = {
		[0] = prd->m2_buf_even_rawdata,
		[1] = prd->m2_buf_odd_rawdata,
	};
	int row_size = param->row;
	int col_size = param->col;
	int lower = 0, upper = 0;
	int invalid = 0;
	int test_cnt = 0;
	int sel_m1 = 0;
	int opt = 1;
	int ret = 0;

	if (prd_get_cmp_tool_str(type, 0) == NULL) {
		t_prd_err(prd, "invalid type, %d\n", type);
		return -EINVAL;
	}

	snprintf(lower_str, sizeof(lower_str), prd_get_cmp_tool_str(type, 0));
	snprintf(upper_str, sizeof(upper_str), prd_get_cmp_tool_str(type, 1));

	sel_m1 = prd_check_type(prd, type, 2);
	if (sel_m1 < 0) {
		return sel_m1;
	}

	if (sel_m1) {
		row_size = param->row;
		col_size = param->m1_col;
		rawdata_buf[0] = prd->m1_buf_even_rawdata;
		rawdata_buf[1] = prd->m1_buf_odd_rawdata;
		test_cnt = param->m1_cnt;
		opt = 0;
	} else {
		test_cnt = param->m2_cnt;
	}

#if defined(__SIW_SUPPORT_PRD_SET_SD)
	table = cmp_raw_tables;
	while (table->type != -1) {
		if (table->type == type) {
			if (table->value == NULL) {
				break;
			}

			lower = table->value[0];
			upper = table->value[1];
			break;
		}
		table++;
	}
	if (lower || upper) {
		spec_by_param = (1<<16);
	}
#endif

	if (spec_by_param) {
		t_prd_info(prd, "type %d, skip reading spec file\n", type);
		goto skip_file;
	}

#if !defined(__SIW_SUPPORT_PRD_SET_SD_ONLY)
	if (prd->file_rd_off) {
		t_prd_info(prd, "skip data comparison by file_rd_off\n");
		siw_prd_buf_snprintf(prd->buf_write, 0,
			"skip data comparison by file_rd_off\n");
		return 0;
	}

	ret = prd_get_limit(prd,
				lower_str, &lower);
	if (ret < 0) {
		return ret;
	}

	ret = prd_get_limit(prd,
				upper_str, &upper);
	if (ret < 0) {
		return ret;
	}
#endif

skip_file:
	invalid |= !!(lower < 0);
	invalid |= !!(upper < 0);
	invalid |= !!(lower >= upper);

	if (invalid) {
		t_prd_err(prd, "wrong condition: type %d, lower %d, upper %d %s\n",
			type, lower, upper,
			(spec_by_param) ? STR_SPEC_BY_PARAM : "");

		siw_prd_buf_snprintf(prd->buf_write, 0,
			"wrong condition: type %d, lower %d, upper %d %s\n",
			type, lower, upper,
			(spec_by_param) ? STR_SPEC_BY_PARAM : "");

		return -EINVAL;
	}

	prd->image_lower = lower;
	prd->image_upper = upper;

	ret = prd_compare_tool(prd, test_cnt,
				rawdata_buf, row_size, col_size, type,
				opt | spec_by_param);

	return ret;
}

/*
 * SIW TOUCH IC F/W Stop HandShake
 */
static int prd_ctrl_firm_not_allowed(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;

	if (siw_addr_is_skip(reg->prd_ic_ait_start_reg) ||
		siw_addr_is_skip(reg->prd_ic_ait_data_readystatus)) {
		return 1;
	}

	return 0;
}

static int prd_stop_firmware(struct siw_hal_prd_data *prd, u32 wdata, int flag)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 read_val;
	u32 check_data = 0;
	int try_cnt = 0;
	int ret = 0;

	if (prd_ctrl_firm_not_allowed(dev)) {
		return 0;
	}

	/*
	 * STOP F/W to check
	 */
	ret = siw_hal_write_value(dev, reg->prd_ic_ait_start_reg, wdata);
	if (ret < 0) {
		goto out;
	}
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_start_reg, &check_data);
	if (ret < 0) {
		goto out;
	}
#if 0
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_start_reg, &check_data);
	if (ret < 0) {
		goto out;
	}
#endif
	t_prd_info_flag(prd, flag, "check_data : %Xh\n", check_data);

	try_cnt = 1000;
	do {
		--try_cnt;
		if (try_cnt == 0) {
			t_prd_err(prd, "[ERR] stop FW: timeout, %08Xh\n", read_val);
			ret = -ETIMEDOUT;
			goto out;
		}
		siw_hal_read_value(dev, reg->prd_ic_ait_data_readystatus, &read_val);
		t_prd_dbg_base_flag(prd, flag, "Read RS_IMAGE = [%x] , OK RS_IMAGE = [%x]\n",
			read_val, (u32)RS_IMAGE);
		touch_msleep(2);
	} while (read_val != (u32)RS_IMAGE);

out:
	return ret;
}

static int prd_start_firmware(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 const cmd = IT_IMAGE_NONE;
	u32 check_data = 0;
	u32 read_val = 0;
	int ret = 0;

	if (prd_ctrl_firm_not_allowed(dev)) {
		return 0;
	}

	/* Release F/W to operate */
	ret = siw_hal_write_value(dev, reg->prd_ic_ait_start_reg, cmd);
	if (ret < 0) {
		goto out;
	}
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_start_reg, &check_data);
	if (ret < 0) {
		goto out;
	}
#if 0
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_start_reg, &check_data);
	if (ret < 0) {
		goto out;
	}
#endif
	t_prd_dbg_base(prd, "check_data : %Xh\n", check_data);

	//for test
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_data_readystatus, &read_val);
	if (ret < 0) {
		goto out;
	}
	t_prd_dbg_base(prd, "Read RS_IMAGE = [%x]\n", read_val);

out:
	return ret;
}

static int prd_read_rawdata(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int __m1_frame_size = ctrl->m1_frame_size;
	int __m2_frame_size = ctrl->m2_frame_size;
	u32 raw_offset_info = 0;
	u32 raw_data_offset[MAX_TEST_CNT] = {0, };
	int16_t *buf_rawdata[MAX_TEST_CNT] = {
		[0] = prd->m2_buf_even_rawdata,
		[1] = prd->m2_buf_odd_rawdata,
	};
	int16_t *tmp_buf = prd->m1_buf_tmp;
	int16_t *raw_buf;
	int addr = reg->prd_m1_m2_raw_offset;
	int sel_m1 = 0;
	int i, j = 0;
	int ret = 0;

	if (__m1_frame_size & 0x3)
		__m1_frame_size = (((__m1_frame_size >> 2) + 1) << 2);
	if (__m2_frame_size & 0x3)
		__m2_frame_size = (((__m2_frame_size >> 2) + 1) << 2);

	ret = siw_hal_read_value(dev,
				addr,
				&raw_offset_info);
	if (ret < 0) {
		goto out;
	}

	switch (touch_chip_type(ts)) {
	case CHIP_SW1828:
	case CHIP_SW42103:
		raw_data_offset[0] = RAW_OFFSET_ODD(raw_offset_info);
		raw_data_offset[1] = RAW_OFFSET_EVEN(raw_offset_info);
		break;
	default:
		raw_data_offset[0] = RAW_OFFSET_EVEN(raw_offset_info);
		raw_data_offset[1] = RAW_OFFSET_ODD(raw_offset_info);
		break;
	}
	t_prd_info(prd, "test[%04Xh] type %d, even offset %04Xh, odd offset %04Xh\n",
		addr, type, raw_data_offset[0], raw_data_offset[1]);

	sel_m1 = prd_check_type(prd, type, 0);
	if (sel_m1 < 0) {
		ret = sel_m1;
		goto out;
	}

	if (sel_m1) {
		buf_rawdata[0] = prd->m1_buf_even_rawdata;
		buf_rawdata[1] = prd->m1_buf_odd_rawdata;

		for (i = 0; i < param->m1_cnt; i++) {
			raw_buf = buf_rawdata[i];

			if (raw_buf == NULL) {
				t_prd_err(prd, "reading rawdata(%d) failed: NULL buf\n", type);
				ret = -EFAULT;
				goto out;
			}

			/* raw data read */
			memset(raw_buf, 0, __m1_frame_size);
			memset(tmp_buf, 0, __m1_frame_size);

			ret = prd_read_raw_memory(prd, raw_data_offset[i], tmp_buf, __m1_frame_size);
			if (ret < 0) {
				goto out;
			}

			for (j = 0; j < param->row; j++) {
				raw_buf[j<<1]   = tmp_buf[j];
				raw_buf[(j<<1)+1] = tmp_buf[param->row+j];
			}

		}
	} else {
		for (i = 0; i < param->m2_cnt; i++) {
			if (buf_rawdata[i] == NULL) {
				t_prd_err(prd, "reading rawdata(%d) failed: NULL buf\n", type);
				ret = -EFAULT;
				goto out;
			}

			/* raw data read */
			memset(buf_rawdata[i], 0, __m2_frame_size);

			ret = prd_read_raw_memory(prd, raw_data_offset[i], buf_rawdata[i], __m2_frame_size);
			if (ret < 0) {
				goto out;
			}
		}
	}

out:
	return ret;
}

enum {
	PRD_TUNE_LEFT = 0,
	PRD_TUNE_RIGHT = 1,
	/* */
	PRD_TUNE_GOFT_M1 = 0x10,
	PRD_TUNE_GOFT_M2,
};

static void prd_tune_display_goft(struct siw_hal_prd_data *prd, char *tc_tune_code,
			int offset, int type, int result_on, int lr)
{
	struct siw_hal_prd_tune *tune = &prd->tune;
//	struct device *dev = prd->dev;
	char *log_buf = prd->log_buf;
	char *sign_str = NULL;
	int sign_mask = BIT(4);
	int data_mask = BIT(4) - 1;
	int sign = 0;
	int data = 0;
	int size = 0;

	/* __SIW_SUPPORT_PRD_TUNE_FLEX */
	if (tune->type) {
		struct code_goft_tune_hdr *tune_hdr = NULL;

		tune_hdr = (struct code_goft_tune_hdr *)&tc_tune_code[offset];

		switch (type) {
		case PRD_TUNE_GOFT_M1:
			sign = tune_hdr->goft_tune_m1_sign;
			data = tune_hdr->goft_tune_m1;
			break;
		case PRD_TUNE_GOFT_M2:
			sign = tune_hdr->goft_tune_m2_sign;
			data = tune_hdr->goft_tune_m2;
			break;
		default:
			t_prd_err(prd, "unknown tune type, %d\n", type);
			return;
		}
	} else {
		offset += (type == PRD_TUNE_GOFT_M2);
		sign = tc_tune_code[offset] & sign_mask;
		data = tc_tune_code[offset] & data_mask;
	}

	t_prd_dbg_base(prd, "tune_display(goft): data offset %Xh\n", offset);

	sign_str = (sign || !data) ? " " : "-";

	size = siw_prd_log_buf_snprintf(log_buf, 0,
				"GOFT(%s) tune : ", (lr == PRD_TUNE_LEFT) ? "L" : "R");

	size += siw_prd_log_buf_snprintf(log_buf, size, "%s%d ",
				sign_str, data);

	size += siw_prd_log_end(prd, log_buf, size);

	if (result_on == RESULT_ON) {
		prd_write_file(prd, log_buf, TIME_INFO_SKIP);
	}
}

static void prd_tune_display_loft(struct siw_hal_prd_data *prd, char *tc_tune_code,
			int offset, int result_on, int lr)
{
	struct siw_hal_prd_tune *tune = &prd->tune;
//	struct device *dev = prd->dev;
	char *log_buf = prd->log_buf;
	char *tune_data = &tc_tune_code[offset];
	char *sign_str = NULL;
	int sign_mask = BIT(5);
	int data_mask = BIT(5) - 1;
	int sign = 0;
	int data = 0;
	int size = 0;
	int i = 0;

	t_prd_dbg_base(prd, "tune_display(loft): data offset %Xh\n", offset);

	size = siw_prd_log_buf_snprintf(log_buf, 0,
				"LOFT(%s) tune : ", (lr == PRD_TUNE_LEFT) ? "L" : "R");

	for (i = 0; i < tune->ch; i++) {
		sign = (*tune_data) & sign_mask;
		data = (*tune_data) & data_mask;
		sign_str = (sign || !data) ? " " : "-";
		size += siw_prd_log_buf_snprintf(log_buf, size, "%s%d ",
					sign_str, data);
		tune_data++;
	}

	size += siw_prd_log_end(prd, log_buf, size);

	if (result_on == RESULT_ON) {
		prd_write_file(prd, log_buf, TIME_INFO_SKIP);
	}
}

/*
 *	tune code result check
 */
static int prd_do_read_tune_code(struct siw_hal_prd_data *prd, u8 *buf, int type, int result_on)
{
	struct siw_hal_prd_tune *tune = &prd->tune;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 grp_offset = 0;
	u32 tune_code_offset = 0;
	u32 offset = 0;
	int ret = 0;

	if (siw_addr_is_skip(reg->prd_tune_result_offset)) {
		t_prd_info(prd, "tune code reading deactivated\n");
		return 0;
	}

	ret = siw_hal_read_value(dev,
				reg->prd_tune_result_offset,
				&tune_code_offset);
	if (ret < 0) {
		goto out;
	}
	offset = (tune_code_offset >> 16) & 0xFFFF;

	t_prd_info(prd, "tune_code: offset %Xh, size %Xh\n",
		offset, tune->code_size);

	ret = prd_read_raw_memory(prd, offset, buf, tune->code_size);
	if (ret < 0) {
		goto out;
	}

	if (result_on == RESULT_ON) {
		prd_write_file(prd, "\n[Read Tune Code]\n", TIME_INFO_SKIP);
	}

	/* __SIW_SUPPORT_PRD_TUNE_FLEX */
	switch (type) {
	case U3_M2_RAWDATA_TEST:
	case U3_M1_RAWDATA_TEST:
		grp_offset = tune->grp_u3_offset;
		break;
	case U0_M2_RAWDATA_TEST:
	case U0_M1_RAWDATA_TEST:
		grp_offset = tune->grp_u0_offset;
		break;
	}

	switch (type) {
	case U3_M1_RAWDATA_TEST:
	case U0_M1_RAWDATA_TEST:
		prd_tune_display_goft(prd, buf,
					grp_offset + tune->code_l_goft_offset,
					PRD_TUNE_GOFT_M1,
					result_on, PRD_TUNE_LEFT);
		prd_tune_display_goft(prd, buf,
					grp_offset + tune->code_r_goft_offset,
					PRD_TUNE_GOFT_M1,
					result_on, PRD_TUNE_RIGHT);

		prd_tune_display_loft(prd, buf,
					grp_offset + tune->code_l_m1_goft_offset,
					result_on, PRD_TUNE_LEFT);
		prd_tune_display_loft(prd, buf,
					grp_offset + tune->code_r_m1_goft_offset,
					result_on, PRD_TUNE_RIGHT);
		break;
	case U3_M2_RAWDATA_TEST:
	case U0_M2_RAWDATA_TEST:
		prd_tune_display_goft(prd, buf,
					grp_offset + tune->code_l_goft_offset,
					PRD_TUNE_GOFT_M2,
					result_on, PRD_TUNE_LEFT);
		prd_tune_display_goft(prd, buf,
					grp_offset + tune->code_r_goft_offset,
					PRD_TUNE_GOFT_M2,
					result_on, PRD_TUNE_RIGHT);

		prd_tune_display_loft(prd, buf,
					grp_offset + tune->code_l_g1_goft_offset,
					result_on, PRD_TUNE_LEFT);
		prd_tune_display_loft(prd, buf,
					grp_offset + tune->code_l_g2_goft_offset,
					result_on, PRD_TUNE_LEFT);
		prd_tune_display_loft(prd, buf,
					grp_offset + tune->code_l_g3_goft_offset,
					result_on, PRD_TUNE_LEFT);
		prd_tune_display_loft(prd, buf,
					grp_offset + tune->code_r_g1_goft_offset,
					result_on, PRD_TUNE_RIGHT);
		prd_tune_display_loft(prd, buf,
					grp_offset + tune->code_r_g2_goft_offset,
					result_on, PRD_TUNE_RIGHT);
		prd_tune_display_loft(prd, buf,
					grp_offset + tune->code_r_g3_goft_offset,
					result_on, PRD_TUNE_RIGHT);
		break;
	}
	if (result_on == RESULT_ON) {
		prd_write_file(prd, "\n", TIME_INFO_SKIP);
	}

out:
	return ret;
}

static int prd_read_tune_code(struct siw_hal_prd_data *prd, int type, int result_on)
{
	struct siw_hal_prd_tune *tune = &prd->tune;
	u8 *buf = NULL;
	int ret = 0;

	if (!tune->code_size) {
		t_prd_warn(prd, "no tune configuration\n");
		return ret;
	}

	buf = kzalloc(tune->code_size, GFP_KERNEL);
	if (buf == NULL) {
		t_prd_err(prd, "reading tune code failed: NULL buf\n");
		return -ENOMEM;
	}

	ret = prd_do_read_tune_code(prd, buf, type, result_on);

	kfree(buf);

	return ret;
}

/*
 * print Frame Data & Compare rawdata & save result
 * return result Pass:0 Fail:1
 */
static int prd_conrtol_rawdata_result(struct siw_hal_prd_data *prd, int type, int result_on)
{
	struct siw_hal_prd_param *param = &prd->param;
	int print_type[2] = {0, };
	int i,test_cnt;
	int sel_m1 = 0;
	int opt = 1;
	int size = 0;
	int result = 0;

	sel_m1 = prd_check_type(prd, type, 1);
	if (sel_m1 < 0) {
		return 1;
	}

	if (sel_m1) {
		print_type[0] = M1_EVEN_DATA;
		print_type[1] = M1_ODD_DATA;
		test_cnt = param->m1_cnt;
		opt = 0;
	} else {
		print_type[0] = M2_EVEN_DATA;
		print_type[1] = M2_ODD_DATA;
		test_cnt = param->m2_cnt;
	}

	/* print Raw Data */
	for (i = 0; i < test_cnt; i++) {
		size = prd_print_rawdata(prd, prd->buf_write, print_type[i], size, opt);
	}

	if (size)
		size += siw_prd_buf_snprintf(prd->buf_write, size, "\n");

	if (result_on == RESULT_ON) {
		/* Frame Data write to Result File */
		prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);

		memset(prd->buf_write, 0, PRD_BUF_SIZE);

		/* rawdata compare result(pass : 0 fail : 1) */
		result = prd_compare_rawdata(prd, type);
		prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);
	}

	return result;
}

/*
 * Index "result_on" -> Save Result File ON: 1 , OFF: 0
 * return - Pass: 0 , Fail: 1
 */
static int prd_do_rawdata_test(struct siw_hal_prd_data *prd,
				int type, int result_on)
{
//	struct device *dev = prd->dev;
	const char *test_str = NULL;
	char test_type[32] = {0, };
	int check_tune_code = 0;
	int cmd = 0;
	int os_test = 0;
	int ret = 0;

	switch (type) {
	case OPEN_NODE_TEST:
		cmd = prd->sd_cmd.cmd_open_node;
		os_test = 1;
		break;
	case SHORT_NODE_TEST:
		cmd = prd->sd_cmd.cmd_short_node;
		os_test = 1;
		break;
	case SHORT_FULL_TEST:
		cmd = prd->sd_cmd.cmd_short_full;
		os_test = 1;
		break;
	case U3_M2_RAWDATA_TEST:
	case U3_M1_RAWDATA_TEST:
	case U0_M2_RAWDATA_TEST:
	case U0_M1_RAWDATA_TEST:
		check_tune_code = 1;
		/* fall thorugh */
	case U3_BLU_JITTER_TEST:
	case U3_JITTER_TEST:
	case U3_M1_JITTER_TEST:
	case U0_JITTER_TEST:
	case U0_M1_JITTER_TEST:
		break;
	default:
		t_prd_err(prd, "rawdata: test type not defined, %d\n", type);
		return 1;
	}

	test_str = prd_get_test_str(type);

	t_prd_info(prd, "========%s========\n", test_str);

	if (result_on == RESULT_ON) {
		snprintf(test_type, sizeof(test_type), "\n[%s]\n", test_str);
		/* Test Type Write */
		prd_write_file(prd, test_type, TIME_INFO_SKIP);
	}

	if (os_test) {
		if (!cmd) {
			t_prd_err(prd, "os test: test type %d, but command is zero\n", type);
			return 1;
		}
	}

	/* Test Start & Finish Check */
	ret = prd_write_test_mode(prd, type);
	if (ret < 0) {
		return ret;
	}
	if (!ret) {
		return 1;
	}

	if (os_test) {
		ret = prd_control_open_short_result(prd, type, result_on);
	} else {
		/* Read Frame Data */
		prd_read_rawdata(prd, type);

		/* Compare Spec & Print & Save result*/
		ret = prd_conrtol_rawdata_result(prd, type, result_on);
	}

	/* tune code result check */
	if (check_tune_code) {
		prd_read_tune_code(prd, type, result_on);
	}

	return ret;
}

static int prd_rawdata_test(struct siw_hal_prd_data *prd,
				int type, int result_on)
{
	int os_test = 0;
	int ret = 0;

	switch (type) {
	case IRQ_TEST:
		return prd_irq_test(prd, result_on);
	case OPEN_NODE_TEST:
	case SHORT_NODE_TEST:
	case SHORT_FULL_TEST:
		os_test = 1;
		break;
	}

	ret = prd_do_rawdata_test(prd, type, result_on);
	if (ret < 0) {
		t_prd_err(prd, "%s(%d) failed, %d\n",
			(os_test) ? "open short test" : "rawdata test",
			type, ret);
	}

	return ret;
}

static void prd_firmware_version_log(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	char *log_buf = prd->log_buf;
	int boot_mode = 0;
	int ret = 0;

	memset(log_buf, 0, sizeof(prd->log_buf));

	boot_mode = siw_touch_boot_mode_check(dev);
	if (boot_mode >= MINIOS_MFTS_FOLDER)
		ret = prd_chip_info(dev);

	ret = snprintf(log_buf, PRD_LOG_BUF_SIZE,
				"======== Firmware Info ========\n");
	if (fw->version_ext) {
		ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
					"version : %08X\n",
					fw->version_ext);
	} else {
		ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
					"version : v%d.%02d\n",
					fw->v.version.major,
					fw->v.version.minor);
	}
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"revision : %d\n",
				fw->revision);
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"product id : %s\n",
				fw->product_id);

	prd_write_file(prd, log_buf, TIME_INFO_SKIP);
}

static void prd_ic_run_info_print(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	char *log_buf = prd->log_buf;
	int ret = 0;
	u32 rdata[4] = {0, };
	struct siw_hal_rw_multi multi[] = {
		{ 0, reg->info_lot_num, &rdata[0], sizeof(rdata[0]), "lot" },
		{ 0, reg->info_serial_num, &rdata[1], sizeof(rdata[1]), "sn" },
		{ 0, reg->info_date, &rdata[2], sizeof(rdata[2]), "date" },
		{ 0, reg->info_time, &rdata[3], sizeof(rdata[3]), "time" },
		{ -1, -1, NULL, }
	};

	memset(log_buf, 0, sizeof(prd->log_buf));

	ret = siw_hal_reg_rw_multi(dev, multi, "prd ic info");
	if (ret < 0) {
		return;
	}

	ret = snprintf(log_buf, PRD_LOG_BUF_SIZE,
				"\n===== Production Info =====\n");
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"lot : %d\n", rdata[0]);
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"serial : 0x%X\n", rdata[1]);
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"date : 0x%X 0x%X\n",
				rdata[2], rdata[3]);
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"date : %04d.%02d.%02d %02d:%02d:%02d Site%d\n",
				rdata[2] & 0xFFFF, (rdata[2] >> 16 & 0xFF),
				(rdata[2] >> 24 & 0xFF), rdata[3] & 0xFF,
				(rdata[3] >> 8 & 0xFF),
				(rdata[3] >> 16 & 0xFF),
				(rdata[3] >> 24 & 0xFF));

	prd_write_file(prd, log_buf, TIME_INFO_SKIP);
}

static int prd_write_test_control(struct siw_hal_prd_data *prd, u32 mode)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 test_mode_enter_cmt = mode;
	u32 test_mode_enter_check = -1;
	unsigned int delay_ms = 30;
	int addr = reg->prd_tc_test_mode_ctl;
	int ret = 0;

	if (siw_addr_is_skip(addr)) {
		return 0;
	}

	ret = siw_hal_write_value(dev,
				addr,
				test_mode_enter_cmt);
	if (ret < 0) {
		goto out;
	}
	t_prd_info(prd, "wr prd_tc_test_mode_ctl[%04Xh] = %d\n",
		addr, test_mode_enter_cmt);

	touch_msleep(delay_ms);

	ret = siw_hal_read_value(dev,
				addr,
				&test_mode_enter_check);
	if (ret < 0) {
		goto out;
	}
	t_prd_info(prd, "rd prd_tc_test_mode_ctl[%04Xh] = %d\n",
		addr, test_mode_enter_check);

out:
	return ret;
}

static int __prd_sd_pre(struct siw_hal_prd_data *prd, char *buf)
{
	struct device *dev = prd->dev;
	int ret = 0;

	ret = prd_write_test_control(prd, CMD_TEST_ENTER);
	if (ret < 0) {
		goto out;
	}

	ret = prd_chip_driving(dev, LCD_MODE_STOP);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int __prd_sd_post(struct siw_hal_prd_data *prd, char *buf)
{
	prd_write_file(prd, buf, TIME_INFO_SKIP);
	t_prd_info(prd, "%s \n", buf);

	prd_write_test_control(prd, CMD_TEST_EXIT);

	return 0;
}

#define __SHOW_SD_PARAM(_t, _r, _m)	\
		{ .type = _t, .result = _r, .msg = _m,	}

static int prd_show_do_sd(struct siw_hal_prd_data *prd, char *buf)
{
//	struct device *dev = prd->dev;
	struct siw_hal_prd_param *param = &prd->param;
#if defined(__SIW_SUPPORT_PRD_SET_SD)
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
#endif
	u32 sd_test_flag = param->sd_test_flag;
	int result_on = prd->result_on;
	int u3_rawdata_ret = 0;
	int u3_m1_rawdata_ret = 0;
	int open_ret = 0;
	int short_ret = 0;
	int short_full_ret = 0;
	int blu_jitter_ret = 0;
	int u3_jitter_ret = 0;
	int u3_m1_jitter_ret = 0;
	int irq_ret = 0;
	struct show_sd_list {
		int type;
		int *result;
		const char *msg;
	} *table, show_sd_tables[] = {
		__SHOW_SD_PARAM(IRQ_TEST, &irq_ret, "IRQ"),
		__SHOW_SD_PARAM(U3_M2_RAWDATA_TEST, &u3_rawdata_ret, "U3 Raw Data"),
		__SHOW_SD_PARAM(U3_M1_RAWDATA_TEST, &u3_m1_rawdata_ret, "U3 M1 Raw Data"),
		__SHOW_SD_PARAM(U3_JITTER_TEST, &u3_jitter_ret, "U3 Jitter"),
		__SHOW_SD_PARAM(U3_M1_JITTER_TEST, &u3_m1_jitter_ret, "U3 M1 Jitter"),
		__SHOW_SD_PARAM(U3_BLU_JITTER_TEST, &blu_jitter_ret, "Blu Jitter"),
		__SHOW_SD_PARAM(OPEN_NODE_TEST, &open_ret, NULL),
		__SHOW_SD_PARAM(SHORT_NODE_TEST, &short_ret, NULL),
		__SHOW_SD_PARAM(SHORT_FULL_TEST, &short_full_ret, NULL),
		__SHOW_SD_PARAM(-1, NULL, NULL),
	};
	int size = 0;
	int ret;

	ret = __prd_sd_pre(prd, buf);
	if (ret < 0) {
		goto out;
	}

#if defined(__SIW_SUPPORT_PRD_SET_SD)
	if ((sd_param->set_sd_class == SET_SD_CLASS_SD) && sd_param->set_sd_flag) {
		sd_test_flag = sd_param->set_sd_flag;
		t_prd_info(prd, "sd_test_flag locally changed %Xh(%Xh)\n",
			sd_test_flag, param->sd_test_flag);
	}
#endif	/* __SIW_SUPPORT_PRD_SET_SD */

	/*
	 * pass : 0, fail : 1
	 * tunecode - pass : 0, fail : 2
	 */
	table = show_sd_tables;
	while (table->type != -1) {
		if (sd_test_flag & BIT(table->type)) {
			ret = prd_rawdata_test(prd, table->type, result_on);
			*table->result = ret;
			if (ret < 0) {
				goto out;
			}
		}
		table++;
	}

	size += siw_snprintf(buf, size,
				"\n========RESULT=======\n");

	table = show_sd_tables;
	while (table->type != -1) {
		if (sd_test_flag & BIT(table->type)) {
			if (table->msg != NULL) {
				size += siw_snprintf_sd_result(buf, size,
							table->msg, *table->result);
			}
		}
		table++;
	}

	if (sd_test_flag & \
		(OPEN_NODE_TEST_FLAG | SHORT_NODE_TEST_FLAG | SHORT_FULL_TEST_FLAG)) {
		if (!open_ret && !short_ret && !short_full_ret) {
			size += siw_snprintf(buf, size,
						"Channel Status : Pass\n");
		} else {
			size += siw_snprintf(buf, size,
						"Channel Status : Fail (open:%s/short:%s)\n",
						(open_ret) ? "F" : "P",
						(short_full_ret | short_ret) ? "F" : "P");
		}
	}

	__prd_sd_post(prd, buf);

out:
	return size;
}

static ssize_t prd_show_sd(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	char log_tmp[64] = {0, };
	int size = 0;
	int ret = 0;

	ret = prd_drv_exception_check(prd);
	if (ret) {
		size += siw_snprintf(buf, size,
					"drv exception(%d) detected, test canceled\n", ret);
		t_prd_err(prd, "%s", buf);
		goto out;
	}

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U3) {
		size += siw_snprintf(buf, size,
					"LCD mode is not U3. Test Result : Fail\n");
		goto out;
	}

	/* ic rev check - MINIOS mode, MFTS mode check */
	ret = prd_ic_exception_check(prd, buf);
	if (ret > 0) {
		t_prd_err(prd, "ic exception(%d) detected, test canceled\n", ret);
		size += siw_snprintf(buf, size,
					"ic exception(%d) detected, test canceled\n", ret);
		goto out;
	}

	siw_touch_mon_pause(dev);

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	/* file create , time log */
	snprintf(log_tmp, sizeof(log_tmp), "\nShow_sd(%Xh) Test Start",
		param->sd_test_flag);
	prd_write_file(prd, log_tmp, TIME_INFO_SKIP);
	prd_write_file(prd, "\n", TIME_INFO_WRITE);

	t_prd_info(prd, "show_sd test(%Xh) begins\n",
		param->sd_test_flag);

	prd_firmware_version_log(prd);
	prd_ic_run_info_print(prd);

	mutex_lock(&ts->lock);
	size = prd_show_do_sd(prd, buf);
	mutex_unlock(&ts->lock);

	prd_write_file(prd, "Show_sd Test End\n", TIME_INFO_WRITE);
	prd_log_file_size_check(prd);

	t_prd_info(prd, "show_sd test terminated\n\n");

	prd_chip_reset(dev);

	siw_touch_mon_resume(dev);

out:
	return (ssize_t)size;
}

static int prd_show_get_data_raw_core(struct device *dev,
					u8 *buf, int size,
					u32 cmd, u32 offset, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int do_fw_ctrl = !!(cmd != IT_DONT_USE_CMD);
	int ret = 0;

	if (do_fw_ctrl) {
		ret = prd_stop_firmware(prd, cmd, flag);
		if (ret < 0) {
			goto out;
		}
	}

	if (buf)
		memset(buf, 0, size);

	ret = prd_read_raw_memory(prd, offset, buf, size);
	if (ret < 0) {
		goto out;
	}

	ret = prd_read_raw_memory_self(prd, cmd, prd->buf_self);
	if (ret < 0) {
		goto out;
	}

	if (do_fw_ctrl) {
		ret = prd_start_firmware(prd);
		if (ret < 0) {
			goto out;
		}
	}

out:
	return ret;
}

static int prd_show_get_data_raw_prd(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	t_prd_info(prd, "======== CMD_RAWDATA_PRD ========\n");

	mutex_lock(&ts->lock);

	ret = prd_write_test_control(prd, CMD_TEST_ENTER);
	if (ret < 0) {
		goto out;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	ret = prd_chip_driving(dev, LCD_MODE_STOP);
	if (ret < 0) {
		goto out;
	}

	if (chip->lcd_mode == LCD_MODE_U3) {
		ret = prd_rawdata_test(prd, U3_M2_RAWDATA_TEST, RESULT_OFF);
	} else if (chip->lcd_mode == LCD_MODE_U0) {
		ret = prd_rawdata_test(prd, U0_M2_RAWDATA_TEST, RESULT_OFF);
	} else {
		t_prd_err(prd, "LCD mode is not U3 or U0!! current mode = %d\n",
				chip->lcd_mode);
		ret = -EINVAL;
		goto out;
	}

	siw_touch_irq_control(dev, INTERRUPT_ENABLE);

	if (ret < 0) {
		goto out;
	}

	ret = prd_write_test_control(prd, CMD_TEST_EXIT);
	if (ret < 0) {
		goto out;
	}

out:
	mutex_unlock(&ts->lock);

	prd_chip_reset(dev);

	return ret;
}

static int prd_show_get_data_raw_tcm(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int __m2_frame_size = ctrl->m2_frame_size;
	void *buf = prd->m2_buf_even_rawdata;
	int buf_size = prd->ctrl.m2_row_col_buf_size;
	int log_size = 0;
	int ret = 0;

	/* 	LCD mode check 	*/
#if 0
	if (chip->lcd_mode != LCD_MODE_U3) {
		t_prd_info(prd, "LCD mode is not U3. Test Result : Fail\n");
		goto out;
	}
#endif

	t_prd_info(prd, "======== rawdata(tcm) ========\n");
#if defined(__PRD_LOG_VIA_SHELL)
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== rawdata(tcm) ========\n");
#endif

	if (buf == NULL) {
		t_prd_err(prd, "getting raw tcm failed: NULL buf\n");
		return -EFAULT;
	}

	/* TCM Offset write 0 */
	ret = siw_hal_write_value(dev, reg->prd_serial_tcm_offset, 0);
	if (ret < 0) {
		goto out;
	}

	/*
	 * TCM Memory Access Select
	 * tc_mem_sel(0x0457) "RAW" or "BASE1" or "BASE2" or "BASE3"
	 */
	ret = siw_hal_write_value(dev, reg->prd_tc_mem_sel, TCM_MEM_RAW);
	if (ret < 0){
		goto out;
	}

	/* 	Read Rawdata	*/
	memset(buf, 0, buf_size);
	ret = siw_hal_reg_read(dev,
				reg->prd_tcm_base_addr,
				buf,
				__m2_frame_size);
	if (ret < 0){
		goto out;
	}

	/*	Print RawData Buffer	*/
	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, log_size, 0);

out:
	return ret;
}

static int prd_show_get_data_do_raw_ait(struct device *dev, u8*buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->m2_buf_even_rawdata;
	int ret = 0;

	ret = prd_show_get_data_raw_core(dev, pbuf, size,
				prd->img_cmd.raw, prd->img_offset.raw, flag);
	if (ret < 0) {
		goto out;
	}

	prd_swap_raw_memory(prd, PRD_BUF_SWAP_RAW, pbuf, size, 0, 0, 0);

out:
	return ret;
}

#if defined(__SIW_SUPPORT_PRD_RAW_CMP)
static void __prd_calc_buff_cmp_16(void *new_raw, void *old_raw, int row, int col, int opt)
{
	int16_t *__new_raw = new_raw;
	int16_t *__old_raw = old_raw;
	int new_val, old_val;
	int total = row * col;
	int i;

	if (opt & 0x01)
		total = row + col;

	for (i = 0; i < total ;i++) {
		new_val = __new_raw[i];
		old_val = __old_raw[i];

		__new_raw[i] = new_val - old_val;
		__old_raw[i] = new_val;
	}
}

static void __prd_calc_raw_cmp(struct siw_hal_prd_data *prd)
{
	struct siw_hal_prd_param *param = &prd->param;

	__prd_calc_buff_cmp_16(prd->m2_buf_even_rawdata, prd->m2_buf_back,
		param->row, param->col + param->col_add, 0);

	if (!prd->panel_type) {
		return;
	}

	__prd_calc_buff_cmp_16(prd->buf_self, prd->buf_self_back,
		param->row, param->col, 1);
}

static void __prd_show_raw_cmp(struct siw_hal_prd_data *prd)
{
	if (!(t_prd_dbg_flag & PRD_DBG_FLAG_RAW_CMP)) {
		return;
	}

	__prd_calc_raw_cmp(prd);

	if (prd->panel_type) {
		prd_print_rawdata_self(prd, prd->img_cmd.raw, prd->buf_write, M2_EVEN_DATA, 0, 0);
		return;
	}

	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, 0, 0);
}
#else	/* __SIW_SUPPORT_PRD_RAW_CMP */
static inline void __prd_show_raw_cmp(struct siw_hal_prd_data *prd)
{

}
#endif	/* __SIW_SUPPORT_PRD_RAW_CMP */

static int prd_show_get_data_raw_ait(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int ret = 0;

	t_prd_info(prd, "======== rawdata ========\n");
#if defined(__PRD_LOG_VIA_SHELL)
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== rawdata ========\n");
#endif

	ret = prd_show_get_data_do_raw_ait(dev,
				(u8 *)prd->m2_buf_even_rawdata,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	if (prd->panel_type) {
		prd_print_rawdata_self(prd, prd->img_cmd.raw, prd->buf_write, M2_EVEN_DATA, log_size, 0);
		goto out_done;
	}

	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, log_size, 0);

out_done:
	__prd_show_raw_cmp(prd);

out:
	return ret;
}

static int prd_show_get_data_do_ait_basedata(struct device *dev,
					u8 *buf, int size, int step, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u32 ait_cmd[MAX_TEST_CNT] = {prd->img_cmd.baseline_even, \
								prd->img_cmd.baseline_odd};
	u32 ait_offset[MAX_TEST_CNT] = {prd->img_offset.baseline_even, \
									prd->img_offset.baseline_odd};
	int16_t *buf_rawdata[MAX_TEST_CNT] = { \
		[0] = prd->m2_buf_even_rawdata, \
		[1] = prd->m2_buf_odd_rawdata, \
	};
	u8 *pbuf = (buf) ? buf : (u8 *)buf_rawdata[step];
	int ret = 0;

	ret = prd_show_get_data_raw_core(dev, pbuf, size,
				ait_cmd[step], ait_offset[step], flag);
	if (ret < 0) {
		goto out;
	}

	prd_swap_raw_memory(prd, PRD_BUF_SWAP_BASE, pbuf, size, 0, 0, 0);

out:
	return ret;
}

static int prd_show_get_data_ait_basedata(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u32 buf_type[MAX_TEST_CNT] = {M2_EVEN_DATA, M2_ODD_DATA};
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int i = 0;
	int ret = 0;

	t_prd_info(prd, "======== basedata ========\n");
#if defined(__PRD_LOG_VIA_SHELL)
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== basedata ========\n");
#endif

	for (i = 0; i < param->m2_cnt; i++) {
		ret = prd_show_get_data_do_ait_basedata(dev,
					NULL,
					size,
					i,
					0);
		if (ret < 0) {
			goto out;
		}

		if (prd->panel_type) {
			prd_print_rawdata_self(prd, prd->img_cmd.baseline_even, prd->buf_write, M2_EVEN_DATA, log_size, 0);
			goto out;
		}

		log_size = prd_print_rawdata(prd, prd->buf_write, buf_type[i], log_size, 0);
	#if !defined(__PRD_LOG_VIA_SHELL)
		log_size = 0;
	#endif
	}

out:
	return ret;
}

static int prd_show_get_data_do_filtered_deltadata(struct device *dev, u8 *buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int16_t *pbuf = (buf) ? (int16_t *)buf : prd->m2_buf_even_rawdata;
	int size_rd = (ctrl->delta_size<<PRD_RAWDATA_SZ_POW);
	int param_col = param->col;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_get_data_raw_core(dev, (u8 *)prd->buf_delta, size_rd,
				prd->img_cmd.f_delta, prd->img_offset.f_delta, flag);
	if (ret < 0) {
		goto out;
	}

	memset(pbuf, 0, size);

	for (i = 0; i < ctrl->m2_row_col_size; i++){
		row = i / param_col;
		col = i % param_col;
		pbuf[i] = prd->buf_delta[(row + 1)*(param_col + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_get_data_filtered_deltadata(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int ret = 0;

	t_prd_info(prd, "======== filtered deltadata ========\n");
#if defined(__PRD_LOG_VIA_SHELL)
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== filtered deltadata ========\n");
#endif

	ret = prd_show_get_data_do_filtered_deltadata(dev,
				(u8 *)prd->m2_buf_even_rawdata,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, log_size, 0);

out:
	return ret;
};

static int prd_show_get_data_do_deltadata(struct device *dev, u8 *buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int16_t *pbuf = (buf) ? (int16_t *)buf : prd->m2_buf_even_rawdata;
	int size_rd = (ctrl->delta_size<<PRD_RAWDATA_SZ_POW);
	int param_col = param->col;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_get_data_raw_core(dev, (u8 *)prd->buf_delta, size_rd,
				prd->img_cmd.delta, prd->img_offset.delta, flag);
	if (ret < 0) {
		goto out;
	}

	prd_swap_raw_memory(prd, PRD_BUF_SWAP_DELTA, prd->buf_delta,
		size_rd, param->row + 2, param->col + 2, 0);

	memset(pbuf, 0, size);

	for (i = 0; i < ctrl->m2_row_col_size; i++){
		row = i / param_col;
		col = i % param_col;
		pbuf[i] = prd->buf_delta[(row + 1)*(param_col + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_get_data_deltadata(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int ret = 0;

	t_prd_info(prd, "======== deltadata ========\n");
#if defined(__PRD_LOG_VIA_SHELL)
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== deltadata ========\n");
#endif

	ret = prd_show_get_data_do_deltadata(dev,
				(u8 *)prd->m2_buf_even_rawdata,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	if (prd->panel_type) {
		prd_print_rawdata_self(prd, prd->img_cmd.delta, prd->buf_write, M2_EVEN_DATA, log_size, 0);
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, log_size, 0);

out:
	return ret;

}

static int prd_show_get_data_do_labeldata(struct device *dev, u8 *buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u8 *pbuf = (buf) ? buf : prd->buf_label;
	int size_rd = ctrl->label_tmp_size;
	int param_col = param->col;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_get_data_raw_core(dev, (u8 *)prd->buf_label_tmp, size_rd,
				prd->img_cmd.label, prd->img_offset.label, flag);
	if (ret < 0) {
		goto out;
	}

	prd_swap_raw_memory(prd, PRD_BUF_SWAP_LABEL, prd->buf_label_tmp,
		size_rd, param->row + 2, param->col + 2, 1);

	memset(pbuf, 0, size);

	for (i = 0; i < ctrl->m2_row_col_size; i++){
		row = i / param_col;
		col = i % param_col;
		pbuf[i] = prd->buf_label_tmp[(row + 1)*(param_col + 2) + (col + 1)];
	}

out:
	return ret;

}

static int prd_show_get_data_labeldata(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = ctrl->m2_row_col_size;
	int log_size = 0;
	int ret = 0;

	t_prd_info(prd, "======== labeldata ========\n");
#if defined(__PRD_LOG_VIA_SHELL)
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== labeldata ========\n");
#endif

	ret = prd_show_get_data_do_labeldata(dev,
				(u8 *)prd->buf_label,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	if (prd->panel_type) {
		prd_print_rawdata_self(prd, prd->img_cmd.label, prd->buf_write, LABEL_DATA, log_size, 0);
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, LABEL_DATA, log_size, 0);

out:
	return ret;
}

static int prd_show_get_data_blu_jitter(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	t_prd_info(prd, "======== CMD_BLU_JITTER ========\n");

	mutex_lock(&ts->lock);

	ret = prd_write_test_control(prd, CMD_TEST_ENTER);
	if (ret < 0) {
		goto out;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	ret = prd_chip_driving(dev, LCD_MODE_STOP);
	if (ret < 0) {
		goto out;
	}

	ret = prd_rawdata_test(prd, U3_BLU_JITTER_TEST, RESULT_OFF);

	siw_touch_irq_control(dev, INTERRUPT_ENABLE);

	if (ret < 0) {
		goto out;
	}

	ret = prd_write_test_control(prd, CMD_TEST_EXIT);
	if (ret < 0) {
		goto out;
	}

out:
	mutex_unlock(&ts->lock);

	prd_chip_reset(dev);

	return ret;
}

static int prd_show_get_data_do_debug_buf(struct device *dev, u8 *buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->buf_debug;
	int ret = 0;

	ret = prd_show_get_data_raw_core(dev, pbuf, size,
				IT_DONT_USE_CMD, prd->img_offset.debug, flag);
	if (ret < 0) {
		goto out;
	}

	prd_swap_raw_memory(prd, PRD_BUF_SWAP_DEBUG, pbuf, size, 0, 0, 0);

out:
	return ret;
}

static int prd_show_get_data_debug_buf(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = (ctrl->debug_buf_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int ret = 0;

	t_prd_info(prd, "======== debugdata ========\n");
#if defined(__PRD_LOG_VIA_SHELL)
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== debugdata ========\n");
#endif

	ret = prd_show_get_data_do_debug_buf(dev,
				(u8 *)prd->buf_debug,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	if (prd->panel_type) {
		prd_print_rawdata_self(prd, prd->img_cmd.debug, prd->buf_write, DEBUG_DATA, log_size, 0);
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, DEBUG_DATA, log_size, 0);

out:
	return ret;
}


static ssize_t prd_show_get_data(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	switch (type) {
	case CMD_RAWDATA_PRD:
		ret = prd_show_get_data_raw_prd(dev);
		break;
	case CMD_RAWDATA_TCM:
		ret = prd_show_get_data_raw_tcm(dev);
		break;
	case CMD_RAWDATA_AIT:
		ret = prd_show_get_data_raw_ait(dev);
		break;
	case CMD_AIT_BASEDATA:
		ret = prd_show_get_data_ait_basedata(dev);
		break;
	case CMD_FILTERED_DELTADATA:
		ret = prd_show_get_data_filtered_deltadata(dev);
		break;
	case CMD_DELTADATA:
		ret = prd_show_get_data_deltadata(dev);
		break;
	case CMD_LABELDATA:
		ret = prd_show_get_data_labeldata(dev);
		break;
	case CMD_BLU_JITTER:
		ret = prd_show_get_data_blu_jitter(dev);
		break;
	case CMD_DEBUG_BUF:
		ret = prd_show_get_data_debug_buf(dev);
		break;
	default:
		t_prd_err(prd, "invalid get_data request CMD");
		ret = -EINVAL;
		break;
	};

	return ret;
}

static ssize_t prd_show_get_data_common(struct device *dev, char *buf, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int size = 0;
	int ret = 0;

	ret = prd_drv_exception_check(prd);
	if (ret) {
		size += siw_prd_buf_snprintf(buf, size,
					"drv exception(%d) detected, test canceled\n", ret);
		t_prd_err(prd, "%s", buf);
		return (ssize_t)size;
	}

	siw_touch_mon_pause(dev);
	ret = prd_show_get_data(dev, type);
	siw_touch_mon_resume(dev);
	if (ret < 0){
		t_prd_err(prd, "prd_show_get_data(%d) failed, %d\n",
			type, ret);
	}

	/*
	 * to prepare the response for APP I/F (not fixed)
	 */
#if defined(__PRD_LOG_VIA_SHELL)
	switch (type) {
	case CMD_BLU_JITTER:
		break;
	default:
		size += siw_prd_buf_snprintf(buf, size, "%s\n", prd->buf_write);
		break;
	}
#endif
	size += siw_prd_buf_snprintf(buf, size, "Get Data[%s] result:\n",
				prd_get_data_cmd_name[type]);
	size += siw_prd_buf_snprintf(buf, size, "%s\n",
				(ret < 0) ? "Fail" : "Pass");
	return (ssize_t)size;
}

static ssize_t prd_show_delta(struct device *dev, char *buf)
{
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_DELTADATA);
}

static ssize_t prd_show_label(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int size = 0;

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U3) {
		size += siw_prd_buf_snprintf(buf, size,
					"Current LCD mode(%d) is not U3, halted\n",
					chip->lcd_mode);
		return (ssize_t)size;
	}
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_LABELDATA);
}

static ssize_t prd_show_rawdata_prd(struct device *dev, char *buf)
{
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_RAWDATA_PRD);
}

static ssize_t prd_show_rawdata_tcm(struct device *dev, char *buf)
{
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_RAWDATA_TCM);
}

static ssize_t prd_show_rawdata_ait(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int size = 0;

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U3) {
		size += siw_prd_buf_snprintf(buf, size,
					"Current LCD mode(%d) is not U3, halted\n",
					chip->lcd_mode);
		return (ssize_t)size;
	}
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_RAWDATA_AIT);
}

static ssize_t prd_show_debug_buf(struct device *dev, char *buf)
{
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_DEBUG_BUF);
}

static ssize_t prd_show_basedata(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int size = 0;

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U3) {
		size += siw_prd_buf_snprintf(buf, size,
					"Current LCD mode(%d) is not U3, halted\n",
					chip->lcd_mode);
		return size;
	}

	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_AIT_BASEDATA);
}

static int prd_show_do_lpwg_sd(struct siw_hal_prd_data *prd, char *buf)
{
//	struct device *dev = prd->dev;
	struct siw_hal_prd_param *param = &prd->param;
#if defined(__SIW_SUPPORT_PRD_SET_SD)
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
#endif
	u32 lpwg_sd_test_flag = param->lpwg_sd_test_flag;
	int result_on = prd->result_on;
	int m1_rawdata_ret = 0;
	int m2_rawdata_ret = 0;
	int u0_jitter_ret = 0;
	int u0_m1_jitter_ret = 0;
	u32 sd_u0_test = 0;
	struct show_lpwg_sd_list {
		int type;
		int *result;
		const char *msg;
	} *table, show_lpwg_sd_tables[] = {
		__SHOW_SD_PARAM(U0_M2_RAWDATA_TEST, &m2_rawdata_ret, "U0 Raw Data"),
		__SHOW_SD_PARAM(U0_M1_RAWDATA_TEST, &m1_rawdata_ret, "U0 M1 Raw Data"),
		__SHOW_SD_PARAM(U0_JITTER_TEST, &u0_jitter_ret, "U0 Jitter"),
		__SHOW_SD_PARAM(U0_M1_JITTER_TEST, &u0_m1_jitter_ret, "U0 Jitter Self"),
		__SHOW_SD_PARAM(-1, NULL, NULL),
	};
	int size = 0;
	int ret = 0;

	ret = __prd_sd_pre(prd, buf);
	if (ret < 0) {
		goto out;
	}

#if defined(__SIW_SUPPORT_PRD_SET_SD)
	if ((sd_param->set_sd_class == SET_SD_CLASS_LPWG_SD) && sd_param->set_sd_flag) {
		lpwg_sd_test_flag = sd_param->set_sd_flag;
		t_prd_info(prd, "lpwg_sd_test_flag locally changed %Xh(%Xh)\n",
			lpwg_sd_test_flag, param->lpwg_sd_test_flag);
	}
#endif	/* __SIW_SUPPORT_PRD_SET_SD */

	/*
	 * pass : 0, fail : 1
	 * tunecode - pass : 0, fail : 2
	 */
	table = show_lpwg_sd_tables;
	while (table->type != -1) {
		if (lpwg_sd_test_flag & BIT(table->type)) {
			ret = prd_rawdata_test(prd, table->type, result_on);
			*table->result = ret;
			if (ret < 0) {
				goto out;
			}
		}
		table++;
	}

	size = siw_snprintf(buf, size, "\n========RESULT=======\n");

	sd_u0_test = lpwg_sd_test_flag & (U0_M2_RAWDATA_TEST_FLAG | U0_M1_RAWDATA_TEST_FLAG);

	if (sd_u0_test) {
		if (!m1_rawdata_ret && !m2_rawdata_ret) {
			size += siw_snprintf(buf, size,
						"LPWG RawData : Pass\n");
		} else {
			int pre_str = 0;

			size += siw_snprintf(buf, size,
						"LPWG RawData : Fail ");

			size += siw_snprintf(buf, size, "(");

			if (sd_u0_test & U0_M2_RAWDATA_TEST_FLAG) {
				size += siw_snprintf(buf, size, "m2 %d", m2_rawdata_ret);
				pre_str = 1;
			}

			if (sd_u0_test & U0_M1_RAWDATA_TEST_FLAG) {
				size += siw_snprintf(buf, size, "%sm1 %d",
					(pre_str) ? ", " : "", m1_rawdata_ret);
				pre_str = 1;
			}

			size += siw_snprintf(buf, size, ")");
		}
	}

	if (lpwg_sd_test_flag & U0_JITTER_TEST_FLAG) {
		size += siw_snprintf_sd_result(buf, size,
				"U0 Jitter", u0_jitter_ret);
	}

	if (lpwg_sd_test_flag & U0_M1_JITTER_TEST_FLAG) {
		size += siw_snprintf_sd_result(buf, size,
				"U0 M1 Jitter", u0_m1_jitter_ret);
	}

	__prd_sd_post(prd, buf);

out:
	return size;
}

static ssize_t prd_show_lpwg_sd(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	char log_tmp[64] = {0, };
	int size = 0;
	int ret = 0;

	ret = prd_drv_exception_check(prd);
	if (ret) {
		size += siw_snprintf(buf, size,
					"drv exception(%d) detected, test canceled\n", ret);
		t_prd_err(prd, "%s", buf);
		goto out;
	}

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U0) {
		size = siw_snprintf(buf, size,
					"LCD mode is not U0. Test Result : Fail\n");
		goto out;
	}

	siw_touch_mon_pause(dev);

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	/* file create , time log */
	snprintf(log_tmp, sizeof(log_tmp), "\nShow_lpwg_sd(%Xh) Test Start",
		param->lpwg_sd_test_flag);
	prd_write_file(prd, log_tmp, TIME_INFO_SKIP);
	prd_write_file(prd, "\n", TIME_INFO_WRITE);

	t_prd_info(prd, "show_lpwg_sd(%Xh) test begins\n",
		param->lpwg_sd_test_flag);

	prd_firmware_version_log(prd);
	prd_ic_run_info_print(prd);

	mutex_lock(&ts->lock);
	size = prd_show_do_lpwg_sd(prd, buf);
	mutex_unlock(&ts->lock);

	prd_write_file(prd, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	prd_log_file_size_check(prd);

	t_prd_info(prd, "show_lpwg_sd test terminated\n");

	prd_chip_reset(dev);

	siw_touch_mon_resume(dev);

out:
	return (ssize_t)size;
}


#define __PRD_FILE_RW_TEST

#if defined(__PRD_FILE_RW_TEST)

enum {
	PRD_FILE_TEST_OPTION_0	= 0,
	PRD_FILE_TEST_OPTION_1,
	PRD_FILE_TEST_OPTION_2,
	PRD_FILE_TEST_OPTION_MAX,
};

static void __prd_show_file_test_usage(struct siw_hal_prd_data *prd)
{
	t_prd_info(prd, "[Usage]\n");
	t_prd_info(prd, " Check : echo 0 {fname} > file_test\n");
	t_prd_info(prd, " Write : echo 1 {fname} > file_test\n");
	t_prd_info(prd, "       : Remove old file and create new one\n");
	t_prd_info(prd, " Write : echo 2 {fname} > file_test\n");
	t_prd_info(prd, "       : Rename old file and create new one\n");
}

static ssize_t prd_show_file_test(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	__prd_show_file_test_usage(prd);

	return 0;
}

static ssize_t prd_store_file_test(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	char *fname = NULL;
	int value = 0;
	int ret = 0;

	fname = touch_getname();
	if (fname == NULL) {
		t_prd_err(prd, "failed to allocate fname\n");
		return -ENOMEM;
	}

	if (sscanf(buf, "%d %s", &value, fname) <= 0) {
		siw_prd_sysfs_err_invalid_param(prd);
		__prd_show_file_test_usage(prd);
		goto out;
	}
	if (value >= PRD_FILE_TEST_OPTION_MAX) {
		t_prd_err(prd, "invalude option, %d\n", value);
		__prd_show_file_test_usage(prd);
		goto out;
	}
	if (fname == NULL) {
		t_prd_err(prd, "target file name is NULL\n");
		__prd_show_file_test_usage(prd);
		goto out;
	}

	ret = prd_vfs_file_chk(prd, fname, O_RDONLY, 0666, NULL);
	if (ret < 0) {
		goto out;
	}

	/* Check mode */
	if (value == PRD_FILE_TEST_OPTION_0) {
		goto out;
	}

	if (!ret) {
		if (value == PRD_FILE_TEST_OPTION_2) {	/* Rename */
			char *buf1 = NULL;
			char *buf2 = NULL;
			int i;

			buf1 = touch_getname();
			if (buf1 == NULL) {
				t_prd_err(prd, "failed to allocate name buf1\n");
				ret = -ENOMEM;
				goto out;
			}

			buf2 = touch_getname();
			if (buf2 == NULL) {
				t_prd_err(prd, "failed to allocate name buf2\n");
				ret = -ENOMEM;
				touch_putname(buf1);
				goto out;
			}

			for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
				if (i == 0) {
					snprintf(buf1, PATH_MAX, "%s", fname);
				} else {
					snprintf(buf1, PATH_MAX, "%s.%d", fname, i);
				}

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					ret = prd_vfs_file_remove(prd, buf1);
					if (ret < 0) {
						break;
					}
				} else {
					snprintf(buf2, PATH_MAX, "%s.%d", fname, (i + 1));

					ret = prd_vfs_file_rename(prd, buf1, buf2);
					if (ret < 0) {
						break;
					}
				}
			}

			touch_putname(buf2);
			touch_putname(buf1);

			if (ret < 0) {
				goto out;
			}
		} else {	/* Remove */
			ret = prd_vfs_file_remove(prd, fname);
			if (ret < 0) {
				goto out;
			}
		}
	}

	ret = prd_do_write_file(prd, fname, "[File Write Test]\n", TIME_INFO_WRITE);
	if (ret < 0) {
		t_prd_err(prd, "File Write Test failed\n");
	}

out:
	touch_putname(fname);

	return (ssize_t)count;
}
#else	/* __PRD_FILE_RW_TEST */
static ssize_t prd_show_file_test(struct device *dev, char *buf)
{
	t_dev_info(dev, "Nop ...\n");
	return 0;
}

static ssize_t prd_store_file_test(struct device *dev,
				const char *buf, size_t count)
{
	t_dev_info(dev, "Nop ...\n");
	return (ssize_t)count;
}
#endif	/* __PRD_FILE_RW_TEST */

enum {
	REPORT_END_RS_NG = 0x05,
	REPORT_END_RS_OK = 0xAA,
};

enum {
	REPORT_OFF = 0,
	REPORT_RAW,
	REPORT_BASE,
	REPORT_DELTA,
	REPORT_LABEL,
	REPORT_DEBUG_BUF,
	REPORT_MAX,
};

static const char *prd_app_mode_str[] = {
	[REPORT_OFF]		= "OFF",
	[REPORT_RAW]		= "RAW",
	[REPORT_BASE]		= "BASE",
	[REPORT_LABEL]		= "LABEL",
	[REPORT_DELTA]		= "DELTA",
	[REPORT_DEBUG_BUF]	= "DEBUG_BUF",
};

static ssize_t prd_show_app_op_end(struct device *dev, char *buf, int prev_mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	buf[0] = REPORT_END_RS_OK;
	if (prev_mode != REPORT_OFF) {
		prd->prd_app_mode = REPORT_OFF;
		ret = prd_start_firmware(prd);
		if (ret < 0) {
			t_prd_err(prd, "prd_start_firmware failed, %d\n", ret);
			buf[0] = REPORT_END_RS_NG;
		}
	}

	return 1;
}

static ssize_t prd_show_app_operator(struct device *dev, char *buf, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u8 *pbuf = (u8 *)prd->m2_buf_even_rawdata;
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int flag = PRD_SHOW_FLAG_DISABLE_PRT_RAW;
	int prev_mode = prd->prd_app_mode;
//	int ret = 0;

	if (mode < REPORT_MAX) {
		t_prd_dbg_base(prd, "show app mode : %s(%d), 0x%X\n",
				prd_app_mode_str[mode], mode, flag);
	}

	if (mode == REPORT_OFF) {
		size = prd_show_app_op_end(dev, buf, prev_mode);
		goto out;
	}

	if (mode < REPORT_MAX) {
		prd->prd_app_mode = mode;
	}

	switch (mode) {
	case REPORT_RAW:
		prd_show_get_data_do_raw_ait(dev, pbuf, size, flag);
		break;
	case REPORT_BASE:
		prd_show_get_data_do_ait_basedata(dev, pbuf, size, 0, flag);
		break;
	case REPORT_DELTA:
		prd_show_get_data_do_deltadata(dev, pbuf, size, flag);
		break;
	case REPORT_LABEL:
		size = ctrl->m2_row_col_size;
		pbuf = (u8 *)prd->buf_label,
		prd_show_get_data_do_labeldata(dev, pbuf, size, flag);
		break;
	case REPORT_DEBUG_BUF:
		size = ctrl->debug_buf_size;
		pbuf = (u8 *)prd->buf_debug,
		prd_show_get_data_do_debug_buf(dev, pbuf, size, flag);
		break;
	default:
		t_prd_err(prd, "unknown mode, %d\n", mode);
		if (prev_mode != REPORT_OFF) {
			prd_show_app_op_end(dev, buf, prev_mode);
			siw_touch_mon_resume(dev);
		}
		size = 0;
		break;
	}

	if (size) {
		memcpy(buf, pbuf, size);
	}

out:
	if (touch_test_prd_quirks(ts, PRD_QUIRK_RAW_RETURN_MODE_VAL)) {
		return (ssize_t)mode;
	}

	return (ssize_t)size;
}

static ssize_t prd_show_app_raw(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_RAW);
}

static ssize_t prd_show_app_base(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_BASE);
}

static ssize_t prd_show_app_label(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_LABEL);
}

static ssize_t prd_show_app_delta(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_DELTA);
}

static ssize_t prd_show_app_debug_buf(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_DEBUG_BUF);
}

static ssize_t prd_show_app_end(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_OFF);
}

static ssize_t prd_show_app_info(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	int temp = prd->sysfs_flag;

	memset(buf, 0, PRD_APP_INFO_SIZE);

	buf[0] = (temp & 0xff);
	buf[1] = ((temp >> 8) & 0xff);
	buf[2] = ((temp >> 16) & 0xff);
	buf[3] = ((temp >> 24) & 0xff);

	buf[8] = param->row;
	buf[9] = param->col;
	buf[10] = param->col_add;
	buf[11] = param->ch;
	buf[12] = param->m1_col;
	buf[13] = param->cmd_type;
	buf[14] = param->second_scr.bound_i;
	buf[15] = param->second_scr.bound_j;

	t_prd_info(prd,
		"prd info: f %08Xh, r %d, c %d, ca %d, ch %d, m1 %d, cmd %d, bi %d, bj %d\n",
		temp, param->row, param->col, param->col_add, param->ch, param->m1_col,
		param->cmd_type, param->second_scr.bound_i, param->second_scr.bound_j);

	if (prd->mon_flag) {
		siw_touch_mon_resume(dev);
	} else {
		siw_touch_mon_pause(dev);
	}
	prd->mon_flag = !prd->mon_flag;

	return PRD_APP_INFO_SIZE;
}

static ssize_t prd_show_dbg_mask(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int size = 0;

	size += siw_snprintf(buf, size,
				"prd->dbg_mask %08Xh\n",
				prd->dbg_mask);
	size += siw_snprintf(buf, size,
				"t_prd_dbg_flag %08Xh\n",
				t_prd_dbg_flag);
	size += siw_snprintf(buf, size,
				"t_prd_dbg_mask %08Xh\n",
				t_prd_dbg_mask);

	size += siw_snprintf(buf, size,
				"\nUsage:\n");
	size += siw_snprintf(buf, size,
				" prd->dbg_mask  : echo 0 {mask_value} > prd_dbg_mask\n");
	size += siw_snprintf(buf, size,
				" t_prd_dbg_flag : echo 8 {mask_value} > prd_dbg_mask\n");
	size += siw_snprintf(buf, size,
				" t_prd_dbg_mask : echo 9 {mask_value} > prd_dbg_mask\n");

	return (ssize_t)size;
}

static void prd_store_dbg_mask_usage(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	t_prd_info(prd, "Usage:\n");
	t_prd_info(prd, " prd->dbg_mask  : echo 0 {mask_value(hex)} > prd_dbg_mask\n");
	t_prd_info(prd, " t_prd_dbg_flag : echo 8 {mask_value(hex)} > prd_dbg_mask\n");
	t_prd_info(prd, " t_prd_dbg_mask : echo 9 {mask_value(hex)} > prd_dbg_mask\n");
}

static ssize_t prd_store_dbg_mask(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int type = 0;
	u32 old_value, new_value = 0;

	if (sscanf(buf, "%d %X", &type, &new_value) <= 0) {
		t_prd_err(prd, "Invalid param\n");
		prd_store_dbg_mask_usage(dev);
		return count;
	}

	switch (type) {
	case 0 :
		old_value = prd->dbg_mask;
		prd->dbg_mask = new_value;
		t_prd_info(prd, "prd->dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	case 8 :
		old_value = t_prd_dbg_flag;
		t_prd_dbg_flag = new_value;
		t_dev_info(dev, "t_prd_dbg_flag changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	case 9 :
		old_value = t_prd_dbg_mask;
		t_prd_dbg_mask = new_value;
		t_dev_info(dev, "t_prd_dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	default :
		prd_store_dbg_mask_usage(dev);
		break;
	}

	return count;
}

#if defined(__SIW_SUPPORT_PRD_SET_SD)
#define siw_prd_set_sd_sprintf(_buf, _title) \
		sprintf(_buf, "%-20s(%d) -", prd_get_test_str(_title), (_title))

#define t_prd_info_set_sd(_prd, _title) \
		t_prd_info(_prd, "  %-20s - %d\n", prd_get_test_str(_title), (_title))

static int prd_show_set_sd_item(char *buf, int size, char *title, int *value)
{
	int lower = 0;
	int upper = 0;
	int disabled = 0;
	int invalid = 0;

	if (value == NULL) {
		size = siw_snprintf(buf, size,
				" %s\n", title);
		return size;
	}

	lower = value[0];
	upper = value[1];

	disabled = !!(!lower && !upper);

	invalid |= !!(lower < 0);
	invalid |= !!(upper < 0);
	invalid |= !!(lower >= upper);

	size = siw_snprintf(buf, size,
				" %s lower %d, upper %d %s\n",
				title, lower, upper,
				(disabled) ? "(disabled)" :
				(invalid) ? "(invalid)" : "(enabled)");

	return size;
}

#define __SHOW_SET_SD_PARAM(_t, _v, _c)	\
		{ .type = _t, .value = _v, .cond = _c }

static ssize_t prd_show_set_sd(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
	struct show_set_sd_list {
		int type;
		int *value;
		int cond;
	} *table, show_store_set_sd_tables[] = {
		__SHOW_SET_SD_PARAM(U3_M2_RAWDATA_TEST, sd_param->u3_m2, 1),
		__SHOW_SET_SD_PARAM(U3_M1_RAWDATA_TEST, sd_param->u3_m1, 1),
		__SHOW_SET_SD_PARAM(U0_M2_RAWDATA_TEST, sd_param->u0_m2, 1),
		__SHOW_SET_SD_PARAM(U0_M1_RAWDATA_TEST, sd_param->u0_m1, 1),
		__SHOW_SET_SD_PARAM(U3_BLU_JITTER_TEST, sd_param->u3_blu_jitter, 1),
		__SHOW_SET_SD_PARAM(U3_JITTER_TEST, sd_param->u3_jitter, 1),
		__SHOW_SET_SD_PARAM(U3_M1_JITTER_TEST, sd_param->u3_m1_jitter, 1),
		__SHOW_SET_SD_PARAM(U0_JITTER_TEST, sd_param->u0_jitter, 1),
		__SHOW_SET_SD_PARAM(U0_M1_JITTER_TEST, sd_param->u0_m1_jitter, 1),
		__SHOW_SET_SD_PARAM(OPEN_NODE_TEST, NULL, 0),
		__SHOW_SET_SD_PARAM(SHORT_NODE_TEST, NULL, 0),
		__SHOW_SET_SD_PARAM(SHORT_FULL_TEST, NULL, 0),
		__SHOW_SET_SD_PARAM(IRQ_TEST, NULL, 0),
		__SHOW_SET_SD_PARAM(-1, NULL, 0),
	};
	u32 sd_test_flag = param->sd_test_flag;
	u32 lpwg_sd_test_flag = param->lpwg_sd_test_flag;
	char title[32] = {0, };
	int valid = 0;
	int disabled = 0;
	int invalid = 0;
	int type;
	int lower, upper;
	int size = 0;

	if (!sd_test_flag) {
		size += siw_snprintf(buf, size, "[sd - disabled]\n");
		goto skip_show_set_sd;
	}

	size += siw_snprintf(buf, size, "[sd]\n");

	size += siw_snprintf(buf, size, "sd_test_flag %Xh\n", sd_test_flag);

	table = show_store_set_sd_tables;
	while (table->type != -1) {
		type = table->type;
		if (sd_test_flag & BIT(type)) {
			siw_prd_set_sd_sprintf(title, type);
			size += prd_show_set_sd_item(buf, size, title,
						(table->cond) ? table->value : NULL);
		}
		table++;
	}
skip_show_set_sd:

	if (!lpwg_sd_test_flag) {
		size += siw_snprintf(buf, size, "[lpwg_sd - disabled]\n");
		goto skip_show_set_lpwg_sd;
	}

	size += siw_snprintf(buf, size, "[lpwg_sd]\n");

	size += siw_snprintf(buf, size, "lpwg_sd_test_flag %Xh\n", lpwg_sd_test_flag);

	table = show_store_set_sd_tables;
	while (table->type != -1) {
		type = table->type;
		if (lpwg_sd_test_flag & BIT(type)) {
			siw_prd_set_sd_sprintf(title, type);
			size += prd_show_set_sd_item(buf, size, title,
						(table->cond) ? table->value : NULL);
		}
		table++;
	}
skip_show_set_lpwg_sd:

	size += siw_snprintf(buf, size, "set_sd_flag %Xh(%d)\n",
			sd_param->set_sd_flag, sd_param->set_sd_class);

	if (!sd_test_flag && !lpwg_sd_test_flag) {
		return (ssize_t)size;
	}

	type = sd_param->last_type;
	lower = sd_param->last_val[0];
	upper = sd_param->last_val[1];
	if ((type >= U3_M2_RAWDATA_TEST) && (type < UX_INVALID)) {
		if (sd_test_flag & (1<<type)) {
			valid |= !!(1<<type);
		}
		if (lpwg_sd_test_flag & (1<<type)) {
			valid |= !!(1<<type);
		}
	}

	disabled = !!(valid && !lower && !upper);

	invalid |= !valid;
	invalid |= !!(lower < 0);
	invalid |= !!(upper < 0);
	invalid |= !!(lower >= upper);

	size += siw_snprintf(buf, size,
				"\nLast:\n");
	size += siw_snprintf(buf, size,
				" type %d, lower %d, upper %d %s\n",
				type,lower, upper,
				(disabled) ? "(disabled)" :
				(invalid) ? "(invalid)" : "(enabled)");

	size += siw_snprintf(buf, size,
				"\nUsage:\n");
	size += siw_snprintf(buf, size,
				" echo {type} {lower} {upper} > set_sd\n\n");

	return (ssize_t)size;
}

static void prd_store_set_sd_usage(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	u32 sd_test_flag = param->sd_test_flag;
	u32 lpwg_sd_test_flag = param->lpwg_sd_test_flag;
	u32 sd_usage_table[] = {
		U3_M2_RAWDATA_TEST,
		U3_M1_RAWDATA_TEST,
		U0_M2_RAWDATA_TEST,
		U0_M1_RAWDATA_TEST,
		U3_BLU_JITTER_TEST,
		U3_JITTER_TEST,
		U3_M1_JITTER_TEST,
		U0_JITTER_TEST,
		U0_M1_JITTER_TEST,
		OPEN_NODE_TEST,
		SHORT_NODE_TEST,
		SHORT_FULL_TEST,
		IRQ_TEST,
		-1,
	};
	u32 *table;

	if (!sd_test_flag && !lpwg_sd_test_flag) {
		return;
	}

	t_prd_info(prd, "Usage:\n");
	t_prd_info(prd, " echo {type} {lower} {upper} > set_sd\n");

	if (!sd_test_flag) {
		goto skip_store_set_sd_usage;
	}

	t_prd_info(prd, " [type for sd]\n");

	table = sd_usage_table;
	while (*table != -1) {
		if (sd_test_flag & BIT(*table)) {
			t_prd_info_set_sd(prd, *table);
		}
		table++;
	}
skip_store_set_sd_usage:

	if (!lpwg_sd_test_flag) {
		return;
	}

	t_prd_info(prd, " [type for lpwd_sd]\n");
	table = sd_usage_table;
	while (*table != -1) {
		if (lpwg_sd_test_flag & BIT(*table)) {
			t_prd_info_set_sd(prd, *table);
		}
		table++;
	}
}

enum {
	CMD_SET_SD_TEST_FLAG		= 32,
	/* */
	CMD_SET_SD_CTRL_FLAG		= 64,
};

static int __prd_store_set_sd_flag(struct device *dev, int type)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
	u32 sd_test_flag = param->sd_test_flag;
	u32 flag = 0;

	if (!sd_test_flag) {
		t_prd_info(prd, "set_sd: sd not supported\n");
		return -EINVAL;
	}

	if (!(sd_test_flag & BIT(type))) {
		t_prd_info(prd, "set_sd: set set_sd_flag, invalid type %d\n", type);
		return -EINVAL;
	}

	sd_param->set_sd_flag = BIT(type);

	flag = OPEN_SHORT_RESULT_DATA_FLAG |	\
		OPEN_SHORT_RESULT_RAWDATA_FLAG |	\
		OPEN_SHORT_RESULT_ALWAYS_FLAG;

	sd_param->set_sd_flag |= (sd_test_flag & flag);
	sd_param->set_sd_class = SET_SD_CLASS_SD;

	return 0;
}

static int __prd_store_set_lpwg_sd_flag(struct device *dev, int type)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
	u32 lpwg_sd_test_flag = param->lpwg_sd_test_flag;

	if (!lpwg_sd_test_flag) {
		t_prd_info(prd, "set_sd: lpwg_sd not supported\n");
		return -EINVAL;
	}

	if (!(lpwg_sd_test_flag & BIT(type))) {
		t_prd_info(prd, "set_sd: set set_sd_flag, invalid type %d\n", type);
		return -EINVAL;
	}

	sd_param->set_sd_flag = BIT(type);
	sd_param->set_sd_class = SET_SD_CLASS_LPWG_SD;

	return 0;
}

static int prd_store_set_sd_flag(struct device *dev,
				int cmd, int onoff, int type)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
//	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
	int ret = 0;

	if (cmd == CMD_SET_SD_TEST_FLAG) {
		if (!onoff) {
			sd_param->set_sd_flag = 0;
			sd_param->set_sd_class = SET_SD_CLASS_NONE;
			t_prd_info(prd, "set_sd: clr set_sd_flag\n");
			return 1;
		}

		/*
		 * echo 32 1 0 > set_sd
		 * => sd_param->set_sd_flag = (1<<0) if valid
		 * (U3_M2_RAWDATA_TEST = 0)
		 *
		 * echo 32 0 0 > set_sd
		 * => sd_param->set_sd_flag = 0
		 */
		if (TEST_CLASS_FLAG_SD & BIT(type)) {
			ret = __prd_store_set_sd_flag(dev, type);
		}

		if (TEST_CLASS_FLAG_LPWG_SD & BIT(type)) {
			ret = __prd_store_set_lpwg_sd_flag(dev, type);
		}

		if (!ret) {
			t_prd_info(prd, "set_sd: set set_sd_flag %s(%Xh)\n",
				prd_get_test_str(type), sd_param->set_sd_flag);
		}

		return 1;
	}

	if (cmd == CMD_SET_SD_CTRL_FLAG) {
		int data = 0;

		switch (type) {
		case 0:
			data = prd->result_on;
			prd->result_on = (onoff) ? RESULT_ON : RESULT_OFF;
			t_prd_info(prd, "set_sd: set ctrl_flag: result_on(%d -> %d)\n",
				data, prd->result_on);
			break;
		case 1:
			data = prd->file_wr_off;
			prd->file_wr_off = !!(onoff);
			t_prd_info(prd, "set_sd: set ctrl_flag: file_wr_off(%d -> %d)\n",
				data, prd->file_wr_off);
			break;
		case 2:
			data = prd->file_rd_off;
			prd->file_rd_off = !!(onoff);
			t_prd_info(prd, "set_sd: set ctrl_flag: file_rd_off(%d -> %d)\n",
				data, prd->file_rd_off);
			break;
		default:
			t_prd_err(prd, "set_sd: set ctrl_flag: unknown type %d\n", type);
			break;
		}

		return 1;
	}

	return 0;
}

#define __STORE_SET_SD_PARAM(_t, _v, _c)	\
		{ .type = _t, .value = _v, .cond = _c }

static ssize_t prd_store_set_sd(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
	struct store_set_sd_list {
		int type;
		int *value;
		int cond;
	} *table, store_set_sd_tables[] = {
		__STORE_SET_SD_PARAM(U3_M2_RAWDATA_TEST, sd_param->u3_m2, 1),
		__STORE_SET_SD_PARAM(U3_M1_RAWDATA_TEST, sd_param->u3_m1, 1),
		__STORE_SET_SD_PARAM(U0_M2_RAWDATA_TEST, sd_param->u0_m2, 1),
		__STORE_SET_SD_PARAM(U0_M1_RAWDATA_TEST, sd_param->u0_m1, 1),
		__STORE_SET_SD_PARAM(U3_BLU_JITTER_TEST, sd_param->u3_blu_jitter, 1),
		__STORE_SET_SD_PARAM(U3_JITTER_TEST, sd_param->u3_jitter, 1),
		__STORE_SET_SD_PARAM(U3_M1_JITTER_TEST, sd_param->u3_m1_jitter, 1),
		__STORE_SET_SD_PARAM(U0_JITTER_TEST, sd_param->u0_jitter, 1),
		__STORE_SET_SD_PARAM(U0_M1_JITTER_TEST, sd_param->u0_m1_jitter, 1),
		__STORE_SET_SD_PARAM(-1, NULL, 0),
	};
	u32 sd_test_flag = param->sd_test_flag;
	u32 lpwg_sd_test_flag = param->lpwg_sd_test_flag;
	int type = 0;
	int valid = 0;
	int lower = 0, upper = 0;
	int set = 0;

	if (sscanf(buf, "%d %d %d", &type, &lower, &upper) <= 0) {
		t_prd_err(prd, "Invalid param\n");
		prd_store_set_sd_usage(dev);
		return count;
	}

	if (prd_store_set_sd_flag(dev, type, lower, upper)) {
		return count;
	}

	if ((type >= U3_M2_RAWDATA_TEST) && (type < UX_INVALID)) {
		if (sd_test_flag & (1<<type)) {
			valid |= !!(1<<type);
		}
		if (lpwg_sd_test_flag & (1<<type)) {
			valid |= !!(1<<type);
		}
	}

	sd_param->last_type = type;
	sd_param->last_val[0] = lower;
	sd_param->last_val[1] = upper;

	if (!valid) {
		prd_store_set_sd_usage(dev);
		return count;
	}

	table = store_set_sd_tables;
	while (table->type != -1) {
		if (table->type == type) {
			if (table->value == NULL) {
				break;
			}

			if (!table->cond) {
				break;
			}

			table->value[0] = lower;
			table->value[1] = upper;
			set |= (1<<type);
			break;
		}
		table++;
	}

	if (!set) {
		return count;
	}

	if (prd_get_cmp_tool_str(type, 0) == NULL) {
		t_prd_info(prd, "set_sd: type %d, lower %d, upper %d\n",
			type, lower, upper);
	} else {
		t_prd_info(prd, "set_sd: %s %d, %s %d\n",
			prd_get_cmp_tool_str(type, 0), lower,
			prd_get_cmp_tool_str(type, 1), upper);
	}

	return count;
}
#endif /* __SIW_SUPPORT_PRD_SET_SD */


#if defined(__SIW_ATTR_PERMISSION_ALL)
#define __TOUCH_PRD_PERM	(S_IRUGO | S_IWUGO)
#else
#define __TOUCH_PRD_PERM	(S_IRUGO | S_IWUSR | S_IWGRP)
#endif

#define SIW_TOUCH_HAL_PRD_ATTR(_name, _show, _store)	\
		__TOUCH_ATTR(_name, __TOUCH_PRD_PERM, _show, _store)

#define _SIW_TOUCH_HAL_PRD_T(_name)	\
		touch_attr_##_name

static SIW_TOUCH_HAL_PRD_ATTR(sd, prd_show_sd, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(lpwg_sd, prd_show_lpwg_sd, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(delta, prd_show_delta, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(label, prd_show_label, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(rawdata_prd, prd_show_rawdata_prd, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(rawdata_tcm, prd_show_rawdata_tcm, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(rawdata, prd_show_rawdata_ait, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(base, prd_show_basedata, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(debug_buf, prd_show_debug_buf, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(file_test, prd_show_file_test, prd_store_file_test);

static SIW_TOUCH_HAL_PRD_ATTR(prd_app_raw, prd_show_app_raw, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_base, prd_show_app_base, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_label, prd_show_app_label, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_delta, prd_show_app_delta, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_debug_buf, prd_show_app_debug_buf, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_end, prd_show_app_end, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_info, prd_show_app_info, NULL);

static SIW_TOUCH_HAL_PRD_ATTR(prd_dbg_mask, prd_show_dbg_mask, prd_store_dbg_mask);

#if defined(__SIW_SUPPORT_PRD_SET_SD)
static SIW_TOUCH_HAL_PRD_ATTR(set_sd, prd_show_set_sd, prd_store_set_sd);
#endif

struct siw_hal_prd_attribute {
	struct attribute *attr;
	int flag;
};

#define _SIW_TOUCH_HAL_PRD_LIST_T(_name, _flag)	\
	{ .attr = &_SIW_TOUCH_HAL_PRD_T(_name).attr, .flag = _flag, }

static const struct siw_hal_prd_attribute siw_hal_prd_attribute_list_all[] = {
	_SIW_TOUCH_HAL_PRD_LIST_T(sd, PRD_SYS_EN_SD),
	_SIW_TOUCH_HAL_PRD_LIST_T(lpwg_sd, PRD_SYS_EN_LPWG_SD),
	_SIW_TOUCH_HAL_PRD_LIST_T(delta, PRD_SYS_EN_DELTA),
	_SIW_TOUCH_HAL_PRD_LIST_T(label, PRD_SYS_EN_LABEL),
	_SIW_TOUCH_HAL_PRD_LIST_T(rawdata_prd, PRD_SYS_EN_RAWDATA_PRD),
	_SIW_TOUCH_HAL_PRD_LIST_T(rawdata_tcm, PRD_SYS_EN_RAWDATA_TCM),
	_SIW_TOUCH_HAL_PRD_LIST_T(rawdata, PRD_SYS_EN_RAWDATA_AIT),
	_SIW_TOUCH_HAL_PRD_LIST_T(base, PRD_SYS_EN_BASE),
	_SIW_TOUCH_HAL_PRD_LIST_T(debug_buf, PRD_SYS_EN_DEBUG_BUF),
	_SIW_TOUCH_HAL_PRD_LIST_T(file_test, PRD_SYS_EN_FILE_TEST),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_raw, PRD_SYS_EN_APP_RAW),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_base, PRD_SYS_EN_APP_BASE),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_label, PRD_SYS_EN_APP_LABEL),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_delta, PRD_SYS_EN_APP_DELTA),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_debug_buf, PRD_SYS_EN_APP_DEBUG_BUF),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_end, PRD_SYS_EN_APP_END),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_info, PRD_SYS_EN_APP_INFO),
	/* */
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_dbg_mask, -1),
#if defined(__SIW_SUPPORT_PRD_SET_SD)
	_SIW_TOUCH_HAL_PRD_LIST_T(set_sd, PRD_SYS_EN_SD),
#endif
	/* */
	{ .attr = NULL, .flag = 0, },	/* end mask */
};

enum {
	PRD_SYS_ATTR_SIZE = ARRAY_SIZE(siw_hal_prd_attribute_list_all),
};

static struct attribute *siw_hal_prd_attribute_list[PRD_SYS_ATTR_SIZE + 1];

static const struct attribute_group __used siw_hal_prd_attribute_group = {
	.attrs = siw_hal_prd_attribute_list,
};

static int siw_hal_prd_create_group(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct kobject *kobj = &ts->kobj;
	struct siw_hal_prd_attribute *prd_attr;
	struct attribute **attr_actual = siw_hal_prd_attribute_list;
	int sysfs_flag = prd->sysfs_flag;
	int attr_size = PRD_SYS_ATTR_SIZE;
	int i = 0;
	int j = 0;
	int added = 0;
	int ret = 0;

	prd_attr = (struct siw_hal_prd_attribute *)siw_hal_prd_attribute_list_all;

	memset(siw_hal_prd_attribute_list, 0, sizeof(siw_hal_prd_attribute_list));

	for(i = 0; i < attr_size; i++){
		added = 0;
		if ((prd_attr->flag == -1) ||
			(prd_attr->flag & sysfs_flag)){
			attr_actual[j] = prd_attr->attr;
			j++;
			added = 1;
		}

		if (prd_attr->attr == NULL) {
			break;
		}

		t_prd_dbg_base(prd, "sysfs(-) %02d(%20s) %s\n",
			i, prd_attr->attr->name,
			(added) ? "added" : "not supported");

		prd_attr++;
	}

	ret = sysfs_create_group(kobj, &siw_hal_prd_attribute_group);
	if (ret < 0) {
		t_prd_err(prd, "sysfs_create_group failed, %d\n", ret);
		goto out;
	}

out:
	return ret;
}

static void siw_hal_prd_remove_group(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct kobject *kobj = &ts->kobj;

	sysfs_remove_group(kobj, &siw_hal_prd_attribute_group);
}

static void siw_hal_prd_free_buffer(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *buf_src = prd->buf_src;
	int buf_size = prd->buf_size;

	t_prd_dbg_base(prd, "[param free buffer]\n");

	if (prd->buf_src == NULL) {
		return;
	}

	prd->buf_src = NULL;
	prd->buf_size = 0;

	t_prd_info(prd, "buffer released: %p(%d)\n",
		buf_src, buf_size);

	kfree(buf_src);
}

static int siw_hal_prd_alloc_buffer(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u8 *buf;
	size_t buf_addr;
	int total_size;

	t_prd_dbg_base(prd, "[param alloc buffer]\n");

	total_size = (ctrl->m2_frame_size * 7);
	total_size += (ctrl->m1_frame_size * 3);
	total_size += (ctrl->delta_size<<PRD_RAWDATA_SZ_POW);
	total_size += (ctrl->debug_buf_size<<PRD_RAWDATA_SZ_POW);
	total_size += ctrl->label_tmp_size;
	total_size += ctrl->m2_row_col_size;
	total_size += (ctrl->buf_self_size<<1);

#if defined(__SIW_SUPPORT_PRD_RAW_CMP)
	total_size += ctrl->m2_frame_size;
	total_size += (ctrl->buf_self_size<<1);
#endif

	buf = kzalloc(total_size + 128, GFP_KERNEL);
	if (buf == NULL) {
		t_prd_err(prd, "falied to allocate buffer(%d)\n", total_size);
		return -ENOMEM;
	}

	t_prd_info(prd, "buffer allocted: %p(%d)\n", buf, total_size);

	prd->buf_src = buf;
	prd->buf_size = total_size;

#if 1
	buf_addr = (size_t)buf;
	if (buf_addr & 0x03) {
		buf_addr += 0x03;
		buf_addr &= ~0x03;
		buf = (u8 *)buf_addr;
		t_prd_info(prd, "buffer align tuned: %p\n", buf);
	}
#endif

	prd->m2_buf_even_rawdata = (int16_t *)buf;
	buf += ctrl->m2_frame_size;
	prd->m2_buf_odd_rawdata = (int16_t *)buf;
	buf += ctrl->m2_frame_size;
	prd->m2_buf_tmp = (int16_t *)buf;
	buf += ctrl->m2_frame_size;
#if defined(__SIW_SUPPORT_PRD_RAW_CMP)
	prd->m2_buf_back = (int16_t *)buf;
	buf += ctrl->m2_frame_size;
#endif

	prd->m1_buf_even_rawdata = (int16_t *)buf;
	buf += ctrl->m1_frame_size;
	prd->m1_buf_odd_rawdata = (int16_t *)buf;
	buf += ctrl->m1_frame_size;
	prd->m1_buf_tmp = (int16_t *)buf;
	buf += ctrl->m1_frame_size;

	prd->open_buf_result_rawdata = (int16_t *)buf;
	buf += ctrl->m2_frame_size;
	prd->short_buf_result_rawdata = (int16_t *)buf;
	buf += ctrl->m2_frame_size;
	prd->open_buf_result_data = (int16_t *)buf;
	buf += ctrl->m2_frame_size;
	prd->short_buf_result_data = (int16_t *)buf;
	buf += ctrl->m2_frame_size;

	prd->buf_delta = (int16_t *)buf;
	buf += (ctrl->delta_size<<PRD_RAWDATA_SZ_POW);

	prd->buf_debug = (int16_t *)buf;
	buf += (ctrl->debug_buf_size<<PRD_RAWDATA_SZ_POW);

	prd->buf_label_tmp = (u8 *)buf;
	buf += ctrl->label_tmp_size;

	prd->buf_label = (u8 *)buf;
	buf += ctrl->m2_row_col_size;

	prd->buf_self = (u16 *)buf;
	buf += (ctrl->buf_self_size<<1);
#if defined(__SIW_SUPPORT_PRD_RAW_CMP)
	prd->buf_self_back = (u16 *)buf;
	buf += (ctrl->buf_self_size<<1);
#endif

	t_prd_dbg_base(prd, "buf: m2_buf_even_rawdata         %p\n",
		prd->m2_buf_even_rawdata);
	t_prd_dbg_base(prd, "buf: m2_buf_odd_rawdata          %p\n",
		prd->m2_buf_odd_rawdata);
	t_prd_dbg_base(prd, "buf: m2_buf_tmp                  %p\n",
		prd->m2_buf_tmp);
	t_prd_dbg_base(prd, "buf: m1_buf_even_rawdata         %p\n",
		prd->m1_buf_even_rawdata);
	t_prd_dbg_base(prd, "buf: m1_buf_odd_rawdata          %p\n",
		prd->m1_buf_odd_rawdata);
	t_prd_dbg_base(prd, "buf: m1_buf_tmp                  %p\n",
		prd->m1_buf_tmp);
	t_prd_dbg_base(prd, "buf: open_buf_result_rawdata     %p\n",
		prd->open_buf_result_rawdata);
	t_prd_dbg_base(prd, "buf: short_buf_result_rawdata    %p\n",
		prd->short_buf_result_rawdata);
	t_prd_dbg_base(prd, "buf: open_buf_result_data        %p\n",
		prd->open_buf_result_data);
	t_prd_dbg_base(prd, "buf: short_buf_result_data       %p\n",
		prd->short_buf_result_data);
	t_prd_dbg_base(prd, "buf: buf_delta                   %p\n",
		prd->buf_delta);
	t_prd_dbg_base(prd, "buf: buf_debug                   %p\n",
		prd->buf_debug);
	t_prd_dbg_base(prd, "buf: buf_label_tmp               %p\n",
		prd->buf_label_tmp);
	t_prd_dbg_base(prd, "buf: buf_label                   %p\n",
		prd->buf_label);
	t_prd_dbg_base(prd, "buf: buf_self                    %p\n",
		prd->buf_self);
	return 0;
}

static void siw_hal_prd_parse_ctrl(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u32 row_size = param->row;
	u32 col_size = param->col;
	u32 col_add = param->col_add;

	t_prd_dbg_base(prd, "[param parse ctrl]\n");

	ctrl->m2_row_col_size = (row_size * col_size);
	ctrl->m2_row_col_buf_size = (row_size * (col_size + col_add));
	ctrl->m1_row_col_size = (row_size * param->m1_col);
	ctrl->m2_frame_size = (ctrl->m2_row_col_buf_size<<PRD_RAWDATA_SZ_POW);
	ctrl->m1_frame_size = (ctrl->m1_row_col_size<<PRD_RAWDATA_SZ_POW);
	ctrl->delta_size = ((row_size + 2) * (col_size + 2));
	ctrl->label_tmp_size = ((row_size + 2) * (col_size + 2));
	ctrl->debug_buf_size = ctrl->m2_row_col_buf_size;

	t_prd_dbg_base(prd, "ctrl: m2_row_col %d, m2_row_col_buf %d, m1_row_col %d\n",
		ctrl->m2_row_col_size, ctrl->m2_row_col_buf_size, ctrl->m1_row_col_size);
	t_prd_dbg_base(prd, "ctrl: m2_frame %d, m1_frame %d\n",
		ctrl->m2_frame_size, ctrl->m1_frame_size);
	t_prd_dbg_base(prd, "ctrl: delta %d, label_tmp %d\n",
		ctrl->delta_size, ctrl->label_tmp_size);

	switch (touch_chip_type(ts)) {
	case CHIP_LG4894:
	case CHIP_LG4895:
	case CHIP_LG4946:
	case CHIP_LG4951:
	case CHIP_SW1828:
	case CHIP_SW49105:
	case CHIP_SW49106:
	case CHIP_SW42103:
		prd->open_result_type = 0;
		break;
	default:
		prd->open_result_type = 1;
		break;
	}

	prd->short_result_type = prd->open_result_type;

	switch (prd->open_result_type) {
	case 1:
		ctrl->open_result_size = ctrl->m2_frame_size;
		ctrl->open_result_col = col_size + col_add;
		break;
	default:
		ctrl->open_result_size = ctrl->m1_frame_size;
		ctrl->open_result_col = param->m1_col;
		break;
	}

	ctrl->open_rawdata_size = ctrl->m2_frame_size;
	ctrl->open_rawdata_col = col_size + col_add;

	t_prd_dbg_base(prd, "ctrl: open: result: size %d, col %d\n",
		ctrl->open_result_size, ctrl->open_result_col);
	t_prd_dbg_base(prd, "ctrl: open: rawdata: size %d, col %d\n",
		ctrl->open_rawdata_size, ctrl->open_rawdata_col);

	switch (prd->short_result_type) {
	case 1:
		ctrl->short_result_size = (row_size * PRD_SHORT_COL_SZ)<<PRD_RAWDATA_SZ_POW;
		ctrl->short_result_col = PRD_SHORT_COL_SZ;
		break;
	default:
		ctrl->short_result_size = ctrl->m1_frame_size;
		ctrl->short_result_col = param->m1_col;
		break;
	}

	ctrl->short_rawdata_size = (row_size * PRD_SHORT_COL_SZ)<<PRD_RAWDATA_SZ_POW;
	ctrl->short_rawdata_col = PRD_SHORT_COL_SZ;

	t_prd_dbg_base(prd, "ctrl: short: result: size %d, col %d\n",
		ctrl->short_result_size, ctrl->short_result_col);
	t_prd_dbg_base(prd, "ctrl: short: rawdata: size %d, col %d\n",
		ctrl->short_rawdata_size, ctrl->short_rawdata_col);

	ctrl->short_full_result_size = ctrl->m2_frame_size;
	ctrl->short_full_result_col = col_size + col_add;
	ctrl->short_full_rawdata_size = ctrl->m2_frame_size;
	ctrl->short_full_rawdata_col = col_size + col_add;

	t_prd_dbg_base(prd, "ctrl: short full: result: size %d, col %d\n",
		ctrl->short_full_result_size, ctrl->short_full_result_col);
	t_prd_dbg_base(prd, "ctrl: short full: rawdata: size %d, col %d\n",
		ctrl->short_full_rawdata_size, ctrl->short_full_rawdata_col);

	ctrl->buf_self_size = (row_size + col_size)<<PRD_RAWDATA_SZ_POW;

	t_prd_dbg_base(prd, "ctrl: buf_self_size %d\n",
		ctrl->buf_self_size);
}

/* __SIW_SUPPORT_PRD_TUNE_FLEX */
static void siw_hal_prd_setup_tune(struct device *dev,
	struct siw_hal_prd_tune *tune, u32 ch)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u32 hdr = 0;
	u32 r_base = 0;
	int nd_offset = 0;

	if (prd->panel_type) {
		/* TBD */
		return;
	}

	tune->ch = ch;

	switch (touch_chip_type(ts)) {
	case CHIP_SW49408:
		tune->type = 1;
		break;
	}

	switch (tune->type) {
	case 2:	/* 'U3 - U0 - U2' or 'U3 - U0' */
		tune->magic_size = sizeof(u32);
		tune->hdr_size = sizeof(struct code_goft_tune_hdr);
		tune->unit_base = tune->hdr_size + (ch<<2) + (ch * nd_offset);
		tune->unit_size = tune->unit_base<<1;
		tune->grp_u3_offset = tune->magic_size;
		tune->grp_u0_offset = tune->magic_size + tune->unit_size;
		if (touch_mode_allowed(ts, LCD_MODE_U2)) {
			tune->grp_u2_offset = tune->magic_size + (tune->unit_size<<1);
		}
		tune->code_size = tune->magic_size + (tune->unit_size * 3);
		break;
	case 1:	/* 'U3 - U2 - U0' or 'U3 - U0' */
		tune->magic_size = sizeof(u32);
		tune->hdr_size = sizeof(struct code_goft_tune_hdr);
		tune->unit_base = tune->hdr_size + (ch<<2) + (ch * nd_offset);
		tune->unit_size = tune->unit_base<<1;
		tune->grp_u3_offset = tune->magic_size;
		if (touch_mode_allowed(ts, LCD_MODE_U2)) {
			tune->grp_u2_offset = tune->magic_size + tune->unit_size;
		}
		tune->grp_u0_offset = tune->magic_size + (tune->unit_size<<1);
		tune->code_size = tune->magic_size + (tune->unit_size * 3);
		break;
	default:
		tune->hdr_size = 2;
		tune->code_size = ((ch<<3)+4);
		break;
	}

	t_prd_dbg_base(prd, "tune: type %d, magic_size %d, hdr_size %d\n",
		tune->type, tune->magic_size, tune->hdr_size);
	t_prd_dbg_base(prd, "tune: u3_offset %Xh, u2_offset %Xh, u0_offset %Xh\n",
		tune->grp_u3_offset, tune->grp_u2_offset, tune->grp_u0_offset);

	hdr = tune->hdr_size;

	tune->code_l_goft_offset = 0;
	tune->code_l_m1_goft_offset = hdr;
	tune->code_l_g1_goft_offset = (tune->code_l_m1_goft_offset + ch);
	tune->code_l_g2_goft_offset = (tune->code_l_g1_goft_offset + ch);
	tune->code_l_g3_goft_offset = (tune->code_l_g2_goft_offset + ch);
	tune->code_l_nd_goft_offset = (nd_offset) ? (tune->code_l_g3_goft_offset + ch) : 0;

	r_base = (nd_offset) ? tune->code_l_nd_goft_offset : tune->code_l_g3_goft_offset;
	r_base += ch;

	tune->code_r_goft_offset = r_base;
	tune->code_r_m1_goft_offset = r_base + hdr;
	tune->code_r_g1_goft_offset = (tune->code_r_m1_goft_offset + ch);
	tune->code_r_g2_goft_offset = (tune->code_r_g1_goft_offset + ch);
	tune->code_r_g3_goft_offset = (tune->code_r_g2_goft_offset + ch);
	tune->code_r_nd_goft_offset = (nd_offset) ? (tune->code_r_g3_goft_offset + ch) : 0;
}

static void siw_hal_prd_parse_tune(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_tune *tune = &prd->tune;
	u32 ch = param->ch;

	t_prd_dbg_base(prd, "[param parse tune]\n");

	memset(tune, 0, sizeof(*tune));

	siw_hal_prd_setup_tune(dev, tune, ch);

	if (!tune->code_size) {
		t_prd_info(prd, "no tune configuration\n");
		return;
	}

	t_prd_dbg_base(prd, "tune: ch %d, code %d\n",
		ch, tune->code_size);

	if (prd->panel_type) {
		/* TBD */
	}

	t_prd_dbg_base(prd, "tune: l_goft %Xh, l_m1_oft %Xh\n",
		tune->code_l_goft_offset,
		tune->code_l_m1_goft_offset);
	t_prd_dbg_base(prd, "tune: l_g1_oft %Xh, l_g2_oft %Xh, l_g3_oft %Xh\n",
		tune->code_l_g1_goft_offset,
		tune->code_l_g2_goft_offset,
		tune->code_l_g3_goft_offset);
	t_prd_dbg_base(prd, "tune: r_goft %Xh, r_m1_oft %Xh\n",
		tune->code_r_goft_offset,
		tune->code_r_m1_goft_offset);
	t_prd_dbg_base(prd, "tune: r_g1_oft %Xh, r_g2_oft %Xh, r_g3_oft %Xh\n",
		tune->code_r_g1_goft_offset,
		tune->code_r_g2_goft_offset,
		tune->code_r_g3_goft_offset);
}

static void siw_hal_prd_set_offset(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u32 idx, addr, old_addr, flag;
	int i;

	t_prd_dbg_base(prd, "[param parse set offset]\n");

	flag = 0;
	for (i = 0; i < IMG_OFFSET_IDX_MAX ; i++) {
		if (!param->addr[i]) {
			continue;
		}

		idx = PRD_OFFSET_QUIRK_GET_IDX(param->addr[i]);
		addr = PRD_OFFSET_QUIRK_GET_OFFSET(param->addr[i]);

		if (flag & (1<<idx)) {
			t_prd_warn(prd, "dupliacted: %Xh (%d)\n", addr, idx);
			continue;
		}
		flag |= (1<<idx);

		switch (idx) {
		case IMG_OFFSET_IDX_RAW:
			old_addr = prd->img_offset.raw;
			prd->img_offset.raw = addr;
			break;
		case IMG_OFFSET_IDX_BASELINE_EVEN:
			old_addr = prd->img_offset.baseline_even;
			prd->img_offset.baseline_even = addr;
			break;
		case IMG_OFFSET_IDX_BASELINE_ODD:
			old_addr = prd->img_offset.baseline_odd;
			prd->img_offset.baseline_odd = addr;
			break;
		case IMG_OFFSET_IDX_DELTA:
			old_addr = prd->img_offset.delta;
			prd->img_offset.delta = addr;
			break;
		case IMG_OFFSET_IDX_LABEL:
			old_addr = prd->img_offset.label;
			prd->img_offset.label = addr;
			break;
		case IMG_OFFSET_IDX_F_DELTA:
			old_addr = prd->img_offset.f_delta;
			prd->img_offset.f_delta = addr;
			break;
		case IMG_OFFSET_IDX_DEBUG:
			old_addr = prd->img_offset.debug;
			prd->img_offset.debug = addr;
			break;
		case IMG_OFFSET_IDX_RAW_S:
			old_addr = prd->img_offset.raw_s;
			prd->img_offset.raw_s = addr;
			break;
		case IMG_OFFSET_IDX_BASELINE_S:
			old_addr = prd->img_offset.baseline_s;
			prd->img_offset.baseline_s = addr;
			break;
		case IMG_OFFSET_IDX_DELTA_S:
			old_addr = prd->img_offset.delta_s;
			prd->img_offset.delta_s = addr;
			break;
		case IMG_OFFSET_IDX_LABEL_S:
			old_addr = prd->img_offset.label_s;
			prd->img_offset.label_s = addr;
			break;
		case IMG_OFFSET_IDX_DEBUG_S:
			old_addr = prd->img_offset.debug_s;
			prd->img_offset.debug_s = addr;
			break;
		default:
			old_addr = 0;
			addr = 0;
			t_prd_info(prd, "unknown idx: %Xh (%d)\n", addr, idx);
			break;
		}

		if (addr) {
			t_prd_info(prd, "debug offset: %Xh(%d)\n", addr, idx);
		}
	}
}

static void siw_hal_prd_set_cmd(struct device *dev, int cmd_type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_img_cmd *img_cmd = &prd->img_cmd;
	char *type_name[] = {
		[PRD_CMD_TYPE_1] = "PRD_CMD_TYPE_1",
		[PRD_CMD_TYPE_2] = "PRD_CMD_TYPE_2",
	};

	t_prd_dbg_base(prd, "[param parse set cmd]\n");

	t_prd_info(prd, "cmd type: %s\n", type_name[cmd_type]);

	if (cmd_type == PRD_CMD_TYPE_2) {
		img_cmd->raw = IT_IMAGE_RAW;
		img_cmd->baseline_even = IT_IMAGE_BASELINE;
		img_cmd->baseline_odd = IT_IMAGE_BASELINE + 1;
		img_cmd->delta = IT_IMAGE_DELTA + 1;
		img_cmd->label = IT_IMAGE_LABEL + 1;
		img_cmd->f_delta = IT_IMAGE_FILTERED_DELTA + 1;
		img_cmd->debug = IT_IMAGE_DEBUG + 1;
		return;
	}

	img_cmd->raw = IT_IMAGE_RAW;
	img_cmd->baseline_even = IT_IMAGE_BASELINE;
	img_cmd->baseline_odd = IT_IMAGE_BASELINE;
	img_cmd->delta = IT_IMAGE_DELTA;
	img_cmd->label = IT_IMAGE_LABEL;
	img_cmd->f_delta = IT_IMAGE_FILTERED_DELTA;
	img_cmd->debug = IT_IMAGE_DEBUG;
}

static void siw_hal_prd_set_sd_cmd(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_sd_cmd *sd_cmd = &prd->sd_cmd;
	int chip_type = touch_chip_type(ts);

	sd_cmd->cmd_open_node = OPEN_NODE_TEST_POST_CMD;
	sd_cmd->cmd_short_node = SHORT_NODE_TEST_POST_CMD;
	sd_cmd->cmd_m2_rawdata = M2_RAWDATA_TEST_POST_CMD;
	sd_cmd->cmd_m1_rawdata = M1_RAWDATA_TEST_POST_CMD;
	sd_cmd->cmd_jitter = JITTER_TEST_POST_CMD;
	sd_cmd->cmd_m1_jitter = 0;
	sd_cmd->cmd_short_full = 0;

	switch (chip_type) {
	/* TBD */
	case CHIP_SW42000:
	case CHIP_SW42000A:
		sd_cmd->cmd_open_node = 1;
		sd_cmd->cmd_short_node = 2;
		sd_cmd->cmd_m2_rawdata = 5;
		sd_cmd->cmd_m1_rawdata = 3;
		sd_cmd->cmd_jitter = 13;
		sd_cmd->cmd_m1_jitter = 4;
		break;
	case CHIP_SW49407:
		sd_cmd->cmd_open_node = 1;
		sd_cmd->cmd_short_node = 2;
		sd_cmd->cmd_m2_rawdata = 5;
		sd_cmd->cmd_m1_rawdata = 3;
		sd_cmd->cmd_jitter = 6;
		sd_cmd->cmd_m1_jitter = 4;
		break;
	case CHIP_SW46104:
	case CHIP_SW49408:
	case CHIP_SW49409:
	case CHIP_SW49501:
		sd_cmd->cmd_open_node = 1;
		sd_cmd->cmd_short_node = 2;
		sd_cmd->cmd_m2_rawdata = 5;
		sd_cmd->cmd_m1_rawdata = 3;
		sd_cmd->cmd_jitter = 10;
		sd_cmd->cmd_m1_jitter = 4;
		sd_cmd->cmd_short_full = 12;
		break;
	case CHIP_LG4946:
		sd_cmd->cmd_jitter = 10;
		break;
	case CHIP_SW49106:
		sd_cmd->cmd_short_full = 13;
		break;
	}

	t_prd_info(prd,
		"cmd_open_node  : %d\n",
		sd_cmd->cmd_open_node);
	t_prd_info(prd,
		"cmd_short_node : %d\n",
		sd_cmd->cmd_short_node);
	t_prd_info(prd,
		"cmd_m2_rawdata : %d\n",
		sd_cmd->cmd_m2_rawdata);
	t_prd_info(prd,
		"cmd_m1_rawdata : %d\n",
		sd_cmd->cmd_m1_rawdata);
	t_prd_info(prd,
		"cmd_jitter     : %d\n",
		sd_cmd->cmd_jitter);
	if (sd_cmd->cmd_m1_jitter) {
		t_prd_info(prd,
			"cmd_m1_jitter  : %d\n",
			sd_cmd->cmd_m1_jitter);
	}
	if (sd_cmd->cmd_short_full) {
		t_prd_info(prd,
			"cmd_short_full : %d\n",
			sd_cmd->cmd_short_full);
	}
}

static void siw_hal_prd_parse_work(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	t_prd_dbg_base(prd, "[param parse work]\n");

	siw_hal_prd_set_offset(dev, param);

	siw_hal_prd_set_cmd(dev, param->cmd_type);

	siw_hal_prd_set_sd_cmd(prd);

	prd->sysfs_flag = PRD_SYS_ATTR_EN_FLAG & ~param->sysfs_off_flag;		//Disable quirk bits

	t_prd_info(prd, "sysfs flag: %Xh (%Xh)\n", prd->sysfs_flag, param->sysfs_off_flag);
}

static void siw_hal_prd_show_param(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int i;

	t_prd_dbg_base(prd, "[current param]\n");

	t_prd_dbg_base(prd, "param: chip_type %Xh, cmd_type %d\n",
			param->chip_type,
			param->cmd_type);
	if (param->name != NULL) {
		char *name;
		int name_idx = 0;
		while (1) {
			name = (char *)param->name[name_idx];
			if (name == NULL) {
				break;
			}
			t_prd_dbg_base(prd,
				"param: name[%d] %s\n",
				name_idx, name);
			name_idx++;
		}
	}

	for (i = 0; i < IMG_OFFSET_IDX_MAX; i++) {
		if (!param->addr[i])
			continue;

		t_prd_dbg_base(prd, "param: addr[%d] %Xh\n",
			i, param->addr[i]);
	}

	t_prd_info(prd, "param: row %d, col %d\n",
		param->row, param->col);
	t_prd_dbg_base(prd, "param: col_add %d, ch %d\n",
		param->col_add, param->ch);
	t_prd_dbg_base(prd, "param: m1_col %d, m1_cnt %d, m2_cnt %d\n",
		param->m1_col, param->m1_cnt, param->m2_cnt);

	t_prd_dbg_base(prd, "param: sysfs_off_flag %Xh\n",
		param->sysfs_off_flag);
}

static int siw_hal_prd_parse_param(struct device *dev, struct siw_hal_prd_param *param)
{
	siw_hal_prd_show_param(dev, param);

	siw_hal_prd_parse_ctrl(dev, param);

	siw_hal_prd_parse_tune(dev, param);

	siw_hal_prd_parse_work(dev, param);

	return siw_hal_prd_alloc_buffer(dev);
}

static void siw_hal_prd_param_quirks(struct device *dev,
			struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int data = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_LG4894:
		data = (param->name == prd_param_name_lg4894_k) |
			(param->name == prd_param_name_lg4894_lv) |
			(param->name == prd_param_name_lg4894_sf);

		reg->prd_m1_m2_raw_offset = (data) ? PRD_M1_M2_RAW_OFFSET : 0x286;

		if (PRD_M1_M2_RAW_OFFSET != reg->prd_m1_m2_raw_offset) {
			t_prd_info(prd, "%s[%s] reg quirks: %04Xh -> %04Xh\n",
				touch_chip_name(ts), fw->product_id,
				PRD_M1_M2_RAW_OFFSET, reg->prd_m1_m2_raw_offset);
		}
		break;
	default:
		break;
	}
}

static int siw_hal_prd_param_lookup(struct device *dev,
	struct siw_hal_prd_param *p)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = (struct siw_hal_prd_param *)prd_params;
	int found = 0;
	int len = 0;
	int idx = 0;

	while (param->chip_type) {
		if (param->chip_type == touch_chip_type(ts)) {
			found = !!(param->name == NULL);
			if (!found) {
				char *name;
				int name_idx = 0;
				while (1) {
					name = (char *)param->name[name_idx];
					if (name == NULL) {
						break;
					}
					len = strlen(name);
					found = !strncmp(fw->product_id, name, len);
					if (found) {
						break;
					}
					name_idx++;
				}
			}
		}

		if (found) {
			memcpy(p, param, sizeof(*param));

			t_prd_dbg_base(prd, "%s[%s] param %d selected\n",
				touch_chip_name(ts), fw->product_id, idx);

			return idx;
		}

		param++;
		idx++;
	}

	return -ENOENT;
}

static int siw_hal_prd_param_setup(struct device *dev,
		struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	/* to disable prd control for a specific type */
	if (!param->row || !param->col) {
		t_prd_info(prd, "%s[%s] disabled\n",
			touch_chip_name(ts), fw->product_id);
		return -EFAULT;
	}

	if (!touch_mode_allowed(ts, LCD_MODE_U0)) {
		param->lpwg_sd_test_flag = 0;
	}

	if (!param->sd_test_flag) {
		param->sysfs_off_flag |= PRD_SYS_EN_SD;
	}
	if (!param->lpwg_sd_test_flag) {
		param->sysfs_off_flag |= PRD_SYS_EN_LPWG_SD;
	}

	siw_hal_prd_param_quirks(dev, param);

	t_prd_info(prd, "sd_test_flag %Xh, lpwg_sd_test_flag %Xh\n",
		param->sd_test_flag, param->lpwg_sd_test_flag);
	t_prd_info(prd, "buf_swap_flag %Xh\n",
		param->buf_swap_flag);

	if (param->sd_test_flag & OPEN_SHORT_RESULT_ALWAYS_FLAG) {
		prd->dbg_mask |= PRD_DBG_OPEN_SHORT_DATA;
	}

	prd->panel_type = chip->opt.t_oled;

	prd->result_on = (param->ctrl_flag & CTRL_FLAG_RESULT_OFF) ?	\
					RESULT_OFF : RESULT_ON;

	prd->file_wr_off = !!(param->ctrl_flag & CTRL_FLAG_FILE_WR_OFF);
	prd->file_rd_off = !!(param->ctrl_flag & CTRL_FLAG_FILE_RD_OFF);

	ret = siw_hal_prd_parse_param(dev, param);
	if (ret < 0) {
		t_prd_err(prd, "%s[%s] param parsing failed, %d\n",
			touch_chip_name(ts), fw->product_id, ret);
		return ret;
	}

	memcpy(&prd->param, param, sizeof(*param));

	return 0;
}

static int siw_hal_prd_init_param(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param param = {0, };
	int ret = 0;

	if (t_prd_dbg_flag & PRD_DBG_FLAG_DISABLE) {
		goto out;
	}

	if (fw->invalid_pid) {
		t_prd_err(prd, "[init] invalid PID - \"%s\" (%03Xh)\n",
			fw->product_id, fw->invalid_pid);
		goto out;
	}

	ret = siw_hal_prd_param_lookup(dev, &param);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_prd_param_setup(dev, &param);

	return ret;

out:
	t_prd_info(prd, "%s[%s] param not found\n",
		touch_chip_name(ts), fw->product_id);

	return -EFAULT;
}

static void siw_hal_prd_free_param(struct device *dev)
{
	siw_hal_prd_free_buffer(dev);
}

static struct siw_hal_prd_data *siw_hal_prd_alloc(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd;

	prd = touch_kzalloc(dev, sizeof(*prd), GFP_KERNEL);
	if (!prd) {
		t_dev_err(dev,
				"failed to allocate memory for prd\n");
		goto out;
	}

	t_dev_dbg_base(dev, "create prd (0x%X)\n", (int)sizeof(*prd));

	prd->dev = ts->dev;

	ts->prd = prd;

#if defined(__SIW_SUPPORT_PRD_SET_SD)
	prd->sd_param.last_type = UX_INVALID;
#endif

	return prd;

out:
	return NULL;
}

static void siw_hal_prd_free(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	if (prd) {
		t_dev_dbg_base(dev, "free prd\n");

		ts->prd = NULL;
		touch_kfree(dev, prd);
	}
}

static int siw_hal_prd_create_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct device *idev = &ts->input->dev;
	struct kobject *kobj = &ts->kobj;
	struct siw_hal_prd_data *prd;
	int ret = 0;

	if (kobj->parent != idev->kobj.parent) {
		t_dev_err(dev, "Invalid kobject\n");
		return -EINVAL;
	}

	if (ts->prd)
		return 0;

	prd = siw_hal_prd_alloc(dev);
	if (!prd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = siw_hal_prd_init_param(dev);
	if (ret < 0) {
		/* Just skip sysfs generation */
		goto out_skip;
	}

	ret = siw_hal_prd_create_group(dev);
	if (ret < 0) {
		t_dev_err(dev, "%s prd sysfs register failed, %d\n",
				touch_chip_name(ts), ret);
		goto out_sysfs;
	}

	t_dev_dbg_base(dev, "%s prd sysfs registered\n",
			touch_chip_name(ts));

	prd->sysfs_done = 1;

out_skip:
	return 0;

out_sysfs:
	siw_hal_prd_free_param(dev);

	siw_hal_prd_free(dev);

out:
	return ret;
}

static void siw_hal_prd_remove_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct device *idev = &ts->input->dev;
	struct kobject *kobj = &ts->kobj;

	if (kobj->parent != idev->kobj.parent) {
		t_dev_err(dev, "Invalid kobject\n");
		return;
	}

	if (prd == NULL) {
		t_dev_dbg_base(dev, "prd sysfs not initialized\n");
		return;
	}

	if (!prd->sysfs_done) {
		t_dev_dbg_base(dev, "prd sysfs group not initialized\n");
		goto skip_prd_normal_remove;
	}

	siw_hal_prd_remove_group(dev);

	siw_hal_prd_free_param(dev);

skip_prd_normal_remove:
	siw_hal_prd_free(dev);

	t_dev_dbg_base(dev, "%s prd sysfs unregistered\n",
			touch_chip_name(ts));
}

int siw_hal_prd_sysfs(struct device *dev, int on_off)
{
	if (on_off == DRIVER_INIT) {
		return siw_hal_prd_create_sysfs(dev);
	}

	siw_hal_prd_remove_sysfs(dev);
	return 0;
}

int siw_hal_set_prd_file(struct device *dev, const char *path, int idx)
{
	char *prd_file_group[] = {
		[0]	= __prd_in_file,
		[1]	= __prd_in_file_m,
		[2] = __prd_out_file,
		[3] = __prd_out_file_mo_aat,
		[4] = __prd_out_file_mo_mfo,
		[5] = __prd_out_file_mo_mfl,
		[6] = __prd_out_file_mo_mcv,
	};

	if (!path) {
		t_dev_err(dev, "NULL prd path (%d)\n", idx);
		return -EINVAL;
	}

	if (idx >= ARRAY_SIZE(prd_file_group)) {
		t_dev_err(dev, "wrong prd file index, %d\n", idx);
		return -EINVAL;
	}

	t_dev_info(dev, "prd file setup: %s -> %s\n",
			prd_file_group[idx], path);
	memcpy(prd_file_group[idx], path, strlen(path));

	return 0;
}

__siw_setup_str("siw_prd_if=", prd_setup_if, __prd_in_file);
__siw_setup_str("siw_prd_if_m=", prd_setup_if_m, __prd_in_file_m);

__siw_setup_str("siw_prd_of_=", prd_setup_of, __prd_out_file);
__siw_setup_str("siw_prd_of_mo_aat=", prd_setup_of_aat, __prd_out_file_mo_aat);
__siw_setup_str("siw_prd_of_mo_mfo=", prd_setup_of_mfo, __prd_out_file_mo_mfo);
__siw_setup_str("siw_prd_of_mo_mfl=", prd_setup_of_mfl, __prd_out_file_mo_mfl);
__siw_setup_str("siw_prd_of_mo_mcv=", prd_setup_of_mcv, __prd_out_file_mo_mcv);

#endif	/* __SIW_SUPPORT_PRD */


