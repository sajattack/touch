/*
 * touch_sw49408.c - SiW touch driver glue for SW49408
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
#include <linux/spi/spi.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"

#define DEBUG

#define CHIP_ID						"49410"
#define CHIP_DEVICE_NAME			"SW49410"
#define CHIP_COMPATIBLE_NAME		"siw,sw49410"
#define CHIP_DEVICE_DESC			"SiW Touch SW49410 Driver"

#define CHIP_TYPE					CHIP_SW49410

#define CHIP_MODE_ALLOWED			(0 |	\
									LCD_MODE_BIT_U0 |	\
									LCD_MODE_BIT_U2_UNBLANK |	\
									LCD_MODE_BIT_U2 |	\
									LCD_MODE_BIT_U3 |	\
									LCD_MODE_BIT_U3_PARTIAL |	\
									LCD_MODE_BIT_U3_QUICKCOVER |	\
									LCD_MODE_BIT_STOP |	\
									0)

#define CHIP_FW_SIZE		        96 * 1024

#define CHIP_FLAGS					(0 |	\
									TOUCH_SKIP_ESD_EVENT |	\
									0)

#define CHIP_IRQFLAGS				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT)


#define CHIP_INPUT_ID_BUSTYPE		BUS_SPI
#define CHIP_INPUT_ID_VENDOR		0xABCD
#define CHIP_INPUT_ID_PRODUCT		0x9876
#define CHIP_INPUT_ID_VERSION		0x1234

#if defined(CONFIG_ARCH_EXYNOS5)
#define __CHIP_QUIRK_ADD			CHIP_QUIRK_NOT_SUPPORT_XFER
#else
#define __CHIP_QUIRK_ADD			0
#endif

#define CHIP_QUIRKS					(0 |	\
									CHIP_QUIRK_NOT_SUPPORT_ASC |	\
									CHIP_QUIRK_NOT_SUPPORT_WATCH |	\
									CHIP_QUIRK_NOT_SUPPORT_IME |	\
									__CHIP_QUIRK_ADD |	\
									0)

#define CHIP_BUS_TYPE				BUS_IF_SPI
#define	CHIP_BUF_SIZE				0
#define CHIP_SPI_MODE				SPI_MODE_0
#define CHIP_BPW					8
#define CHIP_MAX_FREQ				(5 * 1000* 1000)
#define CHIP_TX_HDR_SZ				SPI_BUS_TX_HDR_SZ
#define CHIP_TX_DUMMY_SZ			SPI_BUS_TX_DUMMY_SZ

#if (CHIP_MAX_FREQ >= (30 * 1000* 1000))
#define CHIP_RX_DUMMY_128BIT
#endif

#if defined(CHIP_RX_DUMMY_128BIT)
#define CHIP_RX_HDR_SZ				SPI_BUS_RX_HDR_SZ_128BIT
#define CHIP_RX_DUMMY_SZ			SPI_BUS_RX_DUMMY_SZ_128BIT
#else
#define CHIP_RX_HDR_SZ				SPI_BUS_RX_HDR_SZ_32BIT
#define CHIP_RX_DUMMY_SZ			SPI_BUS_RX_DUMMY_SZ_32BIT
#endif

static const struct tci_info chip_tci_info[2] = {
	[TCI_1] = {
		.tap_count		= 2,
		.min_intertap	= 0,
		.max_intertap	= 70,
		.touch_slop		= 100,
		.tap_distance	= 10,
		.intr_delay		= 0,
	},
	[TCI_2] = {
		.tap_count		= 0,
		.min_intertap	= 0,
		.max_intertap	= 70,
		.touch_slop		= 100,
		.tap_distance	= 255,
		.intr_delay		= 20,
	},
};

static const struct siw_hal_swipe_ctrl chip_swipe_info = {
	.mode	= SWIPE_LEFT_BIT | SWIPE_RIGHT_BIT,
	.info = {
		[SWIPE_R] = {
			.distance		= 10,
			.ratio_thres	= 100,
			.ratio_distance	= 2,
			.ratio_period	= 5,
			.min_time		= 0,
			.max_time		= 150,
			.area.x1		= 419,
			.area.y1		= 0,
			.area.x2		= 1019,
			.area.y2		= 679,
		},
		[SWIPE_L] = {
			.distance		= 10,
			.ratio_thres	= 100,
			.ratio_distance	= 2,
			.ratio_period	= 5,
			.min_time		= 0,
			.max_time		= 150,
			.area.x1		= 419,
			.area.y1		= 0,
			.area.x2		= 1019,
			.area.y2		= 679,
		},
	},
};

static const struct siw_hal_reg_quirk chip_reg_quirks[] = {
	{ .old_addr = SPR_CHIP_TEST, .new_addr = 0x045, },
	{ .old_addr = SPR_SUBDISP_STS, .new_addr = 0x022, },
	{ .old_addr = SPR_CODE_OFFSET, .new_addr = 0x086, },
	{ .old_addr = TC_VERSION, .new_addr = 0x25C, },
	{ .old_addr = TC_PRODUCT_ID1, .new_addr = 0x25E, },
	{ .old_addr = TC_PRODUCT_ID2, .new_addr = 0x25F, },
	{ .old_addr = TC_CONFDN_BASE_ADDR, .new_addr = 0x266, },
	{ .old_addr = TC_FLASH_DN_STS, .new_addr = 0x27F, },
	/* */
	{ .old_addr = (1<<31), .new_addr = 0, },	/* switch : don't show log */
	/* */
	{ .old_addr = CMD_RAW_DATA_REPORT_MODE_READ, .new_addr = 0x091, },
	{ .old_addr = CMD_RAW_DATA_REPORT_MODE_WRITE, .new_addr = 0xC59, },
	{ .old_addr = CODE_ACCESS_ADDR, .new_addr = 0xFD0, },
	{ .old_addr = SERIAL_DATA_OFFSET, .new_addr = 0x08B, },
	{ .old_addr = DATA_I2CBASE_ADDR, .new_addr = 0xFD1, },
	{ .old_addr = PRD_TCM_BASE_ADDR, .new_addr = 0xFD3, },
	{ .old_addr = TC_TSP_TEST_STS, .new_addr = 0x2A9, },
	{ .old_addr = TC_TSP_TEST_PF_RESULT, .new_addr = 0x2AA, },
	{ .old_addr = PRD_SERIAL_TCM_OFFSET, .new_addr = 0x08C, },
	{ .old_addr = PRD_TC_MEM_SEL, .new_addr = 0x84A, },
	{ .old_addr = PRD_TC_TEST_MODE_CTL, .new_addr = 0xF0E, },
	{ .old_addr = PRD_M1_M2_RAW_OFFSET, .new_addr = 0x27B, },
	{ .old_addr = PRD_TUNE_RESULT_OFFSET, .new_addr = 0x30F, },
	{ .old_addr = PRD_OPEN3_SHORT_OFFSET, .new_addr = 0x27D, },
	{ .old_addr = PRD_IC_AIT_START_REG, .new_addr = 0xF0C, },
	{ .old_addr = PRD_IC_AIT_DATA_READYSTATUS, .new_addr = 0xF04, },
	/* */
	{ .old_addr = EXT_WATCH_FONT_OFFSET, .new_addr = 0x08F, },
	{ .old_addr = EXT_WATCH_FONT_ADDR, .new_addr = 0xFD8, },
	/* */
	{ .old_addr = TCI_ENABLE_W, .new_addr = 0xC30, },
	/* */
	{ .old_addr = TAP_COUNT_W, .new_addr = 0xC31, },
	{ .old_addr = MIN_INTERTAP_W, .new_addr = 0xC32, },
	{ .old_addr = MAX_INTERTAP_W, .new_addr = 0xC33, },
	{ .old_addr = TOUCH_SLOP_W, .new_addr = 0xC34, },
	{ .old_addr = TAP_DISTANCE_W, .new_addr = 0xC35, },
	{ .old_addr = INT_DELAY_W, .new_addr = 0xC36, },
	{ .old_addr = ACT_AREA_X1_W, .new_addr = 0xC37, },
	{ .old_addr = ACT_AREA_Y1_W, .new_addr = 0xC38, },
	{ .old_addr = ACT_AREA_X2_W, .new_addr = 0xC39, },
	{ .old_addr = ACT_AREA_Y2_W, .new_addr = 0xC3A, },
	/* */
	{ .old_addr = SWIPE_ENABLE_W, .new_addr = 0xC40, },
	{ .old_addr = SWIPE_DIST_W, .new_addr = 0xC41, },
	{ .old_addr = SWIPE_RATIO_THR_W, .new_addr = 0xC42, },
	{ .old_addr = SWIPE_RATIO_DIST_W, .new_addr = 0xC43, },
	{ .old_addr = SWIPE_RATIO_PERIOD_W, .new_addr = 0xC44, },
	{ .old_addr = SWIPE_TIME_MIN_W, .new_addr = 0xC45, },
	{ .old_addr = SWIPE_TIME_MAX_W, .new_addr = 0xC46, },
	{ .old_addr = SWIPE_ACT_AREA_X1_W, .new_addr = 0xC47, },
	{ .old_addr = SWIPE_ACT_AREA_Y1_W, .new_addr = 0xC48, },
	{ .old_addr = SWIPE_ACT_AREA_X2_W, .new_addr = 0xC49, },
	{ .old_addr = SWIPE_ACT_AREA_Y2_W, .new_addr = 0xC4A, },
	{ .old_addr = SWIPE_FAIL_DEBUG_R, .new_addr = 0x06C, },
	{ .old_addr = SWIPE_DEBUG_R, .new_addr = 0x0D0, },
	/* */
	{ .old_addr = SPR_CHARGER_STS, .new_addr = 0xF20, },
	{ .old_addr = IME_STATE, .new_addr = 0xC61, },
	{ .old_addr = MAX_DELTA, .new_addr = 0x0AC, },
	{ .old_addr = TOUCH_MAX_R, .new_addr = 0x07C, },
	{ .old_addr = TOUCH_MAX_W, .new_addr = 0xC63, },
	{ .old_addr = CALL_STATE, .new_addr = 0xC64, },
	/* */
	{ .old_addr = ~0, .new_addr = ~0, },		// End signal
};

#if defined(__SIW_CONFIG_OF)
/*
 * of_device_is_compatible(dev->of_node, CHIP_COMPATIBLE_NAME)
 */
static const struct of_device_id chip_match_ids[] = {
	{ .compatible = CHIP_COMPATIBLE_NAME },
	{ },
};
#else
enum CHIP_CAPABILITY {
	CHIP_MAX_X			= 1440,	
	CHIP_MAX_Y			= 3120,
	CHIP_MAX_PRESSURE	= 255,
	CHIP_MAX_WIDTH		= 1440,
	CHIP_MAX_ORI		= 90,
	CHIP_MAX_ID			= 10,
	/* */
	CHIP_HW_RST_DELAY	= 90,
	CHIP_SW_RST_DELAY	= 10,
};

#define CHIP_PIN_RESET			32
#define CHIP_PIN_IRQ			31
#define CHIP_PIN_MAKER			-1
#define CHIP_PIN_VDD			-1
#define CHIP_PIN_VIO			-1

#if (CHIP_PIN_RESET == 0) || (CHIP_PIN_IRQ == 0)
	#error Assign external pin & flag first!!!
#endif
#endif	/* __SIW_CONFIG_OF */

/* use eg. cname=arc1 to change name */
static char chip_name[32] = CHIP_DEVICE_NAME;
module_param_string(cname, chip_name, sizeof(chip_name), 0);

/* use eg. dname=arc1 to change name */
static char chip_drv_name[32] = SIW_TOUCH_NAME;
module_param_string(dname, chip_drv_name, sizeof(chip_drv_name), 0);

/* use eg. iname=arc1 to change input name */
static char chip_idrv_name[32] = SIW_TOUCH_INPUT;
module_param_string(iname, chip_idrv_name, sizeof(chip_idrv_name), 0);

/* use eg. ename=arc1 to change ext watch name */
static char chip_ext_watch_name[32] = SIW_TOUCH_EXT_WATCH;
module_param_string(ename, chip_ext_watch_name, sizeof(chip_ext_watch_name), 0);

static const struct siw_touch_pdata chip_pdata = {
	/* Configuration */
	.chip_id			= CHIP_ID,
	.chip_name			= chip_name,
	.drv_name			= chip_drv_name,
	.idrv_name			= chip_idrv_name,
	.ext_watch_name		= chip_ext_watch_name,
	.owner				= THIS_MODULE,
	.chip_type			= CHIP_TYPE,
	.mode_allowed		= CHIP_MODE_ALLOWED,
	.fw_size			= CHIP_FW_SIZE,
	.flags				= CHIP_FLAGS,	/* Caution : MSB(bit31) unavailable */
	.irqflags			= CHIP_IRQFLAGS,
	.quirks				= CHIP_QUIRKS,
	/* */
	.bus_info			= {
		.bus_type			= CHIP_BUS_TYPE,
		.buf_size			= CHIP_BUF_SIZE,
		.spi_mode			= CHIP_SPI_MODE,
		.bits_per_word		= CHIP_BPW,
		.max_freq			= CHIP_MAX_FREQ,
		.bus_tx_hdr_size	= CHIP_TX_HDR_SZ,
		.bus_rx_hdr_size	= CHIP_RX_HDR_SZ,
		.bus_tx_dummy_size	= CHIP_TX_DUMMY_SZ,
		.bus_rx_dummy_size	= CHIP_RX_DUMMY_SZ,
	},
#if defined(__SIW_CONFIG_OF)
	.of_match_table 	= of_match_ptr(chip_match_ids),
#else
	.pins				= {
		.reset_pin		= CHIP_PIN_RESET,
		.reset_pin_pol	= OF_GPIO_ACTIVE_LOW,
		.irq_pin		= CHIP_PIN_IRQ,
		.maker_id_pin	= CHIP_PIN_MAKER,
		.vdd_pin		= CHIP_PIN_VDD,
		.vio_pin		= CHIP_PIN_VIO,
	},
	.caps				= {
		.max_x			= CHIP_MAX_X,
		.max_y			= CHIP_MAX_Y,
		.max_pressure	= CHIP_MAX_PRESSURE,
		.max_width		= CHIP_MAX_WIDTH,
		.max_orientation = CHIP_MAX_ORI,
		.max_id			= CHIP_MAX_ID,
		.hw_reset_delay	= CHIP_HW_RST_DELAY,
		.sw_reset_delay	= CHIP_SW_RST_DELAY,
	},
#endif
	/* Input Device ID */
	.i_id				= {
		.bustype		= CHIP_INPUT_ID_BUSTYPE,
		.vendor 		= CHIP_INPUT_ID_VENDOR,
		.product 		= CHIP_INPUT_ID_PRODUCT,
		.version 		= CHIP_INPUT_ID_VERSION,
	},
	/* */
	//See 'siw_hal_get_default_ops' [siw_touch_hal.c]
	.ops				= (void*)siw_hal_get_default_ops,
	/* */
	//See 'siw_hal_get_tci_info' [siw_touch_hal.c]
	.tci_info			= (void *)chip_tci_info,
	.tci_qcover_open	= NULL,
	.tci_qcover_close	= NULL,
	//See 'siw_hal_get_swipe_info' [siw_touch_hal.c]
	.swipe_ctrl			= (void *)&chip_swipe_info,
	//See 'store_ext_watch_config_font_position' [siw_touch_hal_watch.c]
	.watch_win			= NULL,
	//See 'siw_setup_operations' [siw_touch.c]
	.reg_quirks			= (void *)chip_reg_quirks,
};

static struct siw_touch_chip_data chip_data = {
	.pdata = &chip_pdata,
	.bus_drv = NULL,
};

siw_chip_module_init(CHIP_DEVICE_NAME,
				chip_data,
				CHIP_DEVICE_DESC,
				"kimhh@siliconworks.co.kr");


__siw_setup_str("siw_chip_name=", siw_setup_chip_name, chip_name);
__siw_setup_str("siw_drv_name=", siw_setup_drv_name, chip_drv_name);
__siw_setup_str("siw_idrv_name=", siw_setup_idrv_name, chip_idrv_name);



