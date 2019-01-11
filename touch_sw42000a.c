/*
 * touch_sw42000a.c - SiW touch driver glue for SW42000A
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

#define CHIP_ID						"7600"
#define CHIP_DEVICE_NAME			"SW42000A"
#define CHIP_COMPATIBLE_NAME		"siw,sw42000a"
#define CHIP_DEVICE_DESC			"SiW Touch SW42000A Driver"

#define CHIP_TYPE					CHIP_SW42000A

/* not fixed */
#define CHIP_MODE_ALLOWED			(0 |	\
									LCD_MODE_BIT_U0 |	\
									LCD_MODE_BIT_U2 |	\
									LCD_MODE_BIT_U3 |	\
									LCD_MODE_BIT_STOP |	\
									0)

#define CHIP_FW_SIZE				(128<<10)

#define CHIP_FLAGS					(0 |	\
									TOUCH_SKIP_ESD_EVENT |	\
									0)

#define CHIP_IRQFLAGS				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT)


#define CHIP_INPUT_ID_BUSTYPE		BUS_SPI
#define CHIP_INPUT_ID_VENDOR		0xABCD
#define CHIP_INPUT_ID_PRODUCT		0x9876
#define CHIP_INPUT_ID_VERSION		0x1234

#define __CHIP_QUIRK_ADD			0

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
#define CHIP_MAX_FREQ				(10 * 1000* 1000)
#define CHIP_TX_HDR_SZ				SPI_BUS_TX_HDR_SZ
#define CHIP_TX_DUMMY_SZ			SPI_BUS_TX_DUMMY_SZ
#define CHIP_RX_HDR_SZ				SPI_BUS_RX_HDR_SZ_32BIT
#define CHIP_RX_DUMMY_SZ			SPI_BUS_RX_DUMMY_SZ_32BIT

static const struct siw_hal_reg_quirk chip_reg_quirks[] = {
	{ .old_addr = SPR_CHIP_TEST, .new_addr = 0x01F, },
	{ .old_addr = SPR_RST_CTL, .new_addr = 0x004, },
	{ .old_addr = SPR_SUBDISP_STS, .new_addr = ADDR_SKIP_MASK, },
	{ .old_addr = SPR_CODE_OFFSET, .new_addr = 0x036, },
	{ .old_addr = TC_VERSION, .new_addr = 0x642, },
	{ .old_addr = TC_PRODUCT_ID1, .new_addr = 0x644, },
	{ .old_addr = INFO_CHIP_VERSION, .new_addr = 0x001, },
	{ .old_addr = TC_IC_STATUS, .new_addr = 0x600, },
	{ .old_addr = TC_STS, .new_addr = 0x601, },
	/* */
	{ .old_addr = CODE_ACCESS_ADDR, .new_addr = 0xFD8, },
	{ .old_addr = DATA_I2CBASE_ADDR, .new_addr = 0xFD1, },
	{ .old_addr = SERIAL_DATA_OFFSET, .new_addr = 0x03C, },
	/* */
	{ .old_addr = (1<<31), .new_addr = 0, },	/* switch : don't show log */
	/* */
	{ .old_addr = TCI_ENABLE_W, .new_addr = 0xF30, },
	{ .old_addr = TAP_COUNT_W, .new_addr = 0xF31, },
	{ .old_addr = MIN_INTERTAP_W, .new_addr = 0xF32, },
	{ .old_addr = MAX_INTERTAP_W, .new_addr = 0xF33, },
	{ .old_addr = TOUCH_SLOP_W, .new_addr = 0xF34, },
	{ .old_addr = TAP_DISTANCE_W, .new_addr = 0xF35, },
	{ .old_addr = INT_DELAY_W, .new_addr = 0xF36, },
	{ .old_addr = ACT_AREA_X1_W, .new_addr = 0xF37, },
	{ .old_addr = ACT_AREA_Y1_W, .new_addr = 0xF38, },
	{ .old_addr = ACT_AREA_X2_W, .new_addr = 0xF39, },
	{ .old_addr = ACT_AREA_Y2_W, .new_addr = 0xF3A, },
	/* */
	{ .old_addr = SWIPE_ENABLE_W, .new_addr = 0xF3B, },
	{ .old_addr = SWIPE_DIST_W, .new_addr = 0xF3C, },
	{ .old_addr = SWIPE_RATIO_THR_W, .new_addr = 0xF3D, },
	{ .old_addr = SWIPE_RATIO_DIST_W, .new_addr = ADDR_SKIP_MASK, },
	{ .old_addr = SWIPE_RATIO_PERIOD_W, .new_addr = ADDR_SKIP_MASK, },
	{ .old_addr = SWIPE_TIME_MIN_W, .new_addr = 0xF3E, },
	{ .old_addr = SWIPE_TIME_MAX_W, .new_addr = 0xF40, },
	{ .old_addr = SWIPE_ACT_AREA_X1_W, .new_addr = 0xF42, },
	{ .old_addr = SWIPE_ACT_AREA_Y1_W, .new_addr = 0xF43, },
	{ .old_addr = SWIPE_ACT_AREA_X2_W, .new_addr = 0xF44, },
	{ .old_addr = SWIPE_ACT_AREA_Y2_W, .new_addr = 0xF45, },
	{ .old_addr = SWIPE_FAIL_DEBUG_R, .new_addr = 0xF60, },
	/* */
	{ .old_addr = TCI_DEBUG_FAIL_CTRL, .new_addr = 0xF5D, },
	{ .old_addr = TCI_DEBUG_FAIL_STATUS, .new_addr = 0xF5E, },
	{ .old_addr = TCI_DEBUG_FAIL_BUFFER, .new_addr = 0xF5F, },
	/* */
#if 0
	{ .old_addr = PRD_SERIAL_TCM_OFFSET, .new_addr = 0x028, },
	{ .old_addr = PRD_TC_MEM_SEL, .new_addr = 0x8C8, },
#endif
	{ .old_addr = TC_TSP_TEST_STS, .new_addr = 0x690, },
	{ .old_addr = TC_TSP_TEST_PF_RESULT, .new_addr = 0x691, },
	{ .old_addr = PRD_TC_TEST_MODE_CTL, .new_addr = ADDR_SKIP_MASK, },
	/* not fixed */
	{ .old_addr = PRD_TUNE_RESULT_OFFSET, .new_addr = ADDR_SKIP_MASK, },
#if 0
	{ .old_addr = PRD_M1_M2_RAW_OFFSET, .new_addr = 0x323, },
	{ .old_addr = PRD_TUNE_RESULT_OFFSET, .new_addr = 0x325, },
	{ .old_addr = PRD_OPEN3_SHORT_OFFSET, .new_addr = 0x324, },
#endif
	/* */
	{ .old_addr = PRD_IC_AIT_START_REG, .new_addr = 0xF0C, },
	{ .old_addr = PRD_IC_AIT_DATA_READYSTATUS, .new_addr = 0xF04, },
	/* */
	{ .old_addr = SPR_CHARGER_STS, .new_addr = 0xF68, },
	{ .old_addr = IME_STATE, .new_addr = ADDR_SKIP_MASK, },
	/* */
	{ .old_addr = GLOVE_EN, .new_addr = 0xF69, },
	{ .old_addr = GRAB_EN, .new_addr = 0xF6A, },
	/* */
	{ .old_addr = ~0, .new_addr = ~0 },		// End signal
};


#if defined(__SIW_CONFIG_OF)
/*
 * of_device_is_compatible(dev->of_node, CHIP_COMPATIBLE_NAME)
 */
static const struct of_device_id chip_match_ids[] = {
	{ .compatible = CHIP_COMPATIBLE_NAME },
	{ },
};
MODULE_DEVICE_TABLE(of, chip_match_ids);
#else
enum CHIP_CAPABILITY {
	CHIP_MAX_X			= 1080,	//1440,
	CHIP_MAX_Y			= 2280,	//2880,
	CHIP_MAX_PRESSURE	= 255,
	CHIP_MAX_WIDTH		= 15,
	CHIP_MAX_ORI		= 1,
	CHIP_MAX_ID			= 10,
	/* */
	CHIP_HW_RST_DELAY	= 1000,
	CHIP_SW_RST_DELAY	= 90,
};

#define CHIP_PIN_RESET			0
#define CHIP_PIN_IRQ			0
#define CHIP_PIN_MAKER			-1
#define CHIP_PIN_VDD			-1
#define CHIP_PIN_VIO			-1

#if (CHIP_PIN_RESET == 0) || (CHIP_PIN_IRQ == 0)
//	#error Assign external pin & flag first!!!
#endif
#endif	/* __SIW_CONFIG_OF */

#define __SW42000A_FLASH_ACCESS_VIA_BDMA
#define __SW42000A_FLASH_CRC_PASS

#define FLASH_PAGE_OFFSET		(0x200)
#define FLASH_PAGE_SIZE			(2<<10)
#define SIZEOF_FLASH			(128<<10)
#define FLASH_MAX_RW_SIZE		(1<<10)

#define GDMA_SADDR				0x056
#define GDMA_CTL				0x058
#define GDMA_CTL_READONLY_EN	BIT(17)
#define GDMA_CTL_GDMA_EN		BIT(26)
#define GDMA_START				0x059
#define GDMA_CRC_RESULT			0x05D
#define GDMA_CRC_PASS			0x05E

#define FC_TPROG				0x065
#define FC_CTRL					0x06B
#define FC_STAT					0x06C
#define FC_ADDR					0x06D
#define SPI_FLASH_STATUS		0xFE2

#define __FC_CTRL_PAGE_ERASE	BIT(0)
#define __FC_CTRL_MASS_ERASE	BIT(1)
#define __FC_CTRL_WR_EN			BIT(2)

#define FC_CTRL_PAGE_ERASE		(__FC_CTRL_PAGE_ERASE | __FC_CTRL_WR_EN)
#define FC_CTRL_MASS_ERASE		(__FC_CTRL_MASS_ERASE | __FC_CTRL_WR_EN)
#define FC_CTRL_WR_EN			(__FC_CTRL_WR_EN)

#define BDMA_SADDR				0x072
#define BDMA_DADDR				0x073
#define BDMA_CTL				0x074
#define BDMA_START				0x075
#define BDMA_STS				0x077

#define BDMA_CTL_BDMA_EN		(0x00010000)
#define BDMA_CTL_BDMA_BST_EN	(0x001E0000)
#define BDMA_STS_TR_BUSY		(0x00000040)

#define BDMA_TRANS_SIZE			(SIW_TOUCH_MAX_BUF_SIZE>>1)
#define DATASRAM_ADDR			(0x20000000)

enum {
	E_FW_CODE_SIZE_ERR		= 1,
	E_FW_CODE_ONLY_VALID,
	E_FW_CODE_AND_CFG_VALID,
	E_FW_CODE_CFG_ERR,
};

#define CHIP_BOOT_LOADER_INIT	(0x74696E69)	//"init"
#define CHIP_BOOT_LOADER_CODE	(0x544F4F42)	//"BOOT"

#define CHIP_SPR_RST_CTL		(0x004)
#define CHIP_OSC_CTL			(0x005)
#define CHIP_BOOT_CODE_ADDR		(0x01A)
#define CHIP_INFO_PTR			(0x01B)
#define CHIP_CODE_OFFSET		(0x036)
#define CHIP_DATA_OFFSET		(0x03C)
#define CHIP_CODE_ACCESS_ADDR	(0xFD8)
#define CHIP_DATA_ACCESS_ADDR	(0xFD1)
#define CHIP_CONF_IDX_ADDR		(0x646)	//0x656

#define CRC_FIXED_VALUE			0x800D800D

enum {
	EQ_COND = 0,
	NOT_COND,
};

static int sw42000a_condition_wait(struct device *dev,
					u32 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry, int not_cond)
{
	u32 data = 0;
	int match = 0;
	int ret = 0;

	do {
		touch_msleep(delay);

		ret = siw_hal_read_value(dev, addr, &data);
		if (ret >= 0) {
			match = (not_cond == NOT_COND) ?	\
				!!((data & mask) != expect) : !!((data & mask) == expect);

			if (match) {
				if (value)
					*value = data;

				return 0;
			}
		}
	} while (--retry);

	if (value) {
		*value = data;
	}

	t_dev_err(dev,
		"wait fail: addr[%04Xh] data[%08Xh], "
		"mask[%08Xh], expect[%s%08Xh]\n",
		addr, data, mask,
		(not_cond == NOT_COND) ? "not " : "",
		expect);

	return -EPERM;
}

#if defined(__SW42000A_FLASH_CRC_PASS)
static int sw42000a_read_crc_pass(struct device *dev, u32 *data)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u32 __data = 1;
	int ret = 0;

	if (touch_flags(ts) & TOUCH_SKIP_RESET_PIN) {
		goto out;
	}

	if (chip->fquirks.hw_reset_quirk != NULL) {
		goto out;
	}

	ret = siw_hal_read_value(dev, GDMA_CRC_PASS, &__data);
	if (ret < 0) {
		return ret;
	}

out:
	if (data)
		*data = __data;

	return 0;
}
#else	/* __SW42000A_FLASH_CRC_PASS */
static int sw42000a_read_crc_pass(struct device *dev, u32 *data)
{
	if (data)
		*data = 1;

	return 0;
}
#endif	/* __SW42000A_FLASH_CRC_PASS */

#define FC_ERASE_WAIT_CNT	200
#define FC_ERASE_WAIT_TIME	5

static int __used sw42000a_fwup_flash_mass_erase(struct device *dev)
{
	u32 fc_addr = FC_ADDR;
	u32 fc_ctrl = FC_CTRL;
	u32 fc_stat = FC_STAT;
	u32 spi_flash_status = SPI_FLASH_STATUS;
	int fc_err = 0;
	int busy_time = FC_ERASE_WAIT_TIME;
	int busy_cnt = (FLASH_PAGE_SIZE<<1)/busy_time;
	u32 chk_resp, data;
	int ret = 0;

	ret = siw_hal_write_value(dev, fc_addr, 0);
	if (ret < 0) {
		fc_err = 1;
		goto out;
	}

	ret = siw_hal_write_value(dev, fc_ctrl, FC_CTRL_MASS_ERASE);
	if (ret < 0) {
		fc_err = 2;
		goto out;
	}

	ret = siw_hal_write_value(dev, fc_stat, 1);
	if (ret < 0) {
		fc_err = 3;
		goto out;
	}

	chk_resp = 1;
	ret = sw42000a_condition_wait(dev, spi_flash_status, &data,
			chk_resp, ~0, busy_time, busy_cnt, NOT_COND);
	if (ret < 0) {
		fc_err = 4;
		t_dev_err(dev, "FW upgrade: failed - flash erase wait(%Xh), %Xh\n",
			chk_resp, data);
		goto out;
	}

out:
	touch_msleep(10);

	siw_hal_write_value(dev, fc_ctrl, 0);

	if (fc_err) {
		t_dev_err(dev, "FW upgrade: failed - flash mass erase error, %d, %d\n",
			fc_err, ret);
	} else {
		t_dev_info(dev, "FW upgrade: flash mass erase done\n");
	}

	return ret;
}

#if 0
static int __used sw42000a_fwup_flash_page_erase(struct device *dev)
{
	u32 fc_addr = FC_ADDR;
	u32 fc_ctrl = FC_CTRL;
	u32 fc_stat = FC_STAT;
	u32 spi_flash_status = SPI_FLASH_STATUS;
	int addr = 0;
	int fc_err = 0;
	int i;
	int busy_time = FC_ERASE_WAIT_TIME;
	int busy_cnt = FC_ERASE_WAIT_CNT;
	u32 chk_resp, data;
	int ret = 0;

	for (i = 0; i < (SIZEOF_FLASH/FLASH_PAGE_SIZE); i++) {
		ret = siw_hal_write_value(dev, fc_addr, addr);
		if (ret < 0) {
			fc_err = 1;
			break;
		}

		ret = siw_hal_write_value(dev, fc_ctrl, FC_CTRL_PAGE_ERASE);
		if (ret < 0) {
			fc_err = 2;
			break;
		}

		ret = siw_hal_write_value(dev, fc_stat, 1);
		if (ret < 0) {
			fc_err = 3;
			break;
		}

		chk_resp = 1;
		ret = sw42000a_condition_wait(dev, spi_flash_status, &data,
				chk_resp, ~0, busy_time, busy_cnt, NOT_COND);
		if (ret < 0) {
			fc_err = 4;
			t_dev_err(dev, "FW upgrade: failed - flash page erase wait(%Xh), %Xh\n",
				chk_resp, data);
			break;
		}

		addr += FLASH_PAGE_OFFSET;
	}

	touch_msleep(10);

	siw_hal_write_value(dev, fc_addr, 0);
	siw_hal_write_value(dev, fc_ctrl, 0);

	if (fc_err) {
		t_dev_err(dev, "FW upgrade: failed - flash page erase error on %Xh, %d, %d\n",
			fc_addr, fc_err, ret);
	} else {
		t_dev_info(dev, "FW upgrade: flash page erase done\n");
	}

	return ret;
}
#endif

#if defined(__SW42000A_FLASH_ACCESS_VIA_BDMA)
static int __used sw42000a_fwup_flash_write_data(struct device *dev, int addr, u8 *dn_buf, int dn_size)
{
	u32 fc_offset = CHIP_DATA_OFFSET;
	u32 fc_code_access = CHIP_DATA_ACCESS_ADDR;
	int ret = 0;

	if (!dn_size) {
		return 0;
	}

	ret = siw_hal_write_value(dev, fc_offset, addr);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - flash write addr %Xh(%Xh), %d\n",
			addr, dn_size, ret);
		goto out;
	}

	ret = siw_hal_reg_write(dev, fc_code_access, dn_buf, dn_size);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - flash write data %Xh(%Xh), %d\n",
			addr, dn_size, ret);
		goto out;
	}

out:
	touch_msleep(5);

	return ret;
}
#else	/* __SW42000A_FLASH_ACCESS_VIA_BDMA */
static int __used sw42000a_fwup_flash_write(struct device *dev, int addr, u8 *dn_buf, int dn_size)
{
	u32 fc_offset = CHIP_CODE_OFFSET;
	u32 fc_code_access = CHIP_CODE_ACCESS_ADDR;
	int ret = 0;

	if (!dn_size) {
		return 0;
	}

	ret = siw_hal_write_value(dev, fc_offset, addr>>2);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - flash write addr %Xh(%Xh), %d\n",
			addr, dn_size, ret);
		goto out;
	}

	ret = siw_hal_reg_write(dev, fc_code_access, dn_buf, dn_size);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - flash write data %Xh(%Xh), %d\n",
			addr, dn_size, ret);
		goto out;
	}

out:
	touch_msleep(5);

	return ret;
}
#endif	/* __SW42000A_FLASH_ACCESS_VIA_BDMA */

static int __used sw42000a_fwup_flash_crc(struct device *dev, u32 *crc_val)
{
	u32 data = 0;
	u32 ctrl_data = 0;
	int ret = 0;

	ret = siw_hal_write_value(dev, GDMA_SADDR, 0);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_SADDR(%04Xh) set zero, %d\n",
			GDMA_SADDR, ret);
		goto out;
	}

	ctrl_data = (SIZEOF_FLASH>>2) - 1;
	ctrl_data |= GDMA_CTL_GDMA_EN | GDMA_CTL_READONLY_EN;
	ret = siw_hal_write_value(dev, GDMA_CTL, ctrl_data);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_CTL(%04Xh) write %08Xh, %d\n",
			GDMA_CTL, ctrl_data, ret);
		goto out;
	}
	touch_msleep(10);

	ret = siw_hal_write_value(dev, GDMA_START, 1);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_START(%04Xh) on, %d\n",
			GDMA_START, ret);
		goto out;
	}
	touch_msleep(100);

	ret = siw_hal_read_value(dev, GDMA_CRC_RESULT, &data);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_CRC_RESULT(%04Xh) read, %d\n",
			GDMA_CRC_RESULT, ret);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: flash crc result(%04Xh) %08Xh\n",
		GDMA_CRC_RESULT, data);

	if (crc_val != NULL) {
		*crc_val = data;
	}

	ret = sw42000a_read_crc_pass(dev, &data);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - GDMA_CRC_PASS(%04Xh) read, %d\n",
			GDMA_CRC_PASS, ret);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: flash crc pass(%04Xh) %Xh\n",
		GDMA_CRC_PASS, data);

out:
	touch_msleep(100);

	return ret;
}

#define LOG_SZ	64

static int sw42000a_fwup_rst_ctl(struct device *dev, int val, const char *str)
{
	u32 spr_rst_ctl = CHIP_SPR_RST_CTL;
	char log[LOG_SZ] = { 0, };
	char *name = NULL;
	int ret = 0;

	switch (val) {
	case 2:
		name = "system hold";
		break;
	case 1:
		name = "release cm3";
		break;
	default:
		name = "system release";
		break;
	}

	if (str == NULL) {
		snprintf(log, LOG_SZ, "%s", name);
	} else {
		snprintf(log, LOG_SZ, "%s for %s", name, str);
	}

	ret = siw_hal_write_value(dev, spr_rst_ctl, val);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - spr_rst_ctl(%d) - %s, %d\n", val, log, ret);
		goto out;
	}

	t_dev_dbg_fwup(dev, "FW upgrade: spr_rst_ctl(%d) - %s\n", val, log);

out:
	return ret;
}

static int sw42000a_fwup_fw_pre(struct device *dev)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	int ret = 0;

	ret = sw42000a_fwup_rst_ctl(dev, 2, NULL);
	if (ret < 0) {
		goto out;
	}

	ret = sw42000a_fwup_flash_mass_erase(dev);
	if (ret < 0) {
		goto out;
	}

	ret = sw42000a_fwup_rst_ctl(dev, 0, NULL);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int sw42000a_fwup_osc_ctl(struct device *dev)
{
	u32 data = 0;
	int ret = 0;

	ret = siw_hal_read_value(dev, CHIP_OSC_CTL, &data);
	if (ret < 0) {
		goto out;
	}

	data &= ~0x7F;
	data |= 0x5A;

	ret = siw_hal_write_value(dev, CHIP_OSC_CTL, data);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

#define FW_DN_LOG_UNIT	(8<<10)

static int sw42000a_fwup_fw_core(struct device *dev, u8 *dn_buf, int dn_size)
{
	u32 fc_ctrl = FC_CTRL;
	u8 *fw_data;
	int fw_size;
	int fw_addr;
	int curr_size;
	int fw_size_org = dn_size;
	int fw_dn_size = 0, fw_dn_percent;
	int ret = 0;

	ret = sw42000a_fwup_rst_ctl(dev, 2, "flash write");
	if (ret < 0) {
		goto out;
	}

	ret = sw42000a_fwup_osc_ctl(dev);
	if (ret < 0) {
		goto out;
	}

#if !defined(__SW42000A_FLASH_ACCESS_VIA_BDMA)
	ret = siw_hal_write_value(dev, FC_TPROG, 560);
	if (ret < 0) {
		goto out;
	}
#endif	/* __SW42000A_FLASH_ACCESS_VIA_BDMA */

	ret = siw_hal_write_value(dev, fc_ctrl, FC_CTRL_WR_EN);
	if (ret < 0) {
		goto out;
	}

	fw_data = dn_buf;
	fw_size = dn_size;
	fw_addr = 0;

#if defined(__SW42000A_FLASH_ACCESS_VIA_BDMA)
	while (fw_size) {
		curr_size = min(fw_size, BDMA_TRANS_SIZE);

		t_dev_dbg_fwup(dev, "FW upgrade: flash write %08Xh %04Xh\n", fw_addr, curr_size);

		ret = sw42000a_fwup_flash_write_data(dev, 0, fw_data, curr_size);
		if (ret < 0) {
			t_dev_err(dev, "FW upgrade: failed - flash write %08Xh %04Xh, %d\n",
				fw_addr, curr_size, ret);
			break;
		}

		ret = siw_hal_write_value(dev, BDMA_SADDR, DATASRAM_ADDR);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_write_value(dev, BDMA_CTL, BDMA_CTL_BDMA_EN | (curr_size>>2));
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_write_value(dev, BDMA_DADDR, fw_addr);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_write_value(dev, fc_ctrl, FC_CTRL_WR_EN);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_write_value(dev, BDMA_START, 1);
		if (ret < 0) {
			goto out;
		}

		ret = sw42000a_condition_wait(dev, BDMA_STS, NULL,
			BDMA_STS_TR_BUSY, BDMA_STS_TR_BUSY, 10, 2000, NOT_COND);
		if (ret < 0) {
			goto out;
		}

		fw_addr += curr_size;
		fw_data += curr_size;
		fw_size -= curr_size;

		fw_dn_size += curr_size;
		if (!fw_size || !(fw_dn_size & (FW_DN_LOG_UNIT-1))) {
			fw_dn_percent = (fw_dn_size * 100);
			fw_dn_percent /= fw_size_org;

			t_dev_info(dev, "FW upgrade: downloading...(%d%c)\n",
				fw_dn_percent, '%');
		}
	}
#else	/* __SW42000A_FLASH_ACCESS_VIA_BDMA */
	while (fw_size) {
		curr_size = min(fw_size, FLASH_MAX_RW_SIZE);

		t_dev_dbg_fwup(dev, "FW upgrade: flash write %08Xh %04Xh\n", fw_addr, curr_size);

		ret = sw42000a_fwup_flash_write(dev, fw_addr, fw_data, curr_size);
		if (ret < 0) {
			t_dev_err(dev, "FW upgrade: failed - flash write %08Xh %04Xh, %d\n",
				fw_addr, curr_size, ret);
			break;
		}

		fw_addr += curr_size;
		fw_data += curr_size;
		fw_size -= curr_size;

		fw_dn_size += curr_size;
		if (!fw_size || !(fw_dn_size & (FW_DN_LOG_UNIT-1))) {
			fw_dn_percent = (fw_dn_size * 100);
			fw_dn_percent /= fw_size_org;

			t_dev_info(dev, "FW upgrade: downloading...(%d%c)\n",
				fw_dn_percent, '%');
		}
	}
#endif	/* __SW42000A_FLASH_ACCESS_VIA_BDMA */

out:
	siw_hal_write_value(dev, fc_ctrl, 0);

//	sw42000a_fwup_rst_ctl(dev, 0, NULL);

	return ret;
}

static int sw42000a_fwup_fw_post(struct device *dev)
{
	u32 boot_code_addr = CHIP_BOOT_CODE_ADDR;
	u32 chk_resp, data;
	int ret = 0;

	ret = siw_hal_write_value(dev, boot_code_addr, CHIP_BOOT_LOADER_INIT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - CHIP_BOOT_LOADER_INIT, %d\n", ret);
		goto out;
	}
	touch_msleep(100);

	ret = sw42000a_fwup_rst_ctl(dev, 0, NULL);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - system release, %d\n", ret);
		goto out;
	}
	touch_msleep(200);

	chk_resp = CHIP_BOOT_LOADER_CODE;
	ret = sw42000a_condition_wait(dev, boot_code_addr, &data,
			chk_resp, ~0,
			10, 20, EQ_COND);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - boot check(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: boot loader ready\n");

out:
	return ret;
}

static int __used sw42000a_fwup_fw(struct device *dev,
				u8 *fw_buf, int fw_size)
{
	int fw_max_size = CHIP_FW_SIZE;
	int ret = 0;

	ret = sw42000a_fwup_fw_pre(dev);
	if (ret < 0) {
		goto out;
	}

	ret = sw42000a_fwup_fw_core(dev, fw_buf, fw_max_size);
	if (ret < 0) {
		goto out;
	}

	ret = sw42000a_fwup_fw_post(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int sw42000a_fwup_post(struct device *dev)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	u32 crc_val;
	int ret = 0;

	ret = sw42000a_fwup_rst_ctl(dev, 0, NULL);
	if (ret < 0) {
		goto out;
	}
	touch_msleep(100);

	ret = sw42000a_fwup_rst_ctl(dev, 2, "crc");
	if (ret < 0) {
		goto out;
	}
	touch_msleep(100);

	ret = sw42000a_fwup_flash_crc(dev, &crc_val);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - flash crc, %d\n", ret);
		goto out;
	}
	if (crc_val != CRC_FIXED_VALUE) {
		t_dev_err(dev, "FW upgrade: flash crc error %08Xh != %08Xh, %d\n",
			CRC_FIXED_VALUE, crc_val, ret);
		ret = -EFAULT;
		goto out;
	}
	t_dev_info(dev, "FW upgrade: flash crc check done\n");

	ret = sw42000a_fwup_rst_ctl(dev, 0, NULL);
	if (ret < 0) {
		goto out;
	}
	touch_msleep(100);

	ret = sw42000a_fwup_rst_ctl(dev, 1, NULL);
	if (ret < 0) {
		goto out;
	}
	touch_msleep(100);

	t_dev_dbg_fwup(dev, "FW upgrade: post done\n");

	return 0;

out:
	sw42000a_fwup_rst_ctl(dev, 0, NULL);

	touch_msleep(100);

	return ret;
}

static int sw42000a_fwup_verify(struct device *dev, u8 *fw_buf, int fw_size)
{
	int fw_max_size = CHIP_FW_SIZE;
	u32 fw_code_crc = *(u32 *)&fw_buf[fw_max_size - 4];
	u32 fw_code_size = *(u32 *)&fw_buf[fw_max_size - 8];

	if (fw_size < fw_max_size) {
		t_dev_err(dev, "FW chk_img: too small img size(%Xh), must be >= fw_max_size(%Xh)\n",
			fw_size, fw_max_size);
		return E_FW_CODE_SIZE_ERR;
	}

	t_dev_info(dev, "FW chk_img: code size %Xh, code crc %Xh\n",
		fw_code_size, fw_code_crc);

	if (fw_code_size > fw_max_size) {
		t_dev_err(dev, "FW chk_img: invalid code_size(%Xh), must be <= fw_max_size(%Xh)\n",
			fw_code_size, fw_max_size);
		return E_FW_CODE_SIZE_ERR;
	}

	if (fw_size == fw_max_size) {
		return E_FW_CODE_ONLY_VALID;
	}

	return -EINVAL;
}

static int __used sw42000a_fwup_upgrade(struct device *dev,
					u8 *fw_buf, int fw_size, int retry)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int fw_size_max = CHIP_FW_SIZE;
	u32 include_conf = !!(fw_size > fw_size_max);
	u32 data = 0;
	int ret = 0;

	if (retry) {
		ret = siw_hal_read_value(dev, CHIP_OSC_CTL, &data);
		if (ret < 0) {
			t_dev_err(dev, "FW upgrade: sys_osc_ctl read failed, %d\n", ret);
			goto out;
		}

		data = data & 0x7F;
		if (data == 4) {
			ret = siw_hal_write_value(dev, CHIP_OSC_CTL, 0x3C);
			if (ret < 0) {
				t_dev_err(dev, "FW upgrade: sys_osc_ctl write failed, %d\n", ret);
				goto out;
			}

			t_dev_info(dev, "FW upgrade: sys_osc_ctl adjusted\n");
		}
	}

	chip->fw.conf_index = 0;

	t_dev_info(dev, "FW upgrade:%s include conf data\n",
			(include_conf) ? "" : " not");

	t_dev_dbg_fwup(dev, "FW upgrade: fw size %08Xh, fw_size_max %08Xh\n",
			fw_size, fw_size_max);

	ret = sw42000a_fwup_verify(dev, fw_buf, fw_size);
	switch (ret) {
	case E_FW_CODE_ONLY_VALID:
		include_conf = 0;
		break;
	case E_FW_CODE_AND_CFG_VALID:
		break;
	case E_FW_CODE_SIZE_ERR:
	case E_FW_CODE_CFG_ERR:
	default:
		ret = -EPERM;
		goto out;
	}

	ret = sw42000a_fwup_fw(dev, fw_buf, fw_size);
	if (ret < 0) {
		goto out;
	}

	ret = sw42000a_fwup_post(dev);

out:
	return ret;
}

static int __used sw42000a_boot_status(struct device *dev, u32 *boot_st)
{
	u32 rdata_crc = 0;
	u32 rdata_pass = 0;
	u32 rdata_ptr = 0;
	int err = 0;
	int ret = 0;

	/*
	 * Check CRC
	 */
	ret = siw_hal_read_value(dev, GDMA_CRC_RESULT, &rdata_crc);
	if (ret < 0) {
		t_dev_err(dev, "sw42000a boot status: read %04Xh failed, %d\n",
			GDMA_CRC_RESULT, ret);
		return ret;
	}
	err |= (rdata_crc != CRC_FIXED_VALUE);

	ret = sw42000a_read_crc_pass(dev, &rdata_pass);
	if (ret < 0) {
		return ret;
	}
	err |= (!rdata_pass)<<1;

	/*
	 * Check Boot
	 */
	ret = siw_hal_read_value(dev, CHIP_INFO_PTR, &rdata_ptr);
	if (ret < 0) {
		t_dev_err(dev, "sw42000a boot status: read %04Xh failed, %d\n",
			CHIP_INFO_PTR, ret);
		return ret;
	}
	err |= (!rdata_ptr)<<2;

	if (boot_st != NULL) {
		(*boot_st) = (err) ? BIT(BOOT_STS_POS_DUMP_ERR) : BIT(BOOT_STS_POS_DUMP_DONE);
	}

	if (err) {
		t_dev_err(dev, "sw42000a boot status: %Xh(%Xh, %Xh, %Xh)\n",
			err, rdata_crc, rdata_pass, rdata_ptr);
	}

	return 0;
}


/* use eg. cname=arc1 to change name */
static char chip_name[32] = CHIP_DEVICE_NAME;
module_param_string(cname, chip_name, sizeof(chip_name), 0);

/* use eg. dname=arc1 to change name */
static char chip_drv_name[32] = SIW_TOUCH_NAME;
module_param_string(dname, chip_drv_name, sizeof(chip_drv_name), 0);

/* use eg. iname=arc1 to change input name */
static char chip_idrv_name[32] = SIW_TOUCH_INPUT;
module_param_string(iname, chip_idrv_name, sizeof(chip_idrv_name), 0);

static const struct siw_touch_pdata chip_pdata = {
	/* Configuration */
	.chip_id			= CHIP_ID,
	.chip_name			= chip_name,
	.drv_name			= chip_drv_name,
	.idrv_name			= chip_idrv_name,
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
	.ops				= NULL,
	/* */
	//See 'siw_hal_get_tci_info' [siw_touch_hal.c]
	.tci_info			= NULL,
	.tci_qcover_open	= NULL,
	.tci_qcover_close	= NULL,
	//See 'siw_hal_get_swipe_info' [siw_touch_hal.c]
	.swipe_ctrl			= NULL,
	//See 'store_ext_watch_config_font_position' [siw_touch_hal_watch.c]
	.watch_win			= NULL,
	//See 'siw_setup_operations' [siw_touch.c]
	.reg_quirks			= (void *)chip_reg_quirks,
	//function quirks
	.fquirks = {
		.fwup_upgrade	= sw42000a_fwup_upgrade,
		.boot_status	= sw42000a_boot_status,
	},
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



