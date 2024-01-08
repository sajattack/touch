/*
 * siw_touch_sys.c - SiW touch system interface
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
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
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

#if defined(CONFIG_PLAT_SAMSUNG)
#include <plat/cpu.h>
#include <plat/gpio-core.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-cfg-helpers.h>
#include <plat/pm.h>
#endif

#include "siw_touch.h"
#include "siw_touch_gpio.h"
#include "siw_touch_sys.h"

#if defined(CONFIG_TOUCHSCREEN_SIW_LG4895_F650) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_LG4946_F700)
#include <soc/qcom/lge/board_lge.h>
#endif

int siw_touch_get_boot_mode(void)
{
#if defined(CONFIG_SIW_GET_BOOT_MODE)
	if (sys_get_boot_mode() == BOOT_MODE_CHARGERLOGO) {
		return SIW_TOUCH_CHARGER_MODE;
	}
#elif defined(CONFIG_TOUCHSCREEN_SIW_LG4895_F650) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_LG4946_F700)
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		return SIW_TOUCH_CHARGER_MODE;
	}
#endif

	return 0;
}

int siw_touch_boot_mode_check(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = NORMAL_BOOT;

#if defined(CONFIG_SIW_GET_FACTORY_MODE)
	ret = sys_get_factory_boot();
#elif defined(CONFIG_TOUCHSCREEN_SIW_LG4895_F650) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_LG4946_F700)
	ret = lge_get_factory_boot();
#endif

	if (ret != NORMAL_BOOT) {
		switch (atomic_read(&ts->state.mfts)) {
			case MFTS_NONE :
				ret = MINIOS_AAT;
				break;
			case MFTS_FOLDER :
				ret = MINIOS_MFTS_FOLDER;
				break;
			case MFTS_FLAT :
				ret = MINIOS_MFTS_FLAT;
				break;
			case MFTS_CURVED :
				ret = MINIOS_MFTS_CURVED;
				break;
			default :
				ret = MINIOS_AAT;
				break;
		}
	}

	return ret;
}

int siw_touch_boot_mode_tc_check(struct device *dev)
{
	int ret = 0;

#if defined(CONFIG_SIW_GET_BOOT_MODE)
	if (sys_get_boot_mode() == BOOT_MODE_CHARGERLOGO) {
		return 1;
	}
#elif defined(CONFIG_TOUCHSCREEN_SIW_LG4895_F650)
	if ((lge_get_boot_mode() == LGE_BOOT_MODE_QEM_910K) ||
		(lge_get_boot_mode() == LGE_BOOT_MODE_PIF_910K)) {
		return 1;
	}
#endif

	return ret;
}

int siw_touch_sys_gpio_set_pull(int pin, int value)
{
	int ret = 0;

#if defined(CONFIG_PLAT_SAMSUNG)
	{
		int pull_val;

		switch (value) {
		case GPIO_PULL_UP:
			pull_val = S3C_GPIO_PULL_UP;
			break;
		case GPIO_PULL_DOWN:
			pull_val = S3C_GPIO_PULL_DOWN;
			break;
		default:
			pull_val = S3C_GPIO_PULL_NONE;
			break;
		}
		ret = s3c_gpio_setpull(pin, pull_val);
	}
#endif

	return ret;
}

int siw_touch_sys_panel_reset(struct device *dev)
{
	return 0;
}

int siw_touch_sys_get_panel_bl(struct device *dev)
{
	return 0;
}

int siw_touch_sys_set_panel_bl(struct device *dev, int level)
{
	if (level == 0){
		t_dev_info(dev, "BLU control OFF\n");
		/* To Do : screen off */
	} else {
		t_dev_info(dev, "BLU control ON (level:%d)\n", level);
		/* To Do : screen on with level */
	}
	return 0;
}

int siw_touch_sys_osc(struct device *dev, int onoff)
{
	return 0;
}

int siw_touch_sys_power_state(struct device *dev)
{
	/*
	if (invalid_power_state) {
		t_dev_warn(dev, "power status not invalid\n");
		return -EPERM;
	}
	*/

	return 0;
}

int siw_touch_sys_power_lock(struct device *dev, int set)
{
	if (set) {
		/* keep touch power on */
	} else {
		/* allow touch power off */
	}

	return 0;
}

#define DRM_RET_NOOP		(-EPERM)
#define DRM_RESUME			FB_RESUME
#define DRM_SUSPEND			FB_SUSPEND

#if defined(__SIW_CONFIG_SYS_FB)
/*
 * Example about DRM(Direct Rendering Manager) interface
 * Detail implementation needs more discussion
 * because this highly depends on system architecture.
 */
#if defined(__SIW_SUPPORT_DRM_EXAMPLE)
#include <linux/msm_drm_notify.h>

#define __DRM_EARLY_EVENT_BLANK		MSM_DRM_EARLY_EVENT_BLANK
#define __DRM_EVENT_BLANK			MSM_DRM_EVENT_BLANK
#define __DRM_BLANK_UNBLANK			MSM_DRM_BLANK_UNBLANK
#define __DRM_BLANK_POWERDOWN		MSM_DRM_BLANK_POWERDOWN
#else	/* __SIW_SUPPORT_DRM_EXAMPLE */
#define __DRM_EARLY_EVENT_BLANK		-1
#define __DRM_EVENT_BLANK			-1
#define __DRM_BLANK_UNBLANK			-1
#define __DRM_BLANK_POWERDOWN		-2
#endif	/* __SIW_SUPPORT_DRM_EXAMPLE */

static int __drm_register_client(struct notifier_block *block)
{
#if defined(__SIW_SUPPORT_DRM_EXAMPLE)
	return msm_drm_register_client(block);
#else
	return 0;
#endif
}

static int __drm_unregister_client(struct notifier_block *block)
{
#if defined(__SIW_SUPPORT_DRM_EXAMPLE)
	return msm_drm_unregister_client(block);
#else
	return 0;
#endif
}

static int __drm_blank(void *data)
{
#if defined(__SIW_SUPPORT_DRM_EXAMPLE)
	struct msm_drm_notifier *ev = (struct msm_drm_notifier *)data;

	if (ev) {
		return *(int *)ev->data;
	}
#endif

	return -EINVAL;
}

static int siw_drm_notifier_work(
			struct siw_ts *ts,
			unsigned long event, int blank)
{
	struct device *dev = ts->dev;
	char *fb_msg_unblank = ts->fb_msg_unblank;
	char *fb_msg_blank = ts->fb_msg_blank;
	int ret = DRM_RET_NOOP;

#if 0
	if (event == __DRM_R_EARLY_EVENT_BLANK) {
		switch (blank) {
		case __DRM_BLANK_UNBLANK:
			t_dev_info(dev, "%s(r_early)\n", fb_msg_unblank);
			ret = ts->fb_ret_revert_suspend;
			break;
		case __DRM_BLANK_POWERDOWN:
			t_dev_info(dev, "%s(r_early)\n", fb_msg_blank);
			ret = ts->fb_ret_revert_resume;
			break;
		}
		goto out;
	}
#endif

	if (event == __DRM_EARLY_EVENT_BLANK) {
		switch (blank) {
		case __DRM_BLANK_UNBLANK:
			t_dev_info(dev, "%s(early)\n", fb_msg_unblank);
			ret = ts->fb_ret_early_resume;
		case __DRM_BLANK_POWERDOWN:
			t_dev_info(dev, "%s(early)\n", fb_msg_blank);
			ret = ts->fb_ret_early_suspend;
		}
		goto out;
	}

	if (event == __DRM_EVENT_BLANK) {
		switch (blank) {
		case __DRM_BLANK_UNBLANK:
			t_dev_info(dev, "%s\n", fb_msg_unblank);
			ret = ts->fb_ret_resume;
		case __DRM_BLANK_POWERDOWN:
			t_dev_info(dev, "%s\n", fb_msg_blank);
			ret = ts->fb_ret_suspend;
		}
		goto out;
	}

out:
	return ret;
}

static int siw_drm_notifier_callback(
			struct notifier_block *self,
			unsigned long event, void *data)
{
	struct siw_ts *ts =
		container_of(self, struct siw_ts, sys_fb_notif);
	struct device *dev = ts->dev;
	int blank = __drm_blank(data);
	int ret = 0;

	if (blank < 0) {
		return 0;
	}

	ret = siw_drm_notifier_work(ts, event, blank);
	switch (ret) {
	case DRM_RESUME:
		siw_touch_resume_call(dev);
		break;
	case DRM_SUSPEND:
		siw_touch_suspend_call(dev);
		break;
	}

	return 0;
}

static int siw_drm_register_client(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);

	t_dev_info(dev, "drm_register_client - sys_fb_notif\n");

	ts->fb_ret_revert_resume = DRM_RET_NOOP;
	ts->fb_ret_revert_suspend = DRM_RET_NOOP;
	ts->fb_ret_early_resume = DRM_RET_NOOP;
	ts->fb_ret_early_suspend = DRM_RET_NOOP;
	ts->fb_ret_resume = DRM_RET_NOOP;
	ts->fb_ret_suspend = DRM_RET_NOOP;

#if defined(__SIW_PANEL_CLASS_MOBILE_OLED)
	snprintf(ts->fb_msg_unblank, sizeof(ts->fb_msg_unblank), "drm_unblank");
	snprintf(ts->fb_msg_blank, sizeof(ts->fb_msg_blank), "drm_blank");

	ts->fb_ret_suspend = DRM_SUSPEND;		/* suspend later */
	ts->fb_ret_resume = DRM_RESUME;			/* resume later */
#elif defined(__SIW_CONFIG_SYSTEM_PM)
	snprintf(ts->fb_msg_unblank, sizeof(ts->fb_msg_unblank), "drm_unblank");
	snprintf(ts->fb_msg_blank, sizeof(ts->fb_msg_blank), "drm_blank");

	ts->fb_ret_revert_resume = DRM_RESUME;	/* revert to resume */
	ts->fb_ret_early_suspend = DRM_SUSPEND;	/* suspend early */
	ts->fb_ret_resume = DRM_RESUME;			/* resume later */
#else	/* */
	snprintf(ts->fb_msg_unblank, sizeof(ts->fb_msg_unblank), "DRM_UNBLANK");
	snprintf(ts->fb_msg_blank, sizeof(ts->fb_msg_blank), "DRM_BLANK");
#endif

	ts->sys_fb_notif.notifier_call = siw_drm_notifier_callback;

	return __drm_register_client(&ts->sys_fb_notif);
}

static int siw_drm_unregister_client(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);

	t_dev_info(dev, "drm_unregister_client - sys_fb_notif\n");

	return __drm_unregister_client(&ts->sys_fb_notif);
}
#else	/* __SIW_CONFIG_SYS_FB */
static inline int siw_drm_register_client(struct device *dev)
{
	return 0;
}

static inline int siw_drm_unregister_client(struct device *dev)
{
	return 0;
}
#endif	/* __SIW_CONFIG_SYS_FB */

int siw_touch_sys_fb_register_client(struct device *dev)
{
	int ret = 0;

	ret = siw_drm_register_client(dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to register drm client, %d\n", ret);
	}

	return ret;
}

int siw_touch_sys_fb_unregister_client(struct device *dev)
{
	int ret = 0;

	ret = siw_drm_unregister_client(dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to unregister drm client, %d\n", ret);
	}

	return ret;
}


