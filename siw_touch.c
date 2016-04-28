/*
 * siw_touch.c - SiW touch core driver
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
#include <linux/kthread.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include <linux/input/siw_touch_notify.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"
#include "siw_touch_event.h"
#include "siw_touch_gpio.h"
#include "siw_touch_irq.h"
#include "siw_touch_sys.h"


extern int siw_touch_init_sysfs(struct siw_ts *ts);
extern void siw_touch_free_sysfs(struct siw_ts *ts);

extern int siw_touch_parse_data(struct siw_ts *ts);

#if 0
u32 t_pr_dbg_mask = DBG_NONE | DBG_INFO;
u32 t_dev_dbg_mask = DBG_NONE | DBG_BASE | DBG_INFO | DBG_GPIO | DBG_OF;
#else
u32 t_pr_dbg_mask = DBG_NONE;
u32 t_dev_dbg_mask = DBG_NONE;
/*
 * DBG_NONE | DBG_BASE | DBG_IRQ | DBG_NOTI | DBG_EVENT = 201326721(0xC000081)
 */
#endif

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/pr_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko pr_dbg_mask=<value>
 */
module_param_named(pr_dbg_mask, t_pr_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/dev_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko dev_dbg_mask=<value>
 */
module_param_named(dev_dbg_mask, t_dev_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);


/*
 * SiW Operations
 */
void *siw_setup_operations(struct siw_ts *ts, struct siw_touch_operations *ops_ext)
{
	if (!ops_ext)
		return NULL;

	ts->ops_ext = ops_ext;
	memcpy(&ts->ops_in, ops_ext, sizeof(struct siw_touch_operations));
	ts->ops = &ts->ops_in;

	if (ts->pdata->reg_quirks) {
	//	struct siw_hal_reg *reg = siw_ops_reg(ts);
		u32 *curr_reg;
		struct siw_hal_reg_quirk *reg_quirks = ts->pdata->reg_quirks;
		int cnt = sizeof(struct siw_hal_reg)>>2;
		int i;

		while (1) {
			if (!reg_quirks->old_addr ||
				!reg_quirks->new_addr ||
				(reg_quirks->old_addr == ~0) ||
				(reg_quirks->new_addr == ~0))
				break;

			curr_reg = siw_ops_reg(ts);
			for (i=0 ; i<cnt ; i++) {
				if ((*curr_reg) == reg_quirks->old_addr) {
					(*curr_reg) = reg_quirks->new_addr;
					t_dev_info(ts->dev, "%s reg quirks: %Xh -> %Xh\n",
						touch_chip_name(ts),
						reg_quirks->old_addr, reg_quirks->new_addr);
					break;
				}
				curr_reg++;
			}
			reg_quirks++;
		}
	}

	return ts->ops;
}


/**
 * siw_touch_set() - set touch data
 * @dev: device to use
 * @cmd: set command
 * @buf: data to store
 *
 * Return:
 * On success, the total number of bytes of data stored to device.
 * Otherwise, it returns zero or minus value
 */
int siw_touch_set(struct device *dev, u32 cmd, void *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (!buf)
		t_dev_err(dev, "NULL buf\n");

	mutex_lock(&ts->lock);
	ret = siw_ops_set(ts, cmd, buf);
	mutex_unlock(&ts->lock);

	return ret;
}

/**
 * siw_touch_get() - get touch data
 * @dev: device to use
 * @cmd: set command
 * @buf: data to store
 *
 * Return:
 * On success, the total number of bytes of data loaded from device.
 * Otherwise, it returns zero or minus value
 */
int siw_touch_get(struct device *dev, u32 cmd, void *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (!buf)
		t_dev_err(dev, "NULL buf\n");

	mutex_lock(&ts->lock);
	ret = siw_ops_get(ts, cmd, buf);
	mutex_unlock(&ts->lock);

	return ret;
}

/**
 * siw_touch_suspend() - touch suspend
 * @dev: device to use
 *
 */
static void siw_touch_suspend(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	t_dev_info(dev, "Suspend start\n");

	cancel_delayed_work_sync(&ts->init_work);
	cancel_delayed_work_sync(&ts->upgrade_work);
	atomic_set(&ts->state.uevent, UEVENT_IDLE);

	mutex_lock(&ts->lock);
	siw_touch_report_all_event(ts);
	atomic_set(&ts->state.fb, FB_SUSPEND);
	/* if need skip, return value is not 0 in pre_suspend */
	ret = siw_ops_suspend(ts);
	mutex_unlock(&ts->lock);

	t_dev_info(dev, "Suspend end\n");

	if (ret == 1)
		mod_delayed_work(ts->wq, &ts->init_work, 0);
}

/**
 * siw_touch_resume() - touch resume
 * @dev: device to use
 *
 */
static void siw_touch_resume(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	t_dev_info(dev, "Resume start\n");

	mutex_lock(&ts->lock);
	atomic_set(&ts->state.fb, FB_RESUME);
	/* if need skip, return value is not 0 in pre_resume */
	ret = siw_ops_resume(ts);
	mutex_unlock(&ts->lock);

	t_dev_info(dev, "Resume end\n");

	if (ret == 0)
		mod_delayed_work(ts->wq, &ts->init_work, 0);
}

/**
 * siw_touch_suspend_call() - Helper function for touch suspend
 * @dev: device to use
 *
 */
void siw_touch_suspend_call(struct device *dev)
{
#if !defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_FB)
	siw_touch_suspend(dev);
#endif
}

/**
 * siw_touch_resume_call() - Helper function for touch resume
 * @dev: device to use
 *
 */
void siw_touch_resume_call(struct device *dev)
{
#if !defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_FB)
	siw_touch_resume(dev);
#endif
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
/**
 * touch pm control using early pm
 *
 */
static void siw_touch_early_suspend(struct early_suspend *h)
{
	struct siw_ts *ts =
		container_of(h, struct siw_ts, early_suspend);
	struct device *dev = ts->dev;

	t_dev_info(ts->dev, "early suspend\n");

	siw_touch_suspend(dev);
}

static void siw_touch_early_resume(struct early_suspend *h)
{
	struct siw_ts *ts =
		container_of(h, struct siw_ts, early_suspend);
	struct device *dev = ts->dev;

	t_dev_info(ts->dev, "early resume\n");

	siw_touch_resume(dev);
}

static int __used siw_touch_init_pm(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	t_dev_dbg_pm(ts->dev, "init pm\n");

	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = siw_touch_early_suspend;
	ts->early_suspend.resume = siw_touch_early_resume;
	register_early_suspend(&ts->early_suspend);
	return 0;
}

static int __used siw_touch_free_pm(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	t_dev_dbg_pm(ts->dev, "free pm\n");

	unregister_early_suspend(&ts->early_suspend);

	return 0;
}
#elif defined(CONFIG_FB)
/**
 * touch pm control using FB notifier
 *
 */
static int siw_touch_fb_notifier_callback(
			struct notifier_block *self,
			unsigned long event, void *data)
{
	struct siw_ts *ts =
		container_of(self, struct siw_ts, fb_notif);
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			siw_touch_resume(ts->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			siw_touch_suspend(ts->dev);
	}

	return 0;
}

static int __used siw_touch_init_pm(struct siw_ts *ts)
{
	t_dev_dbg_pm(ts->dev, "fb_register_client - fb_notif\n");

	ts->fb_notif.notifier_call = siw_touch_fb_notifier_callback;
	return fb_register_client(&ts->fb_notif);
}

static int __used siw_touch_free_pm(struct siw_ts *ts)
{
	t_dev_dbg_pm(ts->dev, "fb_unregister_client - fb_notif\n");

	fb_unregister_client(&ts->fb_notif);

	return 0;
}
#else
#pragma message("[SiW - Warning] No core pm operation")
static int __used __siw_touch_init_pm_none(struct siw_ts *ts, int init)
{
	t_dev_dbg_pm(ts->dev, "pm %s none\n", (init)? "free" : "init");
	return 0;
}
#define siw_touch_init_pm(_ts)		__siw_touch_init_pm_none(_ts, 0)
#define siw_touch_free_pm(_ts)		__siw_touch_init_pm_none(_ts, 1)
#endif

#if defined(__SIW_SUPPORT_ASC)
/**
 * siw_touch_get_max_delta -
 * @ts : touch core info
 *
 */
static void siw_touch_get_max_delta(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct asc_info *asc = &(ts->asc);
	int ret;

	if (asc->use_delta_chk == DELTA_CHK_OFF) {
		t_dev_warn(dev, "DELTA_CHK is OFf\n");
		return;
	}

	if (atomic_read(&ts->state.fb) != FB_RESUME) {
		t_dev_warn(dev, "fb state is not FB_RESUME\n");
		return;
	}

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		t_dev_warn(dev, "core state is not CORE_NORMAL\n");
		return;
	}

	mutex_lock(&ts->lock);
	ret = siw_ops_asc(ts, ASC_READ_MAX_DELTA, 0);
	mutex_unlock(&ts->lock);
	if (ret < 0) {
		t_dev_err(dev, "delta change failed, %d\n", ret);
		return;
	}

	asc->delta = ret;
	asc->delta_updated = true;

	t_dev_info(dev, "delta = %d\n", asc->delta);
}

static const char *asc_str[] = {
	"NORMAL",
	"ACUTE",
	"OBTUSE",
};

/**
 * siw_touch_change_sensitivity - change touch sensitivity
 * @ts : touch core info
 * @target : target sensitivity
 *
 */
void siw_touch_change_sensitivity(struct siw_ts *ts,
						int target)
{
	struct device *dev = ts->dev;
	struct asc_info *asc = &(ts->asc);
	int ret;

	if (atomic_read(&ts->state.fb) != FB_RESUME) {
		t_dev_warn(dev, "fb state is not FB_RESUME\n");
		return;
	}

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		t_dev_warn(dev, "core state is not CORE_NORMAL\n");
		return;
	}

	if (asc->curr_sensitivity == target) {
		return;
	}

	t_dev_info(dev, "sensitivity(curr->next) = (%s -> %s)\n",
				asc_str[asc->curr_sensitivity],
				asc_str[target]);

	mutex_lock(&ts->lock);
	ret = siw_ops_asc(ts, ASC_WRITE_SENSITIVITY, target);
	mutex_unlock(&ts->lock);
	if (ret < 0) {
		t_dev_err(dev, "sensitivity change failed, %d\n", ret);
		return;
	}

	asc->curr_sensitivity = target;
}

static void siw_touch_update_sensitivity(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct asc_info *asc = &(ts->asc);
	int target = NORMAL_SENSITIVITY;

	if (asc->use_delta_chk == DELTA_CHK_OFF) {
		t_dev_warn(dev, "DELTA_CHK is OFf\n");
		return;
	}

	if (asc->delta_updated == false) {
		t_dev_warn(dev, "delta is not updated.\n");
		return;
	}

	if (asc->delta < asc->low_delta_thres) {
		target = ACUTE_SENSITIVITY;
	} else if (asc->delta > asc->high_delta_thres) {
		target = OBTUSE_SENSITIVITY;
	} else {
		target = NORMAL_SENSITIVITY;
	}

	asc->delta_updated = false;
	siw_touch_change_sensitivity(ts, target);
}
#endif	/* __SIW_SUPPORT_ASC */

#define SIW_TOUCH_LPWG_LOCK_NAME		"touch_lpwg"

static void __used siw_touch_init_locks(struct siw_ts *ts)
{
	t_dev_dbg_base(ts->dev, "touch init locks\n");

	mutex_init(&ts->lock);
	wake_lock_init(&ts->lpwg_wake_lock,
		WAKE_LOCK_SUSPEND, SIW_TOUCH_LPWG_LOCK_NAME);
}

static void __used siw_touch_free_locks(struct siw_ts *ts)
{
	t_dev_dbg_base(ts->dev, "free locks\n");

	mutex_destroy(&ts->lock);
	wake_lock_destroy(&ts->lpwg_wake_lock);
}

static void siw_touch_initialize(struct siw_ts *ts)
{
	/* lockscreen */
	siw_touch_report_all_event(ts);
}

static void siw_touch_init_work_func(struct work_struct *work)
{
	struct siw_ts *ts =
			container_of(to_delayed_work(work),
						struct siw_ts, init_work);
	struct device *dev = ts->dev;
//	int ret;

	t_dev_dbg_base(dev, "init work start\n");

	mutex_lock(&ts->lock);
	siw_touch_initialize(ts);
	siw_ops_init(ts);
	siw_touch_irq_control(dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	if (ts->role.use_fw_upgrade) {
		if (atomic_read(&ts->state.core) == CORE_PROBE) {
			t_dev_info(dev, "Touch F/W upgrade triggered\n");
			siw_touch_qd_upgrade_work_now(ts);
			return;
		}
	}

#if defined(__SIW_SUPPORT_ASC)
	if (ts->asc.use_asc == ASC_ON) {
		if (atomic_read(&ts->state.core) == CORE_UPGRADE) {
			int ret;

			mutex_lock(&ts->lock);
			ret = siw_ops_asc(ts, ASC_GET_FW_SENSITIVITY, 0);
			mutex_unlock(&ts->lock);
			if (ret < 0) {
				t_dev_warn(dev, "sensitivity change failed, %d\n", ret);
			}
		}

		siw_touch_qd_toggle_delta_work_jiffies(ts, 0);
	}
#endif	/* __SIW_SUPPORT_ASC */

	atomic_set(&ts->state.core, CORE_NORMAL);

	t_dev_dbg_base(dev, "init work done\n");
}

static void siw_touch_upgrade_work_func(struct work_struct *work)
{
	struct siw_ts *ts =
			container_of(to_delayed_work(work),
						struct siw_ts, upgrade_work);
	struct device *dev = ts->dev;
	int ret = 0;

	t_dev_info(dev, "FW upgrade work func\n");

	atomic_set(&ts->state.core, CORE_UPGRADE);
	ts->role.use_fw_upgrade = 0;

	mutex_lock(&ts->lock);
	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	ret = siw_ops_upgrade(ts);
	mutex_unlock(&ts->lock);

	/* init force_upgrade */
	ts->force_fwup = 0;
	ts->test_fwpath[0] = '\0';

	if (ret < 0) {
		if (ret == -EPERM) {
			t_dev_info(dev, "FW upgrade skipped\n");
		} else {
			t_dev_info(dev, "FW upgrade halted, %d\n", ret);
		}
		siw_touch_qd_init_work_now(ts);
	//	atomic_set(&ts->state.core, CORE_NORMAL);
		return;
	}

#if 1
	siw_ops_reset(ts, HW_RESET_ASYNC);
#else
	siw_ops_power(ts, POWER_OFF);
	touch_msleep(1);
	siw_ops_power(ts, POWER_ON);

	siw_touch_qd_init_work_now(ts);
#endif
}

static void siw_touch_fb_work_func(struct work_struct *work)
{
	struct siw_ts *ts =
			container_of(to_delayed_work(work),
				struct siw_ts, fb_work);

	if(atomic_read(&ts->state.fb) == FB_SUSPEND) {
		siw_touch_suspend(ts->dev);
		siwmon_submit_ops_step_core(ts->dev, "FB suspend", 0);
	} else if(atomic_read(&ts->state.fb) == FB_RESUME) {
		siw_touch_resume(ts->dev);
		siwmon_submit_ops_step_core(ts->dev, "FB Resume", 0);
	}
}

static void siw_touch_sys_reset_work_func(struct work_struct *work)
{
	struct siw_ts *ts =
			container_of(to_delayed_work(work),
				struct siw_ts, fb_work);
	struct device *dev = ts->dev;
	int ret;

	ret = siw_touch_sys_panel_reset(dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to reset panel\n");
	}
}

#if defined(__SIW_SUPPORT_ASC)
static void siw_touch_toggle_delta_check_work_func(
			struct work_struct *work)
{
	struct siw_ts *ts =
			container_of(to_delayed_work(work),
				struct siw_ts, toggle_delta_work);
	struct device *dev = ts->dev;
	struct asc_info *asc = &(ts->asc);
	int connect_status = atomic_read(&ts->state.connect);
	int wireless_status = atomic_read(&ts->state.wireless);
	int call_status = atomic_read(&ts->state.incoming_call);
	int onhand_status = atomic_read(&ts->state.onhand);
	int delta = DELTA_CHK_OFF;
	int target = NORMAL_SENSITIVITY;

	if (asc->use_asc == ASC_OFF) {
		t_dev_info(dev, "ASC is off\n");
		return;
	}

	t_dev_info(dev, "connect = %d, wireless = %d, call = %d, onhand = %d\n",
			connect_status, wireless_status,
			call_status, onhand_status);

	if ((connect_status != CONNECT_INVALID) ||
		(wireless_status != 0) ||
		(call_status != INCOMING_CALL_IDLE)) {
		/* */
	} else {
		delta = DELTA_CHK_ON;

		switch (onhand_status) {
		case IN_HAND_ATTN :
		case IN_HAND_NO_ATTN :
			break;
		case NOT_IN_HAND :
			target = ACUTE_SENSITIVITY;
			break;
		default:
			delta = -1;
			break;
		}
	}

	if (delta < 0) {
		t_dev_info(dev, "Unknown onhand_status\n");
		return;
	}

	asc->use_delta_chk = delta;
	siw_touch_change_sensitivity(ts, target);

	t_dev_info(dev, "curr_sensitivity = %s, use_delta_chk = %d\n",
			asc_str[asc->curr_sensitivity], asc->use_delta_chk);

	siwmon_submit_ops_step_core(dev, "Delta work done", 0);
}

static void siw_touch_finger_input_check_work_func(
		struct work_struct *work)
{
	struct siw_ts *ts =
		container_of(to_delayed_work(work),
				struct siw_ts, finger_input_work);

	if ((ts->tcount == 1) && (!ts->asc.delta_updated))
		siw_touch_get_max_delta(ts);

	if ((ts->tcount == 0) && (ts->asc.delta_updated))
		siw_touch_update_sensitivity(ts);

	return;
}
#endif	/* __SIW_SUPPORT_ASC */

static int siw_touch_mon_thread(void *d)
{
	struct siw_ts *ts = d;
	struct siw_ts_thread *ts_thread = &ts->mon_thread;
	struct device *dev = ts->dev;
	unsigned long timeout = ts_thread->interval * HZ;
	int ret = 0;

	atomic_set(&ts_thread->state, TS_THREAD_ON);

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout_interruptible(timeout);

		if (kthread_should_stop()) {
			t_dev_info(dev, "stopping mon thread[%s]\n",
					ts_thread->thread->comm);
			set_current_state(TASK_RUNNING);
			break;
		}

		set_current_state(TASK_RUNNING);

		ret = siw_ops_mon_handler(ts);
	}

	atomic_set(&ts_thread->state, TS_THREAD_OFF);

	return 0;
}

#define siw_touch_kthread_run(__dev, __threadfn, __data, __namefmt, args...) \
({	\
	struct task_struct *__k;	\
	char __name[64];	\
	snprintf(__name, sizeof(__name), __namefmt, ##args);	\
	__k = kthread_run(__threadfn, __data, "%s", __name);	\
	if (IS_ERR(__k))	\
		t_dev_err(__dev, "kthread_run failed : %s\n", __name);	\
	else	\
		t_dev_dbg_base(__dev, "kthread[%s] is running\n", __k->comm);	\
	__k;	\
})

static int __used siw_touch_init_thread(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct siw_ts_thread *ts_thread = NULL;
	struct task_struct *thread;
	int ret = 0;

	if (ts->flags & TOUCH_USE_MON_THREAD) {
		ts_thread = &ts->mon_thread;

		if (siw_ops_is_null(ts, mon_handler)) {
			t_dev_warn(dev, "No mon_handler defined!\n");
			goto mon_skip;
		}

		if (!ts->ops->mon_interval) {
			t_dev_warn(dev, "mon_interval is zero!\n");
			goto mon_skip;
		}

		do {
			touch_msleep(100);
		} while (atomic_read(&ts->state.core) != CORE_NORMAL);

		atomic_set(&ts_thread->state, TS_THREAD_OFF);
		ts_thread->interval = ts->ops->mon_interval;

		thread = siw_touch_kthread_run(dev,
						siw_touch_mon_thread, ts,
						"siw_touch-%d", MINOR(dev->devt));
		if (IS_ERR(thread)) {
			ret = PTR_ERR(thread);
			goto out;
		}
		ts_thread->thread = thread;
		t_dev_info(dev, "mon thread[%s, %d] begins\n",
				thread->comm, ts->ops->mon_interval);
	}
mon_skip:

	return 0;

out:
	return ret;
}

static void __used siw_touch_free_thread(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct siw_ts_thread *ts_thread = NULL;

	if (ts->flags & TOUCH_USE_MON_THREAD) {
		ts_thread = &ts->mon_thread;

		if (ts_thread->thread) {
			char comm[TASK_COMM_LEN];

			memcpy(comm, ts_thread->thread->comm, TASK_COMM_LEN);
			kthread_stop(ts_thread->thread);

			while (1) {
				touch_msleep(10);
				if (atomic_read(&ts_thread->state) == TS_THREAD_OFF)
					break;
				t_dev_info(dev,
					"waiting for killing mon thread[%s]\n", comm);
			};

			ts_thread->thread = NULL;
		}
	}
}

static int __used siw_touch_init_works(struct siw_ts *ts)
{
	ts->wq = create_singlethread_workqueue("touch_wq");
	if (!ts->wq) {
		t_dev_err(ts->dev, "failed to create workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&ts->init_work, siw_touch_init_work_func);
	INIT_DELAYED_WORK(&ts->upgrade_work, siw_touch_upgrade_work_func);
	INIT_DELAYED_WORK(&ts->fb_work, siw_touch_fb_work_func);
#if defined(__SIW_SUPPORT_ASC)
	INIT_DELAYED_WORK(&ts->toggle_delta_work,
					siw_touch_toggle_delta_check_work_func);
	INIT_DELAYED_WORK(&ts->finger_input_work,
					siw_touch_finger_input_check_work_func);
#endif	/* __SIW_SUPPORT_ASC */
	INIT_DELAYED_WORK(&ts->notify_work, siw_touch_atomic_notifer_work_func);
	INIT_DELAYED_WORK(&ts->sys_reset_work, siw_touch_sys_reset_work_func);

	return 0;
}

static void __used siw_touch_free_works(struct siw_ts *ts)
{
	if (ts->wq) {
		cancel_delayed_work(&ts->sys_reset_work);
		cancel_delayed_work(&ts->notify_work);
	#if defined(__SIW_SUPPORT_ASC)
		cancel_delayed_work(&ts->finger_input_work);
		cancel_delayed_work(&ts->toggle_delta_work);
	#endif	/* __SIW_SUPPORT_ASC */
		cancel_delayed_work(&ts->fb_work);
		cancel_delayed_work(&ts->upgrade_work);
		cancel_delayed_work(&ts->init_work);

		destroy_workqueue(ts->wq);
		ts->wq = NULL;
	}

	siw_touch_free_thread(ts);
}

static irqreturn_t __used siw_touch_irq_handler(int irq, void *dev_id)
{
	struct siw_ts *ts = (struct siw_ts *)dev_id;
	struct device *dev = ts->dev;

	t_dev_dbg_irq(dev, "irq_handler\n");

	if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
		t_dev_info(dev, "interrupt in suspend[%d]\n",
				atomic_read(&ts->state.pm));
		atomic_set(&ts->state.pm, DEV_PM_SUSPEND_IRQ);
		wake_lock_timeout(&ts->lpwg_wake_lock, msecs_to_jiffies(1000));
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

static int _siw_touch_do_irq_thread(struct siw_ts *ts)
{
	int ret = 0;

	ts->intr_status = 0;

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		goto out;
	}

	ret = siw_ops_irq_handler(ts);
	if (ret < 0) {
		t_dev_dbg_irq(ts->dev, "Err in irq_handler of %s, %d",
				touch_chip_name(ts), ret);
		if (ret == -ERESTART) {
			siw_touch_qd_sys_reset_work_now(ts);
			siw_ops_reset(ts, HW_RESET_ASYNC);
		}
		return ret;
	}

	if (ts->intr_status & TOUCH_IRQ_FINGER) {
		siw_touch_report_event(ts);

	#if defined(__SIW_SUPPORT_ASC)
		if (ts->asc.use_delta_chk == DELTA_CHK_ON) {
			siw_touch_qd_finger_input_work_jiffies(ts, 0);
		}
	#endif
	}

	if (ts->intr_status & TOUCH_IRQ_KNOCK)
		siw_touch_send_uevent(ts, TOUCH_UEVENT_KNOCK);

	if (ts->intr_status & TOUCH_IRQ_PASSWD)
		siw_touch_send_uevent(ts, TOUCH_UEVENT_PASSWD);

	if (ts->intr_status & TOUCH_IRQ_SWIPE_RIGHT)
		siw_touch_send_uevent(ts, TOUCH_UEVENT_SWIPE_RIGHT);

	if (ts->intr_status & TOUCH_IRQ_SWIPE_LEFT)
		siw_touch_send_uevent(ts, TOUCH_UEVENT_SWIPE_LEFT);

out:
	return ret;
}

static irqreturn_t __used siw_touch_irq_thread(int irq, void *dev_id)
{
	struct siw_ts *ts = (struct siw_ts *)dev_id;
	struct device *dev = ts->dev;
	int ret = 0;

	t_dev_dbg_irq(dev, "irq_thread\n");

	mutex_lock(&ts->lock);
	ret = _siw_touch_do_irq_thread(ts);
	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}

static int __used siw_touch_verify_pdata(struct siw_ts *ts)
{
	struct siw_touch_operations *ops = ts->ops;

	if (0 ||
		!ops->probe ||
		!ops->remove ||
		!ops->suspend ||
		!ops->resume ||
		!ops->init ||
		!ops->reset ||
		!ops->ic_info ||
		!ops->tc_driving ||
		!ops->chk_status ||
		!ops->irq_handler ||
		!ops->irq_abs ||
		!ops->irq_lpwg ||
		!ops->power ||
		!ops->upgrade ||
		!ops->asc ||
		0)
		return -EPERM;

	return 0;
}

static struct siw_touch_pdata *_siw_touch_do_probe_common(struct siw_ts *ts)
{
	struct siw_touch_pdata *pdata = NULL;
	struct device *dev = ts->dev;
	int ret = 0;

	pdata = (struct siw_touch_pdata *)ts->pdata;
	if (!pdata) {
		t_dev_err(dev, "NULL core pdata\n");
		goto out;
	}

	t_dev_info(dev, "%s quirks = 0x%08X\n",
			touch_chip_name(ts), (u32)touch_get_quirks(ts));

	atomic_set(&ts->state.core, CORE_EARLY_PROBE);

	if (!ts->ops) {
		t_dev_warn(dev, "%s ops is NULL : default ops selected\n",
				touch_chip_name(ts));
		siw_setup_operations(ts, siw_hal_get_default_ops(0));
	}

	ret = siw_ops_early_probe(ts);
	if (ret) {
		t_dev_err(dev, "failed to early_probe, %d\n", ret);
		goto out;
	}

	atomic_set(&ts->state.core, CORE_PROBE);

	ret = siw_touch_verify_pdata(ts);
	if (ret) {
		t_dev_err(dev, "failed to check functions, %d\n", ret);
		goto out;
	}

	ret = siw_touch_parse_data(ts);
	if (ret) {
		t_dev_err(dev, "failed to parse touch data, %d\n", ret);
		goto out;
	}

	siw_touch_bus_alloc_buffer(ts);
	siw_touch_bus_pin_get(ts);

	ret = siw_ops_probe(ts);
	if (ret) {
		t_dev_err(dev, "failed to probe, %d\n", ret);
		goto out_init;
	}

	return pdata;

out_init:
	siw_touch_bus_pin_put(ts);
	siw_touch_bus_free_buffer(ts);

out:
	return NULL;
}

static void _siw_touch_do_remove_common(struct siw_ts *ts)
{
//	struct siw_touch_pdata *pdata = (struct siw_touch_pdata *)ts->pdata;

	siw_ops_remove(ts);

	siw_touch_bus_pin_put(ts);
	siw_touch_bus_free_buffer(ts);
}

static int siw_touch_do_probe_normal(void *data)
{
	struct siw_ts *ts = data;
	struct siw_touch_pdata *pdata = NULL;
	struct device *dev = NULL;
	const char *irq_name = NULL;
	int ret = 0;

	pdata = _siw_touch_do_probe_common(ts);
	if (!pdata) {
		return -EINVAL;
	}

	dev = ts->dev;

	/* set defalut lpwg value because of AAT */
	ts->lpwg.screen = 1;
	ts->lpwg.sensor = PROX_FAR;

	siw_ops_power(ts, POWER_OFF);
	siw_ops_power(ts, POWER_ON);

	siw_touch_init_locks(ts);
	ret = siw_touch_init_works(ts);
	if (ret) {
		t_dev_err(dev, "failed to initialize works\n");
		goto out_init_works;
	}

	ret = siw_touch_init_input(ts);
	if (ret) {
		t_dev_err(dev, "failed to register input device, %d\n", ret);
		goto out_init_input;
	}

#if defined(__SIW_TEST_IRQ_OFF)
	ts->irq = 0;
#else	/* __SIW_TEST_IRQ_OFF */
	irq_name = (const char *)touch_drv_name(ts);
	irq_name = (irq_name)? irq_name : SIW_TOUCH_NAME;
	ret = siw_touch_request_irq(ts,
						siw_touch_irq_handler,
						siw_touch_irq_thread,
						ts->irqflags,
						irq_name);
	if (ret) {
		t_dev_err(dev, "failed to request thread irq(%d), %d\n",
					ts->irq, ret);
		goto out_request_irq;
	}
//	irq_set_irq_type(ts->irq, IRQF_ONESHOT | IRQ_TYPE_EDGE_FALLING);
#endif	/* __SIW_TEST_IRQ_OFF */

	siw_touch_disable_irq(dev, ts->irq);
//	t_dev_dbg_irq(dev, "disable irq until init completed\n");

	siw_touch_init_pm(ts);

	ret = siw_touch_init_notify(ts);
	if (ret) {
		t_dev_err(dev, "failed to initialize notifier, %d\n", ret);
		goto out_init_notify;
	}
	siw_touch_blocking_notifier_call(
							LCD_EVENT_TOUCH_DRIVER_REGISTERED,
							NULL);

	ret = siw_touch_init_uevent(ts);
	if (ret) {
		t_dev_err(dev, "failed to initialize uevent, %d\n", ret);
		goto out_init_uevent;
	}

	ret = siw_touch_init_sysfs(ts);
	if (ret) {
		t_dev_err(dev, "failed to initialize sysfs, %d\n", ret);
		goto out_init_sysfs;
	}

	t_dev_dbg_base(dev, "hw_reset_delay : %d ms\n", ts->caps.hw_reset_delay);
	siw_touch_qd_init_work_hw(ts);

	ret = siw_touch_init_thread(ts);
	if (ret) {
		t_dev_err(ts->dev, "failed to create workqueue\n");
		goto out_init_thread;
	}

	return 0;

out_init_thread:

out_init_sysfs:
	siw_touch_free_uevent(ts);

out_init_uevent:
	siw_touch_blocking_notifier_call(
							LCD_EVENT_TOUCH_DRIVER_UNREGISTERED,
							NULL);
	siw_touch_free_notify(ts);

out_init_notify:
	siw_touch_free_pm(ts);
	siw_touch_free_irq(ts);

out_request_irq:
	siw_touch_free_input(ts);

out_init_input:
	siw_touch_free_works(ts);
	siw_touch_free_locks(ts);

out_init_works:
	siw_ops_power(ts, POWER_OFF);

	_siw_touch_do_remove_common(ts);

	return ret;
}

static int siw_touch_do_remove_normal(void *data)
{
	struct siw_ts *ts = data;
//	struct siw_touch_pdata *pdata = (struct siw_touch_pdata *)ts->pdata;
//	struct device *dev = ts->dev;

	siw_touch_free_thread(ts);

	siw_touch_free_sysfs(ts);

	siw_touch_free_uevent(ts);

	siw_touch_blocking_notifier_call(
							LCD_EVENT_TOUCH_DRIVER_UNREGISTERED,
							NULL);
	siw_touch_free_notify(ts);

	siw_touch_free_pm(ts);

	siw_touch_free_irq(ts);

	siw_touch_free_input(ts);

	siw_touch_free_works(ts);
	siw_touch_free_locks(ts);

	siw_ops_power(ts, POWER_OFF);

	_siw_touch_do_remove_common(ts);

	return 0;
}

static int siw_touch_do_probe_charger(void *data)
{
	struct siw_touch_pdata *pdata = NULL;
	struct siw_ts *ts = data;
	struct device *dev;
	int ret = 0;

	pdata = _siw_touch_do_probe_common(ts);
	if (!pdata) {
		return -EINVAL;
	}

	dev = ts->dev;

	siw_touch_init_locks(ts);
	ret = siw_touch_init_works(ts);
	if (ret) {
		t_dev_err(dev, "failed to initialize works, %d\n", ret);
		goto out_init_works;
	}

	siw_touch_init_pm(ts);

	ret = siw_touch_init_notify(ts);
	if (ret) {
		t_dev_err(dev, "failed to initialize notifier, %d\n", ret);
		goto out_init_notify;
	}
	siw_touch_blocking_notifier_call(
							LCD_EVENT_TOUCH_DRIVER_REGISTERED,
							NULL);

	return 0;

out_init_notify:
	siw_touch_free_pm(ts);
	siw_touch_free_works(ts);

out_init_works:
	_siw_touch_do_remove_common(ts);

	return ret;
}

static int siw_touch_do_remove_charger(void *data)
{
	struct siw_ts *ts = data;
//	struct device *dev = ts->dev;
//	struct siw_touch_pdata *pdata = (struct siw_touch_pdata *)ts->pdata;

	siw_touch_blocking_notifier_call(LCD_EVENT_TOUCH_DRIVER_UNREGISTERED, NULL);
	siw_touch_free_notify(ts);

	siw_touch_free_works(ts);

	_siw_touch_do_remove_common(ts);

	return 0;
}

enum {
	SIW_CORE_NORMAL = 0,
	SIW_CORE_CHARGER,
};

static const struct siw_op_dbg siw_touch_probe_ops[2][2] = {
	[SIW_CORE_NORMAL] = {
		_SIW_OP_DBG(DRIVER_FREE, siw_touch_do_remove_normal, NULL, 0),
		_SIW_OP_DBG(DRIVER_INIT, siw_touch_do_probe_normal, NULL, 0),
	},
	[SIW_CORE_CHARGER] = {
		_SIW_OP_DBG(DRIVER_FREE, siw_touch_do_remove_charger, NULL, 0),
		_SIW_OP_DBG(DRIVER_INIT, siw_touch_do_probe_charger, NULL, 0),
	},
};

static int __siw_touch_probe_op(
					struct siw_ts *ts,
					void *op_func)
{
	struct siw_op_dbg *op = op_func;
	int ret = 0;

	t_dev_dbg_base(ts->dev, "%s begins\n", op->name);

	ret = __siw_touch_op_dbg(op, ts);
	if (ret) {
		t_dev_err(ts->dev, "%s failed, %d\n", op->name, ret);
		goto out;
	}

out:
	return ret;
}

static int __siw_touch_probe_normal(struct siw_ts *ts, int on_off)
{
	return __siw_touch_probe_op(ts,
			(void *)&siw_touch_probe_ops[SIW_CORE_NORMAL][on_off]);
}

#define siw_touch_probe_normal(_ts)	\
		__siw_touch_probe_normal(_ts, DRIVER_INIT)
#define siw_touch_remove_normal(_ts)	\
		__siw_touch_probe_normal(_ts, DRIVER_FREE)

static int __siw_touch_probe_charger(struct siw_ts *ts, int on_off)
{
	return __siw_touch_probe_op(ts,
			(void *)&siw_touch_probe_ops[SIW_CORE_CHARGER][on_off]);
}

#define siw_touch_probe_charger(_ts)	\
		__siw_touch_probe_charger(_ts, DRIVER_INIT)
#define siw_touch_remove_charger(_ts)	\
		__siw_touch_probe_charger(_ts, DRIVER_FREE)

int siw_touch_probe(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	BUILD_BUG_ON(sizeof(struct touch_data) != (9<<1));

	t_dev_info(dev, "SiW Touch Probe\n");

	if (siw_touch_get_boot_mode() == SIW_TOUCH_CHARGER_MODE) {
		t_dev_info(dev, "Probe - Charger mode\n");
		return siw_touch_probe_charger(ts);
	}

	return siw_touch_probe_normal(ts);
}

int siw_touch_remove(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	t_dev_info(dev, "SiW Touch Remove\n");

	if (siw_touch_get_boot_mode() == SIW_TOUCH_CHARGER_MODE) {
		t_dev_info(dev, "Remove - Charger mode\n");
		return siw_touch_remove_charger(ts);
	}

	return siw_touch_remove_normal(ts);
}



#if defined(CONFIG_TOUCHSCREEN_SIWMON)

struct siw_mon_operations *siw_mon_ops;
EXPORT_SYMBOL(siw_mon_ops);

int siw_mon_register(struct siw_mon_operations *ops)
{
	if (siw_mon_ops)
		return -EBUSY;

	siw_mon_ops = ops;
	t_pr_dbg(DBG_BASE, "siw mon ops assigned\n");
	mb();
	return 0;
}
EXPORT_SYMBOL_GPL(siw_mon_register);

void siw_mon_deregister(void)
{
	if (siw_mon_ops == NULL) {
		t_pr_err("monitor was not registered\n");
		return;
	}
	siw_mon_ops = NULL;
	t_pr_dbg(DBG_BASE, "siw mon ops released\n");
	mb();
}
EXPORT_SYMBOL_GPL(siw_mon_deregister);

#endif	/* CONFIG_TOUCHSCREEN_SIWMON */


