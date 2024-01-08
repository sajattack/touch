/*
 * siw_touch_event.c - SiW touch event driver
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
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_event.h"

static void siw_touch_finger_cancel_event(struct siw_ts *ts)
{
#if defined(__SIW_SUPPORT_CANCEL_EVENT)
	struct input_dev *input = ts->input;
	u32 old_mask = ts->old_mask;
	u32 event_cnt = ts->event_cnt_finger;
	int i = 0;

	if (!input) {
		t_dev_err(ts->dev, "no input device (cancel)\n");
		return;
	}

	for (i = 0; i < touch_max_finger(ts); i++) {
		if (old_mask & (1 << i)) {
			input_mt_slot(input, i);
		#if defined(__SIW_CONFIG_INPUT_ANDROID)
			input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		#endif	/* __SIW_CONFIG_INPUT_ANDROID */
			siw_input_report_btn_touch(input, 1);
			siw_input_report_btn_tool_finger(input, 1);
			siw_input_report_abs(input, ABS_MT_PRESSURE, 255);
			t_dev_info(ts->dev, "%s: <%d> finger canceled <%d> (%4d, %4d, %4d)\n",
					dev_name(&input->dev),
					event_cnt,
					i,
					ts->tdata[i].x,
					ts->tdata[i].y,
					ts->tdata[i].pressure);
		}
	}

	input_sync(input);
#endif	/* __SIW_SUPPORT_CANCEL_EVENT */
}

static void siw_touch_finger_report_event(struct siw_ts *ts)
{
	struct input_dev *input = ts->input;
	struct device *dev = ts->dev;
	struct device *idev = NULL;
	struct touch_data *tdata = NULL;
	u32 old_mask = ts->old_mask;
	u32 new_mask = ts->new_mask;
	u32 press_mask = 0;
	u32 release_mask = 0;
	u32 change_mask = 0;
	u32 event_cnt = ts->event_cnt_finger;
	int tcount = ts->tcount;
	int i;

//	t_dev_trcf(idev);

	if (!input) {
		t_dev_err(dev, "no input device (report)\n");
		return;
	}

	idev = &input->dev;

	change_mask = old_mask ^ new_mask;
	press_mask = new_mask & change_mask;
	release_mask = old_mask & change_mask;

	t_dev_dbg_abs(dev,
			"%s: <%d> mask [new: %04x, old: %04x]\n",
			dev_name(idev), event_cnt, new_mask, old_mask);
	t_dev_dbg_abs(dev,
			"%s: <%d> mask [change: %04x, press: %04x, release: %04x]\n",
			dev_name(idev), event_cnt, change_mask, press_mask, release_mask);

	/* Palm state - Report Pressure value 255 */
	if (ts->is_cancel) {
		siw_touch_finger_cancel_event(ts);
		ts->is_cancel = 0;
	}

	tdata = ts->tdata;
	for (i = 0; i < touch_max_finger(ts); i++, tdata++) {
		if (new_mask & (1 << i)) {
			input_mt_slot(input, i);
		#if defined(__SIW_CONFIG_INPUT_ANDROID)
			input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		#endif	/* __SIW_CONFIG_INPUT_ANDROID */
			siw_input_report_btn_touch(input, 1);
			siw_input_report_btn_tool_finger(input, 1);
			siw_input_report_abs(input, ABS_MT_TRACKING_ID, tdata->id);
			siw_input_report_abs(input, ABS_MT_POSITION_X, tdata->x);
			siw_input_report_abs(input, ABS_MT_POSITION_Y, tdata->y);
			siw_input_report_abs(input, ABS_MT_PRESSURE, tdata->pressure);
			siw_input_report_abs(input, ABS_MT_WIDTH_MAJOR, tdata->width_major);
			siw_input_report_abs(input, ABS_MT_WIDTH_MINOR, tdata->width_minor);
			siw_input_report_abs(input, ABS_MT_ORIENTATION, tdata->orientation);

			if (press_mask & (1 << i)) {
				t_dev_dbg_button(dev, "%s: <%d> %d finger press <%d> (%4d, %4d, %4d)\n",
						dev_name(idev),
						event_cnt,
						tcount,
						i,
						tdata->x,
						tdata->y,
						tdata->pressure);
			}

			continue;
		}

		if (release_mask & (1 << i)) {
			input_mt_slot(input, i);
		#if defined(__SIW_CONFIG_INPUT_ANDROID)
			input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
		#else	/* __SIW_CONFIG_INPUT_ANDROID */
			siw_input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
		#endif	/* __SIW_CONFIG_INPUT_ANDROID */
			t_dev_dbg_button(dev, "%s: <%d> finger release <%d> (%4d, %4d, %4d)\n",
					dev_name(idev),
					event_cnt,
					i,
					tdata->x,
					tdata->y,
					tdata->pressure);
		}
	}

	if (!tcount) {
		siw_input_report_btn_touch(input, 0);
		siw_input_report_btn_tool_finger(input, 0);
	}

	ts->old_mask = new_mask;

	input_sync(input);

	ts->event_cnt_finger++;
}

void siw_touch_report_event(void *ts_data)
{
	struct siw_ts *ts = ts_data;

	siw_touch_finger_report_event(ts);
}

void siw_touch_report_all_event(void *ts_data)
{
	struct siw_ts *ts = ts_data;

	ts->is_cancel = 1;
	if (ts->old_mask) {
		ts->new_mask = 0;
		siw_touch_finger_report_event(ts);
		ts->tcount = 0;
		memset(ts->tdata, 0, sizeof(struct touch_data) * touch_max_finger(ts));
	}
	ts->is_cancel = 0;
}

#if defined(__SIW_SUPPORT_UEVENT)
static void siw_touch_uevent_release(struct device *dev)
{
	if (dev->platform_data)
		dev->platform_data = NULL;
}

static const struct siw_touch_uevent_ctrl siw_uevent_ctrl_default = {
	.str = {
		[TOUCH_UEVENT_KNOCK] = {"TOUCH_GESTURE_WAKEUP=WAKEUP", NULL},
		[TOUCH_UEVENT_PASSWD] = {"TOUCH_GESTURE_WAKEUP=PASSWORD", NULL},
		[TOUCH_UEVENT_SWIPE_RIGHT] = {"TOUCH_GESTURE_WAKEUP=SWIPE_RIGHT", NULL},
		[TOUCH_UEVENT_SWIPE_LEFT] = {"TOUCH_GESTURE_WAKEUP=SWIPE_LEFT", NULL},
		[TOUCH_UEVENT_SWIPE_UP] = {"TOUCH_GESTURE_WAKEUP=SWIPE_UP", NULL},
		[TOUCH_UEVENT_SWIPE_DOWN] = {"TOUCH_GESTURE_WAKEUP=SWIPE_DOWN", NULL},
		[TOUCH_UEVENT_GESTURE_C] = {"TOUCH_GESTURE_WAKEUP=GESTURE_C", NULL},
		[TOUCH_UEVENT_GESTURE_W] = {"TOUCH_GESTURE_WAKEUP=GESTURE_W", NULL},
		[TOUCH_UEVENT_GESTURE_V] = {"TOUCH_GESTURE_WAKEUP=GESTURE_V", NULL},
		[TOUCH_UEVENT_GESTURE_O] = {"TOUCH_GESTURE_WAKEUP=GESTURE_O", NULL},
		[TOUCH_UEVENT_GESTURE_S] = {"TOUCH_GESTURE_WAKEUP=GESTURE_S", NULL},
		[TOUCH_UEVENT_GESTURE_E] = {"TOUCH_GESTURE_WAKEUP=GESTURE_E", NULL},
		[TOUCH_UEVENT_GESTURE_M] = {"TOUCH_GESTURE_WAKEUP=GESTURE_M", NULL},
		[TOUCH_UEVENT_GESTURE_Z] = {"TOUCH_GESTURE_WAKEUP=GESTURE_Z", NULL},
		[TOUCH_UEVENT_GESTURE_DIR_RIGHT] = {"TOUCH_GESTURE_WAKEUP=GESTURE_DIR_RIGHT", NULL},
		[TOUCH_UEVENT_GESTURE_DIR_DOWN] = {"TOUCH_GESTURE_WAKEUP=GESTURE_DIR_DOWN", NULL},
		[TOUCH_UEVENT_GESTURE_DIR_LEFT] = {"TOUCH_GESTURE_WAKEUP=GESTURE_DIR_LEFT", NULL},
		[TOUCH_UEVENT_GESTURE_DIR_UP] = {"TOUCH_GESTURE_WAKEUP=GESTURE_DIR_UP", NULL},
	},
	.flags = 0,
};

#define siw_kobject_uevent_env(_dev, _type, _kobj, _action, _envp_ext)	\
	do {	\
		kobject_uevent_env(_kobj, _action, _envp_ext);	\
		siwmon_submit_evt(_dev, "@UEVENT", _type, #_action, _action, 0, 0);	\
	} while (0)

void siw_touch_send_uevent(void *ts_data, int type)
{
	struct siw_ts *ts = ts_data;
	struct input_dev *input = ts->input;
	struct device *dev = ts->dev;
	struct siw_touch_uevent_ctrl *uevent_ctrl = touch_uevent_ctrl(ts);
	char *str = NULL;

	if (!input) {
		t_dev_err(dev, "no input device (uevent)\n");
		return;
	}

	if (uevent_ctrl == NULL) {
		uevent_ctrl = (struct siw_touch_uevent_ctrl *)&siw_uevent_ctrl_default;
	}

	if (type >= TOUCH_UEVENT_MAX) {
		t_dev_err(dev, "Invalid event type, %d\n", type);
		return;
	}

	if ((uevent_ctrl->str[type] == NULL) ||
		(uevent_ctrl->str[type][0] == NULL)) {
		t_dev_err(dev, "empty event str, %d\n", type);
		return;
	}

	str = uevent_ctrl->str[type][0];

	if (t_dbg_flag & DBG_FLAG_SKIP_UEVENT) {
		t_dev_dbg_event(dev, "skip uevent, %s\n", str);
		return;
	}

	if (atomic_read(&ts->state.uevent) == UEVENT_IDLE) {
		siw_touch_lpwg_wake_lock(ts->dev, 3000);

		atomic_set(&ts->state.uevent, UEVENT_BUSY);

		siw_kobject_uevent_env(dev, type, &ts->udev.kobj,
						KOBJ_CHANGE, uevent_ctrl->str[type]);

		t_dev_info(dev, "%s\n", str);
		siw_touch_report_all_event(ts);

		/*
		 * Echo reply to lpwg_data(_store_lpwg_data) is required to
		 * clear(UEVENT_IDLE) state.event
		 */
	}
}

int siw_touch_init_uevent(void *ts_data)
{
	struct siw_ts *ts = ts_data;
	char *drv_name = touch_drv_name(ts);
	int ret = 0;

	if (drv_name) {
		ts->ubus.name = drv_name;
		ts->ubus.dev_name = drv_name;
	} else {
		ts->ubus.name = SIW_TOUCH_NAME;
		ts->ubus.dev_name = SIW_TOUCH_NAME;
	}

	memset((void *)&ts->udev, 0, sizeof(ts->udev));
	ts->udev.bus = &ts->ubus;
	ts->udev.release = siw_touch_uevent_release;

	ret = subsys_system_register(&ts->ubus, NULL);
	if (ret < 0) {
		t_dev_err(ts->dev, "bus is not registered, %d\n", ret);
		goto out_subsys;
	}

	ret = device_register(&ts->udev);
	if (ret < 0) {
		t_dev_err(ts->dev, "device is not registered, %d\n",ret);
		goto out_dev;
	}

	return 0;

out_dev:
	bus_unregister(&ts->ubus);

out_subsys:

	return ret;
}

void siw_touch_free_uevent(void *ts_data)
{
	struct siw_ts *ts = ts_data;

	device_unregister(&ts->udev);

	bus_unregister(&ts->ubus);
}
#else	/* __SIW_SUPPORT_UEVENT */
void siw_touch_send_uevent(void *ts_data, int type)
{

}
int siw_touch_init_uevent(void *ts_data)
{
	return 0;
}

void siw_touch_free_uevent(void *ts_data)
{

}
#endif	/* __SIW_SUPPORT_UEVENT */

/* Initialize multi-touch slot */
static int siw_input_mt_init_slots(struct siw_ts *ts, struct input_dev *input)
{
//	struct device *dev = ts->dev;
	struct touch_device_caps *caps = &ts->caps;
	int max_id = caps->max_id;
	int ret = 0;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0))
	ret = input_mt_init_slots(input, max_id);
#else
	ret = input_mt_init_slots(input, max_id, caps->mt_slots_flags);
#endif

	return ret;
}

static int siw_touch_finger_init(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct touch_device_caps *caps = &ts->caps;
	struct input_dev *input = NULL;
	char *input_name = NULL;
	char *phys_name = NULL;
	int ret = 0;

	ts->event_cnt_finger = 0;

	if (ts->input) {
		t_dev_err(dev, "input device has been already allocated!\n");
		return -EINVAL;
	}

	phys_name = touch_kzalloc(dev, SIW_TOUCH_PHYS_NAME_SIZE, GFP_KERNEL);
	if (!phys_name) {
		t_dev_err(dev, "failed to allocate memory for phys_name\n");
		ret = -ENOMEM;
		goto out_phys;
	}

	input = input_allocate_device();
	if (!input) {
		t_dev_err(dev, "failed to allocate memory for input device\n");
		ret = -ENOMEM;
		goto out_input;
	}

	input_name = touch_idrv_name(ts);
	input_name = (input_name) ? input_name : SIW_TOUCH_INPUT;

	snprintf(phys_name, SIW_TOUCH_PHYS_NAME_SIZE,
			"%s/%s - %s",
			dev_name(dev->parent),
			dev_name(dev),
			input_name);

	if (touch_flags(ts) & TOUCH_USE_INPUT_PARENT) {
		input->dev.parent = dev;
	} else {
		/*
		 * To fix sysfs location
		 * With no parent, sysfs folder is created at
		 * '/sys/devices/virtual/input/{input_name}'
		 */
		input->dev.parent = NULL;
	}

	input->name = (const char *)input_name;
	input->phys = (const char *)phys_name;;
	memcpy((void *)&input->id, (void *)&ts->i_id, sizeof(input->id));

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_ABS, input->evbit);
	siw_input_set_ev_key(input);
	siw_input_set_btn_touch(input);
	siw_input_set_btn_tool_finger(input);
#if defined(INPUT_PROP_DIRECT)
	set_bit(INPUT_PROP_DIRECT, input->propbit);
#endif
	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
				caps->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
				caps->max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
				caps->max_pressure, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0,
				caps->max_width, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR, 0,
				caps->max_width, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION, 0,
				caps->max_orientation, 0, 0);

	ret = siw_input_mt_init_slots(ts, input);
	if (ret < 0) {
		t_dev_err(dev, "failed to initialize input device, %d\n", ret);
		goto out_slot;
	}

	input_set_drvdata(input, ts);

	ret = input_register_device(input);
	if (ret < 0) {
		t_dev_err(dev, "failed to register input device, %d\n", ret);
		goto out_reg;
	}

	ts->input = input;

	t_dev_info(dev, "input device[%s, %s] registered\n",
			dev_name(&input->dev), input->phys);
	t_dev_info(dev, "input caps : %d, %d, %d, %d, %d, %d, %d, 0x%X\n",
			caps->max_x,
			caps->max_y,
			caps->max_pressure,
			caps->max_width,
			caps->max_width,
			caps->max_orientation,
			caps->max_id,
			caps->mt_slots_flags);

	return 0;

out_reg:

out_slot:
	input_free_device(input);

out_input:
	touch_kfree(dev, phys_name);

out_phys:

	return ret;
}

static void siw_touch_finger_free(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct input_dev *input = ts->input;

	if (input) {
		char *phys_name = (char *)input->phys;

		ts->input = NULL;

		t_dev_info(dev, "input device[%s] released\n", phys_name);

		input_unregister_device(input);

		touch_kfree(dev, phys_name);
	}
}

int siw_touch_init_input(void *ts_data)
{
	struct siw_ts *ts = ts_data;
	struct device *dev = ts->dev;
	int ret = 0;

#if defined(__SIW_CONFIG_INPUT_ANDROID)
	t_dev_info(dev, "input cfg status : __SIW_CONFIG_INPUT_ANDROID\n");
#endif

	ret = siw_touch_finger_init(ts);
	if (ret < 0) {
		goto out_finger;
	}

	return 0;

out_finger:

	return ret;
}

void siw_touch_free_input(void *ts_data)
{
	struct siw_ts *ts = ts_data;

	siw_touch_finger_free(ts);
}


