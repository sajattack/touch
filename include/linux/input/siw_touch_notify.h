/*
 * SiW touch notifier
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef __SIW_TOUCH_NOTIFIER_H
#define __SIW_TOUCH_NOTIFIER_H

#include <linux/notifier.h>

/* the dsv on */
#define LCD_EVENT_TOUCH_LPWG_ON					0x01
#define LCD_EVENT_TOUCH_LPWG_OFF				0x02

#define LCD_EVENT_TOUCH_PWR_OFF    				0XFF
/* to let lcd-driver know touch-driver's status */
#define LCD_EVENT_TOUCH_DRIVER_REGISTERED		0x03
/* For notifying proxy status to operate ENA control in lcd driver*/
#define LCD_EVENT_TOUCH_PROXY_STATUS			0X04
#define LCD_EVENT_TOUCH_SLEEP_STATUS			0X05
#define LCD_EVENT_TOUCH_SWIPE_STATUS			0X06
/* For PPlus */
#define NOTIFY_TOUCH_RESET						0x07
#define NOTIFY_CONNECTION						0x09
#define NOTIFY_WIRELEES							0x0A
#define NOTIFY_IME_STATE						0x0B
#define NOTIFY_DEBUG_TOOL						0x0C
#define NOTIFY_CALL_STATE						0x0D
#define NOTIFY_FB								0x0E
#define NOTIFY_EARJACK							0x0F
#define LCD_EVENT_HW_RESET						(NOTIFY_TOUCH_RESET)
#define LCD_EVENT_LCD_MODE						0x08
#define LCD_EVENT_READ_REG						0x20
#define LCD_EVENT_TOUCH_WATCH_POS_UPDATE		0x21
#define LCD_EVENT_TOUCH_WATCH_LUT_UPDATE		0x22
#define LCD_EVENT_TOUCH_ESD_DETECTED			0x30
/* For PPlus end */
#define LCD_EVENT_TOUCH_RESET_START				0x10
#define LCD_EVENT_TOUCH_RESET_END				0x11
/* to let lcd-driver know touch-driver's status */
#define LCD_EVENT_TOUCH_DRIVER_UNREGISTERED		0x80

struct siw_touch_event {
		void *data;
};

int siw_touch_blocking_notifier_register(struct notifier_block *nb);
int siw_touch_blocking_notifier_unregister(struct notifier_block *nb);
int siw_touch_blocking_notifier_call(unsigned long val, void *v);

int siw_touch_atomic_notifier_register(struct notifier_block *nb);
int siw_touch_atomic_notifier_unregister(struct notifier_block *nb);
int siw_touch_atomic_notifier_call(unsigned long val, void *v);

int siw_touch_register_client(struct notifier_block *nb);
int siw_touch_unregister_client(struct notifier_block *nb);
int siw_touch_notifier_call_chain(unsigned long val, void *v);

#endif	/* __SIW_TOUCH_NOTIFIER_H */



