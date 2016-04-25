/*
 * SiW touch event
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef __SIW_TOUCH_EVENT_H
#define __SIW_TOUCH_EVENT_H

struct siw_ts;

enum _SIW_TOUCH_UEVENT {
	TOUCH_UEVENT_KNOCK = 0,
	TOUCH_UEVENT_PASSWD,
	TOUCH_UEVENT_SWIPE_RIGHT,
	TOUCH_UEVENT_SWIPE_LEFT,
	TOUCH_UEVENT_MAX,
};

extern void siw_touch_report_event(struct siw_ts *ts);
extern void siw_touch_report_all_event(struct siw_ts *ts);
extern void siw_touch_send_uevent(struct siw_ts *ts, int type);

extern int siw_touch_init_uevent(struct siw_ts *ts);
extern void siw_touch_free_uevent(struct siw_ts *ts);

extern int siw_touch_init_input(struct siw_ts *ts);
extern void siw_touch_free_input(struct siw_ts *ts);

#endif	/* __SIW_TOUCH_EVENT_H */

