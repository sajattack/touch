/*
 * siw_touch_irq.c - SiW touch interrupt driver
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
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/maple_tree.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_irq.h"

#if defined(__SIW_PANEL_CLASS_MOBILE)
#define __SIW_SUPPORT_IRQ_RESUME
#endif

static struct maple_tree sparse_irqs = MTREE_INIT_EXT(sparse_irqs,
					MT_FLAGS_ALLOC_RANGE |
					MT_FLAGS_LOCK_EXTERN |
					MT_FLAGS_USE_RCU,
					sparse_irq_lock);

struct irq_desc *irq_to_desc(unsigned int irq)
{
	return mtree_load(&sparse_irqs, irq);
}

#if defined(__SIW_SUPPORT_IRQ_RESUME)
static void siw_touch_irq_pending_onoff(struct device *dev,
					unsigned int irq, int onoff)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		return;
	}

	raw_spin_lock_irqsave(&desc->lock, flags);
	if (onoff) {
		desc->istate |= IRQS_PENDING;
		t_dev_info(dev, "set IRQS_PENDING(%d)\n", irq);
	} else {
		if (desc->istate & IRQS_PENDING) {
			t_dev_info(dev, "clr IRQS_PENDING(%d)\n", irq);
		}
		desc->istate &= ~IRQS_PENDING;
	}
	raw_spin_unlock_irqrestore(&desc->lock, flags);
}
#else	/* !__SIW_SUPPORT_IRQ_RESUME */
static void siw_touch_irq_pending_onoff(struct device *dev,
					unsigned int irq, int onoff)
{
	t_dev_dbg_base(dev, "no irq pending control\n");
}
#endif	/* __SIW_SUPPORT_IRQ_RESUME */

static void __used siw_touch_set_irq_pending(struct device *dev, unsigned int irq)
{
	siw_touch_irq_pending_onoff(dev, irq, 1);
}

static void __used siw_touch_clr_irq_pending(struct device *dev, unsigned int irq)
{
	siw_touch_irq_pending_onoff(dev, irq, 0);
}

void siw_touch_enable_irq_wake(struct device *dev,
									unsigned int irq)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (!ts->role.use_lpwg)
		return;

	enable_irq_wake(irq);
	t_dev_info(dev, "irq(%d) wake enabled\n", irq);
}

void siw_touch_disable_irq_wake(struct device *dev,
									unsigned int irq)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (!ts->role.use_lpwg)
		return;

	disable_irq_wake(irq);
	t_dev_info(dev, "irq(%d) wake disabled\n", irq);
}

void siw_touch_enable_irq(struct device *dev, unsigned int irq)
{
	siw_touch_clr_irq_pending(dev, irq);
	enable_irq(irq);
	t_dev_info(dev, "irq(%d) enabled\n", irq);
}

void siw_touch_disable_irq(struct device *dev, unsigned int irq)
{
	disable_irq_nosync(irq);	/* No waiting */
	t_dev_info(dev, "irq(%d) disabled\n", irq);
}

#if defined(__SIW_SUPPORT_IRQ_RESUME)
void siw_touch_resume_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int irq = ts->irq;

	t_dev_info(dev, "resume irq(%d)\n", irq);

	disable_irq(irq);
	siw_touch_set_irq_pending(dev, irq);
	enable_irq(irq);
}
#else	/* !__SIW_SUPPORT_IRQ_RESUME */
void siw_touch_resume_irq(struct device *dev)
{
	t_dev_dbg_base(dev, "no irq resume control\n");
}
#endif	/* __SIW_SUPPORT_IRQ_RESUME */

void siw_touch_irq_control(struct device *dev, int on_off)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (!ts->irqflags_curr) {
		t_dev_warn(dev, "irq not initialized\n");
		return;
	}

	t_dev_dbg_irq(dev, "touch_irq_control(%d)\n", on_off);

	if (on_off == INTERRUPT_ENABLE) {
		if (atomic_cmpxchg(&ts->state.irq_enable, 0, 1)) {
			t_dev_warn(dev, "(warn) already irq enabled\n");
			return;
		}

		siw_touch_enable_irq(dev, ts->irq);

		siw_touch_enable_irq_wake(dev, ts->irq);

		return;
	}

	if (!atomic_cmpxchg(&ts->state.irq_enable, 1, 0)) {
		t_dev_warn(dev, "(warn) already irq disabled\n");
		return;
	}

	siw_touch_disable_irq_wake(dev, ts->irq);

	siw_touch_disable_irq(dev, ts->irq);
}

#define __SIW_SUPPORT_IRQ_INDEX_CHECK

/*
 * Verify irq handler index(ts->irq) using gpio_to_irq
 */
#if defined(__SIW_SUPPORT_IRQ_INDEX_CHECK)
static int siw_touch_irq_index_check(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int irq_pin = touch_irq_pin(ts);
	int irq = 0;

	if (!gpio_is_valid(irq_pin)) {
		return 0;
	}

	irq = gpio_to_irq(irq_pin);
	if (irq <= 0) {
		t_dev_warn(dev, "check irq pin: gpio_to_irq(%d) = %d\n",
			irq_pin, irq);
		goto out;
	}

	if (ts->irq) {
		if (ts->irq != irq) {
			t_dev_warn(dev,
				"check irq index: ts->irq = %d vs. gpio_to_irq(%d) = %d\n",
				ts->irq, irq_pin, irq);
		}
		goto out;
	}

	t_dev_info(dev,
		"irq index(%d) is obtained via gpio_to_irq(%d)\n",
		irq, irq_pin);

	ts->irq = irq;

out:
	return irq;
}
#else	/* __SIW_SUPPORT_IRQ_INDEX_CHECK */
#define siw_touch_irq_index_check(_ts)	do {	} while (0)
#endif	/* __SIW_SUPPORT_IRQ_INDEX_CHECK */

int siw_touch_request_irq(struct siw_ts *ts,
								irq_handler_t handler,
							    irq_handler_t thread_fn,
							    unsigned long flags,
							    const char *name)
{
	struct device *dev = ts->dev;
	int ret = 0;

#if defined(__SIW_SUPPORT_IRQ_RESUME)
	t_dev_info(dev, "irq cfg status : __SIW_SUPPORT_IRQ_RESUME\n");
#endif

	siw_touch_irq_index_check(ts);

	if (!ts->irq) {
		t_dev_err(dev, "failed to request irq : zero irq\n");
		ret = -EFAULT;
		goto out;
	}

	ret = request_threaded_irq(ts->irq, handler, thread_fn, flags, name, (void *)ts);
	if (ret) {
		t_dev_err(dev, "failed to request irq(%d, %s, 0x%X), %d\n",
				ts->irq, name, (u32)flags, ret);
		goto out;
	}

	ts->irqflags_curr = flags;

	t_dev_info(dev, "threaded irq request done(%d, %s, 0x%X)\n",
			ts->irq, name, (u32)flags);

out:
	return ret;
}

void siw_touch_free_irq(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	if (ts->irqflags_curr) {
		free_irq(ts->irq, (void *)ts);
		t_dev_info(dev, "irq(%d) release done\n", ts->irq);
		ts->irqflags_curr = 0;
		return;
	}

//	t_dev_warn(dev, "(warn) already free_irq done\n");
}


