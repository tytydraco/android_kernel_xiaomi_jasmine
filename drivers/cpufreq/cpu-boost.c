/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "cpu-boost: " fmt

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/time.h>

static struct workqueue_struct *cpu_boost_wq;

static struct work_struct input_boost_work;
static struct work_struct cooldown_boost_work;
static struct delayed_work input_boost_rem;
static struct delayed_work cooldown_boost_rem;

static __read_mostly unsigned int input_boost_ms = 40;
static __read_mostly unsigned int cooldown_boost_ms = 3000;
static __read_mostly unsigned int input_stune_boost = 15;
static __read_mostly unsigned int cooldown_stune_boost = 12;

module_param(input_boost_ms, uint, 0644);
module_param(cooldown_boost_ms, uint, 0644);
module_param(input_stune_boost, uint, 0644);
module_param(cooldown_stune_boost, uint, 0644);

static int input_stune_slot;
static int cooldown_stune_slot;

static bool input_stune_boost_active;
static bool cooldown_stune_boost_active;

static u64 last_input_time;

/* How long after an input before another input boost can be triggered */
#define MIN_INPUT_INTERVAL (input_boost_ms * USEC_PER_MSEC)

static void do_input_boost_rem(struct work_struct *work)
{
	if (input_stune_boost_active) {
		reset_stune_boost("top-app", input_stune_slot);
		input_stune_boost_active = false;
	}

	queue_work(cpu_boost_wq, &cooldown_boost_work);
}

static void do_input_boost(struct work_struct *work)
{
	cancel_delayed_work_sync(&input_boost_rem);

	if (!input_stune_boost_active) {
		do_stune_boost("top-app", input_stune_boost, &input_stune_slot);
		input_stune_boost_active = true;
	}

	queue_delayed_work(cpu_boost_wq, &input_boost_rem,
					msecs_to_jiffies(input_boost_ms));
}

static void do_cooldown_boost_rem(struct work_struct *work)
{
	if (cooldown_stune_boost_active) {
		reset_stune_boost("top-app", cooldown_stune_slot);
		cooldown_stune_boost_active = false;
	}
}

static void do_cooldown_boost(struct work_struct *work)
{
	cancel_delayed_work_sync(&cooldown_boost_rem);

	if (!cooldown_stune_boost_active) {
		do_stune_boost("top-app", cooldown_stune_boost, &cooldown_stune_slot);
		cooldown_stune_boost_active = true;
	}

	queue_delayed_work(cpu_boost_wq, &cooldown_boost_rem,
					msecs_to_jiffies(cooldown_boost_ms));
}

static void cpuboost_input_event(struct input_handle *handle,
		unsigned int type, unsigned int code, int value)
{
	u64 now;

	now = ktime_to_us(ktime_get());
	if (now - last_input_time < MIN_INPUT_INTERVAL)
		return;

	if (work_pending(&input_boost_work))
		return;

	queue_work(cpu_boost_wq, &input_boost_work);
	last_input_time = ktime_to_us(ktime_get());
}

static int cpuboost_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void cpuboost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpuboost_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler cpuboost_input_handler = {
	.event          = cpuboost_input_event,
	.connect        = cpuboost_input_connect,
	.disconnect     = cpuboost_input_disconnect,
	.name           = "cpu-boost",
	.id_table       = cpuboost_ids,
};

static int cpu_boost_init(void)
{
	int ret;

	cpu_boost_wq = alloc_workqueue("cpuboost_wq", WQ_HIGHPRI, 0);
	if (!cpu_boost_wq)
		return -EFAULT;

	INIT_WORK(&input_boost_work, do_input_boost);
	INIT_WORK(&cooldown_boost_work, do_cooldown_boost);
	INIT_DELAYED_WORK(&input_boost_rem, do_input_boost_rem);
	INIT_DELAYED_WORK(&cooldown_boost_rem, do_cooldown_boost_rem);

	ret = input_register_handler(&cpuboost_input_handler);

	return ret;
}
late_initcall(cpu_boost_init);
