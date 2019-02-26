// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

#define pr_fmt(fmt) "cpu_input_boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>

static __read_mostly unsigned short input_boost_duration = CONFIG_INPUT_BOOST_DURATION_MS;
static __read_mostly int input_stune_boost = CONFIG_INPUT_BOOST_STUNE_LEVEL;
static __read_mostly int general_stune_boost = CONFIG_GENERAL_BOOST_STUNE_LEVEL;
static __read_mostly int suspend_stune_boost = CONFIG_SUSPEND_BOOST_STUNE_LEVEL;

module_param(input_boost_duration, short, 0644);
module_param_named(dynamic_stune_boost, input_stune_boost, int, 0644);
module_param(general_stune_boost, int, 0644);
module_param(suspend_stune_boost, int, 0644);

/* Available bits for boost_drv state */
#define INPUT_STUNE_BOOST	BIT(0)
#define GENERAL_STUNE_BOOST	BIT(1)
#define SUSPEND_STUNE_BOOST	BIT(2)

struct boost_drv {
	struct workqueue_struct *wq;
	struct work_struct input_boost;
	struct work_struct general_boost;
	struct delayed_work input_unboost;
	struct delayed_work general_unboost;
	struct notifier_block fb_notif;
	atomic64_t general_boost_expires;
	atomic_t general_boost_dur;
	atomic_t state;
	atomic64_t prev_input_jiffies;
	int input_stune_slot;
	int general_stune_slot;
	int suspend_stune_slot;
};

static struct boost_drv *boost_drv_g __read_mostly;

bool cpu_input_boost_within_timeout(unsigned int input_boost_timeout) {
	struct boost_drv *b = boost_drv_g;

	if (!b)
		return false;

	return time_before(jiffies, atomic_read(&b->prev_input_jiffies)
		+ msecs_to_jiffies(input_boost_timeout));
}

static u32 get_boost_state(struct boost_drv *b)
{
	return atomic_read(&b->state);
}

static void set_boost_bit(struct boost_drv *b, u32 state)
{
	atomic_or(state, &b->state);
}

static void clear_boost_bit(struct boost_drv *b, u32 state)
{
	atomic_andnot(state, &b->state);
}

static void set_stune_boost(struct boost_drv *b, u32 bit, int level, int *slot)
{
	u32 state = get_boost_state(b);

	if (level && !(state & bit)) {
		if (!do_stune_boost("top-app", level, slot))
			set_boost_bit(b, bit);
	}
}

static void clear_stune_boost(struct boost_drv *b, u32 bit, int slot)
{
	u32 state = get_boost_state(b);

	if (state & bit) {
		reset_stune_boost("top-app", slot);
		clear_boost_bit(b, bit);
	}
}

static void unboost_all_cpus(struct boost_drv *b)
{
	if (!cancel_delayed_work_sync(&b->input_unboost) &&
		!cancel_delayed_work_sync(&b->general_unboost))
		return;

	clear_stune_boost(b, INPUT_STUNE_BOOST, b->input_stune_slot);
	clear_stune_boost(b, GENERAL_STUNE_BOOST, b->general_stune_slot);
	clear_stune_boost(b, SUSPEND_STUNE_BOOST, b->suspend_stune_slot);
}

void cpu_input_boost_kick(void)
{
	struct boost_drv *b = boost_drv_g;

	if (!b)
		return;

	queue_work(b->wq, &b->input_boost);
}

static void __cpu_general_boost_kick(struct boost_drv *b,
	unsigned int duration_ms)
{
	unsigned long curr_expires, new_expires;

	do {
		curr_expires = atomic64_read(&b->general_boost_expires);
		new_expires = jiffies + msecs_to_jiffies(duration_ms);

		/* Skip this boost if there's a longer boost in effect */
		if (time_after(curr_expires, new_expires))
			return;
	} while (atomic64_cmpxchg(&b->general_boost_expires, curr_expires,
		new_expires) != curr_expires);

	atomic_set(&b->general_boost_dur, duration_ms);
	queue_work(b->wq, &b->general_boost);
}

void cpu_general_boost_kick(unsigned int duration_ms)
{
	struct boost_drv *b;

	if (!duration_ms)
		return;

	b = boost_drv_g;

	if (!b)
		return;

	if (get_boost_state(b) & INPUT_STUNE_BOOST)
		return;

	__cpu_general_boost_kick(b, duration_ms);
}

static void input_boost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(work, typeof(*b), input_boost);

	if (!cancel_delayed_work_sync(&b->input_unboost)) {
		set_stune_boost(b, INPUT_STUNE_BOOST, input_stune_boost,
			&b->input_stune_slot);
	}

	queue_delayed_work(b->wq, &b->input_unboost,
		msecs_to_jiffies(input_boost_duration));
}

static void general_boost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(work, typeof(*b), general_boost);

	if (!cancel_delayed_work_sync(&b->general_unboost)) {
		set_stune_boost(b, GENERAL_STUNE_BOOST, general_stune_boost,
			&b->general_stune_slot);
	}

	queue_delayed_work(b->wq, &b->general_unboost,
		msecs_to_jiffies(atomic_read(&b->general_boost_dur)));
}

static void input_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b =
		container_of(to_delayed_work(work), typeof(*b), input_unboost);

	clear_stune_boost(b, INPUT_STUNE_BOOST, b->input_stune_slot);
}

static void general_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b =
		container_of(to_delayed_work(work), typeof(*b), general_unboost);

	clear_stune_boost(b, GENERAL_STUNE_BOOST, b->general_stune_slot);
}

static int fb_notifier_cb(struct notifier_block *nb,
			  unsigned long action, void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), fb_notif);
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	/* Parse framebuffer blank events as soon as they occur */
	if (action != FB_EARLY_EVENT_BLANK)
		return NOTIFY_OK;

	/* Unboost when the screen turns off */
	if (*blank == FB_BLANK_UNBLANK) {
		clear_stune_boost(b, SUSPEND_STUNE_BOOST, b->suspend_stune_slot);
	} else {	
		unboost_all_cpus(b);
		set_stune_boost(b, SUSPEND_STUNE_BOOST, suspend_stune_boost,
			&b->suspend_stune_slot);
	}

	return NOTIFY_OK;
}

static void cpu_input_boost_input_event(struct input_handle *handle,
					unsigned int type, unsigned int code,
					int value)
{
	struct boost_drv *b = handle->handler->private;

	atomic64_set(&b->prev_input_jiffies, jiffies);
	queue_work(b->wq, &b->input_boost);
}

static int cpu_input_boost_input_connect(struct input_handler *handler,
					 struct input_dev *dev,
					 const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_input_boost_handle";

	ret = input_register_handle(handle);
	if (ret)
		goto free_handle;

	ret = input_open_device(handle);
	if (ret)
		goto unregister_handle;

	return 0;

unregister_handle:
	input_unregister_handle(handle);
free_handle:
	kfree(handle);
	return ret;
}

static void cpu_input_boost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_input_boost_ids[] = {
	/* Multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) }
	},
	/* Touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) }
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) }
	},
	{ }
};

static struct input_handler cpu_input_boost_input_handler = {
	.event		= cpu_input_boost_input_event,
	.connect	= cpu_input_boost_input_connect,
	.disconnect	= cpu_input_boost_input_disconnect,
	.name		= "cpu_input_boost_handler",
	.id_table	= cpu_input_boost_ids
};

static int __init cpu_input_boost_init(void)
{
	struct boost_drv *b;
	int ret;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	b->wq = alloc_workqueue("cpu_input_boost_wq", WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!b->wq) {
		ret = -ENOMEM;
		goto free_b;
	}

	INIT_WORK(&b->input_boost, input_boost_worker);
	INIT_WORK(&b->general_boost, general_boost_worker);
	INIT_DELAYED_WORK(&b->input_unboost, input_unboost_worker);
	INIT_DELAYED_WORK(&b->general_unboost, general_unboost_worker);
	atomic_set(&b->state, 0);

	cpu_input_boost_input_handler.private = b;
	ret = input_register_handler(&cpu_input_boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
	}

	b->fb_notif.notifier_call = fb_notifier_cb;
	b->fb_notif.priority = INT_MAX;
	ret = fb_register_client(&b->fb_notif);
	if (ret) {
		pr_err("Failed to register fb notifier, err: %d\n", ret);
		goto unregister_handler;
	}

	/* Allow global boost config access for external boosts */
	boost_drv_g = b;

	return 0;

unregister_handler:
	input_unregister_handler(&cpu_input_boost_input_handler);
free_b:
	kfree(b);
	return ret;
}
late_initcall(cpu_input_boost_init);
