/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 * Copyright (C) 2018 XiaoMi, Inc.
 *
 * $Revision: 14651 $
 * $Date: 2017-07-21 14:58:50 +0800 (周五, 21 七月 2017) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include "nt36xxx.h"

#define NVT_ID_BYTE_MAX 6
#define POINT_DATA_LEN 65

struct nvt_ts_data *ts;
static uint8_t bTouchIsAwake;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);

static const struct nvt_ts_mem_map NT36772_memory_map = {
	.EVENT_BUF_ADDR           = 0x11E00,
	.RAW_PIPE0_ADDR           = 0x10000,
	.RAW_PIPE0_Q_ADDR         = 0,
	.RAW_PIPE1_ADDR           = 0x12000,
	.RAW_PIPE1_Q_ADDR         = 0,
	.BASELINE_ADDR            = 0x10E70,
	.BASELINE_Q_ADDR          = 0,
	.BASELINE_BTN_ADDR        = 0x12E70,
	.BASELINE_BTN_Q_ADDR      = 0,
	.DIFF_PIPE0_ADDR          = 0x10830,
	.DIFF_PIPE0_Q_ADDR        = 0,
	.DIFF_PIPE1_ADDR          = 0x12830,
	.DIFF_PIPE1_Q_ADDR        = 0,
	.RAW_BTN_PIPE0_ADDR       = 0x10E60,
	.RAW_BTN_PIPE0_Q_ADDR     = 0,
	.RAW_BTN_PIPE1_ADDR       = 0x12E60,
	.RAW_BTN_PIPE1_Q_ADDR     = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0x10E68,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0x12E68,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0,
	.READ_FLASH_CHECKSUM_ADDR = 0x14000,
	.RW_FLASH_DATA_ADDR       = 0x14002,
};

static const struct nvt_ts_mem_map NT36525_memory_map = {
	.EVENT_BUF_ADDR           = 0x11A00,
	.RAW_PIPE0_ADDR           = 0x10000,
	.RAW_PIPE0_Q_ADDR         = 0,
	.RAW_PIPE1_ADDR           = 0x12000,
	.RAW_PIPE1_Q_ADDR         = 0,
	.BASELINE_ADDR            = 0x10B08,
	.BASELINE_Q_ADDR          = 0,
	.BASELINE_BTN_ADDR        = 0x12B08,
	.BASELINE_BTN_Q_ADDR      = 0,
	.DIFF_PIPE0_ADDR          = 0x1064C,
	.DIFF_PIPE0_Q_ADDR        = 0,
	.DIFF_PIPE1_ADDR          = 0x1264C,
	.DIFF_PIPE1_Q_ADDR        = 0,
	.RAW_BTN_PIPE0_ADDR       = 0x10634,
	.RAW_BTN_PIPE0_Q_ADDR     = 0,
	.RAW_BTN_PIPE1_ADDR       = 0x12634,
	.RAW_BTN_PIPE1_Q_ADDR     = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0x10AFC,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0x12AFC,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0,
	.READ_FLASH_CHECKSUM_ADDR = 0x14000,
	.RW_FLASH_DATA_ADDR       = 0x14002,
};

static const struct nvt_ts_mem_map NT36870_memory_map = {
	.EVENT_BUF_ADDR           = 0x25000,
	.RAW_PIPE0_ADDR           = 0x20000,
	.RAW_PIPE0_Q_ADDR         = 0x204C8,
	.RAW_PIPE1_ADDR           = 0x23000,
	.RAW_PIPE1_Q_ADDR         = 0x234C8,
	.BASELINE_ADDR            = 0x21350,
	.BASELINE_Q_ADDR          = 0x21818,
	.BASELINE_BTN_ADDR        = 0x24350,
	.BASELINE_BTN_Q_ADDR      = 0x24358,
	.DIFF_PIPE0_ADDR          = 0x209B0,
	.DIFF_PIPE0_Q_ADDR        = 0x20E78,
	.DIFF_PIPE1_ADDR          = 0x239B0,
	.DIFF_PIPE1_Q_ADDR        = 0x23E78,
	.RAW_BTN_PIPE0_ADDR       = 0x20990,
	.RAW_BTN_PIPE0_Q_ADDR     = 0x20998,
	.RAW_BTN_PIPE1_ADDR       = 0x23990,
	.RAW_BTN_PIPE1_Q_ADDR     = 0x23998,
	.DIFF_BTN_PIPE0_ADDR      = 0x21340,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0x21348,
	.DIFF_BTN_PIPE1_ADDR      = 0x24340,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0x24348,
	.READ_FLASH_CHECKSUM_ADDR = 0x24000,
	.RW_FLASH_DATA_ADDR       = 0x24002,
};

static const struct nvt_ts_mem_map NT36676F_memory_map = {
	.EVENT_BUF_ADDR           = 0x11A00,
	.RAW_PIPE0_ADDR           = 0x10000,
	.RAW_PIPE0_Q_ADDR         = 0,
	.RAW_PIPE1_ADDR           = 0x12000,
	.RAW_PIPE1_Q_ADDR         = 0,
	.BASELINE_ADDR            = 0x10B08,
	.BASELINE_Q_ADDR          = 0,
	.BASELINE_BTN_ADDR        = 0x12B08,
	.BASELINE_BTN_Q_ADDR      = 0,
	.DIFF_PIPE0_ADDR          = 0x1064C,
	.DIFF_PIPE0_Q_ADDR        = 0,
	.DIFF_PIPE1_ADDR          = 0x1264C,
	.DIFF_PIPE1_Q_ADDR        = 0,
	.RAW_BTN_PIPE0_ADDR       = 0x10634,
	.RAW_BTN_PIPE0_Q_ADDR     = 0,
	.RAW_BTN_PIPE1_ADDR       = 0x12634,
	.RAW_BTN_PIPE1_Q_ADDR     = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0x10AFC,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0x12AFC,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0,
	.READ_FLASH_CHECKSUM_ADDR = 0x14000,
	.RW_FLASH_DATA_ADDR       = 0x14002,
};

struct nvt_ts_trim_id_table {
	uint8_t id[NVT_ID_BYTE_MAX];
	uint8_t mask[NVT_ID_BYTE_MAX];
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
};

static const struct nvt_ts_trim_id_table trim_id_table[] = {
	{.id = {0x55, 0x00, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
		.mmap = &NT36772_memory_map, .carrier_system = 0},
	{.id = {0x55, 0x72, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
		.mmap = &NT36772_memory_map, .carrier_system = 0},
	{.id = {0xAA, 0x00, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
		.mmap = &NT36772_memory_map, .carrier_system = 0},
	{.id = {0xAA, 0x72, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
		.mmap = &NT36772_memory_map, .carrier_system = 0},
	{.id = {0xFF, 0xFF, 0xFF, 0x72, 0x67, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT36772_memory_map, .carrier_system = 0},
	{.id = {0xFF, 0xFF, 0xFF, 0x70, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT36772_memory_map, .carrier_system = 0},
	{.id = {0xFF, 0xFF, 0xFF, 0x70, 0x67, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT36772_memory_map, .carrier_system = 0},
	{.id = {0xFF, 0xFF, 0xFF, 0x72, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT36772_memory_map, .carrier_system = 0},
	{.id = {0xFF, 0xFF, 0xFF, 0x25, 0x65, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT36525_memory_map, .carrier_system = 0},
	{.id = {0xFF, 0xFF, 0xFF, 0x70, 0x68, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT36870_memory_map, .carrier_system = 1},
	{.id = {0xFF, 0xFF, 0xFF, 0x76, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT36676F_memory_map, .carrier_system = 0}
};

#if WAKEUP_GESTURE
#define WAKEUP_OFF 4
#define WAKEUP_ON 5
#define GESTURE_WORD_C          12
#define GESTURE_WORD_W          13
#define GESTURE_WORD_V          14
#define GESTURE_DOUBLE_CLICK    15
#define GESTURE_WORD_Z          16
#define GESTURE_WORD_M          17
#define GESTURE_WORD_O          18
#define GESTURE_WORD_e          19
#define GESTURE_WORD_S          20
#define GESTURE_SLIDE_UP        21
#define GESTURE_SLIDE_DOWN      22
#define GESTURE_SLIDE_LEFT      23
#define GESTURE_SLIDE_RIGHT     24
#define DATA_PROTOCOL           30
#define FUNCPAGE_GESTURE         1

const uint16_t gesture_key_array[] = {
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
	KEY_WAKEUP,
};

bool enable_gesture_mode = false;
bool delay_gesture = false;
bool suspend_state = false;
static struct wake_lock gestrue_wakelock;

int nvt_gesture_switch(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	if (type == EV_SYN && code == SYN_CONFIG)
	{
		if (suspend_state)
		{
		     if ((value != WAKEUP_OFF) || enable_gesture_mode)
			{
			delay_gesture = true;
			}
		}
		if (value == WAKEUP_OFF){
			enable_gesture_mode = false;
		}else if (value == WAKEUP_ON){
			enable_gesture_mode  = true;
		}
	}
	return 0;
}

void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];

	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE))
		gesture_id = func_id;
	else if (gesture_id > DATA_PROTOCOL)
		return;

	switch (gesture_id) {
		case GESTURE_WORD_C:
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
}
#endif

int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	uint8_t retries;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = &buf[1];

	for (retries = 0; retries < 5; retries++) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			return ret;
	}

	return -EIO;
}

int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	uint8_t retries;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = buf;

	for (retries = 0; retries < 5; retries++) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			return ret;
	}

	return -EIO;
}

void nvt_sw_reset_idle(void)
{
	uint8_t buf[4] = {0};

	buf[0] = 0x00;
	buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(15);
}

void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(35);
}

int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry)
		return -1;
	else
		return 0;
}

int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry)
		return -1;
	else
		return 0;
}

int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	for (;;) {
		msleep(10);

		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if (check_reset_state <= RESET_STATE_REK) {
			if (unlikely(retry > 50)) {
				ret = -1;
				break;
			}
		} else {
			if (unlikely(retry > 100)) {
				ret = -1;
				break;
			}
		}
	}

	return ret;
}

int32_t nvt_read_pid(void)
{
	uint8_t buf[3] = {0};
	int32_t ret = 0;

	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	return ret;
}

int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 17);
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);

	if ((buf[1] + buf[2]) != 0xFF) {
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

		if (retry_count < 3) {
			retry_count++;
			goto info_retry;
		} else
			ret = -1;
	} else
		ret = 0;

	nvt_read_pid();
	return ret;
}

#ifdef CONFIG_OF
static void nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;

	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
}
#else
static void nvt_parse_dt(struct device *dev)
{
	ts->irq_gpio = NVTTOUCH_INT_PIN;
}
#endif

static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret)
			goto err_request_irq_gpio;
	}
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request(ts->reset_gpio, "NVT-reset");
		if (ret)
			goto err_request_irq_gpio;
	}
	return ret;

err_request_irq_gpio:
	return ret;
}

static void nvt_ts_work_func(void)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
	int32_t i = 0;
	int32_t finger_cnt = 0;

	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 1);
	if (unlikely(ret < 0)) {
		return;
	}

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		input_id = (uint8_t) (point_data[1] >> 3);
		nvt_ts_wakeup_gesture_report(input_id, point_data);
		enable_irq(ts->client->irq);
		mutex_unlock(&ts->lock);
		return;
	}
#endif

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t) (point_data[position] >> 3);

		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (likely(((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02))) {
			input_x = (uint32_t) (point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t) (point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TOUCH_FORCE_NUM);
			finger_cnt++;
		}
	}

	for (i = 0; i < ts->max_touch_num; i++) {
		if (likely(press_id[i] != 1)) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
	input_sync(ts->input_dev);
}

static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		wake_lock_timeout(&gestrue_wakelock, msecs_to_jiffies(5000));
	}
#endif

	mutex_lock(&ts->lock);
	nvt_ts_work_func();
	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}

static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	for (retry = 5; retry > 0; retry--) {
		nvt_bootloader_reset();
		nvt_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		msleep(10);

		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF6;
		CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);

		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;
			for (i = 0; i < NVT_ID_BYTE_MAX; i++)
				if (trim_id_table[list].mask[i])
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;

			if (i == NVT_ID_BYTE_MAX)
				found_nvt_chip = 1;

			if (found_nvt_chip) {
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].carrier_system;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}


static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if WAKEUP_GESTURE
	int32_t retry = 0;
#endif
	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		return -ENOMEM;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	nvt_parse_dt(&client->dev);
	
	ret = nvt_gpio_config(ts);
	if (ret) {
		goto err_gpio_config_failed;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	msleep(10);
	ts->vcc_i2c = regulator_get(&client->dev, "vcc_i2c-supply");
	if (IS_ERR(ts->vcc_i2c))
		ret = PTR_ERR(ts->vcc_i2c);

    	if (regulator_count_voltages(ts->vcc_i2c) > 0)
		ret = regulator_set_voltage(ts->vcc_i2c, 1800000, 1800000);

	ret = regulator_enable(ts->vcc_i2c);
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	mutex_init(&ts->lock);
	mutex_lock(&ts->lock);
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	nvt_get_fw_info();
	mutex_unlock(&ts->lock);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;
	ts->int_trigger_type = INT_TRIGGER_TYPE;
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max-1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max-1, 0, 0);
#endif

#if WAKEUP_GESTURE
	ts->input_dev->event = nvt_gesture_switch;
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++)
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	ret = input_register_device(ts->input_dev);
	if (ret)
		goto err_input_register_device_failed;


	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
#if WAKEUP_GESTURE
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_irq_handler, ts->int_trigger_type | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, ts);
#else
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_irq_handler, ts->int_trigger_type | IRQF_ONESHOT, client->name, ts);
#endif
		if (ret != 0)
			goto err_int_request_failed;
		else
			disable_irq(client->irq);
	}

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
#endif
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		goto err_register_fb_notif_failed;

	bTouchIsAwake = 1;
	enable_irq(client->irq);

	return 0;

err_register_fb_notif_failed:
	free_irq(client->irq, ts);
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_chipvertrim_failed:
err_check_functionality_failed:
	gpio_free(ts->irq_gpio);
err_gpio_config_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

static int32_t nvt_ts_remove(struct i2c_client *client)
{
	fb_unregister_client(&ts->fb_notif);
	mutex_destroy(&ts->lock);
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;
}

static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
	uint32_t i = 0;

	if (!bTouchIsAwake)
		return 0;

	mutex_lock(&ts->lock);
	bTouchIsAwake = 0;

#if WAKEUP_GESTURE
	if (enable_gesture_mode){
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		enable_irq_wake(ts->client->irq);

	} else {
#endif
		disable_irq(ts->client->irq);
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#if WAKEUP_GESTURE
	}
#endif

	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);

	msleep(50);
#if WAKEUP_GESTURE
	suspend_state = true;
#endif
	mutex_unlock(&ts->lock);

	return 0;
}

static int32_t nvt_ts_resume(struct device *dev)
{
	if (bTouchIsAwake)
		return 0;

	mutex_lock(&ts->lock);
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_REK);

#if WAKEUP_GESTURE
	if (delay_gesture)
		enable_gesture_mode = !enable_gesture_mode;

	if (!enable_gesture_mode)
		enable_irq(ts->client->irq);

	if (delay_gesture)
		enable_gesture_mode = !enable_gesture_mode;
#else
	enable_irq(ts->client->irq);
#endif

	bTouchIsAwake = 1;
	mutex_unlock(&ts->lock);
#if WAKEUP_GESTURE
	suspend_state = false;
	delay_gesture = false;
#endif
	return 0;
}


static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_POWERDOWN)
			nvt_ts_suspend(&ts->client->dev);
	} else if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			nvt_ts_resume(&ts->client->dev);
	}

	return 0;
}

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts",},
	{ },
};
#endif

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret)
		goto err_driver;

err_driver:
	return ret;
}

static void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif
}


module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
