/*
 *
 * Copyright (C) 2013 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/input.h>
#include <linux/input/mt.h>
#include "bt_athome_input.h"
#include "bt_athome_logging.h"
#include "bt_athome_proto.h"


static struct {
	struct input_dev *idev;
	uint8_t fingers_down;
	char uniq[16];
} inputs[ATHOME_RMT_MAX_CONNS];


/* to see events live: in adb do "getevent -lt /dev/input/event1" */

static unsigned short ikeys[] = {KEY_BACK, KEY_HOMEPAGE, KEY_VOLUMEUP,
					KEY_VOLUMEDOWN, AAH_BT_KEY_DPAD_CENTER,
					AAH_BT_KEY_POWER, AAH_BT_KEY_INPUT};

#define RAW_X_MAX 0xffff
#define RAW_Y_MAX 0xffff

static int athome_bt_input_init_device(struct input_dev **idevP)
{
	int err, i;
	struct input_dev *idev;

	BUG_ON(!idevP);
	*idevP = NULL;

	idev = input_allocate_device();
	if (!idev) {
		aahlog("Failed to allocate touch device\n");
		return -ENOMEM;
	}

	idev->name = "athome_remote";

	/* we support touch buttons */
	set_bit(EV_SYN, idev->evbit);
	set_bit(EV_ABS, idev->evbit);

	/* we support the following buttons */
	set_bit(EV_KEY, idev->evbit);
	for(i = 0; i < ARRAY_SIZE(ikeys); i++)
		__set_bit(ikeys[i], idev->keybit);

	/* key mapping */
	idev->keycode = ikeys;
	idev->keycodesize = sizeof(unsigned short);
	idev->keycodemax = ARRAY_SIZE(ikeys);

	/* touch params */
	input_mt_init_slots(idev, ATHOME_MAX_FINGERS);
	input_set_abs_params(idev, ABS_MT_POSITION_X, 0, RAW_X_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_Y, 0, RAW_Y_MAX, 0, 0);

	/* without misc key capability, volume keys will do nothing */
	input_set_capability(idev, EV_MSC, MSC_SCAN);
	err = input_register_device(idev);
	if (err) {
		aahlog("Failed to register input device\n");
		input_free_device(idev);
		return err;
	}

	*idevP = idev;
	return 0;
}

static void athome_bt_input_del_device(struct input_dev *idev)
{
	input_mt_destroy_slots(idev);
	input_unregister_device(idev);
	input_free_device(idev);
}

int athome_bt_input_init(void)
{
	int err;
	size_t i;

	for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {


		scnprintf(inputs[i].uniq, sizeof(inputs[i].uniq),
						"athome_bt_%d", i);

		err = athome_bt_input_init_device(&inputs[i].idev);
		if (err)
			break;
		inputs[i].idev->uniq = inputs[i].uniq;
		inputs[i].fingers_down = 0;
	}
	if (i == ATHOME_RMT_MAX_CONNS)
		return 0;
	/* we failed */
	while (i) {
		i--;
		athome_bt_input_del_device(inputs[i].idev);
		inputs[i].idev = NULL;
	}
	return err;
}

void athome_bt_input_deinit(void)
{
	int i;

	for(i = 0; i < ATHOME_RMT_MAX_CONNS; i++)
		athome_bt_input_del_device(inputs[i].idev);
}

void athome_bt_input_send_touch(unsigned which, int pointer_idx,
						uint16_t x, uint16_t y)
{
	struct input_dev *idev;
	uint32_t mask = 1UL << pointer_idx;
	bool wasdown, isdown = ((x != RAW_X_MAX) || (y != RAW_Y_MAX));

	BUG_ON(which >= ARRAY_SIZE(inputs));

	idev = inputs[which].idev;
	wasdown = !!(inputs[which].fingers_down & mask);


	if (!isdown && !wasdown)
		return;

	input_mt_slot(idev, pointer_idx);
	input_mt_report_slot_state(idev, MT_TOOL_FINGER, isdown);

	if (isdown) {
		if (LOG_INPUT_EVENTS)
			aahlog_continue(" [%d] raw: ( %5d , %5d)\n",
					pointer_idx, x, y);
		input_report_abs(idev, ABS_MT_POSITION_X, x);
		input_report_abs(idev, ABS_MT_POSITION_Y, y);
		inputs[which].fingers_down |= mask;
	} else { /* was down */

		if (LOG_INPUT_EVENTS)
			aahlog_continue(" [%d] finger release\n", pointer_idx);
		inputs[which].fingers_down &= ~mask;
	}
}

void athome_bt_input_send_buttons(unsigned which, uint32_t mask)
{
	struct input_dev *idev;
	int i;

	BUG_ON(which >= ARRAY_SIZE(inputs));

	idev = inputs[which].idev;

	for (i = 0; i < ARRAY_SIZE(ikeys); i++) {
		input_event(idev, EV_MSC, MSC_SCAN, i);
		input_report_key(idev, ikeys[i], mask & 1);
		mask >>= 1;
	}
}

void athome_bt_input_frame(unsigned which)
{
	BUG_ON(which >= ARRAY_SIZE(inputs));

	input_sync(inputs[which].idev);
}

