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
#include <linux/debugfs.h>
#include <linux/input.h>
#include "bt_athome_input.h"
#include "bt_athome_le_stack.h"
#include "bt_athome_logging.h"

static uint64_t start;
static uint8_t curr_key;
static uint16_t x_0, y_0;
static bool act_analog;
static bool sent_swipe;

static uint16_t swipe_width = 20000;
#define ANALOG_WAIT_NS		(500 * NSEC_PER_MSEC)
#define DPAD_CENTER			32000

int aahbt_input_dpad_init(struct dentry *debugfs_dir) {
	struct dentry *entry;

	entry = debugfs_create_u16("swipe_width", 0600, debugfs_dir,
			&swipe_width);
	if (!entry) {
		aahlog("Failed to create debugfs file for swipe width.\n");
		return -ENODEV;
	}

	return 0;
}

static void send_gesture_key(struct input_dev *idev, unsigned int which,
		int code, int value)
{
	input_report_key(idev, code, value);
	input_sync(idev);
}

static void recenter_origin(int32_t *_x_dist, int32_t *_y_dist) {
	int32_t diff;
	int32_t x_dist;
	int32_t y_dist;

	BUG_ON(!_x_dist);
	BUG_ON(!_y_dist);

	x_dist = *_x_dist;
	y_dist = *_y_dist;

	/* If the distance between our center point is more than half the width of
	 * the panel then we shift the center to facilitate changing direction
	 * after starting from a swipe somewhere off-center
	 */
	if (x_dist > DPAD_CENTER) {
		diff = x_dist - DPAD_CENTER;
		x_dist -= diff;
		x_0 += diff;
	} else if (x_dist < -DPAD_CENTER) {
		diff = x_dist + DPAD_CENTER;
		x_dist -= diff;
		x_0 += diff;
	}

	if (y_dist > DPAD_CENTER) {
		diff = y_dist - DPAD_CENTER;
		y_dist -= diff;
		y_0 += diff;
	} else if (y_dist < -DPAD_CENTER) {
		diff = y_dist + DPAD_CENTER;
		y_dist -= diff;
		y_0 += diff;
	}

	*_x_dist = x_dist;
	*_y_dist = y_dist;
}

/* Updates the global curr_key and returns the key we need to release
 */
static uint8_t determine_gesture_direction(int32_t x_dist, int32_t y_dist)
{
	uint8_t next_key = 0;
	uint8_t old_key = curr_key;
	bool is_vert = false;

	if (abs(y_dist) > swipe_width || abs(x_dist) > swipe_width) {
		if (abs(y_dist) > abs(x_dist))
			is_vert = true;

		if (is_vert) {
			if (y_dist < 0)
				next_key = KEY_UP;
			else
				next_key = KEY_DOWN;
		} else {
			if (x_dist < 0)
				next_key = KEY_LEFT;
			else
				next_key = KEY_RIGHT;
		}
	}

	if (next_key) {
		bool change_key = false;

		if (!curr_key) {
			change_key = true;
		} else {
			/* We want to lock the user to the X or Y axis once they've
			 * swiped so they do not accidentally move around without meaning to
			 */
			if ((curr_key == KEY_RIGHT || curr_key == KEY_LEFT) &&
					(next_key == KEY_RIGHT || next_key == KEY_LEFT))
				change_key = true;

			if ((curr_key == KEY_UP || curr_key == KEY_DOWN) &&
					(next_key == KEY_UP || next_key == KEY_DOWN))
				change_key = true;
		}

		if (change_key) {
			curr_key = next_key;
			return old_key;
		}
	}

	return 0;
}

void aahbt_input_handle_dpad_down(struct input_dev *idev, unsigned int which,
		bool is_down, bool was_down, uint16_t x, uint16_t y)
{
	uint64_t now = aahbt_get_time();
	int32_t x_dist, y_dist;
	uint8_t old_key;

	/* We shouldn't be called if the finger isn't down, but handle it properly
	 * just in case
	 */
	if (!is_down)
		return;

	/* If we had no finger down before but have one now then set up initial
	 * parameters so we can watch for a swipe
	 */
	if (!was_down) {
		start = now;
		x_0 = x;
		y_0 = y;
		return;
	}

	x_dist = x - x_0;
	y_dist = y - y_0;

	if (!sent_swipe) {
		determine_gesture_direction(x_dist, y_dist);
		if (curr_key) {
			send_gesture_key(idev, which, curr_key, AAH_KEY_DOWN);
			sent_swipe = true;
		}
	} else if (!act_analog && now - start > ANALOG_WAIT_NS) {
		act_analog = true;
	} else if (act_analog) {
		/* Adjust the center point */
		recenter_origin(&x_dist, &y_dist);
		old_key = determine_gesture_direction(x_dist, y_dist);
		/* If we've changed keys then we need to make an up -> down transition
		 * TODO cja 3/3/2014: Implement analog sensitivity once we have the
		 * framework changes in place to disable framework auto-repeat
		 */
		if (old_key && old_key != curr_key) {
			send_gesture_key(idev, which, old_key, AAH_KEY_UP);
			send_gesture_key(idev, which, curr_key, AAH_KEY_DOWN);
		}

	}
}

void aahbt_input_handle_dpad_up(struct input_dev *idev, unsigned which)
{
	if (curr_key) {
		if (sent_swipe)
			send_gesture_key(idev, which, curr_key, AAH_KEY_UP);

		curr_key = 0;
		x_0 = 0;
		y_0 = 0;
		start = 0;
		act_analog = false;
		sent_swipe = false;
	}
}
