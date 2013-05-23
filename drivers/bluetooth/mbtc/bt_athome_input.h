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

#ifndef _BT_ATHOME_INPUT_H_
#define _BT_ATHOME_INPUT_H_

#define AAH_BT_KEY_DPAD_CENTER			353
#define AAH_BT_KEY_POWER			177
#define AAH_BT_KEY_INPUT			178


#define ATHOME_MAX_FINGERS			3


int athome_bt_input_init(void);
void athome_bt_input_deinit(void);
void athome_bt_input_send_touch(unsigned which, int pointer_idx,
					uint16_t x, uint16_t y);
void athome_bt_input_send_buttons(unsigned which, uint32_t mask);
void athome_bt_input_frame(unsigned which);


#endif

