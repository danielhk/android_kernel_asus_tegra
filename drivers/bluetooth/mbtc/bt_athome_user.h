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

#ifndef _BT_ATHOME_USER_H_
#define _BT_ATHOME_USER_H_


#include "bt_athome_proto.h"

struct athome_bt_known_remote {
	struct athome_bt_known_remote *next;
	uint8_t bind_mode;
	uint8_t MAC[AAH_BT_MAC_SZ];
	uint8_t LTK[AAH_BT_LTK_SZ];
};

/*
 *	Given a MAC, see if we heard about such a device from the user.
 */
struct athome_bt_known_remote *athome_bt_find_known(const uint8_t *MAC);

/*
 *	Init the user-facind devnode & structures
 */
int athome_bt_user_init(void);

/*
 *	Deinit user-facing devnode & structures
 */
void athome_bt_user_deinit(void);

/*
 *	Send some data to user through the user-facing devnode.
 */
void athome_bt_usr_enqueue(uint8_t typ, const void *data, unsigned len);

/*
 *	See if the user has asked to place the driver into "bind" mode.
 */
bool athome_bt_usr_in_bind_mode(void);

#endif

