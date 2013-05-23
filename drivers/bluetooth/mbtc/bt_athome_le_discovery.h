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

#ifndef _BT_ATHOME_LE_DISCOVERY_H_
#define _BT_ATHOME_LE_DISCOVERY_H_

#include <linux/kernel.h>


/*
 * Take a look at a discovered event. See if we want to connect to any
 * of the devices listed in it. If so, fill "macP" with the MAC and
 * return true. Else return false;
 */
bool athome_bt_discovered(const uint8_t *buf, uint8_t *macP, int len);

struct athome_bt_adv_manuf_data{

	uint8_t ident[6];
	uint8_t ver[4];
	uint8_t snum[0];
};
#define ATHOME_BT_IDENT	"Google"

#endif

