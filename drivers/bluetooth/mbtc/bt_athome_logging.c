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

#include <linux/kernel.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include "bt_athome_logging.h"
#include "bt_athome_splitter.h"



void athome_bt_logpacket(char chip, uint32_t type, const u8 *data, u32 sz,
			uint8_t owned_by)
{
	static const char owner[] = {'E', 'L', '?'};
	int j;

	if (LOG_ONLY_OURS && !owned_by)
		return;

	if (type == HCI_SCODATA_PKT)
		return;

	if (!LOG_BT_ACL && type == HCI_ACLDATA_PKT)
		return;

	if (!LOG_BT_EVTS && type == HCI_EVENT_PKT)
		return;

	if (!LOG_BT_CMDS && (type == HCI_COMMAND_PKT || type == PKT_MARVELL))
		return;


	aahlog("%c**BT** %c: %02X", owner[owned_by], chip ? 'C' : 'H', type);

	for (j = 0; j < sz; j++)
		aahlog_continue(" %02X", data[j]);
	aahlog_continue("\n");
}

