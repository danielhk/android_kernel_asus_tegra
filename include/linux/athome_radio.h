/*
 * Copyright (c) 2012, Google Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _ATHOME_RADIO_H_
#define _ATHOME_RADIO_H_

#include <linux/ioctl.h>

#define ATHOME_RADIO_MOD_NAME		"athome_radio"
#define ATHOME_RADIO_MOD_LOG_NAME	ATHOME_RADIO_MOD_NAME ": "


/* Module reset control ioctl
 *
 *    Passing non zero ioctl arg puts device in reset
 */
#define ATHOME_RADIO_IOCTL_RESET	_IO(0, 0)

/* Buffers flush control ioctl
 *
 *  executing this command causes driver to discard
 *  all pending TX and RX buffers
 */
#define ATHOME_RADIO_IOCTL_FLUSH	_IO(0, 1)

/* TX drain control ioctl
 *
 *  for device opened in non blocking mode
 *  returns -EAGAIN if TX queue is not empty
 *
 *  for device opened in normal mode
 *  blocks until all TX queue is not empty
 */
#define ATHOME_RADIO_IOCTL_DRAIN_TX	_IO(0, 2)

struct athome_platform_data {
	int gpio_num_irq;
	int gpio_num_rst;
	int gpio_spi_cs;
};

#endif

