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

#ifndef _ATHOME_TRANSPORT_H_
#define _ATHOME_TRANSPORT_H_

#include <linux/types.h>

#include <linux/athome_radio.h>

int __init athome_transport_open(struct athome_platform_data *ptd);
void athome_transport_close(void);

int athome_xfer_tx(const uint8_t *msg, size_t len);
int athome_xfer_rx(uint8_t *buf, size_t len);

#endif

