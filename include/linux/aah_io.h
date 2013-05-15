/*
 * AAH IO driver.
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef __LINUX_AAH_IO_H
#define __LINUX_AAH_IO_H

#ifdef __KERNEL__

#define LP5521_CLOCK_EXT	0
#define LP5521_CLOCK_INT	1
#define LP5521_CLOCK_AUTO	2

struct aah_io_platform_data {
	int	key_gpio;
	int	key_code;
};

#endif

struct led_rgb_vals {
	u8 rgb[3];
};

#define AAH_IO_LED_MAGIC 0xE3

#define AAH_LED_MODE_POWER_UP_ANIMATION         1
#define AAH_LED_MODE_DIRECT                     2

#define AAH_IO_LED_SET_RGB          _IOW(AAH_IO_LED_MAGIC, 1, struct led_rgb_vals)
#define AAH_IO_LED_GET_MODE         _IOR(AAH_IO_LED_MAGIC, 2, u8)
#define AAH_IO_LED_SET_MODE         _IOW(AAH_IO_LED_MAGIC, 3, u8)

#endif /* __LINUX_AAH_IO_H */
