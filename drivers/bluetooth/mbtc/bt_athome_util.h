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

#ifndef _BT_ATHOME_UTIL_H_
#define _BT_ATHOME_UTIL_H_

#include <linux/kernel.h>

/* unaligned and endian-agnostic access */
static inline uint16_t r16LE(void *ptr)
{
	uint8_t *p = (uint8_t*)ptr;
	return (((uint16_t)p[1]) << 8) | p[0];
}

static inline uint32_t r32LE(void *ptr)
{
	uint8_t *p = (uint8_t*)ptr;
	return  (((uint32_t)p[3]) << 24) | (((uint32_t)p[2]) << 16) |
		(((uint32_t)p[1]) <<  8) | p[0];
}

static inline uint64_t r64LE(void *ptr)
{
	uint8_t *p = (uint8_t*)ptr;
	return  (((uint64_t)p[7]) << 56) | (((uint64_t)p[6]) << 48) |
		(((uint64_t)p[5]) << 40) | (((uint64_t)p[4]) << 32) |
		(((uint64_t)p[3]) << 24) | (((uint64_t)p[2]) << 16) |
		(((uint64_t)p[1]) <<  8) | p[0];
}

static inline void w16LE(void *ptr, uint16_t val)
{
	uint8_t *p = (uint8_t*)ptr;

	p[0] = val;
	p[1] = val >> 8;
}

static inline void w64LE(void *ptr, uint64_t val)
{
	uint8_t *p = (uint8_t*)ptr;

	p[0] = val >>  0;
	p[1] = val >>  8;
	p[2] = val >> 16;
	p[3] = val >> 24;
	p[4] = val >> 32;
	p[5] = val >> 40;
	p[6] = val >> 48;
	p[7] = val >> 56;
}

/* access to packed non-typedefed structures with autoincrement */
static inline uint8_t get8LE(uint8_t **parPP)
{
	uint8_t ret = **parPP;
	(*parPP)++;

	return ret;
}

static inline uint16_t get16LE(uint8_t **parPP)
{
	uint16_t ret = r16LE(*parPP);
	(*parPP) += 2;

	return ret;
}

static inline void put8LE(uint8_t **parPP, uint8_t v)
{
	*(*parPP)++ = v;
}

static inline void put16LE(uint8_t **parPP, uint16_t v)
{
	w16LE(*parPP, v);
	(*parPP) += 2;
}

static inline void put64LE(uint8_t **parPP, uint64_t v)
{
	w64LE(*parPP, v);
	(*parPP) += 8;
}

#endif

