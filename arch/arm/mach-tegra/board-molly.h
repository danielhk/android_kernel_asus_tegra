/*
 * arch/arm/mach-tegra/board-molly.h
 *
 * Copyright (c) 2012-2013, NVIDIA Corporation. All rights reserved.
 * Copyright (c) 2013, Google, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_MOLLY_H
#define _MACH_TEGRA_BOARD_MOLLY_H

#define MOLLY_ON_DALMORE 0

#include <mach/gpio.h>
#include <mach/irqs.h>

#if MOLLY_ON_DALMORE == 1
#include <linux/mfd/tps65090.h>
#endif

#include "gpio-names.h"

/* External peripheral act as gpio */
#define PALMAS_TEGRA_GPIO_BASE	TEGRA_NR_GPIOS

#if MOLLY_ON_DALMORE == 1
#define TPS65090_TEGRA_IRQ_BASE TEGRA_NR_IRQS
#define TPS65090_TEGRA_IRQ_END	(TPS65090_TEGRA_IRQ_BASE + TPS65090_NUM_IRQ)
/* External peripheral act as interrupt controller */
/* MAX77663 IRQs */
#define PALMAS_TEGRA_IRQ_BASE   TPS65090_TEGRA_IRQ_END
#define PALMAS_TEGRA_IRQ_END	(PALMAS_TEGRA_IRQ_BASE + PALMAS_NUM_IRQ)
#else
#define PALMAS_TEGRA_IRQ_BASE   TEGRA_NR_IRQS
#define PALMAS_TEGRA_IRQ_END	(PALMAS_TEGRA_IRQ_BASE + PALMAS_NUM_IRQ)
#endif

/* I2C related GPIOs */
#define TEGRA_GPIO_I2C1_SCL		TEGRA_GPIO_PC4
#define TEGRA_GPIO_I2C1_SDA             TEGRA_GPIO_PC5
#define TEGRA_GPIO_I2C2_SCL             TEGRA_GPIO_PT5
#define TEGRA_GPIO_I2C2_SDA             TEGRA_GPIO_PT6
#define TEGRA_GPIO_I2C3_SCL             TEGRA_GPIO_PBB1
#define TEGRA_GPIO_I2C3_SDA             TEGRA_GPIO_PBB2
#define TEGRA_GPIO_I2C4_SCL             TEGRA_GPIO_PV4
#define TEGRA_GPIO_I2C4_SDA             TEGRA_GPIO_PV5
#define TEGRA_GPIO_I2C5_SCL             TEGRA_GPIO_PZ6
#define TEGRA_GPIO_I2C5_SDA             TEGRA_GPIO_PZ7

/* HDMI Hotplug detection pin */
#define MOLLY_HDMI_HPD	 TEGRA_GPIO_PN7
/* HDMI level shifter enable on SPDIF_IN - GPIO_PK6 */
#define MOLLY_HDMI_LS_EN TEGRA_GPIO_PK6

enum molly_rev {
	MOLLY_REV_PROTO1 = 0x0,
	MOLLY_REV_PROTO2 = 0x1,
	MOLLY_REV_EVT1   = 0x2,
	MOLLY_REV_EVT2   = 0x3,
	MOLLY_REV_EVT3   = 0x4,
	MOLLY_REV_EVT4   = 0x5,
	MOLLY_REV_EVT5   = 0x6,
	MOLLY_REV_DVT1   = 0x7,
	MOLLY_REV_DVT2   = 0x8,
	MOLLY_REV_DVT3   = 0x9,
	MOLLY_REV_DVT4   = 0xa,
	MOLLY_REV_DVT5   = 0xb,
	MOLLY_REV_PVT1   = 0xc,
	MOLLY_REV_PVT2   = 0xd,
	MOLLY_REV_PVT3   = 0xe,
	MOLLY_REV_PROD   = 0xf,
	MOLLY_REV_INVALID = 0xFF,
};

extern int molly_hw_rev;

int molly_regulator_init(void);
int molly_suspend_init(void);
int molly_sdhci_init(void);
int molly_pinmux_init(void);
int molly_emc_init(void);
int molly_edp_init(void);
int molly_panel_init(void);
int molly_pmon_init(void);
int molly_soctherm_init(void);

#endif
