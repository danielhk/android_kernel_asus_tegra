/*
 * arch/arm/mach-tegra/board-molly-panel.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 * Copyright (c) 2013, Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/ioport.h>
#include <linux/fb.h>
#include <linux/nvmap.h>
#include <linux/nvhost.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>

#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t11.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "board-molly.h"
#include "board-panel.h"
#include "common.h"
#include "tegra11_host1x_devices.h"

struct platform_device * __init molly_host1x_init(void)
{
	struct platform_device *pdev = NULL;

#ifdef CONFIG_TEGRA_GRHOST
	pdev = tegra11_register_host1x_devices();
	if (!pdev) {
		pr_err("host1x devices registration failed\n");
		return NULL;
	}
#endif
	return pdev;
}

#ifdef CONFIG_TEGRA_DC

static struct regulator *molly_hdmi_reg;
static struct regulator *molly_hdmi_pll;
static struct regulator *molly_hdmi_vddio;

static struct resource molly_disp_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by molly_panel_init() */
		.end	= 0, /* Filled in by molly_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static int molly_hdmi_enable(struct device *dev)
{
	int ret;
	if (!molly_hdmi_reg) {
		molly_hdmi_reg = regulator_get(dev, "avdd_hdmi");
		if (IS_ERR_OR_NULL(molly_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			molly_hdmi_reg = NULL;
			return PTR_ERR(molly_hdmi_reg);
		}
	}
	ret = regulator_enable(molly_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!molly_hdmi_pll) {
		molly_hdmi_pll = regulator_get(dev, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(molly_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			molly_hdmi_pll = NULL;
			regulator_put(molly_hdmi_reg);
			molly_hdmi_reg = NULL;
			return PTR_ERR(molly_hdmi_pll);
		}
	}
	ret = regulator_enable(molly_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int molly_hdmi_disable(void)
{
	if (molly_hdmi_reg) {
		regulator_disable(molly_hdmi_reg);
		regulator_put(molly_hdmi_reg);
		molly_hdmi_reg = NULL;
	}

	if (molly_hdmi_pll) {
		regulator_disable(molly_hdmi_pll);
		regulator_put(molly_hdmi_pll);
		molly_hdmi_pll = NULL;
	}
	return 0;
}

static int molly_hdmi_postsuspend(void)
{
	if (molly_hdmi_vddio) {
		regulator_disable(molly_hdmi_vddio);
		regulator_put(molly_hdmi_vddio);
		molly_hdmi_vddio = NULL;
	}
	return 0;
}

static int molly_hdmi_hotplug_init(struct device *dev)
{
#if MOLLY_ON_DALMORE == 1
	if (!molly_hdmi_vddio) {
		molly_hdmi_vddio = regulator_get(dev, "vdd_hdmi_5v0");
		if (WARN_ON(IS_ERR(molly_hdmi_vddio))) {
			pr_err("%s: couldn't get regulator vdd_hdmi_5v0: %ld\n",
				__func__, PTR_ERR(molly_hdmi_vddio));
				molly_hdmi_vddio = NULL;
		} else {
			regulator_enable(molly_hdmi_vddio);
		}
	}
#else
	/* no regulator needed to power the level shifter for
	 * HDMI on molly.  there is a HDMI_EN control GPIO
	 * we need to set to enable the level shifter though.
	 */
#endif

	return 0;
}

static struct tegra_dc_mode hdmi_panel_modes[] = {
	/* 3840x2160p@29.97Hz/30Hz */
	{
		.pclk = 297000000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 88,   /* hsync_len */
		.v_sync_width = 10,   /* vsync_len */
		.h_back_porch = 296,  /* left_margin */
		.v_back_porch = 72,   /* upper_margin */
		.h_active = 3840,     /* xres */
		.v_active = 2160,     /* yres */
		.h_front_porch = 176, /* right_margin */
		.v_front_porch = 8,   /* lower_margin */
		.avi_m = TEGRA_DC_MODE_AVI_M_16_9,
	},
	/* 1920x1080p@59.94Hz/60Hz CEA mode 16 */
	{
		.pclk = 148500000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 44,   /* hsync_len */
		.v_sync_width = 5,    /* vsync_len */
		.h_back_porch = 148,  /* left_margin */
		.v_back_porch = 36,   /* upper_margin */
		.h_active = 1920,     /* xres */
		.v_active = 1080,     /* yres */
		.h_front_porch = 88,  /* right_margin */
		.v_front_porch = 4,   /* lower_margin */
		.avi_m = TEGRA_DC_MODE_AVI_M_16_9,
	},
	/* 1280x720p@59.94Hz/60Hz CEA mode 4 */
	{
		.pclk = 74250000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 40,   /* hsync_len */
		.v_sync_width = 5,    /* vsync_len */
		.h_back_porch = 220,  /* left_margin */
		.v_back_porch = 20,   /* upper_margin */
		.h_active = 1280,     /* xres */
		.v_active = 720,      /* yres */
		.h_front_porch = 110, /* right_margin */
		.v_front_porch = 5,   /* lower_margin */
		.avi_m = TEGRA_DC_MODE_AVI_M_16_9,
	},
	/* 720x480p@59.94Hz/60Hz CEA mode 3 */
	{
		.pclk = 27000000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 62,   /* hsync_len */
		.v_sync_width = 6,    /* vsync_len */
		.h_back_porch = 60,   /* left_margin */
		.v_back_porch = 30,   /* upper_margin */
		.h_active = 720,      /* xres */
		.v_active = 480,      /* yres */
		.h_front_porch = 16,  /* right_margin */
		.v_front_porch = 9,   /* lower_margin */
		.avi_m = TEGRA_DC_MODE_AVI_M_16_9,
	},
};

/* The level shifter's definition of a "low" is higher than what
 * the Tegra's i2c defintion of "low" is.  This causes intermitent failures.
 * So we add pull downs to make sure the Tegra i2c sees lows correctly.
 * This causes increased power draw because the level shifter has internal
 * pull-ups, creating path from 5V to ground through the pulls, so we
 * turn off the pull-down when no HPD.
 */
static void molly_hdmi_hotplug_report(bool state)
{
	if (state) {
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_DDC_SDA,
					    TEGRA_PUPD_PULL_DOWN);
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_DDC_SCL,
					    TEGRA_PUPD_PULL_DOWN);
	} else {
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_DDC_SDA,
					    TEGRA_PUPD_NORMAL);
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_DDC_SCL,
					    TEGRA_PUPD_NORMAL);
	}
}

/* Electrical characteristics for HDMI, all modes must be declared here */
struct tmds_config molly_tmds_config[] = {
	{ /* 480p : 27 MHz and below */
		.pclk = 27000000,
		.pll0 = 0x01003010,
		.pll1 = 0x00301b00,
		.drive_current = 0x23232323,
		.pe_current = 0x00000000,
		.peak_current = 0x00000000,
	},
	{ /* 720p : 74.25MHz modes */
		.pclk = 74250000,
		.pll0 = 0x01003110,
		.pll1 = 0x00301b00,
		.drive_current = 0x25252525,
		.pe_current = 0x00000000,
		.peak_current = 0x03030303,
	},
	{ /* 1080p : 148.5MHz modes */
		.pclk = 148500000,
		.pll0 = 0x01003310,
		.pll1 = 0x00301b00,
		.drive_current = 0x27272727,
		.pe_current = 0x00000000,
		.peak_current = 0x03030303,
	},
	{ /* 4K : 297MHz modes */
		.pclk = INT_MAX,
		.pll0 = 0x01003f10,
		.pll1 = 0x00500f00,
		.drive_current = 0x3D3D3D3D,
		.pe_current = 0x00000000,
		.peak_current = 0x0B0B0B0B,
	},
};

struct tegra_hdmi_out molly_hdmi_out = {
	.tmds_config = molly_tmds_config,
	.n_tmds_config = ARRAY_SIZE(molly_tmds_config),
};

static struct tegra_dc_out molly_disp_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH |
			  TEGRA_DC_OUT_HOTPLUG_WAKE_LP0 |
			  TEGRA_DC_OUT_FILTER_ALLOWED_MODES,
	.parent_clk	= "pll_d_out0",

	.dcc_bus	= 3,
	.hotplug_gpio	= MOLLY_HDMI_HPD,
	.hdmi_out	= &molly_hdmi_out,

	.max_pixclock	= KHZ2PICOS(297000),

	/* defaults until hotplug occurs */
	.modes          = hdmi_panel_modes,
	.n_modes        = ARRAY_SIZE(hdmi_panel_modes),
	.v_refresh_tolerance = 200, /* +/- 0.2Hz in vertical refresh */

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= molly_hdmi_enable,
	.disable	= molly_hdmi_disable,
	.postsuspend	= molly_hdmi_postsuspend,
	.hotplug_init	= molly_hdmi_hotplug_init,
	.hotplug_report = molly_hdmi_hotplug_report,
};

static struct tegra_fb_data molly_disp_fb_data = {
	.win		= 0,
	.xres		= 1920,
	.yres		= 1080,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data molly_disp_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &molly_disp_out,
	.fb		= &molly_disp_fb_data,
	.emc_clk_rate	= 300000000,
#ifdef CONFIG_TEGRA_DC_CMU
	.cmu_enable	= 1,
#endif
};

static struct platform_device molly_disp_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= molly_disp_resources,
	.num_resources	= ARRAY_SIZE(molly_disp_resources),
	.dev = {
		.platform_data = &molly_disp_pdata,
	},
};

static struct nvmap_platform_carveout molly_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		.buddy_size	= 0, /* no buddy allocation for IRAM */
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0, /* Filled in by molly_panel_init() */
		.size		= 0, /* Filled in by molly_panel_init() */
		.buddy_size	= SZ_32K,
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0, /* Filled in by molly_panel_init() */
		.size		= 0, /* Filled in by molly_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data molly_nvmap_data = {
	.carveouts	= molly_carveouts,
	.nr_carveouts	= ARRAY_SIZE(molly_carveouts),
};
static struct platform_device molly_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &molly_nvmap_data,
	},
};

int __init molly_panel_init(void)
{
	int err = 0;
	struct resource __maybe_unused *res;
	struct platform_device *phost1x;

#ifdef CONFIG_TEGRA_NVMAP
	molly_carveouts[1].base = tegra_carveout_start;
	molly_carveouts[1].size = tegra_carveout_size;
	molly_carveouts[2].base = tegra_vpr_start;
	molly_carveouts[2].size = tegra_vpr_size;

	err = platform_device_register(&molly_nvmap_device);
	if (err) {
		pr_err("nvmap device registration failed\n");
		return err;
	}
#endif

	phost1x = molly_host1x_init();
	if (!phost1x) {
		pr_err("host1x devices registration failed\n");
		return -EINVAL;
	}

	gpio_request(MOLLY_HDMI_HPD, "hdmi_hpd");
	gpio_direction_input(MOLLY_HDMI_HPD);

#if MOLLY_ON_DALMORE == 0
	err = gpio_request(MOLLY_HDMI_LS_EN, "hdmi_ls_en");
	pr_info("%s: gpio_request(hdmi_ls_en) returned %d\n",
		__func__, err);
	/* keep level shifter always enabled, otherwise
	 * HPD hotplug detection fails because it's also
	 * coming through the level shifter.
	 */
	err = gpio_direction_output(MOLLY_HDMI_LS_EN, 1);
	pr_info("%s: gpio_direction_output(hdmi_ls_en, 1) returned %d\n",
		__func__, err);
#endif

	/* Copy the bootloader fb to the fb. */
	__tegra_move_framebuffer(&molly_nvmap_device,
		tegra_fb_start, tegra_bootloader_fb_start,
			min(tegra_fb_size, tegra_bootloader_fb_size));

	/* can't use tegra_init_hdmi() because it assumes
	 * fb2 is for hdmi, but we're using fb1
	 */
	molly_disp_resources[2].start = tegra_fb_start;
	molly_disp_resources[2].end = tegra_fb_start + tegra_fb_size - 1;
	molly_disp_device.dev.parent = &phost1x->dev;
	err = platform_device_register(&molly_disp_device);
	if (err)
		return err;

#ifdef CONFIG_TEGRA_NVAVP
	nvavp_device.dev.parent = &phost1x->dev;
	err = platform_device_register(&nvavp_device);
	if (err) {
		pr_err("nvavp device registration failed\n");
		return err;
	}
#endif
	return err;
}
#else
int __init molly_panel_init(void)
{
	if (molly_host1x_init())
		return 0;
	else
		return -EINVAL;
}
#endif
