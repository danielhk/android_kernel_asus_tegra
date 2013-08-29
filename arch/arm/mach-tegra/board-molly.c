/*
 * arch/arm/mach-tegra/board-molly.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2013, Google, Inc.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_data/tegra_xusb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/moduleparam.h>
#include <linux/spi-tegra.h>
#include <linux/rfkill-gpio.h>
#include <linux/skbuff.h>
#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
#include <linux/edp.h>

#include <asm/hardware/gic.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t11.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/gpio-tegra.h>
#include <mach/tegra_fiq_debugger.h>
#include <linux/aah_io.h>
#include <linux/athome_radio.h>
#include <linux/nct1008.h>
#include <mach/hardware.h>
#include <mach/thermal.h>

#include "board.h"
#include "board-common.h"
#include "clock.h"
#include "board-molly.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "pm-irq.h"
#include "common.h"

int molly_hw_rev;
module_param(molly_hw_rev, int, S_IRUGO);
MODULE_PARM_DESC(molly_hw_rev, "hardware revision");

static const char const *molly_hw_name[] = {
    [MOLLY_REV_PROTO1] = "Molly PROTO1",
    [MOLLY_REV_PROTO2] = "Molly PROTO2",
    [MOLLY_REV_EVT1]   = "Molly EVT1",
    [MOLLY_REV_EVT2]   = "Molly EVT2",
    [MOLLY_REV_DVT1]   = "Molly DVT1",
    [MOLLY_REV_DVT2]   = "Molly DVT2",
    [MOLLY_REV_PVT]    = "Molly PVT",
    [MOLLY_REV_PROD]   = "Molly PROD",
};

static const char *molly_hw_rev_name(void)
{
	int num = ARRAY_SIZE(molly_hw_name);

	if (molly_hw_rev >= num ||
	    !molly_hw_name[molly_hw_rev])
		return "Molly unknown version";

	return molly_hw_name[molly_hw_rev];
}

static void __init molly_init_hw_rev(void)
{
	pr_info("Molly HW revision: %02x (%s)\n",
		molly_hw_rev, molly_hw_rev_name());
}

static __initdata struct tegra_clk_init_table molly_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	/* Setting vi_sensor-clk to true for validation purpose, will imapact
	 * power, later set to be false.*/
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data molly_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C1_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C1_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data molly_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_I2C2_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C2_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data molly_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C3_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C3_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data molly_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 10000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C4_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C4_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data molly_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C5_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C5_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

/******************************************************************************
 *                                                                            *
 *           aah_io driver platform data                                      *
 *                                                                            *
 ******************************************************************************/
static struct aah_io_platform_data aah_io_data = {
#if MOLLY_ON_DALMORE == 0
	.key_gpio = TEGRA_GPIO_PQ5, /* molly's UI_SWITCH, KB_COL5/GPIO_PQ5 */
#else
	.key_gpio = TEGRA_GPIO_PR2, /* dalmore's volume+ button for now */
#endif
	.key_code = KEY_MUTE,/* TBD, easiest to test as mute for now */
};

static struct i2c_board_info __initdata aah_io_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("aah-io", 0x32),
		.platform_data = &aah_io_data,
	},
};

/******************************************************************************
 *                                                                            *
 *           temp sensor                                                      *
 *                                                                            *
 ******************************************************************************/

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init molly_throttle_init(void)
{
	pr_info("%s:\n", __func__);
	balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(molly_throttle_init);

static struct nct1008_platform_data nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.shutdown_ext_limit = 105, /* C */
	.shutdown_local_limit = 120, /* C */

	.num_trips = 1,
	.trips = {
		{
			.cdev_type = "suspend_soctherm",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 5000,
		},
	},
};

/* Our real part is TI tmp451, which is a derivative
 * and software compatible with nct1008
 */
static struct i2c_board_info __initdata nct1008_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &nct1008_pdata,
		.irq  = -1,
	},
};

#if MOLLY_ON_DALMORE == 1
#include "tegra-board-id.h"
#else
#define TEMP_ALERT_GPIO TEGRA_GPIO_PJ0
#endif

static void __init temp_sensor_init(void)
{
	int nct1008_port;
	int ret = 0;

#if MOLLY_ON_DALMORE == 1
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	if (board_info.board_id == BOARD_E1611) {
		if (board_info.fab == 0x04)
			nct1008_port = TEGRA_GPIO_PO4;
		else
			nct1008_port = TEGRA_GPIO_PX6;
	} else {
		nct1008_port = TEGRA_GPIO_PX6;
		pr_err("Warning: nct alert_port assumed TEGRA_GPIO_PX6" \
			" for unknown dalmore board id E%d\n",
			board_info.board_id);
	}
#else
	nct1008_port = TEGRA_GPIO_PJ0;
#endif

	tegra_add_cdev_trips(nct1008_pdata.trips, &nct1008_pdata.num_trips);

	nct1008_i2c_board_info[0].irq = gpio_to_irq(nct1008_port);
	pr_info("%s: nct1008 irq %d", __func__, nct1008_i2c_board_info[0].irq);

	ret = gpio_request(nct1008_port, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request() for nct1008_port %d failed\n",
		       __func__, nct1008_port);
		return;
	}

	ret = gpio_direction_input(nct1008_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct1008_port)", __func__);
		gpio_free(nct1008_port);
		return;
	}

#if MOLLY_ON_DALMORE == 1
	/* dalmore has thermal sensor on GEN1-I2C, i.e. instance 0 */
	i2c_register_board_info(0, nct1008_i2c_board_info,
				ARRAY_SIZE(nct1008_i2c_board_info));
#else
	/* molly thermal sensor on I2C3/CAM_I2C, i.e. instance 2 */
	i2c_register_board_info(2, nct1008_i2c_board_info,
				ARRAY_SIZE(nct1008_i2c_board_info));
#endif
}

static void __init molly_i2c_init(void)
{
	/* Tegra4 has five i2c controllers:
	 * I2C_1 is called GEN1_I2C in pinmux/schematics
	 * I2C_2 is called GEN2_I2C in pinmux/schematics
	 * I2C_3 is called CAM_I2C in pinmux/schematics
	 * I2C_4 is called DDC_I2C in pinmux/schematics
	 * I2C_5/PMU is called PWR_I2C in pinmux/schematics
	 *
	 * I2C1/GEN1 is for INA3221 current and bus voltage monitor
	 * I2C2/GEN2 is for LED
	 * I2C3/CAM is for TMP451 (nct1008 temp sensor for
	 *  dalmore is on I2C1)
	 * I2C4 is for HDMI/DDC
	 * I2C5/PWR is for PMIC TPS65913B2B5
	 */

	tegra11_i2c_device1.dev.platform_data = &molly_i2c1_platform_data;
	tegra11_i2c_device2.dev.platform_data = &molly_i2c2_platform_data;
	tegra11_i2c_device3.dev.platform_data = &molly_i2c3_platform_data;
	tegra11_i2c_device4.dev.platform_data = &molly_i2c4_platform_data;
	tegra11_i2c_device5.dev.platform_data = &molly_i2c5_platform_data;

	platform_device_register(&tegra11_i2c_device5);
	platform_device_register(&tegra11_i2c_device4);
	platform_device_register(&tegra11_i2c_device3);
	platform_device_register(&tegra11_i2c_device2);
	platform_device_register(&tegra11_i2c_device1);

	i2c_register_board_info(1, aah_io_i2c_board_info,
				ARRAY_SIZE(aah_io_i2c_board_info));

	temp_sensor_init();
}

static struct platform_device *molly_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartd_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data molly_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = uart_console_debug_init(3);
	if (debug_port_id < 0)
		return;

	molly_uart_devices[1] = uart_console_debug_device;
}

static void __init molly_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	molly_uart_pdata.parent_clk_list = uart_parent_clk;
	molly_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarta_device.dev.platform_data = &molly_uart_pdata;
	tegra_uartd_device.dev.platform_data = &molly_uart_pdata;

	uart_debug_init();

	platform_add_devices(molly_uart_devices,
				ARRAY_SIZE(molly_uart_devices));
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct platform_device *molly_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra11_se_device,
#endif
	&tegra_hda_device,
#if defined(CONFIG_TEGRA_CEC_SUPPORT)
	&tegra_cec_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.unaligned_dma_buf_supported = false,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static void __init molly_wake_sources_init(void)
{
	/* Set HDMI HPD GPIO as wakeup source */
	tegra_set_wake_gpio(4, MOLLY_HDMI_HPD);
}

static void __init molly_usb_init(void)
{
	/* Set USB wake sources for molly */
	tegra_set_usb_wake_source();
	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
}

static struct tegra_xusb_pad_data xusb_padctl_data = {
	.pad_mux = (0x1 << 2),
	.port_cap = (0x1 << 4),
	.snps_oc_map = (0x1fc << 0),
	.usb2_oc_map = (0x2f << 0),
	.ss_port_map = (0x1 << 0),
	.oc_det = (0x2c << 10),
	.rx_wander = (0xf << 4),
	.rx_eq = (0x3070 << 8),
	.cdr_cntl = (0x26 << 24),
	.dfe_cntl = 0x002008EE,
	.hs_slew = (0xE << 6),
	.ls_rslew = (0x3 << 14),
	.otg_pad0_ctl0 = (0x7 << 19),
	.otg_pad1_ctl0 = (0x0 << 19),
	.otg_pad0_ctl1 = (0x0 << 0),
	.otg_pad1_ctl1 = (0x0 << 0),
	.hs_disc_lvl = (0x5 << 2),
	.hsic_pad0_ctl0 = (0x00 << 8),
	.hsic_pad0_ctl1 = (0x00 << 8),

	/* XUSB (USB 3.0) is using UTMI2 phy */
	.portmap = TEGRA_XUSB_USB2_P1,
};

static void __init molly_xusb_init(void)
{
	u32 usb_calib0 = tegra_fuse_readl(FUSE_SKU_USB_CALIB_0);

	pr_info("molly_xusb_init: usb_calib0 = 0x%08x\n", usb_calib0);
	/*
	 * read from usb_calib0 and pass to driver
	 * set HS_CURR_LEVEL (PAD0)	= usb_calib0[5:0]
	 * set TERM_RANGE_ADJ		= usb_calib0[10:7]
	 * set HS_SQUELCH_LEVEL		= usb_calib0[12:11]
	 * set HS_IREF_CAP		= usb_calib0[14:13]
	 * set HS_CURR_LEVEL (PAD1)	= usb_calib0[20:15]
	 */

	xusb_padctl_data.hs_curr_level_pad0 = (usb_calib0 >> 0) & 0x3f;
	xusb_padctl_data.hs_term_range_adj = (usb_calib0 >> 7) & 0xf;
	xusb_padctl_data.hs_squelch_level = (usb_calib0 >> 11) & 0x3;
	xusb_padctl_data.hs_iref_cap = (usb_calib0 >> 13) & 0x3;
	xusb_padctl_data.hs_curr_level_pad1 = (usb_calib0 >> 15) & 0x3f;

	tegra_xhci_device.dev.platform_data = &xusb_padctl_data;
	platform_device_register(&tegra_xhci_device);
}

/* SMSC LAN9730 ethernet controller.
 * Initially reset is asserted.
 * TODO: How to use the phy_int_n signal?  SMSC driver doesn't take
 * platform data.  Maybe just hook up here as a irq for wake?
 */
#define GPIO_ETHERNET_PHY_INT_N     TEGRA_GPIO_PR4  /* KB_ROW4/GPIO_PR4 */
#define GPIO_ETHERNET_PHY_INT_N_3V3 TEGRA_GPIO_PEE4 /* SDMMC3_CLK_LB_OUT/GPIO_PEE4 */
#define GPIO_ETHERNET_RESET_N       TEGRA_GPIO_PEE5 /* SDMMC3_CLK_LB_IN/GPIO_PEE5 */

static struct gpio ethernet_gpios[] __initdata = {
	{GPIO_ETHERNET_PHY_INT_N, GPIOF_IN,           "ethernet_phy_int_n" },
	{GPIO_ETHERNET_PHY_INT_N_3V3, GPIOF_IN,       "ethernet_phy_int_n_3v3" },
	{GPIO_ETHERNET_RESET_N,   GPIOF_OUT_INIT_LOW, "ethernet_reset_n" },
};

static struct gpio ethernet_gpios_dvt1[] __initdata = {
	{GPIO_ETHERNET_PHY_INT_N_3V3, GPIOF_IN,       "ethernet_phy_int_n" },
	{GPIO_ETHERNET_RESET_N,   GPIOF_OUT_INIT_LOW, "ethernet_reset_n" },
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = false,
	},
};

static void __init molly_hsic_init(void)
{
	int ret;

	switch (molly_hw_rev) {
	case MOLLY_REV_PROTO1:
	case MOLLY_REV_PROTO2:
	case MOLLY_REV_EVT1:
	case MOLLY_REV_DVT1:
		ret = gpio_request_array(ethernet_gpios_dvt1,
				ARRAY_SIZE(ethernet_gpios_dvt1));
		break;
	default:
		ret = gpio_request_array(ethernet_gpios,
				ARRAY_SIZE(ethernet_gpios));
	};

	if (ret) {
		pr_warn("%s:gpio request failed\n", __func__);
		return;
	}

	/* delay after reset asserted by gpio_request_array() */
	udelay(100);

	/* take out of reset */
	gpio_set_value(GPIO_ETHERNET_RESET_N, 1);

	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_hsic_pdata;
	platform_device_register(&tegra_ehci2_device);
}

#define ATHOME_RADIO_INT_GPIO     TEGRA_GPIO_PS2 /* KB_ROW10/GPIO_PS2 */
#define ATHOME_RADIO_INT_3V3_GPIO TEGRA_GPIO_PB4 /* SDMMC3_DAT3/GPIO_PB4 */
#define ATHOME_RADIO_RESET_N_GPIO TEGRA_GPIO_PB5 /* SDMMC3_DAT2/GPIO_PB5 */
#define ATHOME_RADIO_SPI_CS_GPIO  TEGRA_GPIO_PA7 /* SDMMC3_CMD/GPIO_PA7 */

static struct athome_platform_data radio_pdata = {
	.gpio_num_irq = ATHOME_RADIO_INT_GPIO,
	.gpio_num_rst = ATHOME_RADIO_RESET_N_GPIO,
	.gpio_spi_cs  = ATHOME_RADIO_SPI_CS_GPIO,
};

static struct athome_platform_data radio_pdata_dvt1 = {
	.gpio_num_irq = ATHOME_RADIO_INT_3V3_GPIO,
	.gpio_num_rst = ATHOME_RADIO_RESET_N_GPIO,
	.gpio_spi_cs  = ATHOME_RADIO_SPI_CS_GPIO,
};

#define ATHOME_RADIO_SPI_BUS_NUM 2 /* bus 2 == spi3 */
#define ATHOME_RADIO_SPI_CS      0
/* 2MHZ is max for sim3 right now.  Need to verify
 * clock values available to SPI for Tegra.
 * Depends on clks (dalmore pll_p is 408MHz and clk_m is 12MHz)
 * and dividers available.
 * 1.5MHz was setting we used in wolfie.
 */
#define ATHOME_RADIO_SPI_MAX_HZ  1500000

static struct spi_board_info molly_radio_spi_info[] __initdata = {
	{
		.modalias	= ATHOME_RADIO_MOD_NAME,
		.platform_data  = &radio_pdata,
		.irq		= -1,
		.max_speed_hz   = ATHOME_RADIO_SPI_MAX_HZ,
		.bus_num	= ATHOME_RADIO_SPI_BUS_NUM,
		.chip_select	= ATHOME_RADIO_SPI_CS,
		.mode           = SPI_MODE_0,
	},
};

static void __init molly_radio_init(void)
{
	switch (molly_hw_rev) {
	case MOLLY_REV_PROTO1:
	case MOLLY_REV_PROTO2:
	case MOLLY_REV_EVT1:
	case MOLLY_REV_DVT1:
		molly_radio_spi_info[0].platform_data = &radio_pdata_dvt1;
	};

	spi_register_board_info(molly_radio_spi_info,
				ARRAY_SIZE(molly_radio_spi_info));
}

static struct platform_device *molly_spi_devices[] __initdata = {
	&tegra11_spi_device3,
};

struct spi_clk_parent spi_parent_clk_molly[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data molly_spi_pdata = {
	.max_dma_buffer         = 16 * 1024,
	.is_clkon_always        = false,
	.max_rate               = 25000000,
	.is_dma_based           = true,
};

static void __init molly_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk_molly); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk_molly[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
			       spi_parent_clk_molly[i].name);
			continue;
		}
		spi_parent_clk_molly[i].parent_clk = c;
		spi_parent_clk_molly[i].fixed_clk_rate = clk_get_rate(c);
		pr_info("%s: clock %s, rate %lu\n",
			__func__, spi_parent_clk_molly[i].name,
			spi_parent_clk_molly[i].fixed_clk_rate);
	}
	molly_spi_pdata.parent_clk_list = spi_parent_clk_molly;
	molly_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk_molly);
	tegra11_spi_device3.dev.platform_data = &molly_spi_pdata;
	platform_add_devices(molly_spi_devices,
			     ARRAY_SIZE(molly_spi_devices));
}

static void __init tegra_molly_init(void)
{
	molly_init_hw_rev();
	tegra_clk_init_from_table(molly_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("molly");
	tegra_enable_pinmux();
	molly_pinmux_init();
	molly_i2c_init();
	molly_spi_init();
	molly_radio_init();
	molly_wake_sources_init();
	molly_usb_init();
	molly_xusb_init();
	molly_hsic_init();
	molly_uart_init();
	platform_add_devices(molly_devices, ARRAY_SIZE(molly_devices));
	tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	molly_regulator_init();
	molly_sdhci_init();
	molly_suspend_init();
	molly_emc_init();
	molly_edp_init();
	molly_panel_init();
	molly_pmon_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_UARTD, NULL, -1, -1);
	molly_soctherm_init();
	tegra_register_fuse();
}

static void __init molly_ramconsole_reserve(unsigned long size)
{
	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_molly_dt_init(void)
{
#ifdef CONFIG_USE_OF
	of_platform_populate(NULL,
		of_default_bus_match_table, NULL, NULL);
#endif

	tegra_molly_init();
}

static void __init tegra_molly_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 1920*1080*4*2 = 16588800 bytes, or 15.8203125MB
	 * 3840*2160*4*2 = 66355200 bytes, or 63.28125MB
	 * We don't run fb at 4K mode.  Movie playback
	 * doesn't use it.
	 */
	tegra_reserve(0, /* carveout */
		      SZ_16M, /* fb_size */
		      0); /* fb2_size: 0, not used */
#else
	tegra_reserve(SZ_128M, SZ_16M, 0);
#endif
	molly_ramconsole_reserve(SZ_1M);
}

static const char * const molly_dt_board_compat[] = {
	"nvidia,dalmore", /*
			   * allows us to work with dalmore
			   * bootloader for now
			   */
	"google,molly",
	NULL
};

MACHINE_START(MOLLY, "molly")
	.atag_offset	= 0x100,
	.soc		= &tegra_soc_desc,
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_molly_reserve,
	.init_early	= tegra11x_init_early,
	.init_irq	= tegra_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra_molly_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= molly_dt_board_compat,
MACHINE_END
