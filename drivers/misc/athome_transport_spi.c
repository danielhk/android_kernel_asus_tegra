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

#define pr_fmt(fmt) ATHOME_RADIO_MOD_LOG_NAME "-spi: " fmt
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include "athome_transport.h"
#include <linux/athome_radio.h>


/* we'll have to do manual control of the CS pin, since linux does not let
us keep cs low while we do some work and start another transaction, but that
is exactly what we need here */

static struct spi_device *spi_dev;
static int cs_gpio;

static int athome_radio_spi_probe(struct spi_device *spi_device)
{
	spi_dev = spi_device;
	return 0;
}

static int athome_radio_spi_remove(struct spi_device *spi_device)
{
	spi_dev = NULL;
	return 0;
}

static struct spi_driver athome_radio_spi_driver = {
	.driver = {
		.name = ATHOME_RADIO_MOD_NAME,
		.owner = THIS_MODULE,
	},
	.probe = athome_radio_spi_probe,
	.remove = athome_radio_spi_remove,
};


int __init athome_transport_open(struct athome_platform_data *pdata)
{
	int ret;

	cs_gpio = pdata->gpio_spi_cs;

	ret = gpio_request(cs_gpio, ATHOME_RADIO_MOD_NAME " cs");
	if (ret) {
		pr_alert("Failed gpio_request err %d\n", ret);
		return ret;
	}
	gpio_direction_output(cs_gpio, 1);

	ret = spi_register_driver(&athome_radio_spi_driver);
	if (ret < 0) {
		pr_alert("Failed to register driver with err %d\n", ret);
		gpio_free(cs_gpio);
	}

	return ret;
}

void athome_transport_close(void)
{
	spi_unregister_driver(&athome_radio_spi_driver);
	gpio_free(cs_gpio);
}

int athome_transport_start(void)
{
	int ret = spi_bus_lock(spi_dev->master);
	gpio_set_value(cs_gpio, 0);
	udelay(10);

	return ret;
}

static int athome_transport_op(const unsigned char *out,
			       unsigned char *in, unsigned len)
{
	struct spi_message m;
	struct spi_transfer t = {0};

	t.tx_buf = out;
	t.rx_buf = in;
	t.len    = len;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync_locked(spi_dev, &m);
}

int athome_transport_send(const unsigned char *buf, unsigned len)
{
	return athome_transport_op(buf, NULL, len);
}

int athome_transport_recv(unsigned char *buf, unsigned len)
{
	return athome_transport_op(NULL, buf, len);
}

int athome_transport_stop(void)
{
	gpio_set_value(cs_gpio, 1);
	udelay(50);
	return spi_bus_unlock(spi_dev->master);
}






