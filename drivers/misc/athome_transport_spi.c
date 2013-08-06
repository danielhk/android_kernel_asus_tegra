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

/*  */
#define XFER_TYPE_TX        0x00
#define XFER_TYPE_RX        0x10
#define XFER_TYPE_NONE      0x33

/* we'll have to do manual control of the CS pin, since linux does not let
us keep cs low while we do some work and start another transaction, but that
is exactly what we need here */

static struct spi_device *spi_dev;
static int cs_gpio;

static int __devinit athome_radio_spi_probe(struct spi_device *spi_device)
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

static int _athome_xfer_start(void)
{
	int ret = spi_bus_lock(spi_dev->master);
	gpio_set_value(cs_gpio, 0);
	udelay(10);
	return ret;
}

static int _athome_xfer_stop(void)
{
	gpio_set_value(cs_gpio, 1);
	udelay(60);
	return spi_bus_unlock(spi_dev->master);
}

/*
 *
 */
static int _athome_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
	struct spi_message m;
	struct spi_transfer t = {0};

	t.tx_buf = tx;
	t.rx_buf = rx;
	t.len    = len;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync_locked(spi_dev, &m);
}


/*
 *
 */
int athome_xfer_tx(const uint8_t *msg, size_t len)
{
	int rc, ret;
	uint8_t hdr[3];
	struct spi_message m;
	struct spi_transfer t0 = {0};
	struct spi_transfer t1 = {0};

	if (!spi_dev) {
		pr_err("%s: no spi_dev\n", __func__);
		return -ENODEV;
	}

	/* grab the bus and assert CS */
	rc = _athome_xfer_start();
	if (rc)
		return rc;

	/* Tx SPI message consists of header followed up by message body */
	hdr[0] = XFER_TYPE_TX;
	hdr[1] = (uint8_t)(len >> 8);  /* network byte order */
	hdr[2] = (uint8_t)(len);

	t0.tx_buf = hdr;
	t0.rx_buf = NULL;
	t0.len    = sizeof(hdr);

	t1.tx_buf = msg;
	t1.rx_buf = NULL;
	t1.len    = len;

	spi_message_init(&m);
	spi_message_add_tail(&t0, &m);
	spi_message_add_tail(&t1, &m);

	ret = spi_sync_locked(spi_dev, &m);

	/* Release the bus, deassert CS */
	rc =_athome_xfer_stop();
	if (rc)
		return rc;

	return ret;
}

/*
 *
 */
int athome_xfer_rx(uint8_t *buf, size_t buf_len)
{
	int rc, ret;
	uint8_t hdr[3];
	size_t msg_len;

	if (!spi_dev) {
		pr_err("%s: no spi_dev\n", __func__);
		return -ENODEV;
	}

	/* grab the bus and assert CS */
	rc = _athome_xfer_start();
	if (rc)
		return rc;

	/* read header */
	hdr[0] = XFER_TYPE_RX;
	hdr[1] = 0;
	hdr[2] = 0;

	rc = _athome_xfer(hdr, hdr, 3);
	if (rc) {
		ret = rc;
		goto done;
	}

	/* parse header */
	if (likely(hdr[0] == XFER_TYPE_RX)) { /* a valid RX msg */
		msg_len = (((uint16_t) hdr[1]) << 8) | hdr[2];
		if (likely(msg_len <= buf_len)) {
			/* read message */
			ret = msg_len;
			rc = _athome_xfer(NULL, buf, msg_len);
		} else { /* message is bigger then buffer */
			/* read maximum length and discard it */
			ret = 0;
			rc = _athome_xfer(NULL, buf, buf_len);
		}
	} else { /* not an valid RX msg */
		if (hdr[0] == XFER_TYPE_NONE) { /* looks like no msg was queued */
			/* just drop it */
			rc = ret = 0;
		} else { /* malformatted message */
			/* read maximum length and discard it */
			ret = 0;
			rc = _athome_xfer(NULL, buf, buf_len);
		}
	}
	if (rc)
		ret = rc;

done:

	/* Release the bus, deassert CS */
	rc =_athome_xfer_stop();
	if (rc)
		return rc;

	return ret;
}






