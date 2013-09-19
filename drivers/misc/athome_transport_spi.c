/*
 * Copyright (c) 2013, Google Inc. All rights reserved.
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
#include <linux/athome_radio.h>

/*  */
#define XFER_TYPE_TX        0x00
#define XFER_TYPE_RX        0x10
#define XFER_TYPE_NONE      0x33

/* we'll have to do manual control of the CS pin, since linux does not let
us keep cs low while we do some work and start another transaction, but that
is exactly what we need here */

static int athome_xfer_tx(struct device *dev, const uint8_t *msg, size_t len);
static int athome_xfer_rx(struct device *dev, uint8_t *buf, size_t buf_len);

static struct athome_transport_ops xfer_ops = {
	.xfer_tx = athome_xfer_tx,
	.xfer_rx = athome_xfer_rx,
};


static int __devinit athome_radio_spi_probe(struct spi_device *spi)
{
	int rc;
	struct athome_platform_data *pd;

	if (!spi) {
		pr_err("%s: no spi_dev\n", __func__);
		return -ENODEV;
	}

	pd = dev_get_platdata(&spi->dev);
	if (!pd) {
		dev_alert(&spi->dev, "no platform data.\n");
		return -ENODEV;
	}

	/* request CS gpio */
	rc = gpio_request(pd->gpio_spi_cs, ATHOME_RADIO_MOD_NAME " cs");
	if (rc) {
		dev_alert(&spi->dev, "failed (%d) to request cs gpio (%d)\n",
		          rc, pd->gpio_spi_cs);
		return rc;
	}
	gpio_direction_output(pd->gpio_spi_cs, 1);

	rc = athome_radio_init (&spi->dev, pd, &xfer_ops);
	if (rc) {
		dev_alert(&spi->dev, "failed (%d) to init driver core\n", rc);
		gpio_free(pd->gpio_spi_cs);
	}

	return rc;
}

static int __exit athome_radio_spi_remove(struct spi_device *spi)
{
	struct athome_platform_data *pd = dev_get_platdata (&spi->dev);
	athome_radio_destroy (&spi->dev);
	gpio_free(pd->gpio_spi_cs);
	return 0;
}

int athome_radio_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
    return athome_radio_suspend(&spi->dev, mesg);
}

int athome_radio_spi_resume(struct spi_device *spi)
{
    return athome_radio_resume(&spi->dev);
}

static struct spi_driver athome_radio_spi_driver = {
	.driver = {
		.name = ATHOME_RADIO_MOD_NAME,
		.owner = THIS_MODULE,
	},
	.probe = athome_radio_spi_probe,
	.remove = athome_radio_spi_remove,
	.suspend = athome_radio_spi_suspend,
	.resume = athome_radio_spi_resume,
};

#define MIN_CS_TO_XFER_DELAY     10 /* min time between CS asserted and XFER */
#define MIN_XFER_TO_XFER_DELAY   60 /* min time between sequential XFERs */

static int _athome_xfer_start(struct spi_device *spi)
{
	struct athome_platform_data *pd = dev_get_platdata(&spi->dev);
	int ret = spi_bus_lock(spi->master);
	gpio_set_value(pd->gpio_spi_cs, 0);
	udelay(MIN_CS_TO_XFER_DELAY);
	return ret;
}

static int _athome_xfer_stop(struct spi_device *spi)
{
	struct athome_platform_data *pd = dev_get_platdata(&spi->dev);
	gpio_set_value(pd->gpio_spi_cs, 1);
	udelay(MIN_XFER_TO_XFER_DELAY);
	return spi_bus_unlock(spi->master);
}

static int _athome_xfer(struct spi_device *spi,
                        const uint8_t *tx, uint8_t *rx, size_t len)
{
	struct spi_message m;
	struct spi_transfer t = {0};

	t.tx_buf = tx;
	t.rx_buf = rx;
	t.len    = len;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync_locked(spi, &m);
}

static int athome_xfer_tx(struct device *dev, const uint8_t *msg, size_t len)
{
	int rc, ret;
	uint8_t hdr[3];
	struct spi_message m;
	struct spi_transfer t0 = {0};
	struct spi_transfer t1 = {0};
	struct spi_device *spi = to_spi_device(dev);

	BUG_ON(!spi);

	/* grab the bus and assert CS */
	rc = _athome_xfer_start(spi);
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

	ret = spi_sync_locked(spi, &m);

	/* Release the bus, deassert CS */
	rc =_athome_xfer_stop(spi);
	if (rc)
		return rc;

	return ret;
}

static int athome_xfer_rx(struct device *dev, uint8_t *buf, size_t buf_len)
{
	int rc, ret;
	uint8_t hdr[3];
	size_t msg_len;
	struct spi_device *spi = to_spi_device(dev);

	BUG_ON(!spi);

	/* grab the bus and assert CS */
	rc = _athome_xfer_start(spi);
	if (rc)
		return rc;

	/* read header */
	hdr[0] = XFER_TYPE_RX;
	hdr[1] = 0;
	hdr[2] = 0;

	rc = _athome_xfer(spi, hdr, hdr, 3);
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
			rc = _athome_xfer(spi, NULL, buf, msg_len);
		} else { /* message is bigger then buffer */
			/* read maximum length and discard it */
			ret = 0;
			rc = _athome_xfer(spi, NULL, buf, buf_len);
		}
	} else { /* not an valid RX msg */
		if (hdr[0] == XFER_TYPE_NONE) { /* looks like no msg was queued */
			/* just drop it */
			rc = ret = 0;
		} else { /* malformatted message */
			/* read maximum length and discard it */
			ret = 0;
			rc = _athome_xfer(spi, NULL, buf, buf_len);
		}
	}
	if (rc)
		ret = rc;

done:

	/* Release the bus, deassert CS */
	rc =_athome_xfer_stop(spi);
	if (rc)
		return rc;

	return ret;
}

static int __init athome_radio_spi_init(void)
{
	return spi_register_driver(&athome_radio_spi_driver);
}

static void __exit athome_radio_spi_exit(void)
{
	return spi_unregister_driver(&athome_radio_spi_driver);
}

module_init(athome_radio_spi_init);
module_exit(athome_radio_spi_exit);


MODULE_DESCRIPTION("SPI transport of the @home radio module driver");
MODULE_LICENSE("GPL");



