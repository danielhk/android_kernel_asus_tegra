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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/athome_radio.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/poll.h>

#include "athome_transport.h"

/* comms config */
#define RX_BUF_NUM      8
#define TX_BUF_NUM      8

/*
 * athome radio buffers should be large enough to hold any packet supported
 * by firmware but not larger then 1024 bytes. It is hard to calculate
 * real value without compiling against firmware software so we will reserve
 * max.
 */
#define MAX_MSG_SIZE    1024

/* protocol defines */
#define PKT_TYP_DATA    0x00
#define PKT_TYP_RTR     0x10

struct athome_msg {
	unsigned len;
	uint8_t *data;
};

struct athome_state {
	struct miscdevice dev;
	struct mutex tx_lock;
	struct mutex rx_lock;

	atomic_t opened;
	int      gpio_irq;
	int      gpio_rst;

	struct workqueue_struct *workq;
	struct work_struct       tx_work;
	struct work_struct       rx_work;
	wait_queue_head_t        rd_waitqh;
	wait_queue_head_t        wr_waitqh;
	struct athome_msg        rx_msg[RX_BUF_NUM];
	struct athome_msg        tx_msg[TX_BUF_NUM];
	unsigned   posRXR;
	unsigned   posRXW;
	unsigned   posTXR;
	unsigned   posTXW;
	atomic_t   rx_num_used;
	atomic_t   tx_num_used;
	uint8_t   *buffers;
};


static uint8_t *get_tx_buf_ptr(struct athome_state *st, uint txidx)
{
	return st->buffers + txidx * MAX_MSG_SIZE;
}

static uint8_t *get_rx_buf_ptr(struct athome_state *st, uint rxidx)
{
	return st->buffers + (TX_BUF_NUM + rxidx) * MAX_MSG_SIZE;
}

static void athome_radio_flush_nolock(struct athome_state *st, int tx, int rx)
{
	if (rx) {
		memset(st->rx_msg, 0, sizeof(st->rx_msg));
		atomic_set(&st->rx_num_used, 0);
		st->posRXW = st->posRXR;
	}

	if (tx) {
		memset(st->tx_msg, 0, sizeof(st->tx_msg));
		atomic_set(&st->tx_num_used, 0);
		st->posTXW = st->posTXR;
	}
}

static int  athome_radio_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct athome_state *st = container_of(filp->private_data,
					       struct athome_state, dev);

	if (atomic_cmpxchg(&st->opened, 0, 1))
		ret = -EBUSY;

	filp->private_data = st;

	return ret;
}

static int athome_radio_close(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct athome_state *st = (struct athome_state *)filp->private_data;

	mutex_lock(&st->rx_lock);
	mutex_lock(&st->tx_lock);
	disable_irq(st->gpio_irq);
	flush_workqueue(st->workq);

	/* flush all data before marking device as closed */
	athome_radio_flush_nolock(st, 1, 1);

	if (atomic_cmpxchg(&st->opened, 1, 0))
		ret = -EBUSY;

	enable_irq(st->gpio_irq);
	mutex_unlock(&st->tx_lock);
	mutex_unlock(&st->rx_lock);

	return ret;
}

static ssize_t athome_radio_read(struct file *filp, char __user *buff,
				 size_t count, loff_t *ppos)
{
	struct athome_msg msg;

	struct athome_state *st = (struct athome_state *)filp->private_data;

	do {
		if (mutex_lock_interruptible(&st->rx_lock))
			return -ERESTARTSYS;

		if (atomic_read(&st->rx_num_used))
			break; /* have data */

		/* nothing to read */
		mutex_unlock(&st->rx_lock);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN; /* non-blocking */

		/* wait for data */
		if (wait_event_interruptible(st->rd_waitqh,
					     atomic_read(&st->rx_num_used)))
			return -ERESTARTSYS;
	} while (1);

	/* we have message */
	msg = st->rx_msg[st->posRXR];

	if (count < msg.len) { /* buffer is too small */
		mutex_unlock(&st->rx_lock);
		return -EINVAL; /* can't find better err code */
	}
	count = msg.len;

	st->rx_msg[st->posRXR].len  = 0;
	st->rx_msg[st->posRXR].data = NULL;

	st->posRXR++;
	if (st->posRXR == RX_BUF_NUM)
		st->posRXR = 0;

	atomic_dec(&st->rx_num_used);

	mutex_unlock(&st->rx_lock);

	return copy_to_user(buff, msg.data, count) ? -EFAULT : count;
}

static ssize_t athome_radio_write(struct file *filp, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	uint8_t *txbuf;
	struct athome_state *st = (struct athome_state *)filp->private_data;

	if (count > MAX_MSG_SIZE)
		return -EINVAL;

	do {
		if (mutex_lock_interruptible(&st->tx_lock))
			return -ERESTARTSYS;

		if (atomic_read(&st->tx_num_used) < TX_BUF_NUM)
			break; /* have space */

		mutex_unlock(&st->tx_lock);

		/* no space is availabe  */
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN; /* non-blocking */

		/* wait for buffers become available */
		if (wait_event_interruptible(st->wr_waitqh,
				atomic_read(&st->tx_num_used) < TX_BUF_NUM))
			return -ERESTARTSYS;
	} while (1);

	txbuf = get_tx_buf_ptr(st, st->posTXW);

	if (copy_from_user(txbuf, buf, count)) {
		mutex_unlock(&st->tx_lock);
		return -EFAULT;
	}

	st->tx_msg[st->posTXW].len  = count;
	st->tx_msg[st->posTXW].data = txbuf;
	st->posTXW++;
	if (st->posTXW == TX_BUF_NUM)
		st->posTXW = 0;
	atomic_inc(&st->tx_num_used);
	mutex_unlock(&st->tx_lock);

	queue_work(st->workq, &st->tx_work);

	return count;
}

static int athome_radio_drain_tx(struct file *filp)
{
	struct athome_state *st = (struct athome_state *)filp->private_data;

	do {
		if (atomic_read(&st->tx_num_used) == 0)
			return 0; /* no data */

		/* we have some data */
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN; /* non-blocking */

		if (wait_event_interruptible(st->wr_waitqh,
				     atomic_read(&st->tx_num_used) == 0))
			return -ERESTARTSYS;
	} while (1);
}

static long athome_radio_unlocked_ioctl(struct file *filp,
					unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct athome_state *st = (struct athome_state *)filp->private_data;

	switch (cmd) {

	case ATHOME_RADIO_IOCTL_RESET:
		gpio_set_value(st->gpio_rst, !arg);
		/* nonzero arg to put into reset, zero to run.
		   This allows userspace to hold chip in reset */
		break;

	case ATHOME_RADIO_IOCTL_FLUSH:
		mutex_lock(&st->rx_lock);
		mutex_lock(&st->tx_lock);
		disable_irq(st->gpio_irq);
		flush_workqueue(st->workq);
		athome_radio_flush_nolock(st, 1, 1);
		mutex_unlock(&st->tx_lock);
		mutex_unlock(&st->rx_lock);
		break;

	case ATHOME_RADIO_IOCTL_DRAIN_TX:
		ret = athome_radio_drain_tx(filp);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static unsigned int athome_radio_poll(struct file *filp,
				      struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct athome_state *st = (struct athome_state *)filp->private_data;

	poll_wait(filp, &st->rd_waitqh, wait);
	poll_wait(filp, &st->wr_waitqh, wait);
	if (atomic_read(&st->rx_num_used))
		mask |= POLLIN  | POLLRDNORM;   /* readable */
	if (atomic_read(&st->tx_num_used) < TX_BUF_NUM)
		mask |= POLLOUT | POLLWRNORM;   /* writable */

	return mask;
}

static void athome_radio_packet_init(uint8_t *pkt, uint8_t type, uint16_t len)
{
	pkt[0] = type;
	pkt[1] = (uint8_t)(len >> 8);  /* network byte order */
	pkt[2] = (uint8_t)(len);
}

static uint8_t athome_radio_packet_get_type(const uint8_t *pkt)
{
	return pkt[0];
}

static uint16_t athome_radio_packet_get_size(const uint8_t *pkt)
{
	return  (((uint16_t) pkt[1]) << 8) | pkt[2];
}

static void athome_radio_tx_work(struct work_struct *w)
{
	uint8_t hdr[3];
	struct  athome_msg msg;
	struct  athome_state *st = container_of(w,
						struct athome_state, tx_work);

	if (atomic_read(&st->tx_num_used) == 0)
		return;

	msg = st->tx_msg[st->posTXR];

	/* send msg */
	athome_transport_start();
	athome_radio_packet_init(hdr, PKT_TYP_DATA, msg.len);

	if (athome_transport_send(hdr, 3))
		pr_alert("TX: failed to send hdr\n");

	if (athome_transport_send(msg.data, msg.len))
		pr_alert("TX: failed to send data\n");

	athome_transport_stop();

	st->tx_msg[st->posTXR].len  = 0;
	st->tx_msg[st->posTXR].data = NULL;
	st->posTXR++;
	if (st->posTXR == TX_BUF_NUM)
		st->posTXR = 0;

	atomic_dec(&st->tx_num_used);

	/* wateup writers */
	wake_up_interruptible(&st->wr_waitqh);

	/* requeue ourself */
	queue_work(st->workq, &st->tx_work);
}

static void athome_radio_rx_work(struct work_struct *w)
{
	uint8_t type;
	uint8_t hdr[3];
	struct  athome_state *st = container_of(w, struct athome_state,
						rx_work);
	size_t  len = MAX_MSG_SIZE;
	uint8_t *rxbuf;

Again:
	/* Note: RX Interrupt is disabled */
	if (gpio_get_value(st->gpio_irq)) {
		/* interrupt line is not asserted */
		pr_alert("%s: spurius interrupt\n", ATHOME_RADIO_MOD_NAME);
		enable_irq(gpio_to_irq(st->gpio_irq));
		return;
	}

	/* grab the bus  */
	athome_transport_start();

	/* read header   */
	athome_radio_packet_init(hdr, PKT_TYP_RTR, 0);
	if (athome_transport_send(hdr, 3)) {
		pr_alert("%s: RX: failed to send header\n", ATHOME_RADIO_MOD_NAME);
		goto rx_drain;
	}

	if (athome_transport_recv(hdr, 3)) {
		pr_alert("%s: RX: failed to recv header\n", ATHOME_RADIO_MOD_NAME);
		goto rx_drain;
	}

	type = athome_radio_packet_get_type(hdr);
	if (type != PKT_TYP_RTR) {
		pr_alert("%s: Got packet with type %d. Ignoring\n",
		          ATHOME_RADIO_MOD_NAME,  type);
		goto rx_drain;
	}

	len = athome_radio_packet_get_size(hdr);
	if (len == 0 || len > MAX_MSG_SIZE) {
		pr_alert("%s: Got packet with length %d. Ignoring\n",
		          ATHOME_RADIO_MOD_NAME, len);
		len = MAX_MSG_SIZE;
		goto rx_drain;
	}

	if (atomic_read(&st->rx_num_used) == RX_BUF_NUM) {
		/* no rx buffers */
		pr_alert("%s: No rx buffers available. Ignoring\n",
			      ATHOME_RADIO_MOD_NAME);
		goto rx_drain;
	}

	/* read body */
	rxbuf = get_rx_buf_ptr(st, st->posRXW);
	if (athome_transport_recv(rxbuf, len)) {
		pr_alert("%s: RX: failed to recv data\n", ATHOME_RADIO_MOD_NAME);
		goto rx_drain; /* this is the best thing we can do to recover */
	}

	if (atomic_read(&st->opened)) {

		st->rx_msg[st->posRXW].len  = len;
		st->rx_msg[st->posRXW].data = rxbuf;
		st->posRXW++;
		if (st->posRXW == RX_BUF_NUM)
			st->posRXW = 0;

		atomic_inc(&st->rx_num_used);

		/* wakeup readers */
		wake_up_interruptible(&st->rd_waitqh);
	}

rx_done:
	/* release the bus */
	athome_transport_stop();

	/* check the state of irq line */
	if (gpio_get_value(st->gpio_irq) == 0) {
		/* DOLATER: Add upper bounds on the number of retries */
		goto Again; /* still asserted */
	}

	/* reenable irq */
	enable_irq(gpio_to_irq(st->gpio_irq));
	return;

rx_drain:
	/* drain rx */
	athome_transport_recv(NULL, len);
	goto rx_done;
}

static irqreturn_t athome_radio_irq(int irq, void *data)
{
	struct  athome_state *st = data;
	disable_irq_nosync(gpio_to_irq(st->gpio_irq));
	queue_work(st->workq, &st->rx_work);

	return IRQ_HANDLED;
}

static const struct file_operations athome_radio_fops = {
	.owner = THIS_MODULE,
	.read  = athome_radio_read,
	.write = athome_radio_write,
	.unlocked_ioctl = athome_radio_unlocked_ioctl,
	.open    = athome_radio_open,
	.release = athome_radio_close,
	.poll    = athome_radio_poll,
};

static void athome_radio_gpio_free(struct  athome_state *st)
{
	gpio_free(st->gpio_irq);
	gpio_free(st->gpio_rst);
}

static int __init athome_radio_gpio_init(struct  athome_state *st)
{
	int ret, irq;
	const struct gpio gpios[] = {
		{ st->gpio_irq, GPIOF_IN,       ATHOME_RADIO_MOD_NAME " irq" },
		/* defaults to RESET */
		{ st->gpio_rst, GPIOF_INIT_LOW, ATHOME_RADIO_MOD_NAME " rst" },
	};

	irq = gpio_to_irq(st->gpio_irq);
	if (irq < 0) {
		pr_err("%s: IRQ gpio not available as an interrupt.\n",
		       ATHOME_RADIO_MOD_NAME);
		return -EIO;
	}
	ret = gpio_request_array(gpios, ARRAY_SIZE(gpios));
	if (ret) {
		pr_err("%s: Failed to request required GPIOs\n",
		        ATHOME_RADIO_MOD_NAME);
		return ret;
	}

	if (!gpio_get_value(st->gpio_irq)) {
		pr_err("%s: IRQ gpio is asserted at boot\n",
		        ATHOME_RADIO_MOD_NAME);
		gpio_free_array(gpios, ARRAY_SIZE(gpios));
		return -EIO;
	}

	return 0;
}


static int athome_radio_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	return 0;
}

static int athome_radio_resume(struct platform_device *pdev)
{
	return 0;
}

static int athome_radio_remove(struct platform_device *pdev)
{
	struct athome_state *st = platform_get_drvdata(pdev);

	if (!st)
		return 0;

	disable_irq(gpio_to_irq(st->gpio_irq));
	free_irq(gpio_to_irq(st->gpio_irq), st);

	/* unregister misc device */
	misc_deregister(&st->dev);

	/* kill workqueue */
	if (st->workq) {
		cancel_work_sync(&st->rx_work);
		cancel_work_sync(&st->tx_work);
		destroy_workqueue(st->workq);
	}

	/* close transport */
	athome_transport_close();

	/* free gpios */
	athome_radio_gpio_free(st);

	/* free buffers */
	kfree(st->buffers);

	/* free state */
	kfree(st);

	pr_info(ATHOME_RADIO_MOD_LOG_NAME "exiting\n");

	return 0;
}

static int __init athome_radio_probe(struct platform_device *pdev)
{
	struct athome_state *st = NULL;
	struct athome_platform_data *pd;
	int ret;
	size_t cb;

	pr_info("%s: Initializing\n", ATHOME_RADIO_MOD_NAME);

	if (!pdev) {
		pr_alert("%s: no platform device.\n", ATHOME_RADIO_MOD_NAME);
		return -ENODEV;
	}
	pd = pdev->dev.platform_data;
	if (!pd) {
		pr_alert("%s: no platform data.\n", ATHOME_RADIO_MOD_NAME);
		return -ENODEV;
	}

	/* allocate state  */
	st = kzalloc(sizeof(struct athome_state), GFP_KERNEL);
	if (!st) {
		pr_err("%s: kzalloc failure for state\n", ATHOME_RADIO_MOD_NAME);
		return -ENOMEM;
	}

	/* preallocate all data buffers at once */
	pr_info("%s: max msg size %d\n", ATHOME_RADIO_MOD_NAME, MAX_MSG_SIZE);
	cb = (TX_BUF_NUM + RX_BUF_NUM) * MAX_MSG_SIZE;
	st->buffers = kmalloc(cb, GFP_KERNEL);
	if (!st->buffers) {
		pr_err("%s: kmalloc failure for buffers\n", ATHOME_RADIO_MOD_NAME);
		ret = -ENOMEM;
		goto fail_buffers;
	}

	atomic_set(&st->opened, 0);

	mutex_init(&st->rx_lock);
	mutex_init(&st->tx_lock);

	init_waitqueue_head(&st->rd_waitqh);
	init_waitqueue_head(&st->wr_waitqh);

	st->dev.minor = MISC_DYNAMIC_MINOR;
	st->dev.name  = ATHOME_RADIO_MOD_NAME;
	st->dev.fops  = &athome_radio_fops;

	st->gpio_irq  = pd->gpio_num_irq;
	st->gpio_rst  = pd->gpio_num_rst;

	/* grab gpios and put module in reset */
	ret = athome_radio_gpio_init(st);
	if (ret) {
		pr_err("%s: gpio_init failure\n", ATHOME_RADIO_MOD_NAME);
		goto fail_gpio;
	}

	/* open transport */
	ret = athome_transport_open(pd);
	if (ret < 0) {
		pr_err("%s: transport open failure\n", ATHOME_RADIO_MOD_NAME);
		goto fail_transport;
	}

	/* Initialize workqueues */
	INIT_WORK(&st->tx_work, athome_radio_tx_work);
	INIT_WORK(&st->rx_work, athome_radio_rx_work);
	st->workq = create_singlethread_workqueue(ATHOME_RADIO_MOD_NAME);
	if (!st->workq) {
		pr_err("%s: failed to create workqueue\n", ATHOME_RADIO_MOD_NAME);
		ret = -ENODEV;
		goto fail_wq;
	}

	/* register interrupt handler */
	ret = request_irq(gpio_to_irq(st->gpio_irq), athome_radio_irq,
			  IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
			  IRQF_TRIGGER_LOW, ATHOME_RADIO_MOD_NAME, st);
	if (ret) {
		pr_err("%s: failed to register irq handler\n", ATHOME_RADIO_MOD_NAME);
		goto fail_irq;
	}

	/* register misc device */
	ret = misc_register(&st->dev);
	if (ret) {
		pr_err("%s: misc_register failure\n", ATHOME_RADIO_MOD_NAME);
		goto fail_misc;
	}

	platform_set_drvdata(pdev, st);

	/* take it out of reset */
	gpio_set_value(st->gpio_rst, 1);

fail_misc:
	if (ret)
		free_irq(gpio_to_irq(st->gpio_irq), st);

fail_irq:
	if (ret)
		destroy_workqueue(st->workq);

fail_wq:
	if (ret)
		athome_transport_close();

fail_transport:
	if (ret)
		athome_radio_gpio_free(st);

fail_gpio:
	if (ret)
		kfree(st->buffers);

fail_buffers:
	if (ret)
		kfree(st);

	pr_info("%s: Init done with return code %d\n",
	         ATHOME_RADIO_MOD_NAME, ret);
	return ret;
}

static struct platform_driver radio_driver = {
	.driver = {
		.name   = ATHOME_RADIO_MOD_NAME,
		.owner  = THIS_MODULE,
	},
	.remove   = athome_radio_remove,
	.suspend  = athome_radio_suspend,
	.resume   = athome_radio_resume,
};

static int __init athome_radio_init(void)
{
	return platform_driver_probe(&radio_driver, athome_radio_probe);
}

static void __exit athome_radio_exit(void)
{
	return platform_driver_unregister(&radio_driver);
}


module_exit(athome_radio_exit);
module_init(athome_radio_init);

MODULE_DESCRIPTION("Driver for the @home radio module");
MODULE_AUTHOR("Dmitry Grinberg <dmitrygr@google.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");



