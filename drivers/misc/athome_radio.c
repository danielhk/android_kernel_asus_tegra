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
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>

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


struct athome_msg {
	unsigned len;
	uint8_t *data;
};

struct athome_state {
	struct miscdevice misc_dev;
	struct mutex tx_lock;
	struct mutex rx_lock;

	atomic_t opened;
	int      gpio_irq;
	int      gpio_rst;
	bool     suspended;
	int      awake_cnt;
	spinlock_t   slock;

	struct device   *owner_dev;
	struct athome_transport_ops *xfer_ops;

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

static void athome_radio_reset(struct athome_state *st)
{
	/* stay in reset until userland services release it */
	gpio_set_value(st->gpio_rst, 0);
	udelay(100);
}

/*
 *  Ref counted wrappers on top of pm_stay_awake/pm_relax pair
 *
 */
static void _pm_lock(struct athome_state *st)
{
	unsigned long flags;
	spin_lock_irqsave(&st->slock, flags);
	if (st->awake_cnt++ == 0) {
		pm_stay_awake(st->misc_dev.this_device);
	}
	spin_unlock_irqrestore(&st->slock, flags);
}

static void _pm_unlock(struct athome_state *st)
{
	unsigned long flags;

	BUG_ON(st->awake_cnt == 0);

	spin_lock_irqsave(&st->slock, flags);
	if (--st->awake_cnt == 0) {
		pm_relax(st->misc_dev.this_device);
	}
	spin_unlock_irqrestore(&st->slock, flags);
}

static void athome_radio_flush_nolock(struct athome_state *st, int tx, int rx)
{
	/*
	 * This function is called with RX interrupt disabled,
	 * workqueue flushed and all kinds of locks held so
	 * we can directly modify counters.
	 */
	if (rx) {
		memset(st->rx_msg, 0, sizeof(st->rx_msg));
		st->awake_cnt -= atomic_read(&st->rx_num_used);
		atomic_set(&st->rx_num_used, 0);
		st->posRXW = st->posRXR;
	}

	if (tx) {
		memset(st->tx_msg, 0, sizeof(st->tx_msg));
		st->awake_cnt -= atomic_read(&st->tx_num_used);
		atomic_set(&st->tx_num_used, 0);
		st->posTXW = st->posTXR;
	}

	BUG_ON(st->awake_cnt < 0);

	if (st->awake_cnt == 0)
		pm_relax(st->misc_dev.this_device);
}

static int  athome_radio_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct athome_state *st = container_of(filp->private_data,
					       struct athome_state, misc_dev);

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

	/* It is expected that caller holds wakelock before calling this */
	_pm_unlock(st);

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

	/* tell pm we have work to do */
	_pm_lock(st);

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

static void athome_radio_tx_work(struct work_struct *w)
{
	int rc;
	struct  athome_msg msg;
	struct  athome_state *st = container_of(w,
						struct athome_state, tx_work);

	if (atomic_read(&st->tx_num_used) == 0)
		return;

	msg = st->tx_msg[st->posTXR];

	rc = st->xfer_ops->xfer_tx(st->owner_dev, msg.data, msg.len);
	if (rc < 0) {
		dev_alert(st->owner_dev, "TX xfer failed (%d)\n", rc);
	}

	st->tx_msg[st->posTXR].len  = 0;
	st->tx_msg[st->posTXR].data = NULL;
	st->posTXR++;
	if (st->posTXR == TX_BUF_NUM)
		st->posTXR = 0;

	atomic_dec(&st->tx_num_used);

	/* wakeup writers */
	wake_up_interruptible(&st->wr_waitqh);

	/* requeue ourself */
	queue_work(st->workq, &st->tx_work);

	/* tell pm we finished a piece of work */
	_pm_unlock(st);
}

static void athome_radio_rx_work(struct work_struct *w)
{
	int rc;
	int rx_cnt = 0;
	struct  athome_state *st = container_of(w, struct athome_state,
						rx_work);
	uint8_t *rxbuf;

Again:
	/* Note: RX Interrupt is disabled */
	if (gpio_get_value(st->gpio_irq)) {
		/* interrupt line is not asserted */
		dev_alert(st->owner_dev, "spurius interrupt\n");
		goto done;
	}

	/* we should always have at least one free buffer */
	rxbuf = get_rx_buf_ptr(st, st->posRXW);

	/* receive data */
	rx_cnt++;
	rc = st->xfer_ops->xfer_rx(st->owner_dev, rxbuf, MAX_MSG_SIZE);
	if (rc < 0) {
		/* rx xfer failed */
		dev_alert(st->owner_dev, "RX xfer failed (%d)\n", rc);
		goto rx_done;
	} else if (rc == 0) {
		/* no data */
		goto rx_done;
	}

	if (atomic_read(&st->rx_num_used) == RX_BUF_NUM-1) {
		/* that was the last rx buffer, and since we will always need
		   at least one, discard it */
		dev_alert(st->owner_dev, "out of rx buffers. Discarding\n");
		goto rx_done;
	}

	if (atomic_read(&st->opened)) {

		/* tell pm we have some work to do */
		_pm_lock(st);

		st->rx_msg[st->posRXW].len  = rc;
		st->rx_msg[st->posRXW].data = rxbuf;
		st->posRXW++;
		if (st->posRXW == RX_BUF_NUM)
			st->posRXW = 0;

		atomic_inc(&st->rx_num_used);

		/* wakeup readers */
		wake_up_interruptible(&st->rd_waitqh);
	}

rx_done:

	/* check the state of irq line */
	if (gpio_get_value(st->gpio_irq) == 0) {
		if (rx_cnt < 50) {
			goto Again; /* still asserted */
		} else {
			/* reset chip */
			athome_radio_reset(st);
			dev_alert(st->owner_dev,
			         "Too much work or IRQ line might be stuck."
			         "Resetting radio\n");
		}
	}

done:
	/* reenable irq */
	enable_irq(gpio_to_irq(st->gpio_irq));
	_pm_unlock (st); /* paired with _pm_lock in irq handler */
	return;
}

static irqreturn_t athome_radio_irq(int irq, void *data)
{
	struct  athome_state *st = data;

	_pm_lock(st);
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

static int __devinit athome_radio_gpio_init(struct  athome_state *st)
{
	int ret, irq;
	const struct gpio gpios[] = {
		{ st->gpio_irq, GPIOF_IN,       ATHOME_RADIO_MOD_NAME " irq" },
		/* defaults to RESET */
		{ st->gpio_rst, GPIOF_INIT_LOW, ATHOME_RADIO_MOD_NAME " rst" },
	};

	irq = gpio_to_irq(st->gpio_irq);
	if (irq < 0) {
		dev_err(st->owner_dev, "IRQ gpio not available as an interrupt.\n");
		return -EIO;
	}
	ret = gpio_request_array(gpios, ARRAY_SIZE(gpios));
	if (ret) {
		dev_err(st->owner_dev, "Failed to request required GPIOs\n");
		return ret;
	}

	/* We just placed device in reset, let it settle before checking
	   irq GPIO state */
	udelay(100);

	if (!gpio_get_value(st->gpio_irq)) {
		dev_err(st->owner_dev, "IRQ gpio is asserted at boot\n");
		gpio_free_array(gpios, ARRAY_SIZE(gpios));
		return -EIO;
	}

	return 0;
}


int athome_radio_suspend(struct device *dev, pm_message_t state)
{
	int irq;
	struct athome_state *st = dev_get_drvdata(dev);

	if (st->suspended)
		return 0;

	irq = gpio_to_irq(st->gpio_irq);

	/* Note: User space is already frozen */

	/* wait untill all tx work is done */
	flush_work_sync(&st->tx_work);

	/* mask rx irq, so there will be no new work */
	disable_irq(irq);

	/* wait until rx work is done */
	flush_work_sync(&st->rx_work);

	/* do not abort suspend or configure wakeup if nobody is listening */
	if (atomic_read(&st->opened)) {

		if (atomic_read(&st->rx_num_used)) { /* we have unhandled messages */
			dev_info(st->owner_dev, "busy rx\n");
			enable_irq(irq);
			return -EBUSY;
		}

		if (gpio_get_value(st->gpio_irq) == 0) { /* irq asserted, abort suspend */
			dev_info(st->owner_dev, "busy irq\n");
			enable_irq(irq);
			return -EBUSY;
		}

		irq_set_irq_wake(irq, 1);
	}

	st->suspended = true;
	dev_info(st->owner_dev, "suspended\n");

	return 0;
}

int athome_radio_resume(struct device *dev)
{
	int irq;
	struct athome_state *st = dev_get_drvdata(dev);

	if (!st->suspended)
		return 0;

	irq = gpio_to_irq(st->gpio_irq);

	/* undo configuring wakeup */
	if (atomic_read(&st->opened))
		irq_set_irq_wake(irq, 0);

	/* reenable irq */
	enable_irq(irq);

	st->suspended = false;
	dev_info(st->owner_dev, "resumed\n");

	return 0;
}

int __exit athome_radio_destroy(struct device *dev)
{
	struct athome_state *st = dev_get_drvdata(dev);

	if (!st)
		return 0;

	dev_info(st->owner_dev, "exiting\n");

	disable_irq(gpio_to_irq(st->gpio_irq));
	free_irq(gpio_to_irq(st->gpio_irq), st);

	/* destroy wakeup source */
	device_wakeup_disable(st->misc_dev.this_device);

	/* unregister misc device */
	misc_deregister(&st->misc_dev);

	/* kill workqueue */
	if (st->workq) {
		cancel_work_sync(&st->rx_work);
		cancel_work_sync(&st->tx_work);
		destroy_workqueue(st->workq);
	}

	/* free gpios */
	athome_radio_gpio_free(st);

	/* free buffers */
	kfree(st->buffers);

	/* free state */
	kfree(st);

	return 0;
}


int __devinit athome_radio_init(struct device *dev,
                                struct athome_platform_data *pd,
                                struct athome_transport_ops *ops)
{
	struct athome_state *st = NULL;
	int ret;
	size_t cb;

	if (!dev) {
		pr_alert("%s: no device.\n", ATHOME_RADIO_MOD_NAME);
		return -ENODEV;
	}
	if (!pd) {
		dev_alert(dev, "no platform data.\n");
		return -ENODEV;
	}
	if (!ops) {
		dev_alert(dev, "no xfer ops.\n");
		return -ENODEV;
	}

	/* allocate state  */
	st = kzalloc(sizeof(struct athome_state), GFP_KERNEL);
	if (!st) {
		dev_err(dev, "kzalloc failure for state\n");
		return -ENOMEM;
	}

	/* preallocate all data buffers at once */
	dev_info(dev, "max msg size %d\n", MAX_MSG_SIZE);
	cb = (TX_BUF_NUM + RX_BUF_NUM) * MAX_MSG_SIZE;
	st->buffers = kmalloc(cb, GFP_KERNEL);
	if (!st->buffers) {
		dev_err(dev, "kmalloc failure for buffers\n");
		ret = -ENOMEM;
		goto fail_buffers;
	}

	atomic_set(&st->opened, 0);

	spin_lock_init(&st->slock);

	mutex_init(&st->rx_lock);
	mutex_init(&st->tx_lock);

	init_waitqueue_head(&st->rd_waitqh);
	init_waitqueue_head(&st->wr_waitqh);

	st->misc_dev.minor = MISC_DYNAMIC_MINOR;
	st->misc_dev.name  = ATHOME_RADIO_MOD_NAME;
	st->misc_dev.fops  = &athome_radio_fops;

	st->gpio_irq  = pd->gpio_num_irq;
	st->gpio_rst  = pd->gpio_num_rst;

	/* attach device */
	st->owner_dev = dev;
	st->xfer_ops  = ops;

	/* grab gpios and put module in reset */
	ret = athome_radio_gpio_init(st);
	if (ret) {
		dev_err(dev, "gpio_init failure (%d)\n", ret);
		goto fail_gpio;
	}

	/* Initialize workqueues */
	INIT_WORK(&st->tx_work, athome_radio_tx_work);
	INIT_WORK(&st->rx_work, athome_radio_rx_work);
	st->workq = create_singlethread_workqueue(ATHOME_RADIO_MOD_NAME);
	if (!st->workq) {
		dev_err(dev, "failed to create workqueue\n");
		ret = -ENODEV;
		goto fail_wq;
	}

	/* register interrupt handler */
	ret = request_irq(gpio_to_irq(st->gpio_irq), athome_radio_irq,
			  IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
			  IRQF_TRIGGER_LOW, ATHOME_RADIO_MOD_NAME, st);
	if (ret) {
		dev_err(dev, "failed to register irq handler\n");
		goto fail_irq;
	}

	/* register misc device */
	ret = misc_register(&st->misc_dev);
	if (ret) {
		dev_err(dev, "misc_register failure\n");
		goto fail_misc;
	}

	/* register wakeup source */
	ret = device_init_wakeup(st->misc_dev.this_device, true);
	if (ret) {
		dev_err(dev, "failed (%d) to init wakeup source\n", ret);
		goto fail_wakeup;
	}

	/* attach state to device object */
	dev_set_drvdata(dev, st);

	/* Note: keep radio in reset until user space opens it */

	goto done;

fail_wakeup:
	if (ret)
		misc_deregister (&st->misc_dev);

fail_misc:
	if (ret)
		free_irq(gpio_to_irq(st->gpio_irq), st);

fail_irq:
	if (ret)
		destroy_workqueue(st->workq);

fail_wq:
	if (ret)
		athome_radio_gpio_free(st);

fail_gpio:
	if (ret)
		kfree(st->buffers);

fail_buffers:
	if (ret)
		kfree(st);

done:
	dev_info(dev, "Init done with return code %d\n", ret);
	return ret;
}

MODULE_DESCRIPTION("Common part of the @home radio module driver");
MODULE_LICENSE("GPL");

