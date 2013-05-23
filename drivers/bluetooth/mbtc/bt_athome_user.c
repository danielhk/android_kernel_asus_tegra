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

#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/atomic.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include "bt_athome_user.h"
#include "bt_athome_logging.h"
#include "bt_athome_le_stack.h"




#define MAX_USERSPACE_MSGS		128


static struct semaphore usr_rx = /* num msgs avail to read by user */
			__SEMAPHORE_INITIALIZER(usr_rx, 0);
static struct sk_buff_head usr_data;
static atomic_t usr_opened = ATOMIC_INIT(0);
static atomic_t in_bind_mode = ATOMIC_INIT(0);
/*
 *	Please do not ever use this value for anything. It is only here for
 *	queue length limiting. it is *NOT* updated atomically with the wait
 *	queue or the semaphore, and if you use it, you might get a very close
 *	but maybe not quite right value and end up blocked forever.
 */
static atomic_t num_usr_msgs = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(usr_wait_q);

static DEFINE_SPINLOCK(device_list_lock);
static struct athome_bt_known_remote *known = NULL;
static bool usr_data_inited = false;



/* only to be called with state lock held */
static struct athome_bt_known_remote *athome_bt_find_known_l(
				const uint8_t *MAC,
				struct athome_bt_known_remote **prevP)
{
	struct athome_bt_known_remote *cur, *prev = NULL;

	cur = known;

	while (cur && memcmp(cur->MAC, MAC, sizeof(cur->MAC))) {
		prev = cur;
		cur = cur->next;
	}

	if (prevP) *prevP = prev;
	return cur;
}

struct athome_bt_known_remote *athome_bt_find_known(const uint8_t *MAC)
{
	struct athome_bt_known_remote *ret;
	unsigned long flags;

	spin_lock_irqsave(&device_list_lock, flags);
	ret = athome_bt_find_known_l(MAC, NULL);
	spin_unlock_irqrestore(&device_list_lock, flags);

	return ret;
}

void athome_bt_usr_enqueue(uint8_t typ, const void *data, unsigned len)
{
	struct sk_buff *skb;
	long new_num;

	/* do not enqueue if file's not opened */
	if (!atomic_read(&usr_opened))
		return;

	/* drop if queue is full */
	smp_mb();
	new_num = atomic_inc_return(&num_usr_msgs);
	smp_mb();

	if (new_num > MAX_USERSPACE_MSGS) {
		atomic_dec(&num_usr_msgs);
		aahlog("dropped userspace msg (%d) due to full queue\n", typ);
		return;
	}

	skb = bt_skb_alloc(len + 1, GFP_ATOMIC);
	if (!skb) {
		atomic_dec(&num_usr_msgs);
		aahlog("fail to alloc skb for %u usr bytes\n", len);
		return;
	}

	*((uint8_t*)skb->data) = typ;
	memcpy(skb->data + 1, data, len);
	skb_put(skb, len + 1);
	skb_queue_tail(&usr_data, skb);

	up(&usr_rx);
	wake_up_interruptible(&usr_wait_q);
}

static ssize_t athome_bt_read(struct file *file, char __user *userbuf,
						size_t bytes, loff_t *off)
{
	struct sk_buff *skb;

	if (down_interruptible(&usr_rx))
		return -EINTR;

	atomic_dec(&num_usr_msgs);
	skb = skb_dequeue(&usr_data);
	BUG_ON(!skb);

	if (bytes > skb->len)
		bytes = skb->len;

	if (copy_to_user(userbuf, skb->data, bytes)) {
		atomic_inc(&num_usr_msgs);
		up(&usr_rx);
		skb_queue_head(&usr_data, skb);
		return -EFAULT;
	} else {
		kfree_skb(skb);
		return bytes;
	}
}

static int athome_bt_add_dev(const uint8_t *MAC, const uint8_t *LTK)
{	/* if LTK == NULL -> bind mode addition */

	unsigned long flags;
	struct athome_bt_known_remote *cur, *item;

	item = kmalloc(sizeof(*item), GFP_KERNEL);
	if (!item)
		return -ENOMEM;

	spin_lock_irqsave(&device_list_lock, flags);

	cur = athome_bt_find_known_l(MAC, NULL);
	if (cur) {
		kfree(item);
		item = cur;
	} else {
		memcpy(item->MAC, MAC, sizeof(item->MAC));
		item->next = known;
		known = item;
	}

	if (LTK) {
		item->bind_mode = 0;
		memcpy(item->LTK, LTK, sizeof(item->LTK));
	} else
		item->bind_mode = 1;
	spin_unlock_irqrestore(&device_list_lock, flags);
	return 0;
}

static void athome_bt_del_dev(uint8_t *MAC, bool bind_mode)
{
	unsigned long flags;
	struct athome_bt_known_remote *cur, *prev;

	spin_lock_irqsave(&device_list_lock, flags);
	cur = athome_bt_find_known_l(MAC, &prev);
	if (cur && (!cur->bind_mode == !bind_mode)) {

		if (prev)
			prev->next = cur->next;
		else
			known = cur->next;
		kfree(cur);
	}

	athome_bt_disc_from_mac(MAC);
	spin_unlock_irqrestore(&device_list_lock, flags);
}

static ssize_t athome_bt_write(struct file *file, const char __user *userbuf,
						size_t bytes, loff_t *off)
{
	uint8_t buf[BT_ATHOME_MAX_USER_LEN] = {0,};
	struct bt_athome_add_dev *add = (struct bt_athome_add_dev*)(buf + 1);
	struct bt_athome_set_bind_mode *bind =
				(struct bt_athome_set_bind_mode*)(buf + 1);
	struct bt_athome_del_dev *del = (struct bt_athome_del_dev*)(buf + 1);
	struct bt_athome_state *state = (struct bt_athome_state*)buf;
	struct bt_athome_do_bind *do_bind =
				(struct bt_athome_do_bind*)(buf + 1);
	struct bt_athome_stop_bind *stop_bind =
				(struct bt_athome_stop_bind*)(buf + 1);
	struct bt_athome_encrypt *encr = (struct bt_athome_encrypt*)(buf + 1);
	struct bt_athome_send_data *data =
				(struct bt_athome_send_data*)(buf + 1);
	struct bt_athome_get_dev_stats *get_stats =
				(struct bt_athome_get_dev_stats*)(buf + 1);
	struct bt_athome_dev_stats *stats = (struct bt_athome_dev_stats*)buf;
	ssize_t ret, orig_bytes = bytes;


	if (!bytes)	/* empty commands do nothing */
		return 0;

	if (bytes > sizeof(buf))
		return -ENOMEM;

	if (copy_from_user(buf, userbuf, bytes))
		return -EFAULT;

	bytes--;
	switch (buf[0]) {

	case BT_ATHOME_MSG_ADD_DEV:
		if (bytes != sizeof(*add))
			return -EIO;
		ret = athome_bt_add_dev(add->MAC, add->LTK);
		if (ret < 0)
			orig_bytes = ret;
		break;

	case BT_ATHOME_MSG_SET_BIND_MODE:
		if (bytes != sizeof(*bind))
			return -EIO;
		atomic_set(&in_bind_mode, bind->bind_mode);
		break;

	case BT_ATHOME_MSG_DO_BIND:
		if (bytes != sizeof(*do_bind))
			return -EIO;
		ret = athome_bt_add_dev(do_bind->MAC, NULL);
		if (ret < 0)
			orig_bytes = ret;
		break;

	case BT_ATHOME_MSG_STOP_BIND:
		if (bytes != sizeof(*stop_bind))
			return -EIO;
		athome_bt_del_dev(del->MAC, 1);
		break;

	case BT_ATHOME_MSG_ENCRYPT:
		if (bytes != sizeof(*encr))
			return -EIO;
		athome_bt_start_encr_for_mac(encr->MAC);
		break;

	case BT_ATHOME_MSG_DEL_DEV:
		if (bytes != sizeof(*del))
			return -EIO;

		athome_bt_del_dev(del->MAC, 0);
		break;

	case BT_ATHOME_MSG_GET_STATE:
		if (bytes)
			return -EIO;

		athome_bt_get_state(state);
		athome_bt_usr_enqueue(BT_ATHOME_EVT_STATE,
						state, sizeof(*state));
		break;

	case BT_ATHOME_MSG_DATA:
		if (bytes < sizeof(*data))
			return -EIO;

		athome_bt_send_data(data->MAC, data->pkt_typ, data->pkt_data,
						bytes - sizeof(*data), true);
		break;

	case BT_ATHOME_MSG_DEV_STATS:
		if (bytes < sizeof(*get_stats))
			return -EIO;
		memcpy(stats->MAC, get_stats->MAC, sizeof(get_stats->MAC));
		if (athome_bt_get_stats(&stats->stats, stats->MAC))
			athome_bt_usr_enqueue(BT_ATHOME_EVT_DEV_STATS,
							stats, sizeof(*stats));
		break;

	default:
		aahlog("unknown user command 0x%x\n", buf[0]);
		/* bad command */
		return -EIO;
	}

	return orig_bytes;
}

static int athome_bt_open(struct inode *inode, struct file *file)
{
	if (atomic_cmpxchg(&usr_opened, 0, 1))
		return -ENOMEM;

	if (!usr_data_inited) {
		usr_data_inited = true;
		skb_queue_head_init(&usr_data);
	}

	return 0;
}

static long athome_bt_ioctl(struct file *fil,
				unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct bt_athome_state state;

	switch (cmd) {
	case BT_ATHOME_IOCTL_GETSTATE:
		memset(&state, 0, sizeof(state));
		athome_bt_get_state(&state);
		if (copy_to_user((void*)arg, &state, sizeof(state)))
			ret = -EFAULT;
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static unsigned int athome_bt_poll(struct file *file,
					struct poll_table_struct *wait)
{
	poll_wait(file, &usr_wait_q, wait);
	if (!down_trylock(&usr_rx)) {
		up(&usr_rx);
		return POLLIN | POLLRDNORM;
	}
	return 0;
}

static int athome_bt_release(struct inode *inode, struct file *file)
{
	struct sk_buff *skb;

	while (!skb_queue_empty(&usr_data)) {
		skb = skb_dequeue(&usr_data);
		if (skb)
			kfree_skb(skb);
	}
	while (!down_trylock(&usr_rx));

	atomic_set(&num_usr_msgs, 0);
	atomic_set(&usr_opened, 0);

	return 0;
}

bool athome_bt_usr_in_bind_mode(void)
{
	return atomic_read(&in_bind_mode);
}

static const struct file_operations athome_bt_fops =
{
	.read = athome_bt_read,
	.write = athome_bt_write,
	.open = athome_bt_open,
	.poll = athome_bt_poll,
	.unlocked_ioctl = athome_bt_ioctl,
	.compat_ioctl = athome_bt_ioctl,
	.release = athome_bt_release
};

static struct miscdevice mdev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aah_remote",
	.fops = &athome_bt_fops
};

int athome_bt_user_init(void)
{
	return misc_register(&mdev);
}

void athome_bt_user_deinit(void)
{
	misc_deregister(&mdev);
}

