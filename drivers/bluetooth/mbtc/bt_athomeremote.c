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

#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/skbuff.h>
#include <linux/semaphore.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/random.h>
#include <stdarg.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include "bt_athomeremote.h"
#include "bt_athomeremote_priv.h"
#include <linux/input.h>
#include <linux/input/mt.h>



/* basic driver limits */
#define ATHOME_MAX_FINGERS		3

/* BT helpers */
#define ACL_CONN_MASK			0x0FFF
#define L2CAP_CHAN_ATHOME_BIT		0x0080
#define EVT_PACKET_LEN			270

/*
	Logging settings - keep in mind that enabling these may slow the stack
	down enough to cause timouts in things like re-encryption.
*/
#define LOG_ONLY_OURS			1
#define LOG_BT_CMDS			0
#define LOG_BT_EVTS			0
#define LOG_BT_ACL			0
#define LOG_INPUT_EVENTS		0
#define LOG_BT_SEM			0
#define LOG_DISCOVERY			0
#define LOG_MODESWITCH			1

/* Protocol-related defines */
#define ENCR_RND_NUM			0x474A204143204744ULL
#define ENCR_DIV			0x6F67
#define MIN_PROTO_VERSION		0x00010000
#define CONNECT_TIMEOUT			5000 /* try for 5 seconds to connect */
#define ENCRYPT_DAT_TIMEOUT		1000 /* 1 sec for encrypt data echo */
#define ENCRYPT_TIMEOUT			1000 /* 1 sec for encryption */

/* Per-remote states */
#define CONN_STATE_INVALID		0 /* not a connection */
#define CONN_STATE_JUST_ESTABLISHED	1 /* it just happened... */
#define CONN_STATE_BINDING		2 /* new remote binding */
#define CONN_STATE_ENCRYPT_SETUP	3 /* encr entropy exchange started */
#define CONN_STATE_ENCRYPTING		4 /* in process or [re]encrypting */
#define CONN_STATE_REENCRYPT_SETUP	5 /* encr entropy exchange started */
#define CONN_STATE_REENCRYPTING		6 /* in process or [re]encrypting */
#define CONN_STATE_DATA			7 /* ready to exchange data */
#define CONN_STATE_DISCONNECT		8 /* disconnect please */
#define CONN_STATE_TEARDOWN		9 /* no longer valid, being torn down */

/* Userspace-facing defines */
#define MAX_USERSPACE_MSGS		128

/* marvell uses this like a command */
#define PKT_MARVELL			0xFE

/* connection params indexed by ATHOME_MODE_* */
static const uint16_t mode_conn_intervals[] = {80, 40, 6};
static const uint16_t mode_slave_latencies[] = {50, 10, 2};
static const uint16_t mode_svc_timeouts[] = {3000, 200, 20};

struct athome_bt_known_remote {
	struct athome_bt_known_remote *next;
	uint8_t bind_mode;
	uint8_t MAC[AAH_BT_MAC_SZ];
	uint8_t LTK[AAH_BT_LTK_SZ];
};

struct athome_bt_conn {
	/* BT state */
	uint16_t handle;
	uint8_t state, pwr, next_pwr;
	uint8_t gone, timeout, outstanding;
	bool in_modeswitch;
	uint8_t MAC[AAH_BT_MAC_SZ];
	uint8_t LTK[AAH_BT_LTK_SZ];
	struct timer_list timer;
	union {
		uint8_t entropy[AAH_BT_ENTROPY_SZ];
	} data;

	/* input state */
	struct input_dev *idev;
	uint8_t fingers_down;
	char uniq[16];

	/* stats */
	uint64_t last_time;
	struct athome_bt_stats stats;
};


/* virtualization state */
static void *drv_priv = NULL;
static struct semaphore send_sem = __SEMAPHORE_INITIALIZER(send_sem, 1);


/* stack state */
#define INVALID			0xFFFF
static struct sk_buff_head rx_evt;
static struct sk_buff_head rx_dat;
static DECLARE_WAIT_QUEUE_HEAD(rx_wait); /* wait to get evt/data/chg */
static atomic_t state_chg = ATOMIC_INIT(0);
static atomic_t in_shutdown = ATOMIC_INIT(0);
static DEFINE_SEMAPHORE(tx_wait); /* wait to send data */
static struct task_struct *aah_thread = NULL;
static struct semaphore cmd_wait = /* wait for command complete/status */
					__SEMAPHORE_INITIALIZER(cmd_wait, 0);
static struct athome_bt_known_remote *known = NULL;

/* userspace-related */
static struct semaphore usr_rx = /* num msgs avail to read by user */
			__SEMAPHORE_INITIALIZER(usr_rx, 0);
static struct sk_buff_head usr_data;
static atomic_t usr_opened = ATOMIC_INIT(0);
static atomic_t in_bind_mode = ATOMIC_INIT(0);
/*
	Please do not ever use this value for anything. It is only here for
	queue length limiting. it is *NOT* updated atomically with the wait
	queue or the semaphore, and if you use it, you might get a very close
	but maybe not quite right value and end up blocked forever.
*/
static atomic_t num_usr_msgs = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(usr_wait_q);

static void aah_bt_usr_enqueue(uint8_t typ, const void* data, unsigned len);

static ssize_t aah_bt_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t aah_bt_write(struct file *, const char __user *, size_t,
								loff_t *);
static int aah_bt_open(struct inode *, struct file *);
static unsigned int aah_bt_poll(struct file *file,
					struct poll_table_struct *wait);
static int aah_bt_release(struct inode *, struct file *);

static long aah_bt_ioctl(struct file *, unsigned int, unsigned long);

static const struct file_operations aah_bt_fops =
{
	.read = aah_bt_read,
	.write = aah_bt_write,
	.open = aah_bt_open,
	.poll = aah_bt_poll,
	.unlocked_ioctl = aah_bt_ioctl,
	.compat_ioctl = aah_bt_ioctl,
	.release = aah_bt_release
};

static struct miscdevice mdev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aah_remote",
	.fops = &aah_bt_fops
};

/* protects the following data */
static DEFINE_SPINLOCK(stack_state_lock);
static uint16_t disconnH = 0;
static uint16_t inflight_ogf = INVALID;
static uint16_t inflight_ocf = INVALID;
static struct athome_bt_conn conns[ATHOME_RMT_MAX_CONNS] = {{0,},};
static uint8_t nconns = 0;
static uint8_t *cmd_rsp_buf;
static bool devices_exist = 0;


static int athome_bt_data_rx(int which, uint8_t *in_data, uint32_t len);

/* input functions */
static int athome_bt_input_init(void);
static void athome_bt_input_deinit(void);
static void athome_bt_input_send_touch(int which, int pointerIdx,
						uint16_t x, uint16_t y);
static void athome_bt_input_send_buttons(int which, uint32_t mask);
static void athome_bt_input_frame(int which);


/* event masks for things we need */
  /* disconnect, encr_change, encr_refresh */
static const uint64_t evtsNeeded = 0x0000800000000090ULL;
  /* LE meta event*/
static const uint64_t evtsLE = 0x2000000000000000ULL;

/* unaligned and endian-agnostic access */
static uint16_t read16(void *ptr)
{
	uint8_t *p = (uint8_t*)ptr;
	return (((uint16_t)p[1]) << 8) | p[0];
}

static uint32_t read32(void *ptr)
{
	uint8_t *p = (uint8_t*)ptr;
	return (((uint32_t)p[3]) << 24) | (((uint32_t)p[2]) << 16) |
		(((uint32_t)p[1]) << 8) | p[0];
}

static uint64_t read64(void *ptr)
{
	uint8_t *p = (uint8_t*)ptr;
	return (((uint64_t)p[7]) << 56) | (((uint64_t)p[6]) << 48) |
		(((uint64_t)p[5]) << 40) | (((uint64_t)p[4]) << 32) |
		(((uint64_t)p[3]) << 24) | (((uint64_t)p[2]) << 16) |
		(((uint64_t)p[1]) << 8) | p[0];
}

static void write16(void *ptr, uint16_t val)
{
	uint8_t *p = (uint8_t*)ptr;

	p[0] = val;
	p[1] = val >> 8;
}

static void write64(void *ptr, uint64_t val)
{
	uint8_t *p = (uint8_t*)ptr;

	p[0] = val;
	p[1] = val >> 8;
	p[2] = val >> 16;
	p[3] = val >> 24;
	p[4] = val >> 32;
	p[5] = val >> 40;
	p[6] = val >> 48;
	p[7] = val >> 56;
}

/* access to packed non-typedefed structures with autoincrement */
static uint8_t get8(uint8_t **parPP)
{
	uint8_t ret = **parPP;
	(*parPP)++;

	return ret;
}

static uint16_t get16(uint8_t **parPP)
{
	uint16_t ret = read16(*parPP);
	(*parPP) += 2;

	return ret;
}

static void put8(uint8_t **parPP, uint8_t v)
{
	*(*parPP)++ = v;
}

static void put16(uint8_t **parPP, uint16_t v)
{
	write16(*parPP, v);
	(*parPP) += 2;
}

static void put64(uint8_t **parPP, uint64_t v)
{
	write64(*parPP, v);
	(*parPP) += 8;
}


/* returns <0 if not found, call only with state lock held */
static int athome_bt_find_connection(uint16_t handle)
{
	int i;

	for(i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
		if (conns[i].state != CONN_STATE_INVALID &&
						!conns[i].gone &&
						conns[i].handle == handle)
			return i;
	}
	return -1;
}

static uint64_t get_time(void)
{
	struct timespec t;
	uint64_t ret;

	getnstimeofday(&t);
	ret = t.tv_sec;
	ret *= NSEC_PER_SEC;
	ret += t.tv_nsec;

	return ret;
}

/* call with spinlock held only */
static void athome_bt_update_time_stats(unsigned which)
{
	uint64_t time = get_time();
	uint64_t spent = time - conns[which].last_time;

	conns[which].stats.mode_times[conns[which].pwr] += spent;
	conns[which].last_time = time;
}

static int athome_bt_filter_disconn(void *dataP)
{
	struct hci_ev_disconn_complete *evt = dataP;
	struct bt_athome_disconnected de;
	int i, ret = 0;
	unsigned long flags;
	uint16_t handle;

	handle = read16(&evt->handle);

	spin_lock_irqsave(&stack_state_lock, flags);
	i = athome_bt_find_connection(handle);
	if (i >= 0) {
		conns[i].gone = 1;
		if (conns[i].outstanding)
			up(&tx_wait);
		ret = 1;
		athome_bt_update_time_stats(i);
		memcpy(&de.MAC, conns[i].MAC, sizeof(de.MAC));
		memcpy(&de.stats, &conns[i].stats, sizeof(de.stats));
		aah_bt_usr_enqueue(BT_ATHOME_EVT_DISCONNECTED, &de,
								sizeof(de));
	} else if (handle == disconnH)
		ret = 1;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	return ret;
}

static int athome_bt_filter_encr_change(void *dataP)
{
	struct hci_ev_encrypt_change *evt = dataP;
	int ret;
	unsigned long flags;
	uint16_t handle;

	handle = read16(&evt->handle);
	spin_lock_irqsave(&stack_state_lock, flags);
	ret = athome_bt_find_connection(handle) >= 0;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	return ret;
}

static int athome_bt_filter_encr_refresh(void *dataP)
{
	struct hci_ev_encrypt_refresh *evt = dataP;
	int ret;
	unsigned long flags;
	uint16_t handle;

	handle = read16(&evt->handle);
	spin_lock_irqsave(&stack_state_lock, flags);
	ret = athome_bt_find_connection(handle) >= 0;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	return ret;
}

static void athome_bt_filter_ftr_page_0(uint8_t *features)
{
	if (features[4] & 0x40) {
		aahlog("features[0]: clearing 4,6 - LE supported\n");
		features[4] &= ~0x40;
	}
	if (features[6] & 0x02) {
		aahlog("features[0]: clearing 6,1 - EDR/LE simult\n");
		features[4] &= ~0x02;
	}
}

static void athome_bt_reset(void)
{
	unsigned long flags;
	int i;

	aahlog("resetting\n");

	if (aah_thread)
		aahlog("athome_bt_reset called while thread exists!\n");

	while(!down_trylock(&send_sem));

	if (!skb_queue_empty(&rx_evt))
		kfree_skb(skb_dequeue(&rx_evt));

	if (!skb_queue_empty(&rx_dat))
		kfree_skb(skb_dequeue(&rx_dat));

	wake_up_interruptible(&rx_wait);

	atomic_set(&state_chg, 0);

	while(!down_trylock(&tx_wait));
	up(&tx_wait);

	while(!down_trylock(&cmd_wait));

	spin_lock_irqsave(&stack_state_lock, flags);
	inflight_ogf = INVALID;
	inflight_ocf = INVALID;
	disconnH = 0;
	nconns = 0;
	cmd_rsp_buf = NULL;
	for(i = 0; i < ATHOME_RMT_MAX_CONNS; i++)
		conns[i].state = CONN_STATE_INVALID;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	aahlog("reset complete\n");
}

static void athome_bt_shutdown(void)
{
	aahlog("shutting down thread\n");
	if (aah_thread) {
		atomic_set(&in_shutdown, 1);
		atomic_set(&state_chg, 1); /* force wake */
		up(&cmd_wait);	/* make sure we're not waiting for a reply */
		wake_up_interruptible(&rx_wait);
		kthread_stop(aah_thread);
		aahlog("thread shut down\n");
		aah_thread = NULL;
	}
}

static int athome_bt_filter_cmd_complete(uint8_t *rx_buf, uint32_t len,
						bool* enq_if_oursP)
{
	struct hci_ev_cmd_complete *evt =
			(struct hci_ev_cmd_complete *)
			(rx_buf + HCI_EVENT_HDR_SIZE);
	uint16_t opcode = read16(&evt->opcode);
	uint16_t ogf = hci_opcode_ogf(opcode);
	uint16_t ocf = hci_opcode_ocf(opcode);
	int i, forus = 0;
	unsigned long flags;

	spin_lock_irqsave(&stack_state_lock, flags);
	if (ogf == inflight_ogf && ocf == inflight_ocf) {

		/* definitely a reply for us */
		forus = 1;
		inflight_ogf = INVALID;
		inflight_ocf = INVALID;
		memcpy(cmd_rsp_buf, rx_buf, len);
		up(&cmd_wait);
		*enq_if_oursP = 0;
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);

	if (ogf == HCI_OGF_Controller_And_Baseband && ocf == HCI_Reset) {

		/*
			any and all command credits we have/had go out the
			window after a reset, and we instead have one. This
			code assures our semaphore acts that way too.
		*/
		while(!down_trylock(&send_sem));
		up(&send_sem);
		if (LOG_BT_SEM)
			aahlog("resetting bt sem to 1 on reset\n");

	 } else if (ogf == HCI_OGF_Controller_And_Baseband &&
		ocf == HCI_Read_LE_Host_Support && !forus) {

		struct read_le_host_supported_reply *repl =
			(struct read_le_host_supported_reply*)(evt + 1);

		if (repl->le)
			aahlog("host_support: clearing le\n");
		if (repl->simul)
			aahlog("host_support: clearing simul\n");
		repl->le = 0;
		repl->simul = 0;

	} else if (ogf == HCI_OGF_Informational) {

		if (ocf == HCI_Read_Local_Version_Information && !forus) {
			struct read_local_version_reply *repl =
				(struct read_local_version_reply*)(evt + 1);

			if (!repl->status) {

				if (repl->hci_version >= BT_HCI_VERSION_4) {
					aahlog("Down BT HCI version from %d\n",
						repl->hci_version);
					repl->hci_version = BT_HCI_VERSION_3;
				}
				if (repl->lmp_version >= BT_LMP_VERSION_4) {
					aahlog("Down bt LMP version from %d\n",
						repl->lmp_version);
					repl->lmp_version = BT_LMP_VERSION_3;
				}
			}
		}
		if (ocf == HCI_Read_Local_Supported_Commands && !forus) {
			struct read_support_cmds_reply *repl =
				(struct read_support_cmds_reply*)(evt + 1);

			if (repl->commands[24] & 0x60)
				aahlog("supported commands: ~LE in byte 24\n");
			repl->commands[24] &= ~0x60;

			/* these are LE commands */
			for (i = 25; i <= 28; i++) {
				if (repl->commands[i])
					aahlog("supported commands: "
						"clearing byte %d\n", i);
				repl->commands[i] = 0;
			}
		}
		if (ocf == HCI_Read_Local_Supported_Features && !forus) {
			struct read_support_ftrs_reply *repl =
				(struct read_support_ftrs_reply*)(evt + 1);

			athome_bt_filter_ftr_page_0(repl->features);
		}
		if (ocf == HCI_Read_Local_Supported_Extended_Features &&
			!forus) {
			struct read_support_ext_ftrs_reply *repl =
				(struct read_support_ext_ftrs_reply*)(evt + 1);

			if (repl->page_nr == 0)
				athome_bt_filter_ftr_page_0(repl->features);
			else if (repl->page_nr == 1) {

				if (repl->features[0] & 0x02) {
					aahlog("features[1]: clearing 0,1 - "
						"LE supported\n");
					repl->features[4] &= ~0x02;
				}
				if (repl->features[0] & 0x04) {
					aahlog("features[1]: clearing 0,2 - "
						"EDR/LE simult\n");
					repl->features[4] &= ~0x04;
				}
			}
		}
		if (ocf == HCI_Read_Buffer_Size) {
			struct hci_rp_read_buffer_size *rsp =
				(struct hci_rp_read_buffer_size*)(evt + 1);
			uint16_t pkts = read16(&rsp->acl_max_pkt);

			aahlog("fixing up acl packet number from %d\n", pkts);
			if (pkts)
				write16(&rsp->acl_max_pkt, pkts - 1);
		}
	} else if (ogf == HCI_OGF_LE && !forus) {
		aahlog("LE command complete not for us (ogf=0x%x ocf=0x%x):",
			ogf, ocf);
		for (i = 0; i < ((uint8_t*)rx_buf)[-1]; i++)
			aahlog_continue(" %02X", ((uint8_t*)rx_buf)[i]);
		aahlog_continue("\n");
	}

	return forus;
}

static int athome_bt_filter_cmd_status(uint8_t *rx_buf, uint32_t len,
						bool* enq_if_oursP)
{
	struct hci_ev_cmd_status *evt =
			(struct hci_ev_cmd_status *)
			(rx_buf + HCI_EVENT_HDR_SIZE);
	uint16_t opcode = read16(&evt->opcode);
	uint16_t ogf = hci_opcode_ogf(opcode);
	uint16_t ocf = hci_opcode_ocf(opcode);
	bool forus = 0;
	unsigned long i, flags;

	spin_lock_irqsave(&stack_state_lock, flags);
	if (ogf == inflight_ogf && ocf == inflight_ocf) {

		/* definitely a reply for us */
		forus = 1;
		inflight_ogf = INVALID;
		inflight_ocf = INVALID;
		memcpy(cmd_rsp_buf, rx_buf, len);
		up(&cmd_wait);
		*enq_if_oursP = 0;
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);

	if (ogf == HCI_OGF_LE && !forus) {
		aahlog("LE command status not for us (ogf=0x%x ocf=0x%x):",
			ogf, ocf);
		for (i = 0; i < ((uint8_t*)rx_buf)[-1]; i++)
			aahlog_continue(" %02X", ((uint8_t*)rx_buf)[i]);
		aahlog_continue("\n");
	}

	return forus;
}

static int athome_bt_filter_num_comp_pkt(void *dataP, uint32_t *rx_len)
{
	struct hci_ev_num_comp_pkts *evt = dataP;
	uint16_t *dataIn = (uint16_t*)(evt + 1);
	uint16_t *dataOut = dataIn;
	int num_skipped = 0;
	int i, j, new_creds = 0;
	unsigned long flags;

	spin_lock_irqsave(&stack_state_lock, flags);
	for (i = 0; i < evt->num_hndl; i++, dataIn += 2) {
		uint16_t conn = read16(dataIn + 0);
		uint16_t pkts = read16(dataIn + 1);

		j = athome_bt_find_connection(conn);

		if (j >= 0) {
			new_creds += pkts;
			num_skipped++;
			if (!conns[j].outstanding)
				aahlog("unexpected send on conn %d\n", j);
			conns[j].outstanding = 0;
		} else {
			write16(dataOut++, conn);
			write16(dataOut++, pkts);
		}
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);

	if (LOG_BT_EVTS)
		aahlog("got %u data credits back, hid %u handles\n",
			new_creds, num_skipped);
	while (new_creds--)
		up(&tx_wait);

	rx_len -= num_skipped * 4;
	evt->num_hndl -= num_skipped;

	return !evt->num_hndl;
}


static void athome_bt_send_to_user(uint32_t pkt_type, uint8_t *data, uint32_t len)
{
	smp_rmb();
	if (!drv_priv)
		aahlog("Attempt to send to user with no private data\n");
	else
		athome_bt_pkt_to_user(drv_priv, pkt_type, data, len);
}

static void athome_bt_send_to_chip(uint32_t pkt_type, uint8_t *data,
								uint32_t len)
{
	smp_rmb();
	if (!drv_priv)
		aahlog("Attempt to send to chip with no private data\n");
	else
		athome_bt_pkt_to_chip(drv_priv, pkt_type, data, len);
}

/* send command and then wait for complete or status event. nonzero -> quit */
static int __attribute__((warn_unused_result))
		athome_bt_simple_cmd(uint32_t ogf, uint32_t ocf, uint8_t plen,
					uint8_t *dataIn, unsigned outLen,
					uint8_t *dataOut)
{
	uint8_t cmd[255 + HCI_COMMAND_HDR_SIZE];
	uint8_t rsp[255 + HCI_COMMAND_HDR_SIZE];
	struct hci_command_hdr *hdr = (struct hci_command_hdr*)cmd;
	unsigned long flags;
	unsigned i;

	for (i = 0; i < plen; i++)
		cmd[i + HCI_COMMAND_HDR_SIZE] = dataIn[i];

	hdr->plen = plen;
	write16(&hdr->opcode, hci_opcode_pack(ogf, ocf));

	spin_lock_irqsave(&stack_state_lock, flags);
	if (inflight_ogf != INVALID || inflight_ocf != INVALID)
		aahlog("cmd enqueue while prev not invalid\n");
	inflight_ogf = ogf;
	inflight_ocf = ocf;
	cmd_rsp_buf = rsp;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	athome_bt_send_to_chip(HCI_COMMAND_PKT, cmd,
						plen + HCI_COMMAND_HDR_SIZE);
	down(&cmd_wait);

	for (i = 0; i < outLen; i++)
		dataOut[i] = rsp[i];

	return atomic_read(&in_shutdown);
}

static int athome_bt_disconnect(uint16_t handle)
{
	uint8_t buf[EVT_PACKET_LEN];
	uint8_t *parP = buf;


	put16(&parP, handle);
	put8(&parP, 0x13);	/* reason = user request */
	return athome_bt_simple_cmd(HCI_OGF_Link_Control, HCI_CMD_Disconnect,
				parP - buf, buf, 0, NULL);
}

/* something timed out - find out what and why */
static void athome_bt_timeout(unsigned long which)
{
	unsigned long flags;

	if (which == ATHOME_RMT_MAX_CONNS) {

		/* shared timeout - just wake the thread */

	} else {

		/* per-connection timeout - marck which & wake the thread */
		spin_lock_irqsave(&stack_state_lock, flags);
		conns[which].timeout = 1;
		spin_unlock_irqrestore(&stack_state_lock, flags);
	}
	atomic_set(&state_chg, 1);
	wake_up_interruptible(&rx_wait);
}

/* only to be called with state lock held */
static struct athome_bt_known_remote *aah_bt_find_known(const uint8_t* MAC,
				struct athome_bt_known_remote ** prevP)
{
	struct athome_bt_known_remote *cur = known, *prev = NULL;

	while (cur && memcmp(cur->MAC, MAC, sizeof(cur->MAC))) {
		prev = cur;
		cur = cur->next;
	}

	if (prevP) *prevP = prev;
	return cur;
}

/* process adv response, see if anything of use is in it, if so, what? */
static bool athome_bt_handle_scan_evt(const uint8_t *buf, uint8_t *macP)
{
	static const char *advTypes[] = {"ADV_IND", "ADV_DIRECT_IND",
					"ADV_SCAN_IND", "ADV_NONCONN_IND",
					"SCAN_RSP"};
	uint8_t num;
	bool try_connect = false;
	unsigned long flags;


	buf += sizeof(struct hci_ev_le_meta);
	num = *buf++;

	while (num--) {

		bool needflags = 1, needmanuf = 1;
		uint8_t slen = 0;
		const uint8_t *snum = NULL; /* only devices in bind mode */
		uint8_t evtType, addrType, mac[AAH_BT_MAC_SZ];
		uint8_t len, j;
		const uint8_t *data, *end;
		struct athome_bt_known_remote *item;
		bool found = false, want_bind = false, can_connect = false;
		uint8_t ver[4] = {0,};
		int8_t rssi;

		evtType = *buf++;
		addrType = *buf++;
		memcpy(mac, buf, sizeof(mac));
		buf += sizeof(mac);
		len = *buf++;
		data = buf;
		buf += len;
		rssi = *buf++;

		if (LOG_DISCOVERY) {
			aahlog("DEV %02X:%02X:%02X:%02X:%02X:%02X(%c) "
				"%s: RSSI %d (%u):", mac[5], mac[4],
				mac[3], mac[2], mac[1], mac[0],
				addrType ? 'R' : 'P',
				advTypes[evtType], rssi, len);
			for (j = 0; j < len; j++)
				aahlog_continue(" %02X", data[j]);
			aahlog_continue("\n");
			aahlog("  DATA:");
		}
		end = data + len;

		while (data < end) {

			uint8_t t, L = *data++;
			if (end - data < L) {
				if (LOG_DISCOVERY)
					aahlog_continue("ERR(want %d have %d)",
								L, end - data);
				break;
			}
			if (!L) {
				if (LOG_DISCOVERY)
					aahlog_continue("ERR(ZLP)");
				continue;
			}

			t = *data++;
			L--;
			switch (t) {
			case 1: //flags
				if (!L)
					break;
				t = *data;

				/* our flags? */
				if (needflags && t == 0x06)
					needflags = 0;

				if (!LOG_DISCOVERY)
					break;

				aahlog_continue(" FLAGS(");
				if (t & 0x01)
					aahlog_continue(" LimitedDisc");
				if (t & 0x02)
					aahlog_continue(" GeneralDisc");
				if (t & 0x04)
					aahlog_continue(" NoEDR");
				if (t & 0x08)
					aahlog_continue(" SimulEDRcontroller");
				if (t & 0x10)
					aahlog_continue(" SimulEDR_Host");
				aahlog_continue(" )");
				break;

			case 2:	//16-bit uuids (more)
			case 3:	//16-bit uuids (complete)
				if (!LOG_DISCOVERY)
					break;

				aahlog_continue(" UUID16(%scomplete)(",
						t == 2 ? "in" : "");
				while (L >= 2) {
					aahlog_continue(" %02X%02X",
							data[1], data[0]);
					data += 2;
					L -= 2;
				}
				aahlog_continue(" )");
				break;

			case 4:	//32-bit uuids (more)
			case 5:	//32-bit uuids (complete)
				if (!LOG_DISCOVERY)
					break;

				aahlog_continue(" UUID32(%scomplete)(",
						t == 4 ? "in" : "");
				while (L >= 4) {
					aahlog_continue(" %02X%02X%02X%02X",
							data[3], data[2],
							data[1], data[0]);
					data += 4;
					L -= 4;
				}
				aahlog_continue(" )");
				break;

			case 6:	//128-bit uuids (more)
			case 7:	//128-bit uuids (complete)
				if (!LOG_DISCOVERY)
					break;

				aahlog_continue(" UUID(%scomplete)(",
						t == 6 ? "in" : "");
				while (L >= 16) {
					aahlog_continue(" ");
					for (j = 0; j < 16; j++)
						aahlog_continue("%02X",
								data[15 - j]);
					data += 16;
					L -= 16;
				}
				aahlog_continue(" )");
				break;

			case 8:	//shortened name
			case 9:	//complete
				if (!LOG_DISCOVERY)
					break;

				aahlog_continue(" NAME(%s)(",
						t == 8 ?
							"shortened" :
							"complete");
				while (L--)
					aahlog_continue("%c", *data++);
				aahlog_continue(")");
				break;

			case 10: //tx power
				if (!LOG_DISCOVERY)
					break;

				aahlog_continue(" TXPOWER(");
				while (L--)
					aahlog_continue("%d", (int8_t)*data++);
				aahlog_continue(")");
				break;

			case 255: //custom
				if (needmanuf && L >= 10) do {

					uint32_t ver_val;

					if (data[0] != 'G')
						break;
					if (data[1] != 'o')
						break;
					if (data[2] != 'o')
						break;
					if (data[3] != 'g')
						break;
					if (data[4] != 'l')
						break;
					if (data[5] != 'e')
						break;

					memcpy(ver, data + 6, sizeof(ver));
					ver_val = ver[0];
					ver_val = (ver_val << 8) | ver[1];
					ver_val = (ver_val << 8) | ver[2];
					ver_val = (ver_val << 8) | ver[3];

					if (LOG_DISCOVERY)
						aahlog_continue(" AAH(v%08X)",
								ver_val);
					if (ver_val >= MIN_PROTO_VERSION)
						needmanuf = 0;

					if (L == 10)
						break;

					slen = L - 10;
					snum = data + 10;
					if (LOG_DISCOVERY) {

						aahlog_continue(" SN(");
						for(j = 0 ;j < slen; j++)
							aahlog_continue("%c",
								snum[j]);
						aahlog_continue(")");
					}
				} while (0);

				if (!LOG_DISCOVERY)
					break;

				aahlog_continue(" MANUF_DATA(");
				while (L--)
					aahlog_continue(" %02X", *data++);
				aahlog_continue(" )");
				break;

			default:
				if (!LOG_DISCOVERY)
					break;

				aahlog_continue(" UNKNOWN(%u)(", t);
				while (L--)
					aahlog_continue(" %02X", *data++);
				aahlog_continue(" )");
				break;
			}
			/* consume all unconsumed chunk data */
			data += L;
		}
		if (LOG_DISCOVERY)
			aahlog_continue("\n");

		/* It may be a device we know about. Check that. */
		spin_lock_irqsave(&stack_state_lock, flags);
		item = aah_bt_find_known(mac, NULL);
		if (item) {
			found = true;
			want_bind = item->bind_mode;
		}
		spin_unlock_irqrestore(&stack_state_lock, flags);

		if (!needmanuf && !needflags && addrType &&
					evtType == SCAN_EVT_ADV_IND) {
			if (found) {
				if (snum && want_bind) {

					if (LOG_DISCOVERY)
						aahlog(" -> wanted unbound "
							"remote found\n");
					can_connect = true;
				} else if (snum) {

					if (LOG_DISCOVERY)
						aahlog(" -> bind-like adv from"
							" slave not expected "
							"in bind mode\n");
				} else if (want_bind) {

					if (LOG_DISCOVERY)
						aahlog(" -> non-bind-like adv "
							"from slave expected "
							"in bind mode\n");
				} else {
					if (LOG_DISCOVERY)
						aahlog(" -> known half bound "
							"remote found\n");
					can_connect = true;
				}
			} else if (snum && atomic_read(&in_bind_mode)) {

				uint8_t usr[64] = {0,};
				struct bt_athome_discovered_v1 *dd =
					(struct bt_athome_discovered_v1*)usr;

				if (LOG_DISCOVERY)
					aahlog(" -> advertising & unbound\n");

				/* prepare event & send it to userspace */
				memcpy(dd->MAC, mac, sizeof(dd->MAC));
				memcpy(dd->ver, ver, sizeof(dd->ver));
				memcpy(dd->ser_num, snum, slen);
				dd->RSSI = rssi;

				aah_bt_usr_enqueue(BT_ATHOME_EVT_DISCOVERED_V1,
						usr, sizeof(*dd) + slen);

			} else if (LOG_DISCOVERY) {

				aahlog(" -> strange adv {snum=%d "
					"slen=%d found=%d want_bind=%d}\n",
					!!snum, slen, found,
					want_bind);
			}
		} else if (evtType == SCAN_EVT_ADV_DIRECT_IND) {
			if (!found) {
				if (LOG_DISCOVERY)
					aahlog(" -> unexpected direct adv "
								"not found\n");
			} else if (want_bind) {
				if (LOG_DISCOVERY)
					aahlog(" -> unexpected direct adv "
								"want bind\n");
			} else {
				if (LOG_DISCOVERY)
					aahlog(" -> wanted bound remote\n");
				can_connect = true;
			}
		}

		/* Do not connect to it if we are already connected */
		if (can_connect) {
			int i;

			spin_lock_irqsave(&stack_state_lock, flags);
			for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
				if (conns[i].state == CONN_STATE_INVALID)
					continue;

				if (conns[i].gone)
					continue;

				if (memcmp(conns[i].MAC, mac, sizeof(mac)))
					continue;

				can_connect = 0;
				break;
			}
			spin_unlock_irqrestore(&stack_state_lock, flags);
			if (i != ATHOME_RMT_MAX_CONNS && LOG_DISCOVERY)
					aahlog(" -> already conn as %d\n", i);
			if (can_connect) {
				try_connect = true;
				memcpy(macP, mac, sizeof(mac));
			}
		}

	}

	return try_connect;
}

static bool athome_bt_send_data(uint8_t *MAC, uint8_t typ,
			const void *dataP, uint8_t data_sz, bool for_user)
{

	uint8_t buf[27 + HCI_ACL_HDR_SIZE] = {0, };
	struct hci_acl_hdr *h = (struct hci_acl_hdr*)buf;
	uint8_t *packet = (uint8_t*)(h + 1);
	const uint8_t *data = (const uint8_t*)dataP;
	uint16_t handle;
	unsigned long flags;
	unsigned i, which;

	if (data_sz > 26) {
		aahlog("trying to send large packet (%d)\n", data_sz);
		return false;
	}

	/* split data as per spec */
	for (i = 0; i < 2 && i < data_sz; i++)
		packet[i] = data[i];
	for (i = 2; i < data_sz; i++)
		packet[i + 1] = data[i];

	/* minimums must be met */
	if (data_sz < 2)
		data_sz = 2;

	/* set our packet type */
	packet[2] = typ | L2CAP_CHAN_ATHOME_BIT;

	/* wait till we can send */
	down(&tx_wait);

	/* get handle info and record we're sending */
	spin_lock_irqsave(&stack_state_lock, flags);
	for (which = 0; which < ATHOME_RMT_MAX_CONNS; which++) {

		if (memcmp(MAC, conns[which].MAC, sizeof(conns[which].MAC)))
			continue;

		if (for_user && conns[which].state == CONN_STATE_ENCRYPT_SETUP)
			continue;

		if (conns[which].state == CONN_STATE_JUST_ESTABLISHED ||
			conns[which].state == CONN_STATE_INVALID ||
			conns[which].state == CONN_STATE_ENCRYPTING ||
			conns[which].state == CONN_STATE_REENCRYPTING ||
			conns[which].state == CONN_STATE_DISCONNECT ||
			conns[which].state == CONN_STATE_TEARDOWN ||
			conns[which].gone)
			continue;

		/* only some packets allowed in binding mode */
		/*
			XXX: TODO: once binding is proper, stop allowing
			set_param command here
		*/
		if (conns[which].state == CONN_STATE_BINDING &&
					typ != ATHOME_PKT_TX_ACK &&
					typ != ATHOME_PKT_TX_SET_PARAM &&
					typ != ATHOME_PKT_TX_ENC)
			continue;

		break;
	}
	if (which == ATHOME_RMT_MAX_CONNS) {

		spin_unlock_irqrestore(&stack_state_lock, flags);
		up(&tx_wait);

		/* cannot send - we drop it */
		aahlog("dropping impossible send request\n");
		return false;
	}

	if (conns[which].outstanding)
		aahlog("send while outstanding on %d\n", i);
	conns[which].outstanding = 1;
	handle = conns[which].handle;
	conns[which].stats.pkts_tx++;
	conns[which].stats.bytes_tx += data_sz;
	if (typ == ATHOME_PKT_TX_NFC)
		conns[which].stats.nfc_tx++;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	/* craft ACL header */
	write16(&h->handle, handle);
	write16(&h->dlen, data_sz + 1);

	/* let the chip have it */
	athome_bt_send_to_chip(HCI_ACLDATA_PKT, buf,
					data_sz + 1 + HCI_ACL_HDR_SIZE);

	return true;
}

static int athome_bt_do_encrypt(int which, const uint8_t* LTK, bool initial)
{
	uint8_t buf[EVT_PACKET_LEN], *parP, k[AAH_BT_LTK_SZ];
	uint8_t *evt_data = buf + HCI_EVENT_HDR_SIZE;
	struct hci_ev_cmd_status* stat =
				(struct hci_ev_cmd_status*)evt_data;
	uint8_t *cmd_cmpl_data = evt_data +
				sizeof(struct hci_ev_cmd_complete);
	unsigned long flags;
	unsigned i;

	parP = buf;
	for (i = 0; i < AAH_BT_LTK_SZ; i++)
		put8(&parP, LTK[i]);
	for (i = 0; i < AAH_BT_ENTROPY_SZ; i++)
		put8(&parP, conns[which].data.entropy[i]);
	i = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Encrypt, parP - buf, buf,
							sizeof(buf), buf);
	if (i)
		return i;

	parP = cmd_cmpl_data;
	if (get8(&parP)) {	/* check status */
		aahlog("failed to AES encrypt\n");
		conns[which].state = CONN_STATE_DISCONNECT;
		return 0;
	}

	aahlog("KEY [%d]:", which);
	for(i = 0; i < AAH_BT_LTK_SZ; i++) {
		k[i] = get8(&parP);
		aahlog_continue(" %02X", k[i]);
	}
	aahlog_continue("\n");

	parP = buf;
	put16(&parP, conns[which].handle);
	put64(&parP, ENCR_RND_NUM);
	put16(&parP, ENCR_DIV);
	for (i = 0; i < AAH_BT_LTK_SZ; i++)
		put8(&parP, k[i]);
	i = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Start_Encryption,
				parP - buf, buf,sizeof(buf), buf);
	if (i)
		return i;
	if (stat->status) {
		aahlog("failed to encrypt conection\n");
		conns[which].state = CONN_STATE_DISCONNECT;
		return 0;
	}
	spin_lock_irqsave(&stack_state_lock, flags);
	if (initial && conns[which].state == CONN_STATE_ENCRYPT_SETUP)
		conns[which].state = CONN_STATE_ENCRYPTING;
	else if (!initial && conns[which].state == CONN_STATE_REENCRYPT_SETUP)
		conns[which].state = CONN_STATE_REENCRYPTING;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	return 0;
}

/* pass i < 0 to do a lookup by mac */
static void athome_bt_start_encr(uint8_t* MAC, int i)
{
	struct athome_tx_pkt_enc enc_pkt;
	struct athome_bt_known_remote *K;
	unsigned long flags;
	unsigned j;
	bool initial = false;

	get_random_bytes(&enc_pkt.entropy, sizeof(enc_pkt.entropy));


	spin_lock_irqsave(&stack_state_lock, flags);
	if (i < 0) {
		for(i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
			if (memcmp(conns[i].MAC, MAC, AAH_BT_MAC_SZ) ||
						conns[i].gone)
				continue;

			if (conns[i].state == CONN_STATE_JUST_ESTABLISHED ||
					conns[i].state == CONN_STATE_BINDING) {
				initial = true;
				break;
			}

			if (conns[i].state == CONN_STATE_DATA)
				break;
		}
	} else {

		initial = (conns[i].state == CONN_STATE_JUST_ESTABLISHED ||
					conns[i].state == CONN_STATE_BINDING);
	}
	if (i >= ATHOME_RMT_MAX_CONNS) {
		spin_unlock_irqrestore(&stack_state_lock, flags);
		aahlog("start encr failed - no such connection\n");
		return;
	}

	K = aah_bt_find_known(MAC, NULL);
	if (K)
		memcpy(conns[i].LTK, &K->LTK, sizeof(K->LTK));
	else {
		spin_unlock_irqrestore(&stack_state_lock, flags);
		aahlog("device unknown unexpectedly\n");
		return;
	}

	conns[i].timeout = 0;
	conns[i].state = initial ?
			CONN_STATE_ENCRYPT_SETUP :
			CONN_STATE_REENCRYPT_SETUP;

	for (j = 0; j < AAH_BT_ENTROPY_SZ; j++)
			conns[i].data.entropy[j] = enc_pkt.entropy[j];
	mod_timer((struct timer_list*)&conns[i].timer,
			jiffies + msecs_to_jiffies(ENCRYPT_DAT_TIMEOUT));

	spin_unlock_irqrestore(&stack_state_lock, flags);

	/* send the packet */
	athome_bt_send_data(MAC, ATHOME_PKT_TX_ENC, &enc_pkt, sizeof(enc_pkt),
									false);
}


static int athome_bt_modeswitch(int which, unsigned mode)
{
	const uint16_t intr = mode_conn_intervals[mode];
	const uint16_t slat = mode_slave_latencies[mode];
	const uint16_t svto = mode_svc_timeouts[mode];
	uint8_t buf[EVT_PACKET_LEN];
	uint8_t *parP = buf;
	int ret;
	struct hci_ev_cmd_status* stat =
			(struct hci_ev_cmd_status*)(buf + HCI_EVENT_HDR_SIZE);

	put16(&parP, conns[which].handle);
	put16(&parP, intr); /* min conn intr */
	put16(&parP, intr); /* max conn intr */
	put16(&parP, slat);
	put16(&parP, svto);
	put16(&parP, 20);   /* min durX.62ms */
	put16(&parP, 20);   /* max durX.62ms */

	if (LOG_MODESWITCH)
		aahlog("Starting modeswitch to %d for conn %d\n", mode, which);

	conns[which].in_modeswitch = true;
	ret = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Connection_Update,
		parP - buf, buf, sizeof(buf), buf);
	if (ret)
		return ret;

	if (stat->status)
		aahlog("failed conn update try.\n");

	return 0;
}

/* stack thread */
static int athome_bt_thread(void *unusedData)
{
	static const unsigned clockA[] = {500,250,150,100,75,50,30,20};
	uint8_t buf[EVT_PACKET_LEN];
	uint8_t *evt_data = buf + HCI_EVENT_HDR_SIZE;
	uint8_t *cmd_cmpl_data = evt_data + sizeof(struct hci_ev_cmd_complete);
	struct hci_ev_cmd_status *stat = (struct hci_ev_cmd_status*)evt_data;
	bool is_scanning = 0, is_conecting = 0;
	uint8_t *parP, sta, aclBufNum;
	uint16_t aclBufSz, handle;
	struct timer_list timeout;
	unsigned long flags;
	unsigned i;
	int ret = 0;

	if (!devices_exist) {
		aahlog("devices_created\n");
		if (misc_register(&mdev)) {
			aahlog("failed to register misc device\n");
			ret = -1;
			goto err_return;
		}

		if (athome_bt_input_init()) {
			aahlog("failed to register input device\n");
			ret = -2;
			goto err_deregister_misc;
		}
		devices_exist = true;
	}

	aahlog("host setup\n");
	parP = buf;
	put8(&parP, 1);		//le on
	put8(&parP, 1);		//le simul on
	ret = athome_bt_simple_cmd(HCI_OGF_Controller_And_Baseband,
				HCI_Write_LE_Host_Support, parP - buf,
				buf, sizeof(buf), buf);
	if (ret)
		goto exit_return;
	parP = cmd_cmpl_data;
	if (get8(&parP)) {	/* check status */
		aahlog("failed to set LE support. bailing.\n");
		ret = -3;
		goto err_deregister_input;
	}

	ret = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Read_Buffer_Size, 0, NULL,
				sizeof(buf), buf);
	if (ret)
		goto exit_return;
	parP = cmd_cmpl_data;
	if (get8(&parP)) {	/* check status */
		aahlog("failed to get buffer sizes. bailing. \n");
		ret = -3;
		goto err_deregister_input;
	}
	aclBufSz = get16(&parP);
	aclBufNum = get8(&parP);
	aahlog("LE chip features %d %d-byte buffers\n", aclBufNum, aclBufSz);

	skb_queue_head_init(&rx_dat);
	skb_queue_head_init(&rx_evt);
	skb_queue_head_init(&usr_data);

	for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++)
		setup_timer((struct timer_list*)&conns[i].timer,
						athome_bt_timeout, i);
	setup_timer(&timeout, athome_bt_timeout, ATHOME_RMT_MAX_CONNS);

	aahlog("LE INFO:\n");

	ret = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Read_Local_Supported_Features,
				0, NULL, sizeof(buf), buf);
	if (ret)
		goto exit_return;
	if (cmd_cmpl_data[1] & 1)
		aahlog(" -> chip supports LE encryption\n");
	cmd_cmpl_data[1] &= ~1;
	for (i = 0; i < 64; i++)
		if (cmd_cmpl_data[i / 8 + 1]  & (1 << (i & 7)))
			aahlog(" -> chip supports unknown LE feature %d\n", i);


	ret = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Read_Supported_States, 0,
				NULL, sizeof(buf), buf);
	if (ret)
		goto exit_return;

	aahlog(" -> normally:\n");
	if (!(cmd_cmpl_data[1] & 0xFF))
		aahlog("    NONE\n");
	if (cmd_cmpl_data[1] & 0x01)
		aahlog("   -> non-connectible advertising\n");
	if (cmd_cmpl_data[1] & 0x02)
		aahlog("   -> scannable advertising\n");
	if (cmd_cmpl_data[1] & 0x04)
		aahlog("   -> connectible advertising\n");
	if (cmd_cmpl_data[1] & 0x08)
		aahlog("   -> directed advertising\n");
	if (cmd_cmpl_data[1] & 0x10)
		aahlog("   -> passive scanning\n");
	if (cmd_cmpl_data[1] & 0x20)
		aahlog("   -> active scanning\n");
	if (cmd_cmpl_data[1] & 0x40)
		aahlog("   -> master role\n");
	if (cmd_cmpl_data[1] & 0x80)
		aahlog("   -> slave role\n");

	aahlog(" -> while passive scanning:\n");
	if (!(cmd_cmpl_data[2] & 0x0F))
		aahlog("    NONE\n");
	if (cmd_cmpl_data[2] & 0x01)
		aahlog("   -> non-connectible advertising\n");
	if (cmd_cmpl_data[2] & 0x02)
		aahlog("   -> scannable advertising\n");
	if (cmd_cmpl_data[2] & 0x04)
		aahlog("   -> connectible advertising\n");
	if (cmd_cmpl_data[2] & 0x08)
		aahlog("   -> directed advertising\n");

	aahlog(" -> while active scanning:\n");
	if (!(cmd_cmpl_data[2] & 0xF0))
		aahlog("    NONE\n");
	if (cmd_cmpl_data[2] & 0x10)
		aahlog("   -> non-connectible advertising\n");
	if (cmd_cmpl_data[2] & 0x20)
		aahlog("   -> scannable advertising\n");
	if (cmd_cmpl_data[2] & 0x40)
		aahlog("   -> connectible advertising\n");
	if (cmd_cmpl_data[2] & 0x80)
		aahlog("   -> directed advertising\n");

	aahlog(" -> while initiating as master:\n");
	if (!(cmd_cmpl_data[3] & 0x0F))
		aahlog("    NONE\n");
	if (cmd_cmpl_data[3] & 0x01)
		aahlog("   -> non-connectible advertising\n");
	if (cmd_cmpl_data[3] & 0x02)
		aahlog("   -> scannable advertising\n");
	if (cmd_cmpl_data[3] & 0x04)
		aahlog("   -> connectible advertising\n");
	if (cmd_cmpl_data[3] & 0x08)
		aahlog("   -> directed advertising\n");

	aahlog(" -> while slave:\n");
	if (!(cmd_cmpl_data[3] & 0xF0))
		aahlog("    NONE\n");
	if (cmd_cmpl_data[3] & 0x10)
		aahlog("   -> non-connectible advertising\n");
	if (cmd_cmpl_data[3] & 0x20)
		aahlog("   -> scannable advertising\n");
	if (cmd_cmpl_data[3] & 0x40)
		aahlog("   -> connectible advertising\n");
	if (cmd_cmpl_data[3] & 0x80)
		aahlog("   -> directed advertising\n");


	aahlog("scan setup\n");
	parP = buf;
	put8(&parP, 0);		//passive scan
	put16(&parP, 32);	//20ms interval per channel
	put16(&parP, 20);	//12.5ms of listening per interval
	put8(&parP,  0);	//use real mac address on packets we send
	put8(&parP, 0);		//accept all advertisement packets
	ret = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Set_Scan_Parameters,
				parP - buf, buf, sizeof(buf), buf);
	if (ret)
		goto exit_return;

	parP = cmd_cmpl_data;
	if (get8(&parP))	/* check status */
		aahlog("failed to set scan parameters.\n");

	aahlog("set clock accuracy\n");
	parP = buf;
	put8(&parP, HCI_Marvell_MCA_30_ppm);
	put8(&parP, HCI_Marvell_MCA_30_ppm);
	ret = athome_bt_simple_cmd(HCI_OGF_Vendor, HCI_CMD_Marvell_Set_MCA,
				parP - buf, buf, 0, NULL);
	if (ret)
		goto exit_return;


	while (1) {	/* stack loop */

		struct sk_buff *skb = NULL;
		bool did_something = 0;

		/* always scanning, if possible */
		if (!is_scanning && !is_conecting) {

			aahlog("scan start\n");
			parP = buf;
			put8(&parP, 1);	//scanning on
			put8(&parP, 0);	//duplicate filter off
			ret = athome_bt_simple_cmd(HCI_OGF_LE,
						HCI_LE_Set_Scan_Enable,
						parP - buf, buf,
						sizeof(buf), buf);
			if (ret)
				goto exit_return;
			parP = cmd_cmpl_data;
			sta = get8(&parP);
			if (sta)	/* check status */
				aahlog("failed to set scan on sta=%d.\n", sta);

			aahlog("scanning...\n");
			is_scanning = 1;
			did_something = 1;
		}

		/* see if we have events to process */
		if (!skb_queue_empty(&rx_evt))
			skb = skb_dequeue(&rx_evt);

		if (skb) {
			const uint16_t intr =
				mode_conn_intervals[ATHOME_MODE_ACTIVE];
			const uint16_t slat =
				mode_slave_latencies[ATHOME_MODE_ACTIVE];
			const uint16_t svto =
				mode_svc_timeouts[ATHOME_MODE_ACTIVE];

			struct hci_event_hdr *evt =
					(struct hci_event_hdr*)skb->data;
			uint8_t *data = (uint8_t*)(evt + 1);
			struct hci_ev_le_meta* meta = 
					(struct hci_ev_le_meta*)data;

			if (evt->evt == HCI_EV_LE_META &&
					meta->subevent ==
						HCI_EV_LE_CONN_COMPLETE) {
				bool need_encr = true;
				struct hci_ev_le_conn_complete *ci =
					(struct hci_ev_le_conn_complete*)
								(meta + 1);

				is_conecting = 0;
				did_something = 1;
				del_timer(&timeout);

				if (ci->status) {
					aahlog("Failed to connect. Err = %d\n",
						ci->status);
					goto evt_done;
				}

				/* we connected successfully */
				handle = read16(&ci->handle) & ACL_CONN_MASK;
				aahlog("CONNECTED with handle %d as %c with "
					"interval  %uuS, latency %u, sv_to "
					"%umS & MCA %uppm\n", handle,
					ci->role ? 'S' : 'M',
					1250 * read16(&ci->interval),
					read16(&ci->latency),
					10 * read16(&ci->supervision_timeout),
					clockA[ci->clk_accurancy]);

				spin_lock_irqsave(&stack_state_lock, flags);
				for (i = 0; i < ATHOME_RMT_MAX_CONNS &&
					conns[i].state !=
						CONN_STATE_INVALID; i++);
				if (i != ATHOME_RMT_MAX_CONNS) do {
					struct athome_bt_known_remote *K;
					struct bt_athome_connected ce;

					K = aah_bt_find_known(ci->bdaddr.b,
									NULL);
					if (!K) {
						aahlog("device became unknown "
							"unexpectedly\n");
						/* pretend we found no slots */
						i = ATHOME_RMT_MAX_CONNS + 1;
						break;
					}
					conns[i].state =
						CONN_STATE_JUST_ESTABLISHED;
					if (K->bind_mode) {
						need_encr = false;
						conns[i].state =
							CONN_STATE_BINDING;
					}
					conns[i].handle = handle;
					conns[i].outstanding = 0;
					conns[i].gone = 0;
					conns[i].pwr = ATHOME_MODE_ACTIVE;
					conns[i].next_pwr = ATHOME_MODE_ACTIVE;
					conns[i].in_modeswitch = false;
					memcpy(conns[i].MAC, ci->bdaddr.b,
								AAH_BT_MAC_SZ);
					memset(&conns[i].stats, 0,
						sizeof(conns[i].stats));
					conns[i].last_time = get_time();
					nconns++;

					memcpy(ce.MAC, ci->bdaddr.b,
							sizeof(ce.MAC));
					aah_bt_usr_enqueue(
						BT_ATHOME_EVT_CONNECTED,
						&ce, sizeof(ce));
				} while (0);
				spin_unlock_irqrestore(&stack_state_lock,
									flags);
				if (i >= ATHOME_RMT_MAX_CONNS) {

					if (i == ATHOME_RMT_MAX_CONNS)
						aahlog("no more connection "
								"slots!\n");
					spin_lock_irqsave(&stack_state_lock,
									flags);
					disconnH = handle;
					spin_unlock_irqrestore(
						&stack_state_lock, flags);
					ret = athome_bt_disconnect(handle);
					if (ret)
						goto exit_return;
					goto evt_done;
				} else if (need_encr)
					athome_bt_start_encr(ci->bdaddr.b, i);

			} else if (evt->evt == HCI_EV_LE_META &&
				meta->subevent ==
					HCI_EV_LE_ADVERTISING_REPORT) {

				uint8_t mac[AAH_BT_MAC_SZ];

				did_something = 1;
				if (athome_bt_handle_scan_evt(data, mac) &&
					!is_conecting &&
					nconns != ATHOME_RMT_MAX_CONNS) {

					is_scanning = 0;
					aahlog("Trying to connect to %02X:%02X"
						":%02X:%02X:%02X:%02X\n",
						mac[5], mac[4], mac[3], mac[2],
						mac[1], mac[0]);

					/* try to connect */
					parP = buf;
					put16(&parP, 32);   /* 20ms/channel */
					put16(&parP, 20);   /* 12.5ms listen */
					put8(&parP, 0);	    /* no whitelist */
					put8(&parP, 1);	    /* random addr */
					put8(&parP, mac[0]);/* mac addr[5] */
					put8(&parP, mac[1]);/* mac addr[4] */
					put8(&parP, mac[2]);/* mac addr[3] */
					put8(&parP, mac[3]);/* mac addr[2] */
					put8(&parP, mac[4]);/* mac addr[1] */
					put8(&parP, mac[5]);/* mac addr[0] */
					put8(&parP, 0);     /* send pub. adr */
					put16(&parP, intr);
					put16(&parP, intr);
					put16(&parP, slat);
					put16(&parP, svto);
					put16(&parP, 20);   /* min durX.62ms */
					put16(&parP, 20);   /* max durX.62ms */
					ret = athome_bt_simple_cmd(HCI_OGF_LE,
						HCI_LE_Create_Connection,
						parP - buf, buf, sizeof(buf),
						buf);
					if (ret)
						goto exit_return;
					if (stat->status)
						aahlog("failed conn. try.\n");
					else {
						is_conecting = 1;
						mod_timer(&timeout, jiffies +
							msecs_to_jiffies(
							CONNECT_TIMEOUT));
					}
				}
			} else if (evt->evt == HCI_EV_LE_META &&
					meta->subevent ==
						HCI_EV_LE_CONN_UPDATE) {
				struct hci_ev_le_conn_update *cu =
					(struct hci_ev_le_conn_update*)
								(meta + 1);
				int which;
				unsigned interval = read16(&cu->interval);


				did_something = 1;

				if (cu->status) {
					aahlog("Failed to update. Err = %d\n",
						cu->status);
					goto evt_done;
				}

				/* we updated successfully */
				handle = read16(&cu->handle) & ACL_CONN_MASK;
				if (LOG_MODESWITCH)
					aahlog("UPDATED handle %d with interval"
						" %uuS, latency %u, sv_to %umS"
						"\n", handle, 1250 * interval,
						read16(&cu->latency),
						10 * read16(&cu->
							supervision_timeout));

				spin_lock_irqsave(&stack_state_lock, flags);
				which = athome_bt_find_connection(handle);
				if (which < 0) {
					aahlog("connection update on a conn. "
						"we don't have: %d\n", handle);
				} else if (conns[which].state ==
							CONN_STATE_TEARDOWN) {
					aahlog("connection update on a conn. "
						"being torn down: %d\n",
						handle);
				} else {
					struct bt_athome_mode_switched mse;
					for (i = 0; i < ARRAY_SIZE(
						mode_conn_intervals) &&
						mode_conn_intervals[i] !=
							interval; i++);

					/* if unknown mode, pretend idle */
					if (i == ARRAY_SIZE(
							mode_conn_intervals))
						i = ATHOME_MODE_IDLE;

					athome_bt_update_time_stats(which);

					conns[which].pwr = i;
					conns[which].in_modeswitch = false;
					if (LOG_MODESWITCH)
						aahlog("Conn %d now in state "
							"%d\n", which, i);
					memcpy(mse.MAC, conns[which].MAC,
						sizeof(mse.MAC));
					mse.new_mode = i;
					aah_bt_usr_enqueue(
						BT_ATHOME_EVT_MODE_SWITCHED,
						&mse, sizeof(mse));
				}
				spin_unlock_irqrestore(&stack_state_lock,
									flags);

			} else if (evt->evt == HCI_EV_ENCRYPT_CHANGE) {

				struct hci_ev_encrypt_change *evt =
					(struct hci_ev_encrypt_change*)data;

				handle = read16(&evt->handle);

				did_something = 1;
				spin_lock_irqsave(&stack_state_lock, flags);
				i = athome_bt_find_connection(handle);
				if (i >= 0) {
					aahlog("conn%d encr. s=%d o=%d\n",
						handle, evt->status,
						evt->encrypt);
					if (evt->encrypt) {

						if (conns[i].state ==
						CONN_STATE_ENCRYPTING) {
							del_timer((struct timer_list*)&conns[i].timer);
							conns[i].state = CONN_STATE_DATA;
						} else {

							aahlog("Encr event "
								"for invalid "
								"state %d\n",
							conns[i].state);
						}
					}
				}
				spin_unlock_irqrestore(&stack_state_lock,
									flags);
			} else if (evt->evt == HCI_EV_ENCR_REFRESH) {

				struct hci_ev_encrypt_refresh *evt =
					(struct hci_ev_encrypt_refresh*)data;

				handle = read16(&evt->handle);

				did_something = 1;
				spin_lock_irqsave(&stack_state_lock, flags);
				i = athome_bt_find_connection(handle);
				if (i >= 0) {
					aahlog("conn%d encr refr s=%d\n",
						handle, evt->status);

					if (conns[i].state ==
					CONN_STATE_REENCRYPTING) {
						del_timer((struct timer_list*)&conns[i].timer);
						conns[i].state = CONN_STATE_DATA;
					} else {

						aahlog("Reencr event "
							"for invalid "
							"state %d\n",
						conns[i].state);
					}
				}
				spin_unlock_irqrestore(&stack_state_lock,
									flags);
			} else if (evt->evt == HCI_EV_DISCONN_COMPLETE) {

				/* it's already been handled in the filter callback */

			} else {

				aahlog("unexpected event %d\n", evt->evt);
			}
evt_done:
			kfree_skb(skb);
			skb = NULL;
		}

		/* see if we have events to process */
		if (!skb_queue_empty(&rx_dat))
			skb = skb_dequeue(&rx_dat);

		if (skb) {

			struct hci_acl_hdr *acl =
					(struct hci_acl_hdr*)skb->data;

			handle = read16(&acl->handle) & ACL_CONN_MASK;
			spin_lock_irqsave(&stack_state_lock, flags);
			i = athome_bt_find_connection(handle);
			spin_unlock_irqrestore(&stack_state_lock, flags);

			if (i >= 0) {
				ret = athome_bt_data_rx(i, skb->data, skb->len);
				if (ret)
					goto exit_return;
			}
			else
				aahlog("data for unknown conection dropped\n");

			kfree_skb(skb);
			skb = NULL;
		}

		/* maybe we timed out connecting? */
		if (!timer_pending(&timeout) && is_conecting) {

			did_something = 1;
			ret = athome_bt_simple_cmd(HCI_OGF_LE,
					HCI_LE_Create_Connection_Cancel,
					0, NULL, 0, NULL);
			if (ret)
				goto exit_return;
		}

		/* maybe something timed out ? */
		spin_lock_irqsave(&stack_state_lock, flags);
		for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
			if (conns[i].timeout) {
				conns[i].timeout = 0;
				break;
			}
		}
		if (i != ATHOME_RMT_MAX_CONNS) switch (conns[i].state) {
		case CONN_STATE_ENCRYPT_SETUP:
		case CONN_STATE_REENCRYPT_SETUP:
			aahlog("[re]encrypt setup timeout\n");
			conns[i].state = CONN_STATE_DISCONNECT;
			break;
		case CONN_STATE_ENCRYPTING:
		case CONN_STATE_REENCRYPTING:
			aahlog("[re]encrypt cmd timeout\n");
			conns[i].state = CONN_STATE_DISCONNECT;
			break;
		default:
			aahlog("timeout in state %d unexpected\n",
							conns[i].state);
		}
		spin_unlock_irqrestore(&stack_state_lock, flags);

		/* maybe a connection requires our attention (via state) ? */
		spin_lock_irqsave(&stack_state_lock, flags);
		ret = 0;
		for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
		//	aahlog("conn[%d] state = %d\n", i, conns[i].state);
			if (conns[i].state == CONN_STATE_DISCONNECT) {

				uint16_t handle;

				conns[i].state = CONN_STATE_TEARDOWN;
				handle = conns[i].handle;
				spin_unlock_irqrestore(&stack_state_lock,
									flags);
				ret = athome_bt_disconnect(handle);
				did_something = 1;
				break;
			} else if (conns[i].gone) {
				conns[i].state = CONN_STATE_INVALID;
				conns[i].gone = 0;
				nconns--;
				did_something = 1;
			} else if (conns[i].pwr != conns[i].next_pwr &&
						!conns[i].in_modeswitch) {

				uint8_t next_pwr = conns[i].next_pwr;

				spin_unlock_irqrestore(&stack_state_lock,
									flags);
				ret = athome_bt_modeswitch(i, next_pwr);
				did_something = 1;
				break;
			}
		}
		if (i == ATHOME_RMT_MAX_CONNS)
			spin_unlock_irqrestore(&stack_state_lock, flags);
		if (ret)
			goto exit_return;

		if (!did_something)
			wait_event_interruptible(rx_wait,
				(!skb_queue_empty(&rx_evt) ||
					!skb_queue_empty(&rx_dat) ||
					atomic_read(&state_chg)));

		atomic_set(&state_chg, 0);

		if (kthread_should_stop() || atomic_read(&in_shutdown)) {
			aahlog("thread exiting normally\n");
			ret = 1;
			goto exit_return;
		}
	}

err_deregister_input:
	athome_bt_input_deinit();

err_deregister_misc:
	misc_deregister(&mdev);

err_return:
exit_return:
	aahlog("thread exiting with sta %d\n", ret);
	return ret;
}

static void logpacket(char chip, uint32_t type, const u8 *data, u32 sz,
			uint8_t owned_by)
{
	static char x[8192];
	static const char hexch[] = "0123456789ABCDEF";
	int j, i = 0;
	static const char owner[] = {'E', 'L', '?'};

	if (LOG_ONLY_OURS && !owned_by)
		return;

	if (type == HCI_SCODATA_PKT)
		return;

	if (!LOG_BT_ACL && type == HCI_ACLDATA_PKT)
		return;

	if (!LOG_BT_EVTS && type == HCI_EVENT_PKT)
		return;

	if (!LOG_BT_CMDS && (type == HCI_COMMAND_PKT || type == PKT_MARVELL))
		return;


	x[i++] = owner[owned_by];
	x[i++] = '*';
	x[i++] = '*';
	x[i++] = 'B';
	x[i++] = 'T';
	x[i++] = '*';
	x[i++] = '*';
	x[i++] = ' ';
	x[i++] = chip ? 'C' : 'H';
	x[i++] = ':';
	x[i++] = ' ';
	x[i++] = hexch[type >> 4];
	x[i++] = hexch[type & 15];
	for (j = 0; j < sz; j++) {
		x[i++] = ' ';
		x[i++] = hexch[data[j] >> 4];
		x[i++] = hexch[data[j] & 15];
	}
	x[i++] = '\n';
	x[i] = 0;
	aahlog("%s", x);
}

/*
 * Function in Wolfie audio ALSA driver that decodes SBC.
 * TODO put this prototype in a header file.
 */
void athome_bt_audio_dec(int which, int format, const uint8_t *data, unsigned len)
{
}

/* returns 1 if we've been asked to quit */
static int athome_bt_data_rx(int which, uint8_t *in_data, uint32_t len)
{
	uint8_t *payload = in_data + HCI_ACL_HDR_SIZE;
	bool ok = false, secure = true;
	uint8_t i, type, data_buf[sizeof(struct bt_athome_send_data) + 26];
	struct bt_athome_send_data *usr_data =
					(struct bt_athome_send_data*)data_buf;
	uint8_t *data = usr_data->pkt_data;
	uint64_t *stats_incr = NULL;
	unsigned long flags;
	bool initial = true;
	struct athome_pkt_rx_modeswitch *msw =
					(struct athome_pkt_rx_modeswitch*)data;
	struct athome_pkt_rx_input *inp = (struct athome_pkt_rx_input*)data;
	struct athome_pkt_rx_input_btn *inp_btn = NULL;
	struct athome_pkt_rx_input_touch *inp_tch = NULL;

	struct hci_acl_hdr *acl = (struct hci_acl_hdr*)in_data;
	if (len < HCI_ACL_HDR_SIZE) {
		aahlog("acl data too small (%d)\n", len);
		return 0;
	}
	if (len - HCI_ACL_HDR_SIZE != read16(&acl->dlen)) {
		aahlog("acl data bad size (got %u expected %u)\n",
			len - HCI_ACL_HDR_SIZE, read16(&acl->dlen));
		return 0;
	}
	len = read16(&acl->dlen);

	/* read and reassemble the data */
	for(i = 0; i < 2; i++)
		data[i] = payload[i];
	for(i = 3; i < 26 && i < len; i++)
		data[i - 1] = payload[i];
	type = payload[2];
	len--;

	if (!(type & L2CAP_CHAN_ATHOME_BIT)) {
		aahlog("got ACL data with missing bit. Dropping\n");
		return 0;
	}
	type &= 0x7F;
	usr_data->pkt_typ = type;

	spin_lock_irqsave(&stack_state_lock, flags);
	conns[which].stats.pkts_rx++;
	conns[which].stats.bytes_rx += len + 1; /* type byte */
	memcpy(usr_data->MAC, conns[which].MAC, sizeof(usr_data->MAC));
	switch(conns[which].state) {
	case CONN_STATE_REENCRYPT_SETUP:
		if (type != ATHOME_PKT_RX_ACK || data[0] != ATHOME_PKT_TX_ENC) {
			/* on re-encrypt other packets are ok */
			ok = true;
			break;
		} else
			initial = false;
		/* fallthrough intentional */

	case CONN_STATE_ENCRYPT_SETUP:
		if (type != ATHOME_PKT_RX_ACK) {
			aahlog("entropy echo not ACK packet - drop\n");
			break;
		}
		if (data[0] != ATHOME_PKT_TX_ENC) {
			aahlog("entropy echo not ACK for ENC - drop\n");
			break;
		}
		if (len != sizeof(struct athome_pkt_ack) + AAH_BT_LTK_SZ) {
			aahlog("entropy echo not good sz(%d) - drop\n", len);
			break;
		}
		for (i = 0; i < AAH_BT_LTK_SZ; i++) {
			if (conns[which].data.entropy[i] != data[i + 1])
				break;
		}
		if (i != AAH_BT_LTK_SZ) {
			aahlog("entropy echo doesn't match(b %d) - drop\n", i);
			conns[which].state = CONN_STATE_DISCONNECT;
			break;
		}
		aahlog("entropy echo ok - encryption approved\n");
		mod_timer((struct timer_list*)&conns[which].timer,
				jiffies + msecs_to_jiffies(ENCRYPT_TIMEOUT));
		conns[which].timeout = 0;
		spin_unlock_irqrestore(&stack_state_lock, flags);
		return athome_bt_do_encrypt(which, conns[which].LTK, initial);

	case CONN_STATE_DATA:
		ok = true;
		break;

	case CONN_STATE_BINDING:
		/* only some packets allowed in bind mode */
		ok = type == ATHOME_PKT_RX_ACK || type == ATHOME_PKT_RX_INPUT;
		secure = false;
		break;
	}

	/* done here so as to be under spinlock */
	if (ok) switch (type) {
	case ATHOME_PKT_RX_INPUT:
		stats_incr = &conns[which].stats.input;
		break;
	case ATHOME_PKT_RX_ACCEL:
		stats_incr = &conns[which].stats.accel;
		break;
	case ATHOME_PKT_RX_AUDIO_0:
		stats_incr = conns[which].stats.audio + 0;
		break;
	case ATHOME_PKT_RX_AUDIO_1:
		stats_incr = conns[which].stats.audio + 1;
		break;
	case ATHOME_PKT_RX_AUDIO_2:
		stats_incr = conns[which].stats.audio + 2;
		break;
	case ATHOME_PKT_RX_AUDIO_3:
		stats_incr = conns[which].stats.audio + 3;
		break;
	case ATHOME_PKT_RX_NFC:
		stats_incr = &conns[which].stats.nfc_rx;
		break;
	}
	if (stats_incr)
		(*stats_incr)++;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	if (!ok) {
		aahlog("got data while not allowed. Dropping\n");
		return 0;
	}

	switch (type) {
	case ATHOME_PKT_RX_INPUT:
		if (LOG_INPUT_EVENTS)
			aahlog("[%d] input event with time %d and data: {\n",
				which,
				inp->info & ATHOME_INPUT_INFO_MASK_TIMESTAMP);

		if (inp->info & ATHOME_INPUT_INFO_MASK_HAS_BTN) {
			inp_btn = (struct athome_pkt_rx_input_btn*)(inp + 1);
		}
		if (inp->info & ATHOME_INPUT_INFO_MASK_HAS_TOUCH) {
			inp_tch = inp_btn ?
				(struct athome_pkt_rx_input_touch*)(inp_btn + 1) :
				(struct athome_pkt_rx_input_touch*)(inp + 1);
		}
		if (inp_btn) {
			uint32_t buttons = read32(&inp_btn->btn_mask);
			if (LOG_INPUT_EVENTS)
				aahlog(" -> buttons: 0x%08X\n", buttons);

			if (!secure && (buttons & ~SECURE_BTN_MASK)) {
				buttons &= SECURE_BTN_MASK;
				if (LOG_INPUT_EVENTS)
					aahlog(" ->  now: 0x%08X\n", buttons);
			}

			athome_bt_input_send_buttons(which,
						buttons & ~SECURE_BTN_MASK);

			/* send binding buttons to userspace */
			if (buttons & SECURE_BTN_MASK) {
				struct bt_athome_bind_key bb;
				memcpy(&bb.MAC, conns[which].MAC,
							sizeof(bb.MAC));
				bb.key = !!(buttons & ATHOME_PAIR_BTN_CHAR);
				aah_bt_usr_enqueue(BT_ATHOME_EVT_BIND_KEY,
							&bb, sizeof(bb));
			}
		}
		if (inp_tch && secure) {
			uint16_t x, y;

			for (i = 0; i < ATHOME_MAX_FINGERS; i++) {
				x = read16(&inp_tch->fingers[i].X);
				y = read16(&inp_tch->fingers[i].Y);

				athome_bt_input_send_touch(which, i, x, y);
			}
		}
		if (LOG_INPUT_EVENTS)
			aahlog_continue("}\n");
		athome_bt_input_frame(which);
		break;

	case ATHOME_PKT_RX_AUDIO_0:
	case ATHOME_PKT_RX_AUDIO_1:
	case ATHOME_PKT_RX_AUDIO_2:
	case ATHOME_PKT_RX_AUDIO_3:
		athome_bt_audio_dec(which, type - ATHOME_PKT_RX_AUDIO_0, data, len);
		break;

	case ATHOME_PKT_RX_MODESWITCH:
		if (msw->mode != ATHOME_MODE_ACTIVE &&
				msw->mode != ATHOME_MODE_SEMIIDLE)
			msw->mode = ATHOME_MODE_IDLE;
		conns[which].next_pwr = msw->mode;
		if (LOG_MODESWITCH)
			aahlog("Requested modeswitch to %d for conn %d\n",
								msw->mode, which);
		break;

	default:
		aah_bt_usr_enqueue(BT_ATHOME_EVT_DATA, data_buf,
					len + sizeof(struct bt_athome_got_data));
		break;
	}
	return 0;
}

static void athome_bt_enqueue_data(struct sk_buff_head* Q, const void* ptr,
								uint32_t len)
{
	struct sk_buff *skb;

	skb = bt_skb_alloc(len, GFP_ATOMIC);
	if (!skb) {
		aahlog("fail to alloc skb (%u bytes)\n", len);
		return;
	}
	memcpy(skb->data, ptr, len);
	skb_put(skb, len);
	skb_queue_tail(Q, skb);
}

int athome_bt_remote_filter_rx_data(void *priv, u32 pkt_type, u8 *rx_buf,
					u32 *rx_len)
{
	int i = 0;
	bool isours = 0, enq_if_ours = 1;

	smp_rmb();
	if (!drv_priv)
		drv_priv = priv;
	smp_wmb();

	if (pkt_type == HCI_EVENT_PKT) {

		uint8_t *evt_data = rx_buf + HCI_EVENT_HDR_SIZE;
		struct hci_event_hdr *evt = (struct hci_event_hdr*)rx_buf;
		struct hci_ev_cmd_complete *cmd_complete =
			(struct hci_ev_cmd_complete*)evt_data;
		struct hci_ev_cmd_status *cmd_status =
			(struct hci_ev_cmd_status*)evt_data;


		switch (evt->evt) {

		case HCI_EV_DISCONN_COMPLETE:
			isours = athome_bt_filter_disconn(evt_data);
			break;

		case HCI_EV_ENCRYPT_CHANGE:
			isours = athome_bt_filter_encr_change(evt_data);
			break;

		case HCI_EV_CMD_COMPLETE:
			if (cmd_complete->ncmd) {
				if (LOG_BT_SEM)
					aahlog("up(cmpl)\n");
				up(&send_sem);
			}
			isours = athome_bt_filter_cmd_complete(rx_buf, *rx_len,
								&enq_if_ours);
			break;

		case HCI_EV_CMD_STATUS:
			if (cmd_status->ncmd) {
				if (LOG_BT_SEM)
					aahlog("up(sta)\n");
				up(&send_sem);
			}
			isours = athome_bt_filter_cmd_status(rx_buf, *rx_len,
								&enq_if_ours);
			break;

		case HCI_EV_NUM_COMP_PKTS:
			isours = athome_bt_filter_num_comp_pkt(evt_data,
								rx_len);
			enq_if_ours = 0;
			break;

		case HCI_EV_ENCR_REFRESH:
			isours = athome_bt_filter_encr_refresh(evt_data);
			break;

		case HCI_EV_LE_META:
			isours = 1;
			break;
		}

		if (isours && enq_if_ours) {
			athome_bt_enqueue_data(&rx_evt, rx_buf, *rx_len);
			wake_up_interruptible(&rx_wait);
		}

	} else if (pkt_type == HCI_ACLDATA_PKT) {

		struct hci_acl_hdr *acl = (struct hci_acl_hdr*)rx_buf;
		unsigned long flags;
		uint16_t hnd;

		hnd = read16(&acl->handle) & ACL_CONN_MASK;
		spin_lock_irqsave(&stack_state_lock, flags);
		i = athome_bt_find_connection(hnd);
		spin_unlock_irqrestore(&stack_state_lock, flags);

		if (i >= 0) {
			athome_bt_enqueue_data(&rx_dat, rx_buf, *rx_len);
			wake_up_interruptible(&rx_wait);
			isours = 1;
		}
	}

	logpacket(1, pkt_type, rx_buf, *rx_len, isours);

	return isours ? AAH_BT_PKT_DROP : AAH_BT_PKT_PROCEED;
}

int athome_bt_pkt_send_req(void *priv, struct sk_buff *skb)
{
	struct hci_command_hdr *cmd = (struct hci_command_hdr*)skb->data;
	bool wasours = 0, log_it = 1, skipsem = false;
	int i, ret = AAH_BT_PKT_PROCEED;

	smp_rmb();
	if (!drv_priv)
		drv_priv = priv;
	smp_wmb();

	if (bt_cb(skb)->pkt_type == AAH_BT_PKT_TYPE_CMD) {
		bt_cb(skb)->pkt_type = HCI_COMMAND_PKT;
		wasours = 1;
	}

	if (bt_cb(skb)->pkt_type == HCI_COMMAND_PKT) {

		uint16_t ogf = hci_opcode_ogf(cmd->opcode);
		uint16_t ocf = hci_opcode_ocf(cmd->opcode);

		if (ogf == HCI_OGF_Controller_And_Baseband &&
			ocf == HCI_Reset) {

			static int first = 1;

			if (!first)
				aahlog("unexpected chip reset\n");
			first = 0;

			athome_bt_shutdown();
			athome_bt_reset();
			skipsem = true;

		} else if (ogf == HCI_OGF_Controller_And_Baseband &&
				ocf == HCI_Write_LE_Host_Support &&
				!wasours) {

			struct hci_cp_write_le_host_supported *params =
				(struct hci_cp_write_le_host_supported*)
								(cmd + 1);

			if (params->le)
				aahlog("EDR stack enabling LE  -> bad\n");
			if (params->simul)
				aahlog("EDR stack enabling simul -> bad\n");

			params->le = 1;
			params->simul = 1;

		} else if (ogf == HCI_OGF_Controller_And_Baseband &&
				ocf == HCI_Set_Event_Mask) {

			struct task_struct *thread;
			uint64_t mask = read64(((uint8_t*)skb->data) +
						HCI_COMMAND_HDR_SIZE);
			uint64_t oldmask = mask;

			if ((mask & evtsNeeded) != evtsNeeded) {
				aahlog("EDR stack blocked some vital "
					"events - BAD - fixing\n");
				mask |= evtsNeeded;
			}
			if ((mask & evtsLE) && !wasours) {
				aahlog("EDR stack unmasked some events "
					"unexpectedly - OK, just weird\n");
			}
			mask |= evtsLE;
			write64(((uint8_t*)skb->data) + HCI_COMMAND_HDR_SIZE,
					mask);

			aahlog("modified event mask 0x%08X%08X -> 0x%08X%08X"
				"\n", (uint32_t)(oldmask >> 32),
				(uint32_t)oldmask, (uint32_t)(mask >> 32),
				(uint32_t)mask);

			/* it is now safe to start our stack */
			if (!aah_thread) {
				aahlog("starting LE stack!\n");
				atomic_set(&in_shutdown, 0);
				thread = kthread_run(athome_bt_thread, NULL,
							"athome_bt");
				if (IS_ERR(thread)) {
					aahlog("Failed to start stack "
						"thread\n");
					thread = NULL;
				}
				aah_thread = thread;
			}
		} else if (ogf == HCI_OGF_LE && !wasours) {

			uint8_t buf[64];
			uint8_t* end = NULL;
			struct hci_event_hdr *evt = (struct hci_event_hdr*)buf;
			struct hci_ev_cmd_complete *comp =
					(struct hci_ev_cmd_complete*)(evt + 1);
			struct hci_rp_le_read_buffer_size *bs =
				(struct hci_rp_le_read_buffer_size*)(comp + 1);
			uint8_t *stat = (uint8_t*)(comp + 1);

			evt->evt = HCI_EV_CMD_COMPLETE;
			comp->ncmd = 1;
			write16(&comp->opcode, cmd->opcode);

			aahlog("LE command (0x%x) from EDR stack!\n", ocf);
			switch (ocf) {
			case HCI_LE_Read_Buffer_Size:
				bs->status = 0;
				write16(&bs->le_mtu, 256); /* why not ? */
				bs->le_max_pkt = 16; /* why not */
				end = (uint8_t*)(bs + 1);
				break;

			case HCI_LE_Set_Scan_Parameters:
			case HCI_LE_Set_Advertising_Data:
			case HCI_LE_Set_Scan_Enable:
			case HCI_LE_Set_Event_Mask:
			case HCI_LE_Set_Random_Address:
			case HCI_LE_Set_Scan_Response_Data:
			case HCI_LE_Clear_White_List:
			case HCI_LE_Add_Device_To_White_List:
			case HCI_LE_Remove_Device_From_White_List:
			case HCI_LE_Set_Host_Channel_Classification:
				*stat = 0;
				end = stat + 1;
				break;

			case HCI_LE_Read_Advertising_Channel_Tx_Power:
				*stat++ = 0;  /* status = 0 */
				*stat = 0; /* good tx power */
				end = stat + 1;
				break;

			case HCI_LE_Read_White_List_Size:
				*stat++ = 0;  /* status = 0 */
				*stat = 1; /* small list */
				end = stat + 1;
				break;

			case HCI_LE_Read_Local_Supported_Features:
			case HCI_LE_Read_Supported_States:
				*stat++ = 0; /* status = 0 */
				for (i = 0; i < 8; i++)
					*stat++ = 0; /* nothing supported */
				end = stat;
				break;

			case HCI_LE_Encrypt:
			case HCI_LE_Rand:
				/* we allow this - it is safe */
				break;

			default:
				/* this is unsafe, but what can we do?*/
				aahlog(" -> command unknown - BAD!\n");
			}

			if (end) {

				evt->plen = end - (uint8_t*)(evt + 1);

				log_it = 0; /* log now so we can log "reply" */

				logpacket(0, HCI_COMMAND_PKT,
						(const uint8_t*)skb->data,
						skb->len, 0);

				logpacket(1, HCI_EVENT_PKT, buf, end - buf, 0);

				athome_bt_send_to_user(HCI_EVENT_PKT, buf,
								end - buf);


				ret = AAH_BT_PKT_DROP;
			}
		}
	}

	if (ret == AAH_BT_PKT_PROCEED && !skipsem &&
			(bt_cb(skb)->pkt_type == HCI_COMMAND_PKT ||
				bt_cb(skb)->pkt_type == PKT_MARVELL)) {
		if (LOG_BT_SEM)
			aahlog("down\n");
		down(&send_sem);
	}

	if (log_it)
		logpacket(0, bt_cb(skb)->pkt_type, (const uint8_t*)skb->data,
							skb->len, wasours);

	return ret;
}

/* ========================== MISC. DEVICE DRIVER ========================== */


static void aah_bt_usr_enqueue(uint8_t typ, const void* data, unsigned len)
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
		aahlog("dropped userspace message (%d) due to full queue\n", typ);
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

static ssize_t aah_bt_read(struct file *file, char __user *userbuf,
						size_t bytes, loff_t *off)
{
	struct sk_buff *skb;

	if (down_interruptible(&usr_rx))
		return -EINTR;

	atomic_dec(&num_usr_msgs);
	skb = skb_dequeue(&usr_data);

	if (bytes > skb->len)
		bytes = skb->len;

	if (copy_to_user(userbuf, skb->data, bytes)) {
		up(&usr_rx);
		skb_queue_head(&usr_data, skb);
		return -EFAULT;
	} else {
		kfree_skb(skb);
		return bytes;
	}
}

static int aah_bt_add_dev(uint8_t* MAC, uint8_t* LTK)
{	/* if LTK == NULL -> bind mode addition */

	unsigned long flags;
	struct athome_bt_known_remote *cur, *item;

	item = kmalloc(sizeof(*item), GFP_KERNEL);
	if (!item)
		return -ENOMEM;

	pr_info("%s: MAC = %x %x %x %x %x %x\n",
		__func__, MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);
	if (LTK)
		pr_info("%s: LTK = %x %x %x %x %x %x %x %x %x %x %x %x %x "
			"%x %x %x\n", __func__,
			LTK[0], LTK[1], LTK[2], LTK[3],
			LTK[4], LTK[5], LTK[6], LTK[7],
			LTK[8], LTK[9], LTK[10], LTK[11],
			LTK[12], LTK[13], LTK[14], LTK[15]);
	spin_lock_irqsave(&stack_state_lock, flags);

	cur = aah_bt_find_known(MAC, NULL);
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

	spin_unlock_irqrestore(&stack_state_lock, flags);
	return 0;
}

static void aah_bt_del_dev(uint8_t* MAC, bool bind_mode)
{
	unsigned long flags;
	int i;
	struct athome_bt_known_remote *cur, *prev;

	spin_lock_irqsave(&stack_state_lock, flags);
	cur = aah_bt_find_known(MAC, &prev);
	if (cur && (!cur->bind_mode == !bind_mode)) {

		if (prev)
			prev->next = cur->next;
		else
			known = cur->next;
		kfree(cur);
	}

	for(i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
		if (conns[i].state == CONN_STATE_INVALID)
			continue;

		if (conns[i].gone)
			continue;

		if (memcmp(conns[i].MAC, MAC, sizeof(conns[i].MAC)))
			continue;

		aahlog("Disconnecting %d\n", conns[i].handle);
		conns[i].state = CONN_STATE_DISCONNECT;
		atomic_set(&state_chg, 1);
		wake_up_interruptible(&rx_wait);
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);
}

static void aah_bt_getstate(struct bt_athome_state *state)
{
	unsigned long flags;
	int i;


	state->num = 0;
	spin_lock_irqsave(&stack_state_lock, flags);
	for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
		uint8_t conn_state = BT_ATHOME_STATE_UNKNOWN;

		if (conns[i].state == CONN_STATE_INVALID)
			continue;

		memcpy(state->remotes[state->num].MAC,
				conns[i].MAC, sizeof(conns[i].MAC));

		switch (conns[i].state) {
		case CONN_STATE_JUST_ESTABLISHED:
		case CONN_STATE_ENCRYPT_SETUP:
			/* not ready for data */
			conn_state = BT_ATHOME_STATE_CONNECTING;
			break;

		case CONN_STATE_BINDING:
			/* unsecure data only */
			conn_state = BT_ATHOME_STATE_BINDING;
			break;

		case CONN_STATE_ENCRYPTING:
		case CONN_STATE_REENCRYPTING:
			/* encrypt in progress -> no data */
			conn_state = BT_ATHOME_STATE_ENCRYPTING;
			break;

		case CONN_STATE_REENCRYPT_SETUP:
		case CONN_STATE_DATA:
			/* data allowed */
			conn_state = BT_ATHOME_STATE_CONNECTED;
			break;

		case CONN_STATE_DISCONNECT:
		case CONN_STATE_TEARDOWN:
			/* soon to be gone */
			conn_state = BT_ATHOME_STATE_DISCONNECTING;
			break;
		}
		state->remotes[state->num].con_state = conn_state;
		state->remotes[state->num].pwr_state = conns[i].pwr;
		state->remotes[state->num].input_dev_idx = i;
		state->num++;
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);
}

static ssize_t aah_bt_write(struct file *file, const char __user *userbuf,
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
	unsigned long flags, i;

	pr_info("%s: bytes = %d\n", __func__, bytes);

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
		ret = aah_bt_add_dev(add->MAC, add->LTK);
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
		ret = aah_bt_add_dev(do_bind->MAC, NULL);
		if (ret < 0)
			orig_bytes = ret;
		break;

	case BT_ATHOME_MSG_STOP_BIND:
		if (bytes != sizeof(*stop_bind))
			return -EIO;
		aah_bt_del_dev(del->MAC, 1);
		break;

	case BT_ATHOME_MSG_ENCRYPT:
		if (bytes != sizeof(*encr))
			return -EIO;
		athome_bt_start_encr(encr->MAC, -1);
		break;

	case BT_ATHOME_MSG_DEL_DEV:
		if (bytes != sizeof(*del))
			return -EIO;

		aah_bt_del_dev(del->MAC, 0);
		break;

	case BT_ATHOME_MSG_GET_STATE:
		if (bytes)
			return -EIO;

		aah_bt_getstate(state);
		aah_bt_usr_enqueue(BT_ATHOME_EVT_STATE, state, sizeof(*state));
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

		spin_lock_irqsave(&stack_state_lock, flags);
		for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
			if (conns[i].state == CONN_STATE_INVALID)
				continue;

			if (conns[i].gone)
				continue;

			if (memcmp(conns[i].MAC,
					get_stats->MAC, sizeof(conns[i].MAC)))
				continue;

			/*
			 * We must do an update here so that the time in
			 * "current" mode, whatever it is, is reported.
			 */
			athome_bt_update_time_stats(i);
			memcpy(&stats->stats, &conns[i].stats,
							sizeof(stats->stats));
			memcpy(stats->MAC, conns[i].MAC, sizeof(conns[i].MAC));
			break;
		}
		spin_unlock_irqrestore(&stack_state_lock, flags);
		if (i != ATHOME_RMT_MAX_CONNS)
			aah_bt_usr_enqueue(BT_ATHOME_EVT_DEV_STATS,
							stats, sizeof(*stats));
		break;

	default:
		aahlog("unknown user command 0x%x\n", buf[0]);
		/* bad command */
		return -EIO;
	}

	return orig_bytes;
}

static int aah_bt_open(struct inode * inode, struct file * file)
{
	if (atomic_cmpxchg(&usr_opened, 0, 1))
		return -ENOMEM;

	return 0;
}

static long aah_bt_ioctl(struct file *fil, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct bt_athome_state state;

	switch (cmd) {
	case BT_ATHOME_IOCTL_GETSTATE:
		memset(&state, 0, sizeof(state));
		aah_bt_getstate(&state);
		if (copy_to_user((void*)arg, &state, sizeof(state)))
			ret = -EFAULT;
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static unsigned int aah_bt_poll(struct file *file,
					struct poll_table_struct *wait)
{
	poll_wait(file, &usr_wait_q, wait);
	if (!down_trylock(&usr_rx)) {
		up(&usr_rx);
		return POLLIN | POLLRDNORM;
	}
	return 0;
}

static int aah_bt_release(struct inode * inode, struct file * file)
{
	struct sk_buff *skb;

	while (!skb_queue_empty(&usr_data))
		skb = skb_dequeue(&usr_data);
	while (!down_trylock(&usr_rx));

	atomic_set(&num_usr_msgs, 0);
	atomic_set(&usr_opened, 0);

	return 0;
}


/* =========================== TOUCH & BTN STUFF =========================== */
/* to see events live: in adb do "getevent -lt /dev/input/event1" */


static unsigned short ikeys[] = {KEY_BACK, KEY_HOMEPAGE, KEY_VOLUMEUP,
					KEY_VOLUMEDOWN, AAH_BT_KEY_DPAD_CENTER,
					AAH_BT_KEY_POWER, AAH_BT_KEY_INPUT};

#define RAW_X_MAX 0xffff
#define RAW_Y_MAX 0xffff

static int athome_bt_input_init_device(struct input_dev **idevP)
{
	int err, i;
	struct input_dev *idev;

	idev = input_allocate_device();
	if (!idev) {
		aahlog("Failed to allocate touch device\n");
		return -ENOMEM;
	}

	idev->name = "athome_remote";

	/* we support touch buttons */
	set_bit(EV_SYN, idev->evbit);
	set_bit(EV_ABS, idev->evbit);

	/* we support the following buttons */
	set_bit(EV_KEY, idev->evbit);
	for(i = 0; i < ARRAY_SIZE(ikeys); i++)
		__set_bit(ikeys[i], idev->keybit);

	/* key mapping */
	idev->keycode = ikeys;
	idev->keycodesize = sizeof(unsigned short);
	idev->keycodemax = ARRAY_SIZE(ikeys);

	/* touch params */
	/* don't use input_mt_init_slots() because it causes the input
	 * system to drop events if the current coordinate is the same
	 * as the last coordinate, which Android can't handle
	 */
	input_mt_init_slots(idev, ATHOME_MAX_FINGERS);
	input_set_abs_params(idev, ABS_MT_POSITION_X, 0, RAW_X_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_Y, 0, RAW_Y_MAX, 0, 0);

	/* without misc key capability, volume keys will do nothing... */
	input_set_capability(idev, EV_MSC, MSC_SCAN);
	err = input_register_device(idev);
	if (err) {
		aahlog("Failed to register input device\n");
		input_free_device(idev);
		return err;
	}

	*idevP = idev;
	return 0;
}

static void athome_bt_input_del_device(struct input_dev *idev)
{
	input_mt_destroy_slots(idev);
	input_unregister_device(idev);
	input_free_device(idev);
}

static int athome_bt_input_init(void)
{
	int ret, i;

	for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {


		sprintf((char*)conns[i].uniq, "athome_bt_%d", i);

		ret = athome_bt_input_init_device(
			(struct input_dev **)&conns[i].idev);
		if (ret)
			break;
		conns[i].idev->uniq = (char*)conns[i].uniq;
		conns[i].fingers_down = 0;
	}
	if (i == ATHOME_RMT_MAX_CONNS)
		return 0;
	/* we failed */
	while (i) {
		i--;
		athome_bt_input_del_device(conns[i].idev);
	}
	return ret;
}

static void athome_bt_input_deinit(void)
{
	int i;

	for(i = 0; i < ATHOME_RMT_MAX_CONNS; i++)
		athome_bt_input_del_device(conns[i].idev);
}

static void athome_bt_input_send_touch(int which, int pointerIdx,
						uint16_t x, uint16_t y)
{
	struct input_dev *idev = conns[which].idev;
	uint32_t mask = 1UL << pointerIdx;
	bool wasdown = !!(conns[which].fingers_down & mask);
	bool isdown = ((x != RAW_X_MAX) || (y != RAW_Y_MAX));

	/* currently, remote sends us events for fingers that aren't down
	 * and haven't been down, but input system doesn't want those
	 */
	if (!isdown && !wasdown)
		return;

	input_mt_slot(idev, pointerIdx);
	input_mt_report_slot_state(idev, MT_TOOL_FINGER, isdown);

	if (isdown) {
		if (LOG_INPUT_EVENTS)
			aahlog_continue(" [%d] raw: ( %5d , %5d)\n",
					pointerIdx, x, y);
		input_report_abs(idev, ABS_MT_POSITION_X, x);
		input_report_abs(idev, ABS_MT_POSITION_Y, y);
		conns[which].fingers_down |= mask;
	} else {
		/* was down case */
		if (LOG_INPUT_EVENTS)
			aahlog_continue(" [%d] finger release\n", pointerIdx);
		conns[which].fingers_down &= ~mask;
	}
}

static void athome_bt_input_send_buttons(int which, uint32_t mask)
{
	struct input_dev *idev = conns[which].idev;
	int i;

	for (i = 0; i < ARRAY_SIZE(ikeys); i++) {
		input_event(idev, EV_MSC, MSC_SCAN, i);
		input_report_key(idev, ikeys[i], mask & 1);
		mask >>= 1;
	}
}

static void athome_bt_input_frame(int which)
{
	input_sync(conns[which].idev);
}


