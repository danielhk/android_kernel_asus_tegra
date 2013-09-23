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

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include <linux/kthread.h>
#include "bt_athome_le_discovery.h"
#include "bt_athome_hci_extra.h"
#include "bt_athome_le_stack.h"
#include "bt_athome_splitter.h"
#include "bt_athome_logging.h"
#include "bt_athome_proto.h"
#include "bt_athome_user.h"
#include "bt_athome_input.h"
#include "bt_athome_util.h"



#define L2CAP_CHAN_ATHOME_BIT		0x0080
#define EVT_PACKET_LEN			270

/* Calculate the timeout value for (num) missed replies.
 * The +9 is to round up when dividing by 10.
 * The 1+ is so that the timeout will occur slightly after the missed reply, for safety.
 */
#define AAH_BT_CALC_SVC_TIMEOUT(conint, latency, num) \
		((uint16_t)(1 + ((num) * ((((conint) * 1.25 * ((latency) + 1)) + 9) / 10))))

/*
 * Increasing AAH_BT_ACTIVE_CONN_INTERVAL will reduce the quality of recorded audio and touch
 * but improves WiFi bandwidth.
 * Time interval is 1.25 msec per unit.
 *   6 =>  7.50 msec
 *   7 =>  8.75 msec
 *   8 => 10.00 msec
 *   9 => 11.25 msec
 *  10 => 12.50 msec
 *  11 => 13.75 msec
 *  12 => 15.00 msec
 */
#define AAH_BT_ACTIVE_CONN_INTERVAL     8

/*
 * The scan interval determines how often we will listen for advertising packets.
 * Time interval per channel is 0.625 msec per unit.
 * To calculate the time in msec, multiply the last number of the scan setting
 * by the connection interval time. For example, if the connection interval is 8
 * then (AAH_BT_ACTIVE_CONN_INTERVAL * 2 * 23) will give us a scan interval of
 *
 *    10.00 msec * 23 = 230 msec
 *
 * Increasing the normal scan interval can improve WiFi bandwidth slightly.
 * But increasing it will also increase the time it takes for devices
 * to be discovered. But it is very quick in human time.
 */

/* Scan interval while looking for the first device to connect with. */
#define AAH_BT_NORMAL_SCAN_INTERVAL    (AAH_BT_ACTIVE_CONN_INTERVAL * 2 * 11)

/* Scan interval while actively connecting to a remote device. */
#define AAH_BT_CONN_SCAN_INTERVAL      (AAH_BT_ACTIVE_CONN_INTERVAL * 2 * 2)

/*
 * Increasing AAH_BT_ACTIVE_SLAVE_LATENCY will improve battery life in an attached BTLE device.
 * But increasing it will also increase the time it takes for the remote to respond to
 * commands from the master, and can increase the time it takes to exchange keys.
 */
#define AAH_BT_ACTIVE_SLAVE_LATENCY    75

/*
 * Increasing AAH_BT_SERVICE_TIMEOUT_COUNT will make it less likely for the remote
 * control to spontaneously disconnect. This is mainly a problem in spatial coexistence
 * mode when WiFi is transmitting, causing BTLE replies to be lost.
 * But increasing it will also make it take longer for the master to detect a
 * dead remote control and allow reconnection.
 *
 * Spec requires that this be set to a minimum of 2!
 */
#define AAH_BT_SERVICE_TIMEOUT_COUNT   10

#define AAH_BT_ACTIVE_SVC_TIMEOUT     AAH_BT_CALC_SVC_TIMEOUT(AAH_BT_ACTIVE_CONN_INTERVAL,  \
				AAH_BT_ACTIVE_SLAVE_LATENCY, AAH_BT_SERVICE_TIMEOUT_COUNT)

/* AAH_BT_SCAN_WINDOW = time spent listening per interval, 1.6 units/msec,
 * eg. 10 => 6.25 msec  RECOMMENDED
 * eg. 20 => 12.5 msec
 * */
#define AAH_BT_SCAN_WINDOW             10

/*
 * Set "max event length", which determines the number of
 * times to retry when BTLE TX fails.
 * 6 = 2 retries
 * 4 = 1 retry
 * 2 = 0 retries
 * This was ignored by some Marvell drivers before July 2013.
 */
#define AAH_BT_MAX_EVENT_LENGTH         4

/* Tell Marvell to give passive BTLE scans a lower priority than WiFi TX. */
#define AAH_BT_USE_LOWER_SCAN_PRIORITY   1

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
#define CONN_STATE_TEARDOWN		9 /* no longer OK, being torn down */

/* Per-remote state struct */
struct athome_bt_conn {
	/* BT state */
	uint16_t handle;
	uint8_t state, pwr, next_pwr;
	bool gone, timeout, outstanding, in_modeswitch;
	bdaddr_t MAC;
	uint8_t LTK[AAH_BT_LTK_SZ];
	uint32_t proto_ver;
	struct timer_list timer;

	uint8_t entropy[AAH_BT_ENTROPY_SZ];

	/* stats */
	uint64_t last_time;
	struct athome_bt_stats stats;
};

struct athome_bt_mode_settings
{
	uint16_t conn_interval;
	uint16_t slave_latency;
	uint16_t svc_timeout;
};

struct athome_bt_thread_context
{
	uint8_t nconns;
	/* TODO Move other globals that are only used by the thread into this context. */
};

/* stack state */
#define INVALID			0xFFFF
static struct sk_buff_head rx_evt;
static struct sk_buff_head rx_dat;
static DECLARE_WAIT_QUEUE_HEAD(rx_wait); /* wait to get evt/data/chg */
static atomic_t state_chg = ATOMIC_INIT(0);
static atomic_t in_shutdown = ATOMIC_INIT(0);
/* This reflects the credits we have for sending data.  We take
 * one credit from the available pool.  The send code uses it
 * to make sure we never send more than one data packet at a time.
 */
static DEFINE_SEMAPHORE(tx_credit);
static struct task_struct *aah_thread = NULL;

/* Completion for command response */
static DECLARE_COMPLETION(cmd_response);

/* protects the following data */
static DEFINE_SPINLOCK(stack_state_lock);
static uint16_t disconnH = 0;
static uint16_t inflight_ogf = INVALID;
static uint16_t inflight_ocf = INVALID;
static struct athome_bt_conn conns[ATHOME_RMT_MAX_CONNS] = {{0,},};
static uint8_t *cmd_rsp_buf;
static size_t cmd_rsp_buf_size;
static bool devices_exist = false;



/*
 * These settings are tuned to work with the Bemote Power manager.
 * The connections intervals are kept low so that audio can send packets quickly enough,
 * even in idle mode.
 * The latency is high in idle mode to reduce power consumption when hibernating.
 * The index into these arrays is the sleep_level.
 *
 * WARNING the settings for different modes must differ in some way or we cannot
 * detect which mode we are in.
 *
 * interval and latency units are 1.25ms
 * timeout min value: ci * 1.25 * (lat + 1) * 2 / 10
 *
 * These settings are buried in the function to limit their scope.
 */
static const struct athome_bt_mode_settings *athome_bt_get_mode_settings(uint32_t proto_ver,
			int mode)
{
	static const struct athome_bt_mode_settings old_mode_settings[ATHOME_MODE_MAX] =
	{
		{.conn_interval = 80, .slave_latency = 50, .svc_timeout = 3000}, /* IDLE */
		{.conn_interval = 40, .slave_latency = 10, .svc_timeout =  200}, /* SEMI_IDLE */
		{.conn_interval =  6, .slave_latency =  2, .svc_timeout =   20}, /* ACTIVE */
	};

	static const struct athome_bt_mode_settings new_mode_settings[ATHOME_MODE_MAX] =
	{
		{.conn_interval = 17, .slave_latency =  60, .svc_timeout = 600}, /* IDLE */
		{.conn_interval = 11, .slave_latency = 101, .svc_timeout = 1000}, /* SEMI_IDLE */
		{  .conn_interval =  AAH_BT_ACTIVE_CONN_INTERVAL,
		   .slave_latency = AAH_BT_ACTIVE_SLAVE_LATENCY,
		   .svc_timeout = AAH_BT_ACTIVE_SVC_TIMEOUT}, /* ACTIVE */
	};

	/*
	 * There was a problem in early versions of Bemote that caused it to
	 * timeout during encryption if the slave latency was too high.
	 * b/10198360
	 */
	return (proto_ver < PROTO_VERSION_SPAKE)
		? &old_mode_settings[mode]
		: &new_mode_settings[mode];
}

/*
 * About "disconnH": We have a finite number of connection slots (n), but due
 * to the asyncronous nature of all we do, sometimes we can end up with n+1
 * connections made before we realize it. This means we need to drop the last
 * one. To drop it we issue a "disconnect" command. This command produces a
 * response. How do we know that a "disconnect response" is ours? We check our
 * connection list. But... our list only fits n items, so we need a slot to
 * keep an extra connection ID just to track its disconnect event to make sure
 * we do not send it to the userspace EDR stack by mistake. "disconnH" is used
 * for exactly that purpose. Since ACL connection IDs are 12-bits. We use 12th
 * bit of it to indicate it is valid. Whenever we need to close a connection
 * due to not having enough slots, we set "disconnH" to the connection's ID,
 * set its 12th bit, and then issue the "disconnect" command. When the response
 * comes, we detect it by having it match the connection ID to lower 12 bits of
 * "disconnH" and checking 12th bit of "disconnH" for validity. Then we know
 * that the connection closed successfully and we reset "disconnH" to zero,
 * which marks it invalid due to a clear 12th bit.
 */



/* returns <0 if not found, call only with state lock held */
static int athome_bt_find_by_mac_l(const bdaddr_t *macP)
{
	int i;

	for(i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
		if (bacmp(&conns[i].MAC, macP))
			continue;

		if (conns[i].gone)
			continue;

		if (conns[i].state == CONN_STATE_INVALID)
			continue;

		return i;
	}

	return -1;
}
/* returns <0 if not found, call only with state lock held */
static int athome_bt_find_connection_l(uint16_t handle)
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

/* returns <0 if not found, call only with state lock not held */
static int athome_bt_find_connection(uint16_t handle)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&stack_state_lock, flags);
	i = athome_bt_find_connection_l(handle);
	spin_unlock_irqrestore(&stack_state_lock, flags);

	return i;
}

static uint64_t get_time(void)
{
	struct timespec t;
	uint64_t ret;

	ktime_get_ts(&t);
	ret = t.tv_sec;
	ret *= NSEC_PER_SEC;
	ret += t.tv_nsec;

	return ret;
}

/* call with spinlock held only */
static void athome_bt_update_time_stats_l(unsigned which)
{
	uint64_t time = get_time();
	uint64_t spent = time - conns[which].last_time;

	conns[which].stats.mode_times[conns[which].pwr] += spent;
	conns[which].last_time = time;
}

bool athome_bt_connection_went_down(uint16_t handle)
{
	struct bt_athome_disconnected de;
	int i;
	bool ret = true;
	unsigned long flags;

	spin_lock_irqsave(&stack_state_lock, flags);
	i = athome_bt_find_connection_l(handle);
	if (i >= 0) {
		conns[i].gone = true;
		if (conns[i].outstanding)
			up(&tx_credit);
		athome_bt_update_time_stats_l(i);
		bacpy(&de.MAC, &conns[i].MAC);
		memcpy(&de.stats, &conns[i].stats, sizeof(de.stats));
		athome_bt_usr_enqueue(BT_ATHOME_EVT_DISCONNECTED, &de,
								sizeof(de));
	} else if (handle == (disconnH & ACL_CONN_MASK))
		disconnH = 0;
	else
		ret = false;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	if (i >= 0)
		aahlog("device #%d went down\n", i);

	return ret;
}

bool athome_bt_encr_changed(uint16_t handle)
{
	return athome_bt_find_connection(handle) >= 0;
}


bool athome_bt_encr_refreshed(uint16_t handle)
{
	return athome_bt_find_connection(handle) >= 0;
}

bool athome_bt_cmd_sta_or_compl(uint16_t ogf, uint16_t ocf,
				const uint8_t *buf_p, uint32_t len)
{
	unsigned long flags;
	/* Guarantee atomic update to these three globals
	 * and that SMP access to them have fresh data.
	 */
	spin_lock_irqsave(&stack_state_lock, flags);
	if (ogf == inflight_ogf && ocf == inflight_ocf) {
		/* definitely a reply for us */
		inflight_ogf = INVALID;
		inflight_ocf = INVALID;
		if (cmd_rsp_buf_size > 0) {
			WARN(len > cmd_rsp_buf_size,
			     "ble response size %d larger than rsp buf size %d\n",
			     len, cmd_rsp_buf_size);
			memcpy(cmd_rsp_buf, buf_p, min(cmd_rsp_buf_size, len));
		}
		spin_unlock_irqrestore(&stack_state_lock, flags);
		complete(&cmd_response);
		return true;
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);
	return false;
}

bool athome_bt_compl_pkts(uint16_t handle, uint16_t num)
{
	int i;
	bool ret = false;
	unsigned long flags;

	spin_lock_irqsave(&stack_state_lock, flags);
	i = athome_bt_find_connection_l(handle);
	if (i >= 0) {
		if (num) {
			if (!conns[i].outstanding)
				aahlog("unexpected send on conn %d\n", i);
			conns[i].outstanding = 0;
			while(num--)
				up(&tx_credit);
		}
		ret = true;
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);

	return ret;
}

/* pass i < 0 to do a lookup by mac */
static void athome_bt_start_encr(int i)
{
	struct athome_tx_pkt_enc enc_pkt;
	struct athome_bt_known_remote *remote;
	unsigned long flags;
	bool initial;

	get_random_bytes(&enc_pkt.entropy, sizeof(enc_pkt.entropy));

	spin_lock_irqsave(&stack_state_lock, flags);
	BUG_ON(i >= ATHOME_RMT_MAX_CONNS);
	initial = (conns[i].state == CONN_STATE_JUST_ESTABLISHED ||
			conns[i].state == CONN_STATE_BINDING);

	remote = athome_bt_find_known(&conns[i].MAC);
	if (remote)
		memcpy(conns[i].LTK, &remote->LTK, sizeof(remote->LTK));
	else {
		spin_unlock_irqrestore(&stack_state_lock, flags);
		aahlog("device unknown unexpectedly\n");
		return;
	}

	conns[i].timeout = false;
	conns[i].state = initial ? CONN_STATE_ENCRYPT_SETUP :
					CONN_STATE_REENCRYPT_SETUP;

	memcpy(conns[i].entropy, enc_pkt.entropy, AAH_BT_ENTROPY_SZ);
	mod_timer(&conns[i].timer,
		  jiffies + msecs_to_jiffies(ENCRYPT_DAT_TIMEOUT));

	spin_unlock_irqrestore(&stack_state_lock, flags);

	/* send the packet */
	athome_bt_send_data(&conns[i].MAC, ATHOME_PKT_TX_ENC, &enc_pkt,
						sizeof(enc_pkt), false);
}

/* called by userland */
void athome_bt_start_encr_for_mac(const bdaddr_t *macP)
{
	int i;
	struct athome_bt_known_remote *remote;
	unsigned long flags;

	spin_lock_irqsave(&stack_state_lock, flags);
	i = athome_bt_find_by_mac_l(macP);
	if (i < 0) {
		spin_unlock_irqrestore(&stack_state_lock, flags);
		aahlog("start encr failed - no such connection\n");
		return;
	} else if (conns[i].state != CONN_STATE_JUST_ESTABLISHED &&
		   conns[i].state != CONN_STATE_BINDING &&
		   conns[i].state != CONN_STATE_DATA) {
		spin_unlock_irqrestore(&stack_state_lock, flags);
		aahlog("start encr failed - not in correct state,"
		       " state is %d\n", conns[i].state);
		return;
	}
	remote = athome_bt_find_known(&conns[i].MAC);
	if (remote->bind_mode == 0) {
		spin_unlock_irqrestore(&stack_state_lock, flags);
		aahlog("start encr failed - half bound remote "
		       "will be auto encrypted\n");
		return;
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);
	athome_bt_start_encr(i);
}

void athome_bt_disc_from_mac(const bdaddr_t *macP)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&stack_state_lock, flags);
	i = athome_bt_find_by_mac_l(macP);

	if (i >= 0) {
		conns[i].state = CONN_STATE_DISCONNECT;
		atomic_set(&state_chg, 1);
		wake_up_interruptible(&rx_wait);
	}

	spin_unlock_irqrestore(&stack_state_lock, flags);
}

void athome_bt_get_state(struct bt_athome_state *state)
{
	unsigned long flags;
	int i;

	state->num = 0;
	spin_lock_irqsave(&stack_state_lock, flags);
	for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
		uint8_t conn_state = BT_ATHOME_STATE_UNKNOWN;

		if (conns[i].state == CONN_STATE_INVALID || conns[i].gone)
			continue;

		bacpy(&state->remotes[state->num].MAC, &conns[i].MAC);

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

bool athome_bt_get_stats(struct athome_bt_stats *stats, const bdaddr_t *macP)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&stack_state_lock, flags);
	i = athome_bt_find_by_mac_l(macP);
	if (i >= 0) {
		athome_bt_update_time_stats_l(i);
		memcpy(stats, &conns[i].stats, sizeof(*stats));
	}
	spin_unlock_irqrestore(&stack_state_lock, flags);
	return (i != ATHOME_RMT_MAX_CONNS);
}

static void athome_bt_enqueue_data(struct sk_buff_head *Q, const void *ptr,
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

void athome_bt_process_le_evt(const uint8_t *buf_p, uint32_t len)
{
	athome_bt_enqueue_data(&rx_evt, buf_p, len);
	wake_up_interruptible(&rx_wait);
}

bool athome_bt_process_le_data(uint16_t handle, const uint8_t *buf_p,
								uint32_t len)
{
	if (athome_bt_find_connection(handle) >= 0) {
		athome_bt_enqueue_data(&rx_dat, buf_p, len);
		wake_up_interruptible(&rx_wait);
		return true;
	}

	return false;
}

bool athome_bt_send_data(const bdaddr_t *macP, uint8_t typ,
			 const void *dataP, uint8_t data_sz, bool for_user)
{

	uint8_t buf[ACL_AAH_PKT_SZ + HCI_ACL_HDR_SIZE] = {0, };
	struct hci_acl_hdr *h = (struct hci_acl_hdr*)buf;
	uint8_t *packet = (uint8_t*)(h + 1);
	const uint8_t *data = (const uint8_t*)dataP;
	uint16_t handle;
	unsigned long flags;
	int i, which;

	if (data_sz > ACL_AAH_PKT_SZ) {
		aahlog("trying to send too large packet (%d)\n", data_sz);
		return false;
	}

	switch (typ) {
		case ATHOME_PKT_TX_SET_PARAM:
			aahlog("TX_SET_PARAM p#=%d, d[0]=0x%02X,  d[1]=0x%02X,\n",
				data[0], data[1], data[2]);
			break;
		default:
			break;
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
	down(&tx_credit);

	/* get handle info and record we're sending */
	spin_lock_irqsave(&stack_state_lock, flags);
	which = athome_bt_find_by_mac_l(macP);
	if (which < 0) {
		/* we failed to find it */
		aahlog("could not find by MAC\n");
	} else if (for_user && conns[which].state == CONN_STATE_ENCRYPT_SETUP) {
		/* users cannot send before we encrypt */
		aahlog("cannot send data before encryption\n");
		which = -1;
	} else if (conns[which].state == CONN_STATE_JUST_ESTABLISHED ||
			conns[which].state == CONN_STATE_INVALID ||
			conns[which].state == CONN_STATE_ENCRYPTING ||
			conns[which].state == CONN_STATE_REENCRYPTING ||
			conns[which].state == CONN_STATE_DISCONNECT ||
			conns[which].state == CONN_STATE_TEARDOWN) {
		/* in these states nobody can send */
		aahlog("cannot send data in state %d\n", conns[which].state);
		which = -1;
	} else if (conns[which].state == CONN_STATE_BINDING &&
					typ != ATHOME_PKT_TX_ACK &&
					typ != ATHOME_PKT_TX_SET_PARAM &&
					typ != ATHOME_PKT_TX_ENC &&
					typ != ATHOME_PKT_TX_PAIRING) {
		/* only some packet types allowed in binding mode */
		/*
			XXX: TODO: once binding is proper, stop allowing
			set_param command here
		*/
		aahlog("cannot send packet type %d when binding\n", typ);
		which = -1;
	}

	if (which < 0) {

		spin_unlock_irqrestore(&stack_state_lock, flags);
		up(&tx_credit);

		/* cannot send - we drop it */
		aahlog("dropping impossible send request\n");
		return false;
	}

	if (conns[which].outstanding)
		aahlog("send while outstanding on %d\n", i);
	conns[which].outstanding = true;
	handle = conns[which].handle;
	conns[which].stats.pkts_tx++;
	conns[which].stats.bytes_tx += data_sz;
	if (typ == ATHOME_PKT_TX_NFC)
		conns[which].stats.nfc_tx++;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	/* craft ACL header */
	w16LE(&h->handle, handle);
	w16LE(&h->dlen, data_sz + 1);

	/* let the chip have it */
	athome_bt_send_to_chip(HCI_ACLDATA_PKT, buf,
					data_sz + 1 + HCI_ACL_HDR_SIZE);

	return true;
}

/* send command and then wait for complete or status event. nonzero -> quit */
static int __attribute__((warn_unused_result))
		athome_bt_simple_cmd(uint32_t ogf, uint32_t ocf, uint8_t plen,
					const uint8_t *dataIn, unsigned outLen,
					uint8_t *dataOut)
{
	uint8_t cmd[255 + HCI_COMMAND_HDR_SIZE];
	struct hci_command_hdr *hdr = (struct hci_command_hdr*)cmd;
	unsigned long flags;

	memcpy(&cmd[HCI_COMMAND_HDR_SIZE], dataIn, plen);
	hdr->plen = plen;
	hdr->opcode = cpu_to_le16(hci_opcode_pack(ogf, ocf));

	/* this is only used by the le stack thread, so no locking
	 * needed.
	 */
	BUG_ON(current != aah_thread);

	/* Guarantee atomic update to the globals
	 * and that SMP access to them have fresh data.
	 */
	spin_lock_irqsave(&stack_state_lock, flags);
	BUG_ON(inflight_ogf != INVALID || inflight_ocf != INVALID);
	inflight_ogf = ogf;
	inflight_ocf = ocf;
	cmd_rsp_buf = dataOut;
	cmd_rsp_buf_size = outLen;
	spin_unlock_irqrestore(&stack_state_lock, flags);

	athome_bt_send_to_chip(HCI_COMMAND_PKT, cmd,
			       plen + HCI_COMMAND_HDR_SIZE);
	wait_for_completion(&cmd_response);

	return atomic_read(&in_shutdown);
}

static int athome_bt_do_encrypt(int which, const uint8_t *LTK, bool initial)
{
	uint8_t buf[EVT_PACKET_LEN], *parP, k[AAH_BT_LTK_SZ];
	uint8_t *evt_data = buf + HCI_EVENT_HDR_SIZE;
	struct hci_ev_cmd_status *stat =
				(struct hci_ev_cmd_status*)evt_data;
	uint8_t *cmd_cmpl_data = evt_data +
				sizeof(struct hci_ev_cmd_complete);
	unsigned long flags;
	unsigned i;

	parP = buf;
	for (i = 0; i < AAH_BT_LTK_SZ; i++)
		put8LE(&parP, LTK[i]);
	for (i = 0; i < AAH_BT_ENTROPY_SZ; i++)
		put8LE(&parP, conns[which].entropy[i]);

	i = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Encrypt, parP - buf, buf,
							sizeof(buf), buf);
	if (i)
		return i;

	parP = cmd_cmpl_data;
	if (get8LE(&parP)) {	/* check status */
		aahlog("failed to AES encrypt\n");
		conns[which].state = CONN_STATE_DISCONNECT;
		return 0;
	}

	memcpy(k, parP, AAH_BT_LTK_SZ);
	aahlog("KEY [%d]: %02X %02X %02X %02X %02X %02X %02X %02X"
	       "%02X %02X %02X %02X %02X %02X %02X %02X\n", which,
	       k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7],
	       k[8], k[9], k[10], k[11], k[12], k[13], k[14], k[15]);

	parP = buf;
	put16LE(&parP, conns[which].handle);
	put64LE(&parP, ENCR_RND_NUM);
	put16LE(&parP, ENCR_DIV);
	for (i = 0; i < AAH_BT_LTK_SZ; i++)
		put8LE(&parP, k[i]);
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

/* returns 1 if we've been asked to quit */
static int athome_bt_data_rx(int which, uint8_t *in_data, uint32_t len)
{
	uint8_t *payload = in_data + HCI_ACL_HDR_SIZE;
	bool ok = false, secure = true;
	uint8_t i, type = 0, data_buf[sizeof(struct bt_athome_send_data) +
							ACL_AAH_PKT_SZ];
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
	if (len - HCI_ACL_HDR_SIZE != r16LE(&acl->dlen)) {
		aahlog("acl data bad size (got %u expected %u)\n",
			len - HCI_ACL_HDR_SIZE, r16LE(&acl->dlen));
		return 0;
	}
	len = r16LE(&acl->dlen);
	if (len > ACL_AAH_PKT_SZ)
		len = ACL_AAH_PKT_SZ;

	/* read and reassemble the data */
	if (len > 2) {
		/* all our valid packets have len >= 3 */
		memcpy(data, payload, 2);
		type = payload[2];
	}
	if (len > 3)
		memcpy(data + 2, payload + 3, len - 3);
	len--;

	if (!(type & L2CAP_CHAN_ATHOME_BIT)) {
		aahlog("got ACL data with missing bit. Dropping\n");
		return 0;
	}
	type &= 0x7F;
	usr_data->pkt_typ = type;

	BUG_ON(which >= ATHOME_RMT_MAX_CONNS);
	spin_lock_irqsave(&stack_state_lock, flags);
	conns[which].stats.pkts_rx++;
	conns[which].stats.bytes_rx += len + 1; /* type byte */
	bacpy(&usr_data->MAC, &conns[which].MAC);
	switch(conns[which].state) {
	case CONN_STATE_REENCRYPT_SETUP:
		if (type != ATHOME_PKT_RX_ACK ||
					data[0] != ATHOME_PKT_TX_ENC) {
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
			if (conns[which].entropy[i] != data[i + 1])
				break;
		}
		if (i != AAH_BT_LTK_SZ) {
			aahlog("entropy echo doesn't match(b %d) - drop\n", i);
			conns[which].state = CONN_STATE_DISCONNECT;
			aahlog("expected: %02x%02x%02x%02x%02x%02x%02x%02x"
			       "%02x%02x%02x%02x%02x%02x%02x%02x\n",
			       conns[which].entropy[0],
			       conns[which].entropy[1],
			       conns[which].entropy[2],
			       conns[which].entropy[3],
			       conns[which].entropy[4],
			       conns[which].entropy[5],
			       conns[which].entropy[6],
			       conns[which].entropy[7],
			       conns[which].entropy[8],
			       conns[which].entropy[9],
			       conns[which].entropy[10],
			       conns[which].entropy[11],
			       conns[which].entropy[12],
			       conns[which].entropy[13],
			       conns[which].entropy[14],
			       conns[which].entropy[15]);
			aahlog("received: %02x%02x%02x%02x%02x%02x%02x%02x"
			       "%02x%02x%02x%02x%02x%02x%02x%02x\n",
			       data[1], data[2], data[3], data[4],
			       data[5], data[6], data[7], data[8],
			       data[9], data[10], data[11], data[12],
			       data[13], data[14], data[15], data[16]);
			break;
		}
		aahlog("entropy echo ok - encryption approved\n");
		mod_timer(&conns[which].timer,
			  jiffies + msecs_to_jiffies(ENCRYPT_TIMEOUT));
		conns[which].timeout = false;
		spin_unlock_irqrestore(&stack_state_lock, flags);
		return athome_bt_do_encrypt(which, conns[which].LTK, initial);

	case CONN_STATE_DATA:
		ok = true;
		break;

	case CONN_STATE_BINDING:
		/* only some packets allowed in bind mode */
		ok = type == ATHOME_PKT_RX_ACK ||
			/*
			 * TODO Why isn't this code also looking for ATHOME_PKT_RX_TOUCH_V2
			 * and ATHOME_PKT_RX_BTN_V2? Binding works.
			 * Maybe it doesn't need to look for input at all.
			 */
				type == ATHOME_PKT_RX_INPUT ||
				type == ATHOME_PKT_RX_PAIRING;
		secure = false;
		break;
	}

	/* done here so as to be under spinlock */
	if (ok) switch (type) {
	case ATHOME_PKT_RX_INPUT:
	case ATHOME_PKT_RX_TOUCH_V2:
	case ATHOME_PKT_RX_BTN_V2:
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
		aahlog("got data while not allowed. Dropping. type = %d, state = %d\n",
			type, conns[which].state);
		return 0;
	}

	switch (type) {
	case ATHOME_PKT_RX_INPUT:
		if (LOG_INPUT_SPEW)
			aahlog("[%d] input event with time %d and data: {\n",
				which,
				inp->info & ATHOME_INPUT_INFO_MASK_TIMESTAMP);

		if (inp->info & ATHOME_INPUT_INFO_MASK_HAS_BTN)
			inp_btn = (struct athome_pkt_rx_input_btn*)(inp + 1);
		if (inp->info & ATHOME_INPUT_INFO_MASK_HAS_TOUCH) {
			inp_tch = inp_btn ?
				(struct athome_pkt_rx_input_touch*)
								(inp_btn + 1) :
				(struct athome_pkt_rx_input_touch*)
								(inp + 1);
		}
		if (inp_btn) {
			uint32_t buttons = r32LE(&inp_btn->btn_mask);
			if (LOG_INPUT_SPEW)
				aahlog(" -> buttons: 0x%08X\n", buttons);

			if (!secure && (buttons & ~SECURE_BTN_MASK)) {
				buttons &= SECURE_BTN_MASK;
				if (LOG_INPUT_SPEW)
					aahlog(" ->  now: 0x%08X\n", buttons);
			}

			athome_bt_input_send_buttons(which,
						buttons & ~SECURE_BTN_MASK);

			/* send binding buttons to userspace */
			if (buttons & SECURE_BTN_MASK) {
				struct bt_athome_bind_key bb;
				bacpy(&bb.MAC, &conns[which].MAC);
				bb.key = !!(buttons & ATHOME_PAIR_BTN_CHAR);
				athome_bt_usr_enqueue(BT_ATHOME_EVT_BIND_KEY,
							&bb, sizeof(bb));
			}
		}

		if (inp_tch && secure) {
			uint16_t x, y;

			for (i = 0; i < ATHOME_MAX_FINGERS; i++) {
				bool is_down;

				x = r16LE(&inp_tch->fingers[i].X);
				y = r16LE(&inp_tch->fingers[i].Y);
				is_down = ((x != AAH_RAW_X_MAX) ||
					   (y != AAH_RAW_Y_MAX));

				athome_bt_input_send_touch(which, i, x, y,
							   is_down);
			}
		}

		if (LOG_INPUT_SPEW)
			aahlog("}\n");
		i = inp->info & ATHOME_INPUT_INFO_MASK_TIMESTAMP;

		athome_bt_input_calculate_time(which,
				i == ATHOME_INPUT_INFO_MASK_TIMESTAMP
				? AAH_BT_UNKNOWN_TS_DELTA
				: ((long)i * 10000));
		athome_bt_input_frame(which);
		break;

	case ATHOME_PKT_RX_TOUCH_V2: {
		if (secure) {
			struct athome_pkt_rx_touch_v2 *tv2;
			bool is_down;

			tv2 = (struct athome_pkt_rx_touch_v2*)data;

			is_down = (tv2->x & 0x8000) != 0;
			tv2->x <<= 1;
			tv2->y <<= 1;

			athome_bt_input_calculate_time(which,
					tv2->ts_delta == ((uint16_t)(-1))
					? AAH_BT_UNKNOWN_TS_DELTA
					: ((long)tv2->ts_delta * 10));
			athome_bt_input_send_touch(which, 0,
					tv2->x, tv2->y, is_down);
			athome_bt_input_frame(which);
		}
	} break;

	case ATHOME_PKT_RX_BTN_V2: {
		if (secure) {
			struct athome_pkt_rx_btn_v2 *bv2;
			bool is_down;

			bv2 = (struct athome_pkt_rx_btn_v2*)data;

			is_down = (bv2->data & 0x80) != 0;

			athome_bt_input_calculate_time(which,
					bv2->ts_delta == ((uint16_t)(-1))
					? AAH_BT_UNKNOWN_TS_DELTA
					: ((long)bv2->ts_delta * 10));
			athome_bt_input_send_button(which,
					(bv2->data & ~0x80),
					is_down);
			athome_bt_input_frame(which);
		}
	} break;


	case ATHOME_PKT_RX_AUDIO_0:
	case ATHOME_PKT_RX_AUDIO_1:
	case ATHOME_PKT_RX_AUDIO_2:
	case ATHOME_PKT_RX_AUDIO_3:
#ifdef CONFIG_SND_MIC_BTLE_SBC
		athome_bt_audio_dec(which, type - ATHOME_PKT_RX_AUDIO_0,
			data, len,
			conns[which].proto_ver >= PROTO_VERSION_AUDIO_V2);
#endif
		break;

	case ATHOME_PKT_RX_MODESWITCH:
		if (msw->mode != ATHOME_MODE_ACTIVE &&
				msw->mode != ATHOME_MODE_SEMIIDLE)
			msw->mode = ATHOME_MODE_IDLE;
		conns[which].next_pwr = msw->mode;
		athome_bt_led_show_event((msw->mode == ATHOME_MODE_ACTIVE)
			? HACK_LED_EVENT_AWAKE : HACK_LED_EVENT_ASLEEP);
		if (LOG_MODESWITCH) {
			if (conns[which].pwr == ATHOME_MODE_ACTIVE) {
				aahlog("Requested modeswitch to %d, conn %d, "
				       "#pkts_rx = %6llu, "
				       "#pkts_input = %6llu\n",
				       msw->mode, which,
				       conns[which].stats.pkts_rx,
				       conns[which].stats.input);
			} else {
				aahlog("Requested modeswitch to %d, conn %d\n",
				       msw->mode, which);
			}
		}
		/* Deliberate fall-through to the default case in order to send
		 * the mode switch packet to user-land for further
		 * logging/backend analysis.
		 */

	default:
		athome_bt_usr_enqueue(BT_ATHOME_EVT_DATA, data_buf,
				len + sizeof(struct bt_athome_got_data));
		break;
	}
	return 0;
}

static int athome_bt_modeswitch(int which, unsigned mode)
{
	/* TODO:
	 * Re-enable this functionality once we have figured out why it takes so
	 * long to change from a long slave-latency to a short one while waking
	 * up.
	 * OR maybe not. We have seen intermittent problems with the TI chip and PSoC5 getting
	 * out of sync when enabled. https://b.corp.google.com/issue?id=9206731
	 *
	 * And Doug has tested WiFi and is not seeing any impact on WiFi from BTLE.
	 */
#define CHANGE_SETTINGS_ON_MODESWITCH  0
#if CHANGE_SETTINGS_ON_MODESWITCH
	const struct athome_bt_mode_settings *mode_settings =
		athome_bt_get_mode_settings(conns[which].proto_ver, mode);
	uint8_t buf[EVT_PACKET_LEN];
	uint8_t *parP = buf;
	int ret;
	struct hci_ev_cmd_status *stat =
			(struct hci_ev_cmd_status*)(buf + HCI_EVENT_HDR_SIZE);

	put16LE(&parP, conns[which].handle);
	put16LE(&parP, mode_settings->conn_interval); /* min conn intr */
	put16LE(&parP, mode_settings->conn_interval); /* max conn intr */
	put16LE(&parP, mode_settings->slave_latency);
	put16LE(&parP, mode_settings->svc_timeout);
	put16LE(&parP, 20);   /* min durX.62ms */
	put16LE(&parP, 20);   /* max durX.62ms */

	if (LOG_MODESWITCH)
		aahlog("Starting modeswitch to %d for conn %d\n", mode, which);

	conns[which].in_modeswitch = true;
	ret = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Connection_Update,
		parP - buf, buf, sizeof(buf), buf);
	if (ret)
		return ret;

	if (stat->status)
		aahlog("failed conn update try.\n");
#else
	conns[which].pwr = conns[which].next_pwr;
#endif

	return 0;
}

static int athome_bt_disconnect(uint16_t handle)
{
	uint8_t buf[EVT_PACKET_LEN];
	uint8_t *parP = buf;


	put16LE(&parP, handle);
	put8LE(&parP, 0x13);	/* reason = user request */
	return athome_bt_simple_cmd(HCI_OGF_Link_Control, HCI_CMD_Disconnect,
				parP - buf, buf, 0, NULL);
}

/* something timed out - find out what and why */
static void athome_bt_timeout(unsigned long which)
{
	unsigned long flags;

	if (which != ATHOME_RMT_MAX_CONNS) {

		/* per-connection timeout - mark which */
		spin_lock_irqsave(&stack_state_lock, flags);
		conns[which].timeout = true;
		spin_unlock_irqrestore(&stack_state_lock, flags);
	}
	/* wake the thread */
	atomic_set(&state_chg, 1);
	wake_up_interruptible(&rx_wait);
}

bool athome_bt_is_connected_to(const bdaddr_t *macP)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&stack_state_lock, flags);
	i = athome_bt_find_by_mac_l(macP);
	spin_unlock_irqrestore(&stack_state_lock, flags);

	return i >= 0;
}

/*
 * The buf array should contain at least EVT_PACKET_LEN bytes.
 * We pass in the buf to avoid allocating large arrays on the stack.
 */
static int athome_bt_set_scan_enabled(bool enabled, uint8_t *buf_p, size_t blen)
{
	char *mode = enabled ? "enabled" : "disabled";
	uint8_t *evt_data = buf_p + HCI_EVENT_HDR_SIZE;
	uint8_t *cmd_cmpl_data = evt_data + sizeof(struct hci_ev_cmd_complete);
	uint8_t *parP;
	int err;
	int sta;

	parP = buf_p;
	put8LE(&parP, enabled ? 1 : 0);	/* scanning enabled or disabled */
	put8LE(&parP, 0);	/* duplicate filter off */
	err = athome_bt_simple_cmd(HCI_OGF_LE,
				HCI_LE_Set_Scan_Enable,
				parP - buf_p, buf_p,
				blen, buf_p);
	if (err) {
		aahlog("failed to send cmd to set scan parameters.\n");
		return err;
	}
	sta = *cmd_cmpl_data;
	if (sta)	/* check status */
		aahlog("failed to set scan %s, sta=%d.\n", mode, sta);
	else
		aahlog("scan %s\n", mode);
	return 0;
}

/*
 * The buf array should contain at least EVT_PACKET_LEN bytes.
 * We pass in the buf to avoid allocating large arrays on the stack.
 */
static int athome_bt_set_scan_timing(int scan_interval, int scan_window,
				     uint8_t *buf_p, size_t blen)
{
	uint8_t *evt_data = buf_p + HCI_EVENT_HDR_SIZE;
	uint8_t *cmd_cmpl_data = evt_data + sizeof(struct hci_ev_cmd_complete);
	uint8_t *parP;
	int err;
	int sta;

	aahlog("set scan timing: interval = %d, window = %d\n", scan_interval, scan_window);
	parP = buf_p;
	put8LE(&parP, 0);	/* passive scan */
	put16LE(&parP, scan_interval); /* time between listens per channel */
	put16LE(&parP, scan_window); /* time spent listening per interval */
	put8LE(&parP,  0);	/* use real mac address on packets we send */
	put8LE(&parP, 0);	/* accept all advertisement packets */
	err = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Set_Scan_Parameters,
				   parP - buf_p, buf_p, blen, buf_p);
	if (err) {
		aahlog("failed to send cmd to set scan parameters.\n");
		return err;
	}

	sta = *cmd_cmpl_data;
	if (sta)	/* check status */
		aahlog("failed to set scan timing, sta = %d\n", sta);
	return 0;
}

static int athome_bt_host_setup(void)
{
	uint8_t buf[EVT_PACKET_LEN];
	uint8_t *evt_data = buf + HCI_EVENT_HDR_SIZE;
	uint8_t *cmd_cmpl_data = evt_data + sizeof(struct hci_ev_cmd_complete);
	uint8_t aclBufNum;
	uint16_t aclBufSz;
	uint8_t *parP;
	uint i;
	int err;

	aahlog("host setup\n");
	parP = buf;

	aahlog("turning le and le simul on\n");
	put8LE(&parP, 1);		/* le on */
	put8LE(&parP, 1);		/* le simul on */
	err = athome_bt_simple_cmd(HCI_OGF_Controller_And_Baseband,
				HCI_Write_LE_Host_Support, parP - buf,
				buf, sizeof(buf), buf);
	if (err)
		return err;
	parP = cmd_cmpl_data;
	if (get8LE(&parP)) {	/* check status */
		aahlog("failed to set LE support. bailing.\n");
		return -3;
	}

	aahlog("read buffer size\n");
	err = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Read_Buffer_Size,
						0, NULL, sizeof(buf), buf);
	if (err)
		return err;
	parP = cmd_cmpl_data;
	if (get8LE(&parP)) {	/* check status */
		aahlog("failed to get buffer sizes. bailing. \n");
		return -3;
	}
	aclBufSz = get16LE(&parP);
	aclBufNum = get8LE(&parP);
	aahlog("LE chip features %d %d-byte buffers\n", aclBufNum, aclBufSz);

	aahlog("LE INFO:\n");

	aahlog("read local supported features\n");
	err = athome_bt_simple_cmd(HCI_OGF_LE,
				HCI_LE_Read_Local_Supported_Features,
				0, NULL, sizeof(buf), buf);
	if (err)
		return err;
	if (cmd_cmpl_data[1] & 1)
		aahlog(" -> chip supports LE encryption\n");
	cmd_cmpl_data[1] &= ~1;
	for (i = 0; i < 64; i++)
		if (cmd_cmpl_data[i / 8 + 1]  & (1 << (i & 7)))
			aahlog(" -> chip supports unknown LE feature %d\n", i);


	aahlog("read supported states\n");
	err = athome_bt_simple_cmd(HCI_OGF_LE, HCI_LE_Read_Supported_States, 0,
				NULL, sizeof(buf), buf);
	if (err)
		return err;

	aahlog(" -> normally:\n");
	if (!cmd_cmpl_data[1])
		aahlog("    NONE\n");
	else {
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
	}

	aahlog(" -> while passive scanning:\n");
	if (!(cmd_cmpl_data[2] & 0x0F))
		aahlog("    NONE\n");
	else {
		if (cmd_cmpl_data[2] & 0x01)
			aahlog("   -> non-connectible advertising\n");
		if (cmd_cmpl_data[2] & 0x02)
			aahlog("   -> scannable advertising\n");
		if (cmd_cmpl_data[2] & 0x04)
			aahlog("   -> connectible advertising\n");
		if (cmd_cmpl_data[2] & 0x08)
			aahlog("   -> directed advertising\n");
	}

	aahlog(" -> while active scanning:\n");
	if (!(cmd_cmpl_data[2] & 0xF0))
		aahlog("    NONE\n");
	else {
		if (cmd_cmpl_data[2] & 0x10)
			aahlog("   -> non-connectible advertising\n");
		if (cmd_cmpl_data[2] & 0x20)
			aahlog("   -> scannable advertising\n");
		if (cmd_cmpl_data[2] & 0x40)
			aahlog("   -> connectible advertising\n");
		if (cmd_cmpl_data[2] & 0x80)
			aahlog("   -> directed advertising\n");
	}

	aahlog(" -> while initiating as master:\n");
	if (!(cmd_cmpl_data[3] & 0x0F))
		aahlog("    NONE\n");
	else {
		if (cmd_cmpl_data[3] & 0x01)
			aahlog("   -> non-connectible advertising\n");
		if (cmd_cmpl_data[3] & 0x02)
			aahlog("   -> scannable advertising\n");
		if (cmd_cmpl_data[3] & 0x04)
			aahlog("   -> connectible advertising\n");
		if (cmd_cmpl_data[3] & 0x08)
			aahlog("   -> directed advertising\n");
	}

	aahlog(" -> while slave:\n");
	if (!(cmd_cmpl_data[3] & 0xF0))
		aahlog("    NONE\n");
	else {
		if (cmd_cmpl_data[3] & 0x10)
			aahlog("   -> non-connectible advertising\n");
		if (cmd_cmpl_data[3] & 0x20)
			aahlog("   -> scannable advertising\n");
		if (cmd_cmpl_data[3] & 0x40)
			aahlog("   -> connectible advertising\n");
		if (cmd_cmpl_data[3] & 0x80)
			aahlog("   -> directed advertising\n");
	}

	err = athome_bt_set_scan_timing(AAH_BT_NORMAL_SCAN_INTERVAL,
					AAH_BT_SCAN_WINDOW, buf, sizeof(buf));
	if (err)
		return err;

	aahlog("set clock accuracy\n");
	parP = buf;
	put8LE(&parP, AAH_BT_MARVELL_MCA);
	put8LE(&parP, AAH_BT_MARVELL_MCA);
	err = athome_bt_simple_cmd(HCI_OGF_Vendor, HCI_CMD_Marvell_Set_MCA,
				parP - buf, buf, 0, NULL);
	if (err)
		return err;

#if (AAH_BT_USE_LOWER_SCAN_PRIORITY)
	aahlog("set coexistence mode configuration\n");
/*
 * Change coexistence behavior based on suggestions from Marvell.
 * Changes LE scan priority to be lower than WiFi.
 */
	{
		static const uint8_t coex_cmd_params[] = {
			0x03, 0x01, 0x01, 0x01,  0x01, 0x03, 0x01, 0x00,
			0x02, 0x02, 0x02, 0x04,  0x02, 0x00, 0x00, 0x00,
			0x04, 0x02, 0x00, 0x00,  0x00};
		err = athome_bt_simple_cmd(
				0x3F,   /* OGF = HCI_OGF_VENDOR */
				0x008A, /* OCF = HCI_CMD_MARVELL_BLE_COEX_MODE_CONFIG*/
				sizeof(coex_cmd_params), coex_cmd_params, 0, NULL);
		if (err)
			return err;
	}

#endif /* AAH_BT_USE_LOWER_SCAN_PRIORITY */
	return 0;
}

static int athome_bt_process_evt(struct sk_buff *skb, bool *is_scanning,
			bool *is_connecting, uint32_t *proto_ver,
			bool *did_something,
			struct timer_list *timeout,
			struct athome_bt_thread_context *thread_context)
{
	/* clock accuracy velues indexed as per BT 4.0 spec */
	static const unsigned clockA[] = {500,250,150,100,75,50,30,20};
	uint8_t buf[EVT_PACKET_LEN];
	uint8_t *evt_data = buf + HCI_EVENT_HDR_SIZE;
	struct hci_ev_cmd_status *stat = (struct hci_ev_cmd_status*)evt_data;
	unsigned long flags;
	uint16_t handle;
	uint8_t *parP;
	unsigned i;
	int err;

	struct hci_event_hdr *evt =
			(struct hci_event_hdr*)skb->data;
	uint8_t *data = (uint8_t*)(evt + 1);
	struct hci_ev_le_meta *meta =
			(struct hci_ev_le_meta*)data;

	if (evt->evt == HCI_EV_LE_META &&
			meta->subevent ==
				HCI_EV_LE_CONN_COMPLETE) {
		bool need_encr = true;
		struct hci_ev_le_conn_complete *ci =
			(struct hci_ev_le_conn_complete*)
						(meta + 1);

		*is_connecting = false;
		*did_something = true;
		del_timer(timeout);

		if (ci->status) {
			aahlog("Failed to connect. Err = %d\n",
				ci->status);
			return 0;
		}

		/* we connected successfully */
		handle = r16LE(&ci->handle) & ACL_CONN_MASK;
		aahlog("CONNECTED with handle %d as %c with "
			"interval  %uuS, latency %u, sv_to "
			"%umS & MCA %uppm (v%08X)\n", handle,
			ci->role ? 'S' : 'M',
			1250 * r16LE(&ci->interval),
			r16LE(&ci->latency),
			10 * r16LE(&ci->supervision_timeout),
			clockA[ci->clk_accurancy],
			*proto_ver);

		athome_bt_led_show_event(HACK_LED_EVENT_CONNECT);

		spin_lock_irqsave(&stack_state_lock, flags);
		for (i = 0; i < ATHOME_RMT_MAX_CONNS &&
				conns[i].state != CONN_STATE_INVALID; i++)
			/* no loop body intended - just finds a free slot */
			;
		if (i != ATHOME_RMT_MAX_CONNS) {
			struct athome_bt_known_remote *K;
			struct bt_athome_connected ce;

			K = athome_bt_find_known(&ci->bdaddr);
			if (!K) {
				aahlog("device became unknown unexpectedly\n");
				/* pretend we found no slots */
				i = ATHOME_RMT_MAX_CONNS + 1;
				goto device_handled;
			}
			conns[i].state = CONN_STATE_JUST_ESTABLISHED;
			if (K->bind_mode) {
				need_encr = false;
				conns[i].state = CONN_STATE_BINDING;
			}
			conns[i].handle = handle;
			conns[i].outstanding = false;
			conns[i].gone = false;
			conns[i].pwr = ATHOME_MODE_ACTIVE;
			conns[i].next_pwr = ATHOME_MODE_ACTIVE;
			conns[i].in_modeswitch = false;
			conns[i].proto_ver = *proto_ver;
			bacpy(&conns[i].MAC, &ci->bdaddr);
			memset(&conns[i].stats, 0, sizeof(conns[i].stats));
			conns[i].last_time = get_time();
			thread_context->nconns++;

			bacpy(&ce.MAC, &ci->bdaddr);
			athome_bt_usr_enqueue( BT_ATHOME_EVT_CONNECTED,
							&ce, sizeof(ce));
		}
device_handled:
		spin_unlock_irqrestore(&stack_state_lock, flags);
		if (i >= ATHOME_RMT_MAX_CONNS) {

			if (i == ATHOME_RMT_MAX_CONNS)
				aahlog("no more connection slots!\n");
			spin_lock_irqsave(&stack_state_lock, flags);
			disconnH = handle | (ACL_CONN_MASK + 1);
			spin_unlock_irqrestore( &stack_state_lock, flags);
			return athome_bt_disconnect(handle);
		} else if (need_encr)
			athome_bt_start_encr(i);

	} else if (evt->evt == HCI_EV_LE_META &&
		meta->subevent ==
			HCI_EV_LE_ADVERTISING_REPORT) {

		bdaddr_t mac;
		uint32_t new_proto_ver = 0;

		*did_something = true;
		if (athome_bt_discovered(data, &mac,
			skb->len - sizeof(*evt), &new_proto_ver) &&
				!*is_connecting &&
				thread_context->nconns != ATHOME_RMT_MAX_CONNS) {

			const struct athome_bt_mode_settings *mode_settings;
			BUG_ON(!new_proto_ver);

			mode_settings = athome_bt_get_mode_settings(new_proto_ver, ATHOME_MODE_ACTIVE);
			*is_scanning = false;
			aahlog("Trying to connect to %02X:%02X"
			       ":%02X:%02X:%02X:%02X (v%08X), "
			       "conn_scan interval = %d, window = %d, "
			       "max event length (retries) = %d\n",
			       mac.b[5], mac.b[4], mac.b[3], mac.b[2],
			       mac.b[1], mac.b[0], new_proto_ver,
			       AAH_BT_CONN_SCAN_INTERVAL, AAH_BT_SCAN_WINDOW,
			       AAH_BT_MAX_EVENT_LENGTH);
			/* try to connect */
			parP = buf;
			put16LE(&parP, AAH_BT_CONN_SCAN_INTERVAL);   /* time between listens per channel */
			put16LE(&parP, AAH_BT_SCAN_WINDOW);   /* time to listen */
			put8LE(&parP, 0);     /* no whitelist */
			put8LE(&parP, 1);     /* random addr */
			put8LE(&parP, mac.b[0]);/* mac addr[5] */
			put8LE(&parP, mac.b[1]);/* mac addr[4] */
			put8LE(&parP, mac.b[2]);/* mac addr[3] */
			put8LE(&parP, mac.b[3]);/* mac addr[2] */
			put8LE(&parP, mac.b[4]);/* mac addr[1] */
			put8LE(&parP, mac.b[5]);/* mac addr[0] */
			put8LE(&parP, 0);     /* send pub. adr */
			put16LE(&parP, mode_settings->conn_interval);
			put16LE(&parP, mode_settings->conn_interval);
			put16LE(&parP, mode_settings->slave_latency);
			put16LE(&parP, mode_settings->svc_timeout);
			put16LE(&parP, AAH_BT_MAX_EVENT_LENGTH); /* affects num retries */
			put16LE(&parP, AAH_BT_MAX_EVENT_LENGTH);
			err = athome_bt_simple_cmd(HCI_OGF_LE,
				HCI_LE_Create_Connection,
				parP - buf, buf, sizeof(buf),
				buf);
			if (err)
				return err;
			if (stat->status)
				aahlog("failed conn. try.\n");
			else {
				*proto_ver = new_proto_ver;
				*is_connecting = true;
				mod_timer(timeout, jiffies +
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
		/* Use {0,} to be sure entire structure is zero for later comparison. */
		struct athome_bt_mode_settings mode_settings = {0,};
		mode_settings.conn_interval = r16LE(&cu->interval);
		mode_settings.slave_latency = r16LE(&cu->latency);
		mode_settings.svc_timeout = r16LE(&cu->supervision_timeout);

		*did_something = true;

		if (cu->status) {
			aahlog("Failed to update. Err = %d\n",
				cu->status);
			return 0;
		}

		/* we updated successfully */
		handle = r16LE(&cu->handle) & ACL_CONN_MASK;
		if (LOG_MODESWITCH)
			aahlog("UPDATED handle %d with interval"
				" %uuS, latency %u, sv_to %umS"
				"\n", handle, 1250 * mode_settings.conn_interval,
				mode_settings.slave_latency,
				10 * mode_settings.svc_timeout);

		spin_lock_irqsave(&stack_state_lock, flags);
		which = athome_bt_find_connection_l(handle);
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
			for (i = 0; i < ATHOME_MODE_MAX; i++) {
				const struct athome_bt_mode_settings *candidate
					= athome_bt_get_mode_settings(conns[which].proto_ver, i);
				if (memcmp(&mode_settings, candidate, sizeof(mode_settings)) == 0)
					break;
			}

			/* if unknown mode, pretend idle */
			if (i == ATHOME_MODE_MAX)
				i = ATHOME_MODE_IDLE;

			athome_bt_update_time_stats_l(which);

			conns[which].pwr = i;
			conns[which].in_modeswitch = false;
			if (LOG_MODESWITCH)
				aahlog("Conn %d now in state "
					"%d\n", which, i);
			bacpy(&mse.MAC, &conns[which].MAC);
			mse.new_mode = i;
			athome_bt_usr_enqueue(
				BT_ATHOME_EVT_MODE_SWITCHED,
				&mse, sizeof(mse));
		}
		spin_unlock_irqrestore(&stack_state_lock,
							flags);

	} else if (evt->evt == HCI_EV_ENCRYPT_CHANGE) {

		struct hci_ev_encrypt_change *evt =
			(struct hci_ev_encrypt_change*)data;

		handle = r16LE(&evt->handle);

		*did_something = true;
		spin_lock_irqsave(&stack_state_lock, flags);
		i = athome_bt_find_connection_l(handle);
		if (i >= 0) {
			aahlog("conn%d encr. s=%d o=%d\n",
				handle, evt->status,
				evt->encrypt);
			if (evt->encrypt) {
				struct athome_bt_known_remote *remote;

				if (conns[i].state ==
						CONN_STATE_ENCRYPTING) {
					del_timer((struct timer_list*)
							&conns[i].timer);
					conns[i].state = CONN_STATE_DATA;
					remote = athome_bt_find_known(
							&conns[i].MAC);
					if (remote && remote->bind_mode)
						remote->bind_mode = 0;
				} else {

					aahlog("Encr event  for invalid "
						"state %d\n", conns[i].state);
				}
			}
		}
		spin_unlock_irqrestore(&stack_state_lock, flags);
	} else if (evt->evt == HCI_EV_ENCR_REFRESH) {

		struct hci_ev_encrypt_refresh *evt =
			(struct hci_ev_encrypt_refresh*)data;

		handle = r16LE(&evt->handle);

		*did_something = true;
		spin_lock_irqsave(&stack_state_lock, flags);
		i = athome_bt_find_connection_l(handle);
		if (i >= 0) {
			aahlog("conn%d encr refr s=%d\n", handle, evt->status);

			if (conns[i].state == CONN_STATE_REENCRYPTING) {
				del_timer((struct timer_list*)&conns[i].timer);
				conns[i].state = CONN_STATE_DATA;
			} else {

				aahlog("Reencr event  for invalid  state %d\n",
							conns[i].state);
			}
		}
		spin_unlock_irqrestore(&stack_state_lock,
							flags);
	} else if (evt->evt != HCI_EV_DISCONN_COMPLETE) {

		/* disconnects were handled in filter cbk */

		aahlog("unexpected event %d\n", evt->evt);
	}

	return 0;
}

static int athome_bt_thread(void *unusedData)
{
	uint8_t buf[EVT_PACKET_LEN];
	bool is_scanning = false, is_connecting = false;
	uint32_t proto_ver;
	uint16_t handle;
	unsigned long flags;
	struct timer_list timeout;
	unsigned i;
	int ret = 0;
	struct athome_bt_thread_context thread_context = {0};

	if (!devices_exist) {
		aahlog("devices_created\n");
		if (athome_bt_user_init()) {
			aahlog("failed to register misc device\n");
			return -1;
		}

		if (athome_bt_input_init()) {
			aahlog("failed to register input device\n");
			athome_bt_user_deinit();
			return -2;
		}
		devices_exist = true;
	} else {
		athome_bt_input_reset_state();
	}

	if (athome_bt_host_setup()) {
		/* we don't unwind the one-time creation of the devices */
		aahlog("host setup failed\n");
		return -3;
	}

	skb_queue_head_init(&rx_dat);
	skb_queue_head_init(&rx_evt);

	for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++)
		setup_timer((struct timer_list*)&conns[i].timer,
						athome_bt_timeout, i);
	setup_timer(&timeout, athome_bt_timeout, ATHOME_RMT_MAX_CONNS);

	while (1) {	/* stack loop */

		struct sk_buff *skb = NULL;
		bool did_something = false;
		bool is_disconnecting = false;

		/* Only scan if we can connect to more Bemotes. */
		if (!is_scanning && !is_connecting) {
			if (thread_context.nconns < ATHOME_RMT_MAX_CONNS) {
				ret = athome_bt_set_scan_enabled(true, buf,
								 sizeof(buf));
				if (ret)
					break; /* exit while loop */

				is_scanning = 1;
				did_something = 1;
			}
		}

		/* see if we have events to process */
		if (!skb_queue_empty(&rx_evt))
			skb = skb_dequeue(&rx_evt);

		if (skb) {
			athome_bt_process_evt(skb, &is_scanning,
					&is_connecting, &proto_ver,
					&did_something, &timeout, &thread_context);
			kfree_skb(skb);
			skb = NULL;
		}

		/* see if we have events to process */
		if (!skb_queue_empty(&rx_dat))
			skb = skb_dequeue(&rx_dat);

		if (skb) {

			struct hci_acl_hdr *acl =
					(struct hci_acl_hdr*)skb->data;

			handle = r16LE(&acl->handle) & ACL_CONN_MASK;
			i = athome_bt_find_connection(handle);

			if (i >= 0) {
				ret = athome_bt_data_rx(i, skb->data,
								skb->len);
				if (ret)
					break; /* exit while loop */
			}
			else
				aahlog("data for unknown conection dropped\n");

			kfree_skb(skb);
			skb = NULL;
		}

		/* maybe we timed out connecting? */
		if (!timer_pending(&timeout) && is_connecting) {

			did_something = 1;
			ret = athome_bt_simple_cmd(HCI_OGF_LE,
					HCI_LE_Create_Connection_Cancel,
					0, NULL, 0, NULL);
			if (ret)
				break; /* exit while loop */
		}

		/* maybe something timed out ? */
		spin_lock_irqsave(&stack_state_lock, flags);
		for (i = 0; i < ATHOME_RMT_MAX_CONNS; i++) {
			if (conns[i].timeout) {
				conns[i].timeout = false;
				did_something = 1;
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
				conns[i].gone = false;
				is_disconnecting = true;
				if (thread_context.nconns == 0)
					aahlog("more disconnections than connections!\n");
				else
					thread_context.nconns--;
				did_something = 1;
			} else if (conns[i].pwr != conns[i].next_pwr &&
						!conns[i].in_modeswitch) {

				uint8_t next_pwr = conns[i].next_pwr;

				athome_bt_update_time_stats_l(i);
				spin_unlock_irqrestore(&stack_state_lock,
									flags);
				ret = athome_bt_modeswitch(i, next_pwr);
				did_something = 1;
				break;
			}
		}
		if (i == ATHOME_RMT_MAX_CONNS)
			spin_unlock_irqrestore(&stack_state_lock, flags);

		if (is_disconnecting)
			athome_bt_led_show_event(HACK_LED_EVENT_DISCONNECT);

		if (ret)
			break; /* exit while loop */

		if (!did_something)
			wait_event_interruptible(rx_wait,
				(!skb_queue_empty(&rx_evt) ||
					!skb_queue_empty(&rx_dat) ||
					atomic_read(&state_chg)));

		atomic_set(&state_chg, 0);

		if (kthread_should_stop() || atomic_read(&in_shutdown)) {
			aahlog("thread exiting normally\n");
			ret = 1;
			break; /* exit while loop */
		}
	}

	del_timer(&timeout);
	aahlog("thread exiting with sta %d\n", ret);
	return ret;
}

void athome_bt_stack_prepare(void)
{
	static struct task_struct *thread;

	if (!aah_thread) {
		aahlog("starting LE stack!\n");
		atomic_set(&in_shutdown, 0);
		thread = kthread_create(athome_bt_thread, NULL,
					"athome_bt");
		if (IS_ERR(thread)) {
			aahlog("Failed to start stack "
				"thread\n");
			thread = NULL;
		}
		aah_thread = thread;
	}
	smp_wmb();
}

void athome_bt_stack_start(void)
{
	smp_rmb();
	BUG_ON(!aah_thread);

	wake_up_process(aah_thread);
}


void athome_bt_stack_shutdown(void)
{
	unsigned long flags;
	int i;

	if (aah_thread) {
		aahlog("shutting down thread\n");
		atomic_set(&in_shutdown, 1);
		atomic_set(&state_chg, 1); /* force wake */
		/* unblock it if waiting for a cmd response */
		complete_all(&cmd_response);
		INIT_COMPLETION(cmd_response);
		wake_up_interruptible(&rx_wait);
		kthread_stop(aah_thread);
		aahlog("thread shut down\n");
		aah_thread = NULL;
	}

	if (!skb_queue_empty(&rx_evt))
		kfree_skb(skb_dequeue(&rx_evt));

	if (!skb_queue_empty(&rx_dat))
		kfree_skb(skb_dequeue(&rx_dat));

	wake_up_interruptible(&rx_wait);

	atomic_set(&state_chg, 0);

	/* Reset tx_credit to a count of 1  */
	while(!down_trylock(&tx_credit))
		;
	up(&tx_credit);

	spin_lock_irqsave(&stack_state_lock, flags);
	inflight_ogf = INVALID;
	inflight_ocf = INVALID;
	disconnH = 0;
	cmd_rsp_buf = NULL;
	for(i = 0; i < ATHOME_RMT_MAX_CONNS; i++)
		conns[i].state = CONN_STATE_INVALID;
	spin_unlock_irqrestore(&stack_state_lock, flags);
}

