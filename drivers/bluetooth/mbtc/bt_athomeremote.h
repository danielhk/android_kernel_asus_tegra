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

#ifndef _BT_ATHOME_REMOTE_H_
#define _BT_ATHOME_REMOTE_H_


#include <linux/skbuff.h>
#include <linux/ioctl.h>


#define aahlog(...)			printk("AahBtRemote: " __VA_ARGS__)
#define aahlog_continue(...)		printk(__VA_ARGS__)

#define AAH_BT_KEY_DPAD_CENTER			353
#define AAH_BT_KEY_POWER			177
#define AAH_BT_KEY_INPUT			178


/* a type val BT will not use */
#define AAH_BT_PKT_TYPE_CMD			5

#define AAH_BT_PKT_PROCEED			0
#define AAH_BT_PKT_DROP				1

#define AAH_BT_MAC_SZ				6
#define AAH_BT_LTK_SZ				16
#define AAH_BT_ENTROPY_SZ			16


/*  ====== userspace-relevant defines ====== */
#define ATHOME_RMT_MAX_CONNS			4
#define BT_ATHOME_MAX_USER_LEN			280

#define BT_ATHOME_STATE_BINDING		0 /* data ok, unsecure only */
#define BT_ATHOME_STATE_CONNECTING	1 /* no data, maybe later */
#define BT_ATHOME_STATE_ENCRYPTING	2 /* no data, maybe later */
#define BT_ATHOME_STATE_CONNECTED	3 /* data ok, secure */
#define BT_ATHOME_STATE_DISCONNECTING	4 /* no data, bye bye */
#define BT_ATHOME_STATE_UNKNOWN		5 /* who knows ... */

#define ATHOME_MODE_IDLE	0
#define ATHOME_MODE_SEMIIDLE	1
#define ATHOME_MODE_ACTIVE	2
#define ATHOME_MODE_MAX		3


struct athome_bt_stats {
	/* nanoseconds in each mode */
	uint64_t mode_times[ATHOME_MODE_MAX];

	/* number of various packets */
	uint64_t audio[4];
	uint64_t accel;
	uint64_t input;
	uint64_t nfc_tx;
	uint64_t nfc_rx;
	uint64_t pkts_rx;
	uint64_t pkts_tx;

	/* number of bytes total */
	uint64_t bytes_rx;
	uint64_t bytes_tx;

	/* RFU */
	uint64_t RFU[8];
};

/* messages TO driver */
#define BT_ATHOME_MSG_ADD_DEV			0x00
	struct bt_athome_add_dev {
		uint8_t MAC[AAH_BT_MAC_SZ];
		uint8_t LTK[AAH_BT_LTK_SZ];
	} __packed;
#define BT_ATHOME_MSG_SET_BIND_MODE		0x01
	struct bt_athome_set_bind_mode {
		uint8_t bind_mode;
	} __packed;
#define BT_ATHOME_MSG_DO_BIND			0x02
	struct bt_athome_do_bind {
		uint8_t MAC[AAH_BT_MAC_SZ];
	} __packed;
#define BT_ATHOME_MSG_STOP_BIND			0x03
	struct bt_athome_stop_bind {
		uint8_t MAC[AAH_BT_MAC_SZ];
	} __packed;
#define BT_ATHOME_MSG_ENCRYPT			0x04
	struct bt_athome_encrypt {
		uint8_t MAC[AAH_BT_MAC_SZ];
	} __packed;
#define BT_ATHOME_MSG_DEL_DEV			0x05
	struct bt_athome_del_dev {
		uint8_t MAC[AAH_BT_MAC_SZ];
	} __packed;
#define BT_ATHOME_MSG_GET_STATE			0x06
	/* no params -> generate BT_ATHOME_EVT_STATE */
#define BT_ATHOME_MSG_DATA			0x07
	struct bt_athome_send_data {
		uint8_t MAC[AAH_BT_MAC_SZ];
		uint8_t pkt_typ;
		uint8_t pkt_data[]; /* len <= 25 */
	} __packed;
#define BT_ATHOME_MSG_DEV_STATS			0x08
	/* MAC -> generate BT_ATHOME_EVT_DEV_STATS */
	struct bt_athome_get_dev_stats {
		uint8_t MAC[AAH_BT_MAC_SZ];
	} __packed;

/* events FROM driver */
#define BT_ATHOME_EVT_STATE			0x81
	struct bt_athome_state {
		uint8_t num;
		struct {
			uint8_t MAC[AAH_BT_MAC_SZ];
			/* see BT_ATHOME_STATE_* */
			uint8_t con_state;
			/* see ATHOME_MODE_* */
			uint8_t pwr_state;
			uint8_t input_dev_idx;
			uint8_t RFU[7];
		} remotes [ATHOME_RMT_MAX_CONNS];
	} __packed;
#define BT_ATHOME_EVT_CONNECTED			0x82
	struct bt_athome_connected {
		uint8_t MAC[AAH_BT_MAC_SZ];
	} __packed;
#define BT_ATHOME_EVT_DISCONNECTED		0x83
	struct bt_athome_disconnected {
		uint8_t MAC[AAH_BT_MAC_SZ];
		struct athome_bt_stats stats;
	} __packed;
#define BT_ATHOME_EVT_MODE_SWITCHED		0x84
	struct bt_athome_mode_switched {
		uint8_t MAC[AAH_BT_MAC_SZ];
		uint8_t new_mode;
	} __packed;
#define BT_ATHOME_EVT_DATA			0x85
	struct bt_athome_got_data {
		uint8_t MAC[AAH_BT_MAC_SZ];
		uint8_t pkt_typ;
		uint8_t pkt_data[]; /* len <= 25 */
	} __packed;
#define BT_ATHOME_EVT_DISCOVERED_V1		0x86
	struct bt_athome_discovered_v1 {
		uint8_t MAC[AAH_BT_MAC_SZ];
		uint8_t ver[4];
		int8_t RSSI;
		uint8_t RFU[8];
		uint8_t ser_num[]; /* len <= 20 */
	} __packed;
#define BT_ATHOME_EVT_DEV_STATS			0x87
	struct bt_athome_dev_stats {
		uint8_t MAC[AAH_BT_MAC_SZ];
		struct athome_bt_stats stats;
	} __packed;
#define BT_ATHOME_EVT_BIND_KEY			0x88
	struct bt_athome_bind_key {
		uint8_t MAC[AAH_BT_MAC_SZ];
		uint8_t key;
	} __packed;

#define BT_ATHOME_IOCTL_GETSTATE 		_IOR(0, 0, struct bt_athome_state)
/* param is pointer to state struct to fill out */

/*  ============= air protocol ============= */

#define ATHOME_PKT_RX_ACK		0
	struct athome_pkt_ack {
		uint8_t orig_pkt_typ;
		uint8_t data[]; /* up to 24 bytes */
	} __packed;
#define ATHOME_PKT_RX_INPUT		1

	#define ATHOME_INPUT_INFO_MASK_HAS_BTN		0x40
	#define ATHOME_INPUT_INFO_MASK_HAS_TOUCH	0x80
	#define ATHOME_INPUT_INFO_MASK_TIMESTAMP	0x3F
	struct athome_pkt_rx_input {
		uint8_t info;
	} __packed;
	#define ATHOME_PAIR_BTN_CHAR			0x40000000
	#define ATHOME_PAIR_BTN_BKSP			0x80000000
	#define SECURE_BTN_MASK				(ATHOME_PAIR_BTN_CHAR | \
							 ATHOME_PAIR_BTN_BKSP)
	struct athome_pkt_rx_input_btn {
		uint32_t btn_mask;
	} __packed;
	struct athome_pkt_rx_input_touch {
		struct {
			uint16_t X, Y;
		} fingers [3];
	} __packed;

#define ATHOME_PKT_RX_AUDIO_0		2
#define ATHOME_PKT_RX_AUDIO_1		3
#define ATHOME_PKT_RX_AUDIO_2		4
#define ATHOME_PKT_RX_AUDIO_3		5
#define ATHOME_PKT_RX_ACCEL		6
	struct athome_pkt_rx_accel {
		int16_t X, Y, Z;
	} __packed;
#define ATHOME_PKT_RX_MODESWITCH	7
	struct athome_pkt_rx_modeswitch {
		/* see ATHOME_MODE_* */
		uint8_t mode;
	} __packed;
#define ATHOME_PKT_RX_NFC		8

#define ATHOME_PKT_TX_ACK		0
	/* see athome_pkt_ack */
#define ATHOME_PKT_TX_SET_PARAM		1
	struct athome_tx_pkt_set_param {
		uint8_t param;
		uint8_t val[]; /* up to 25 bytes */
	} __packed;
#define ATHOME_PKT_TX_GET_PARAM		2
	struct athome_tx_pkt_get_param {
		uint8_t param;
	} __packed;
#define ATHOME_PKT_TX_FW_UPDATE		3
#define ATHOME_PKT_TX_ENC		4
	struct athome_tx_pkt_enc {
		uint8_t entropy[AAH_BT_ENTROPY_SZ];
	} __packed;
#define ATHOME_PKT_TX_NFC		5



/*  ========== functions provided ========== */

/*
	Process an incoming data packet. The options are:
	return AAH_BT_PKT_*
*/
int athome_bt_remote_filter_rx_data(void *priv, u32 pkt_type, u8 *rx_buf, u32 *rx_len);


/*
	Called when driver about to send a packet to chip.
	return AAH_BT_PKT_*
*/
int athome_bt_pkt_send_req(void *priv, struct sk_buff *skb);



/*  ========== functions needed ========== */

/*
	Send packet to user, as if it came from the chip
*/
void athome_bt_pkt_to_user(void *priv, uint32_t pkt_type, uint8_t *data, uint32_t len);

/*
	Send packet to chip, as if it came from the user
*/
void athome_bt_pkt_to_chip(void *priv, uint32_t pkt_type, uint8_t *data, uint32_t len);

/*
	Wake up the main thread of the bt driver (if we had previously refused
	to send a packet, this should retry it)

	//TODO: use wake_up_interruptible(&priv->MainThread.waitQ);
*/
void athome_bt_main_thread_wake(void *priv);


#endif

