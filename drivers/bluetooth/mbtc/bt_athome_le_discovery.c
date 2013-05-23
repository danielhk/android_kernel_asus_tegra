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
#include "bt_athome_le_discovery.h"
#include "bt_athome_hci_extra.h"
#include "bt_athome_le_stack.h"
#include "bt_athome_logging.h"
#include "bt_athome_proto.h"
#include "bt_athome_user.h"


#define ADV_DATA_FLAGS			1
#define ADV_DATA_UUID16S_INCOMPLETE	2
#define ADV_DATA_UUID16S_COMPLETE	3
#define ADV_DATA_UUID32S_INCOMPLETE	4
#define ADV_DATA_UUID32S_COMPLETE	5
#define ADV_DATA_UUID128S_INCOMPLETE	6
#define ADV_DATA_UUID128S_COMPLETE	7
#define ADV_DATA_NAME_SHORT		8
#define ADV_DATA_NAME_COMPLETE		9
#define ADV_DATA_TX_POWER		10
#define ADV_DATA_MANUF_SPECIFIC		255

static void athome_bt_log_uuids(const uint8_t **data, uint8_t* L, unsigned len)
{
	unsigned i;
	while ((*L) > len) {
		(*L) -= len;
		aahlog_continue(" ");
		for(i = 0; i < len; i++)
			aahlog_continue("%02X", *(*data)++);
	}
}

/*
 * If we find a device to connect to, copy its mac into macP and return
 * true. Else leave macP alone and return try_connect. This allows us to
 * chain multiple calls of this func on multiple discovery results and to
 * produce a simple final decision to or not to connect, and whom to.
 */
static bool athome_bt_discovered_device(const uint8_t **bufP,
			bool try_connect, uint8_t *macP, int* remain_bytes)
{
	static const char *advTypes[] = {"ADV_IND", "ADV_DIRECT_IND",
					"ADV_SCAN_IND", "ADV_NONCONN_IND",
					"SCAN_RSP"};
	const uint8_t *buf = *bufP;
	bool needflags = true, needmanuf = true;
	uint8_t slen = 0;
	const uint8_t *snum = NULL; /* only devices in bind mode */
	uint8_t evtType, is_addr_rand, mac[AAH_BT_MAC_SZ];
	uint8_t len, j;
	const uint8_t *data, *end;
	struct athome_bt_known_remote *item;
	bool found = false, want_bind = false, can_connect = false;
	struct hci_ev_le_advertising_info *adv_info =
			(struct hci_ev_le_advertising_info*)buf;
	struct athome_bt_adv_manuf_data *manuf;
	uint8_t ver[4] = {0,};
	int8_t rssi;


	/*
	 * This code here parses a BTLE Advertising Report event as defined by
	 * BT 4.0 spec section 2.E.7.7.65.2. There is not an easy way to
	 * overlay a struct over the data since it is mostly free-form. Here we
	 * focus on logging the entire struct (parsed as best as we can) and
	 * also decising if this is a remote we want to connect to, and if so,
	 * how (secured or not). The "data" field itself is a sequence of EIR
	 * data sturctures as defined by BT 4.0 spec section 3.C.8. The remote
	 * detection is based on "flags" being "LE General Discoverable" and
	 * "no EDR support", and a custom manufacturer-specific data chunk
	 * containing the word "Google" as well as a protocol version number
	 * and possibly the remote serial number.
	 */

	if (*remain_bytes < sizeof(*adv_info)) {
		aahlog("invalid adv device (too short)\n");
		*remain_bytes = 0;
		return try_connect;
	}
	(*remain_bytes) -= sizeof(*adv_info);
	buf += sizeof(*adv_info);

	evtType = adv_info->evt_type;;
	is_addr_rand = adv_info -> bdaddr_type;
	memcpy(mac, adv_info->bdaddr.b, sizeof(mac));
	len = adv_info->length;
	data = buf;
	buf += len;
	(*remain_bytes) -= len;

	if (*remain_bytes < 1 || len > HCI_LE_ADV_DATA_MAX_LEN) {
		aahlog("invalid adv device data (too long)\n");
		*remain_bytes = 0;
		return try_connect;
	}
	rssi = *buf++;
	(*remain_bytes)--;

	if (LOG_DISCOVERY) {
		aahlog("DEV %02X:%02X:%02X:%02X:%02X:%02X(%c) "
			"%s: RSSI %d (%u):", mac[5], mac[4],
			mac[3], mac[2], mac[1], mac[0],
			is_addr_rand ? 'R' : 'P',
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
		case ADV_DATA_FLAGS: //flags
			if (!L)
				break;
			t = *data;

			/* our flags? */
			/* 6 is LE_general_discoverable + no_EDR_support */
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

		case ADV_DATA_UUID16S_INCOMPLETE:  //16-bit uuids (more)
		case ADV_DATA_UUID16S_COMPLETE:    //16-bit uuids (complete)
			if (!LOG_DISCOVERY)
				break;

			aahlog_continue(" UUID16(%scomplete)(",
				t == ADV_DATA_UUID16S_INCOMPLETE ? "in" : "");
			athome_bt_log_uuids(&data, &L, 2);
			aahlog_continue(" )");
			break;

		case ADV_DATA_UUID32S_INCOMPLETE:  //32-bit uuids (more)
		case ADV_DATA_UUID32S_COMPLETE:    //32-bit uuids (complete)
			if (!LOG_DISCOVERY)
				break;

			aahlog_continue(" UUID32(%scomplete)(",
				t == ADV_DATA_UUID32S_INCOMPLETE ? "in" : "");
			athome_bt_log_uuids(&data, &L, 4);
			aahlog_continue(" )");
			break;

		case ADV_DATA_UUID128S_INCOMPLETE: //128-bit uuids (more)
		case ADV_DATA_UUID128S_COMPLETE:   //128-bit uuids (complete)
			if (!LOG_DISCOVERY)
				break;

			aahlog_continue(" UUID(%scomplete)(",
				t ==ADV_DATA_UUID128S_INCOMPLETE ? "in" : "");
			athome_bt_log_uuids(&data, &L, 16);
			aahlog_continue(" )");
			break;

		case ADV_DATA_NAME_SHORT:	//shortened name
		case ADV_DATA_NAME_COMPLETE:	//complete
			if (!LOG_DISCOVERY)
				break;

			aahlog_continue(" NAME(%s)(",
				t == ADV_DATA_NAME_SHORT ?
					"shortened" : "complete");
			while (L--)
				aahlog_continue("%c", *data++);
			aahlog_continue(")");
			break;

		case ADV_DATA_TX_POWER: //tx power
			if (!LOG_DISCOVERY)
				break;

			aahlog_continue(" TXPOWER(");
			while (L--)
				aahlog_continue("%d", (int8_t)*data++);
			aahlog_continue(")");
			break;

		case ADV_DATA_MANUF_SPECIFIC: //custom
			if (needmanuf && L >= sizeof(*manuf)) do {

				uint32_t ver_val;
				manuf = (struct athome_bt_adv_manuf_data*)data;

				if (memcmp(manuf->ident, ATHOME_BT_IDENT,
							sizeof(manuf->ident)))
					/* breaks out of if, not of case */
					break;

				memcpy(ver, manuf->ver, sizeof(ver));
				ver_val = ver[0];
				ver_val = (ver_val << 8) | ver[1];
				ver_val = (ver_val << 8) | ver[2];
				ver_val = (ver_val << 8) | ver[3];

				if (LOG_DISCOVERY)
					aahlog_continue(" AAH(v%08X)",
							ver_val);
				if (ver_val >= MIN_PROTO_VERSION)
					needmanuf = 0;

				/* bind mode devices send serial number too */
				if (L == sizeof(*manuf))
					/* breaks out of if, not of case */
					break;

				slen = L - 10;
				snum = manuf->snum;
				if (LOG_DISCOVERY) {

					aahlog_continue(" SN(");
					for(j = 0 ;j < slen; j++)
						aahlog_continue("%c", snum[j]);
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
	item = athome_bt_find_known(mac);
	if (item) {
		found = true;
		want_bind = item->bind_mode;
	}

	if (!needmanuf && !needflags && is_addr_rand &&
				evtType == SCAN_EVT_ADV_IND) {
		if (found) {
			if (snum && want_bind) {

				if (LOG_DISCOVERY)
					aahlog(" -> wanted unbound remote"
						" found\n");
				can_connect = true;
			} else if (snum) {

				if (LOG_DISCOVERY)
					aahlog(" -> bind-like adv from slave "
						"not expected in bind mode\n");
			} else if (want_bind) {

				if (LOG_DISCOVERY)
					aahlog(" -> non-bind adv from slave "
						"expected in bind mode\n");
			} else {
				if (LOG_DISCOVERY)
					aahlog(" -> known half bound remote "
								"found\n");
				can_connect = true;
			}
		} else if (snum && athome_bt_usr_in_bind_mode()) {

			uint8_t usr[64] = {0,};
			struct bt_athome_discovered *dd =
				(struct bt_athome_discovered*)usr;

			if (LOG_DISCOVERY)
				aahlog(" -> advertising & unbound\n");

			/* prepare event & send it to userspace */
			if (slen >= AAH_BT_SLEN_MAX)
				slen = AAH_BT_SLEN_MAX;
			memcpy(dd->MAC, mac, sizeof(dd->MAC));
			memcpy(dd->ver, ver, sizeof(dd->ver));
			memcpy(dd->ser_num, snum, slen);
			dd->RSSI = rssi;

			athome_bt_usr_enqueue(
					BT_ATHOME_EVT_DISCOVERED,
					usr, sizeof(*dd) + slen);

		} else if (LOG_DISCOVERY) {

			aahlog(" -> strange adv {snum=%d slen=%d found=%d "
						"want_bind=%d}\n", !!snum,
						slen, found, want_bind);
		}
	} else if (evtType == SCAN_EVT_ADV_DIRECT_IND) {
		if (!found) {
			if (LOG_DISCOVERY)
				aahlog(" -> unexpected direct adv\n");
		} else if (want_bind) {
			if (LOG_DISCOVERY)
				aahlog(" -> unexpected direct, want bind\n");
		} else {
			if (LOG_DISCOVERY)
				aahlog(" -> wanted bound remote\n");
			can_connect = true;
		}
	}

	/* Do not connect to it if we are already connected */
	if (can_connect) {
		if (athome_bt_is_connected_to(mac))
			aahlog(" -> already connected\n");
		else {
			try_connect = true;
			memcpy(macP, mac, sizeof(mac));
		}
	}

	*bufP = buf;
	return try_connect;
}

/* process adv response, see if anything of use is in it, if so, what */
bool athome_bt_discovered(const uint8_t *buf, uint8_t *macP, int len)
{
	bool try_connect = false;
	uint8_t num;


	buf += sizeof(struct hci_ev_le_meta);
	len -= sizeof(struct hci_ev_le_meta);
	if (len < 1) {
		aahlog("invalid adv packet (too short)\n");
		return false;
	}
	num = *buf++;
	len --;

	while (num--)
		try_connect = athome_bt_discovered_device(&buf,
						try_connect, macP, &len);

	return try_connect;
}
