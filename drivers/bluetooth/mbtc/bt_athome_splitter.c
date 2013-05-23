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
#include <linux/semaphore.h>
#include <linux/kernel.h>
#include "bt_athome_hci_extra.h"
#include "bt_athome_le_stack.h"
#include "bt_athome_splitter.h"
#include "bt_athome_logging.h"
#include "bt_athome_util.h"




/* virtualization state */
static struct semaphore send_sem = __SEMAPHORE_INITIALIZER(send_sem, 1);
/*
 * Only one copy of this data exists in the kernel:
 * This is the driver-private data for the MBT driver whom we're piggy-backing
 * off. We make no use of it ourselves, but we must pass it back to MBT anytime
 * we call into it. MBT itself is thread-safe and handles all necesary safety
 * and locking for this data.
 */
static void *drv_priv = NULL;



/* event masks for things we need */
  /* disconnect, encr_change, encr_refresh */
static const uint64_t evtsNeeded = 0x0000800000000090ULL;
  /* LE meta event*/
static const uint64_t evtsLE = 0x2000000000000000ULL;


static int athome_bt_filter_disconn(void *dataP)
{
	struct hci_ev_disconn_complete *evt = dataP;

	return athome_bt_connection_went_down(r16LE(&evt->handle) & ACL_CONN_MASK);
}

static int athome_bt_filter_encr_change(void *dataP)
{
	struct hci_ev_encrypt_change *evt = dataP;

	return athome_bt_encr_changed(r16LE(&evt->handle) & ACL_CONN_MASK);
}

static int athome_bt_filter_encr_refresh(void *dataP)
{
	struct hci_ev_encrypt_refresh *evt = dataP;

	return athome_bt_encr_refreshed(r16LE(&evt->handle) & ACL_CONN_MASK);
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
	aahlog("resetting\n");

	athome_bt_stack_shutdown();

	while(!down_trylock(&send_sem));
	up(&send_sem);

	aahlog("reset complete\n");
}

static int athome_bt_filter_cmd_complete(uint8_t *rx_buf, uint32_t len,
						bool *enq_if_oursP)
{
	struct hci_ev_cmd_complete *evt =
			(struct hci_ev_cmd_complete *)
			(rx_buf + HCI_EVENT_HDR_SIZE);
	uint16_t opcode = r16LE(&evt->opcode);
	uint16_t ogf = hci_opcode_ogf(opcode);
	uint16_t ocf = hci_opcode_ocf(opcode);
	int i, forus = 0;

	if (athome_bt_cmd_sta_or_compl(ogf, ocf, rx_buf, len)) {
		forus = 1;
		*enq_if_oursP = 0;
	}

	if (ogf == HCI_OGF_Controller_And_Baseband && ocf == HCI_Reset) {

		/*
		 *	any and all command credits we have/had go out the
		 *	window after a reset, and we instead have one. This
		 *	code assures our semaphore acts that way too.
		 */
		while(!down_trylock(&send_sem));
		up(&send_sem);
		if (LOG_BT_SEM)
			aahlog("resetting bt sem to 1 on reset\n");

	 } else if (ogf == HCI_OGF_Controller_And_Baseband &&
		ocf == HCI_Read_LE_Host_Support && !forus) {

		struct read_le_host_supported_reply *repl =
			(struct read_le_host_supported_reply*)(evt + 1);

		BUG_ON(len < sizeof(*repl) + sizeof(*evt));

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

			BUG_ON(len < sizeof(*repl) + sizeof(*evt));

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

			BUG_ON(len < sizeof(*repl) + sizeof(*evt));

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

			BUG_ON(len < sizeof(*repl) + sizeof(*evt));

			athome_bt_filter_ftr_page_0(repl->features);
		}
		if (ocf == HCI_Read_Local_Supported_Extended_Features &&
			!forus) {
			struct read_support_ext_ftrs_reply *repl =
				(struct read_support_ext_ftrs_reply*)(evt + 1);

			BUG_ON(len < sizeof(*repl) + sizeof(*evt));

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
			struct hci_rp_read_buffer_size *repl =
				(struct hci_rp_read_buffer_size*)(evt + 1);
			uint16_t pkts;

			BUG_ON(len < sizeof(*repl) + sizeof(*evt));
			pkts = r16LE(&repl->acl_max_pkt);

			aahlog("fixing up acl packet number from %d\n", pkts);
			if (pkts)
				w16LE(&repl->acl_max_pkt, pkts - 1);
		}
	} else if (ogf == HCI_OGF_LE && !forus) {
		aahlog("LE command complete not for us (ogf=0x%x ocf=0x%x):",
			ogf, ocf);
		/*
		 * This is ugly and bad. This code here is for LOGGING ONLY and
		 * it logs our userspace EDR stack doing bad things (sending LE
		 * commands to a chip claiming no LE support). Once these bugs
		 * in the userspace stack are fixed, this code can go away
		 */
		for (i = 0; i < ((uint8_t*)rx_buf)[-1]; i++)
			aahlog_continue(" %02X", ((uint8_t*)rx_buf)[i]);
		aahlog_continue("\n");
	} else if (ogf == HCI_OGF_Controller_And_Baseband &&
						ocf == HCI_Set_Event_Mask) {

		/* it is now safe to start our stack */
		athome_bt_stack_start();
	}

	return forus;
}

static int athome_bt_filter_cmd_status(uint8_t *rx_buf, uint32_t len,
						bool *enq_if_oursP)
{
	struct hci_ev_cmd_status *evt =
			(struct hci_ev_cmd_status *)
			(rx_buf + HCI_EVENT_HDR_SIZE);
	uint16_t opcode = r16LE(&evt->opcode);
	uint16_t ogf = hci_opcode_ogf(opcode);
	uint16_t ocf = hci_opcode_ocf(opcode);
	bool forus = 0;
	unsigned long i;

	if (athome_bt_cmd_sta_or_compl(ogf, ocf, rx_buf, len)) {
		forus = 1;
		*enq_if_oursP = 0;
	}

	if (ogf == HCI_OGF_LE && !forus) {
		aahlog("LE command status not for us (ogf=0x%x ocf=0x%x):",
			ogf, ocf);
		/*
		 * This is ugly and bad. This code here is for LOGGING ONLY and
		 * it logs our userspace EDR stack doing bad things (sending LE
		 * commands to a chip claiming no LE support). Once these bugs
		 * in the userspace stack are fixed, this code can go away
		 */
		for (i = 0; i < ((uint8_t*)rx_buf)[-1]; i++)
			aahlog_continue(" %02X", ((uint8_t*)rx_buf)[i]);
		aahlog_continue("\n");
	}

	return forus;
}

static int athome_bt_filter_num_comp_pkt(void *dataP, uint32_t *rx_len)
{
	struct num_comp_pkts *evt = dataP;
	struct comp_pkts_info *in = evt->handles, *out = evt->handles;
	int num_skipped = 0;
	int i, new_creds = 0;

	for (i = 0; i < evt->num_hndl; i++, in++) {
		uint16_t conn = r16LE(&in->handle);
		uint16_t pkts = r16LE(&in->count);

		if (athome_bt_compl_pkts(conn, pkts)) {
			new_creds += pkts;
			num_skipped++;
		} else {
			w16LE(&out->handle, conn);
			w16LE(&out->count, pkts);
			out++;
		}
	}

	if (LOG_BT_EVTS)
		aahlog("got %u data credits back, hid %u handles\n",
			new_creds, num_skipped);

	*rx_len -= num_skipped * 2 * sizeof(uint16_t);
	evt->num_hndl -= num_skipped;

	return !evt->num_hndl;
}

void athome_bt_send_to_user(uint32_t pkt_type, uint8_t *data, uint32_t len)
{
	smp_rmb();
	if (!drv_priv)
		aahlog("Attempt to send to user with no private data\n");
	else
		athome_bt_pkt_to_user(drv_priv, pkt_type, data, len);
}

void athome_bt_send_to_chip(uint32_t pkt_type, uint8_t *data, uint32_t len)
{
	smp_rmb();
	if (!drv_priv)
		aahlog("Attempt to send to chip with no private data\n");
	else
		athome_bt_pkt_to_chip(drv_priv, pkt_type, data, len);
}

int athome_bt_remote_filter_rx_data(void *priv, u32 pkt_type, u8 *rx_buf,
					u32 *rx_len)
{
	bool isours = false, enq_if_ours = true;

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

		if (isours && enq_if_ours)
			athome_bt_process_le_evt(rx_buf, *rx_len);

	} else if (pkt_type == HCI_ACLDATA_PKT) {

		struct hci_acl_hdr *acl = (struct hci_acl_hdr*)rx_buf;

		isours = athome_bt_process_le_data(r16LE(&acl->handle) & ACL_CONN_MASK, rx_buf, *rx_len);
	}

	athome_bt_logpacket(1, pkt_type, rx_buf, *rx_len, isours);

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

			uint64_t mask = r64LE(((uint8_t*)skb->data) +
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
			w64LE(((uint8_t*)skb->data) + HCI_COMMAND_HDR_SIZE,
					mask);

			aahlog("modified event mask 0x%08X%08X -> 0x%08X%08X"
				"\n", (uint32_t)(oldmask >> 32),
				(uint32_t)oldmask, (uint32_t)(mask >> 32),
				(uint32_t)mask);

			/* it is now safe to prepare our stack */
			athome_bt_stack_prepare();

		} else if (ogf == HCI_OGF_LE && !wasours) {

			uint8_t buf[64];
			uint8_t *end = NULL;
			struct hci_event_hdr *evt = (struct hci_event_hdr*)buf;
			struct hci_ev_cmd_complete *comp =
					(struct hci_ev_cmd_complete*)(evt + 1);
			struct hci_rp_le_read_buffer_size *bs =
				(struct hci_rp_le_read_buffer_size*)(comp + 1);
			uint8_t *stat = (uint8_t*)(comp + 1);

			evt->evt = HCI_EV_CMD_COMPLETE;
			comp->ncmd = 1;
			w16LE(&comp->opcode, cmd->opcode);

			aahlog("LE command (0x%x) from EDR stack!\n", ocf);
			switch (ocf) {
			case HCI_LE_Read_Buffer_Size:
				bs->status = 0;
				w16LE(&bs->le_mtu, 256); /* why not ? */
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
				break;
			}

			if (end) {

				evt->plen = end - (uint8_t*)(evt + 1);

				log_it = 0; /* log now so we can log "reply" */

				athome_bt_logpacket(0, HCI_COMMAND_PKT,
						(const uint8_t*)skb->data,
						skb->len, 0);

				athome_bt_logpacket(1, HCI_EVENT_PKT, buf, end - buf, 0);

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
		athome_bt_logpacket(0, bt_cb(skb)->pkt_type, (const uint8_t*)skb->data,
							skb->len, wasours);

	return ret;
}

