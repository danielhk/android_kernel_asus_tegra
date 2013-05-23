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

#ifndef _BT_ATHOME_LE_STACK_H_
#define _BT_ATHOME_LE_STACK_H_


#include "bt_athome_proto.h"

/*
 * If connected to a given MAC, schedule a disconnection soon. Async.
 */
void athome_bt_disc_from_mac(const uint8_t *mac);

/*
 * Get driver state (list of connections and their states)
 */
void athome_bt_get_state(struct bt_athome_state *state);

/*
 * If connected to a given MAC, schedule encryption to start soon
 */
void athome_bt_start_encr_for_mac(const uint8_t *mac);

/*
 * Get a connection's statistics
 */
bool athome_bt_get_stats(struct athome_bt_stats *stats, const uint8_t *mac);

/*
 * Send a packet to a remote. Returns false on certain failures, but a return
 * of "true" does not guarantee anything more than that the packet was sent to
 * the BTLE chip. Higher level code, if it wants to, can deal with higher-level
 * reliability.
 */
bool athome_bt_send_data(const uint8_t *mac, uint8_t typ,
			const void *data, uint8_t data_sz, bool for_user);

/*
 * Check if we're currently connected to a given MAC
 */
bool athome_bt_is_connected_to(const uint8_t *mac);

/*
 * Filters:
 * Return true if it was meant for LE. Basic work is OK. Try to be fast
 */
bool athome_bt_connection_went_down(uint16_t handle);
bool athome_bt_encr_changed(uint16_t handle);
bool athome_bt_encr_refreshed(uint16_t handle);
bool athome_bt_cmd_sta_or_compl(uint16_t ogf, uint16_t ocf,
				const uint8_t *buf, uint32_t len);
bool athome_bt_compl_pkts(uint16_t handle, uint16_t num);
bool athome_bt_process_le_data(uint16_t handle,
				const uint8_t *buf, uint32_t len);

/*
 * An LE event was delivered. Do quick processing on it or save it for later.
 * Please do not make this function slow - the other BT stack will be waiting
 * for you.
 */
void athome_bt_process_le_evt(const uint8_t *buf, uint32_t len);

/*
 * Prepare the LE stack to start. This setus up all the stack needs to start
 * and does everything except starting. This function may sleep/block as
 * needed. If all goes well, athome_bt_stack_start() may be called later.
 */
void athome_bt_stack_prepare(void);

/*
 * Actually start the stack. Possibly called from atomic context, Cannot block.
 * Will always be preceded by a call to athome_bt_stack_prepare().
 */
void athome_bt_stack_start(void);



/*
 *	cleanup state in prep for future restart of LE stack
 */
void athome_bt_stack_shutdown(void);


/*
 *	This must exist elsewhere in the kernel. Used for audio packets
 */
void athome_bt_audio_dec(int which, int format,
				const uint8_t *data, unsigned len);



#endif

