#ifndef _SBC_DEC_H_
#define _SBC_DEC_H_

#include <linux/types.h>

#define SBC_MAX_SAMPLES_PER_PACKET   128
#define SBC_MAX_PACKET_SIZE          262  /* 2 + 8 / 2 + 16 * 8 * 16 / 8 */

/**
 * Reset the SBC audio decoder state.
 */
void sbc_decoder_reset(void);

/**
 * Decode a packet of SBC audio data to PCM.
 */
uint32_t sbc_decode(uint8_t blocks_per_packet, uint8_t num_bits,
	const uint8_t* buf, uint16_t len, int16_t* outbuf);	//return num SAMPLES produced


#endif


