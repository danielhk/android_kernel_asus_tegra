/*
 * ALSA driver for Bemote audio
 *
 * SBC encoded packets are received over BTLE from Bemote.
 * They are then decoded and written to a PCM buffer.
 *
 * Copyright (C) 2013 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/math64.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/info.h>
#include <sound/initval.h>

#include "sbcdec.h"

/*
 * TODO Report underflow up to Audio HAL so it can close the driver.
 */

/* If set to 1 then spew info about the underflow
 * watchdog and silence generator. */
#define LOG_UNDERFLOW_TIMER    0
/* If set to 1, occasionally print # SBC packets have been sent from BTLE. */
#define LOG_PACKET_COUNTS    1

#define btlesbc_log(...)         pr_info("mic_btle_sbc: " __VA_ARGS__)

MODULE_AUTHOR("Phil Burk <philburk@google.com>");
MODULE_DESCRIPTION("BTLE SBC Mic");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,RemoteMic soundcard}}");

/* defaults */
#define MAX_PCM_DEVICES     1
#define MAX_PCM_SUBSTREAMS  4
#define MAX_MIDI_DEVICES    0

/* Define these all in one place so they stay in sync. */
#define USE_RATE_MIN          8000
#define USE_RATE_MAX         16000
#define USE_RATES_ARRAY     { USE_RATE_MIN, 11025, USE_RATE_MAX }
#define USE_RATES_MASK \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000)	

#define MAX_FRAMES_PER_BUFFER  (8192)

#define USE_CHANNELS_MIN   1
#define USE_CHANNELS_MAX   1
#define USE_PERIODS_MIN    1
#define USE_PERIODS_MAX    1024

#define MAX_PCM_BUFFER_SIZE  (MAX_FRAMES_PER_BUFFER * sizeof(int16_t))
#define MIN_PERIOD_SIZE      64
#define MAX_PERIOD_SIZE      (MAX_PCM_BUFFER_SIZE / 8)
#define USE_FORMATS          (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)

struct sbc_fifo_packet {
	uint8_t  which; /* Bemote index */
	uint8_t  format; /* SBC encoding type, 0-3 */
	uint8_t  new_timings;
	uint8_t  num_bytes;
	/* Expect no more than 25 bytes. But align struct size to power of 2. */
	uint8_t  sbc_data[28];
};

/* Actual minimum is 40 but we want a power of 2. */
#define MIN_SAMPLES_PER_SBC_PACKET_P2  32
#define MAX_SBC_PACKETS_PER_BUFFER  \
		(MAX_FRAMES_PER_BUFFER / MIN_SAMPLES_PER_SBC_PACKET_P2)
#define MAX_SBC_BUFFER_SIZE  \
		(MAX_SBC_PACKETS_PER_BUFFER * sizeof(struct sbc_fifo_packet))

#define SND_BTLESBC_RUNNING_TIMEOUT_MSEC    (500)


#define TIMER_STATE_BEFORE_SBC    0
#define TIMER_STATE_DURING_SBC    1
#define TIMER_STATE_AFTER_SBC     2

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;  /* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;   /* ID for this card */
static bool enable[SNDRV_CARDS] = {true, false};
/* Linux does not like NULL initialization. */
static char *model[SNDRV_CARDS]; /* = {[0 ... (SNDRV_CARDS - 1)] = NULL}; */
static int pcm_devs[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};
static int pcm_substreams[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for btlesbc soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for btlesbc soundcard.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable this btlesbc soundcard.");
module_param_array(model, charp, NULL, 0444);
MODULE_PARM_DESC(model, "Soundcard model.");
module_param_array(pcm_devs, int, NULL, 0444);
MODULE_PARM_DESC(pcm_devs, "PCM devices # (0-4) for btlesbc driver.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams,
	"PCM substreams # (1-128) for btlesbc driver?");

static struct platform_device *devices[SNDRV_CARDS];

/*
 * Static substream is needed so Bluetooth can pass SBC to a running stream.
 * This also serves to enable or disable the decoding of audio in the callback.
 */
static struct snd_pcm_substream *s_substream_for_btle;
static DEFINE_SPINLOCK(s_substream_lock);

struct simple_atomic_fifo {
	/* Read and write cursors are modified by different threads. */
	uint read_cursor;
	uint write_cursor;
	/* Size must be a power of two. */
	uint size;
	/* internal mask is 2*size - 1
	 * This allows us to tell the difference between full and empty. */
	uint internal_mask;
	uint external_mask;
};

struct snd_btlesbc {
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;

	uint32_t sample_rate;

	uint previous_jiffies; /* Used to detect underflows. */
	uint timeout_jiffies;
	struct timer_list decoding_timer;
	uint timer_state;
	bool timer_enabled;
	uint timer_callback_count;

	/* Decoder */
	struct simple_atomic_fifo sbc_fifo_controller;
	struct sbc_fifo_packet *sbc_fifo_packet_buffer;
	int16_t sbc_output[SBC_MAX_SAMPLES_PER_PACKET];
	int16_t peak_level;

	/*
	 * Write_index is the circular buffer position.
	 * It is advanced by the BTLE thread after decoding SBC.
	 * It is read by ALSA in btlesbc_pcm_pointer().
	 * It is not declared volatile because that is not
	 * allowed in the Linux kernel.
	 */
	uint32_t write_index;
	uint32_t frames_per_buffer;
	/* count frames generated so far in this period */
	uint32_t frames_in_period;
	int16_t *pcm_buffer;
};

/***************************************************************************/
/************* Atomic FIFO *************************************************/
/***************************************************************************/
/*
 * This FIFO is atomic if used by no more than 2 threads.
 * One thread modifies the read cursor and the other
 * thread modifies the write_cursor.
 * Size and mask are not modified while being used.
 *
 * The read and write cursors range internally from 0 to (2*size)-1.
 * This allows us to tell the difference between full and empty.
 * When we get the cursors for external use we mask with size-1.
 *
 * Memory barriers required on SMP platforms.
 */
static int atomic_fifo_init(struct simple_atomic_fifo *fifo_ptr, uint size)
{
	/* Make sure size is a power of 2. */
	if ((size & (size-1)) != 0) {
		pr_err("%s:%d - ERROR FIFO size = %d, not power of 2!\n",
			__func__, __LINE__, size);
		return -EINVAL;
	}
	fifo_ptr->read_cursor = 0;
	fifo_ptr->write_cursor = 0;
	fifo_ptr->size = size;
	fifo_ptr->internal_mask = (size * 2) - 1;
	fifo_ptr->external_mask = size - 1;
	smp_wmb();
	return 0;
}


static uint atomic_fifo_available_to_read(struct simple_atomic_fifo *fifo_ptr)
{
	smp_rmb();
	return (fifo_ptr->write_cursor - fifo_ptr->read_cursor)
			& fifo_ptr->internal_mask;
}

static uint atomic_fifo_available_to_write(struct simple_atomic_fifo *fifo_ptr)
{
	smp_rmb();
	return fifo_ptr->size - atomic_fifo_available_to_read(fifo_ptr);
}

static void atomic_fifo_advance_read(
		struct simple_atomic_fifo *fifo_ptr,
		uint frames)
{
	smp_rmb();
	BUG_ON(frames > atomic_fifo_available_to_read(fifo_ptr));
	fifo_ptr->read_cursor = (fifo_ptr->read_cursor + frames)
			& fifo_ptr->internal_mask;
	smp_wmb();
}

static void atomic_fifo_advance_write(
		struct simple_atomic_fifo *fifo_ptr,
		uint frames)
{
	smp_rmb();
	BUG_ON(frames > atomic_fifo_available_to_write(fifo_ptr));
	fifo_ptr->write_cursor = (fifo_ptr->write_cursor + frames)
		& fifo_ptr->internal_mask;
	smp_wmb();
}

static uint atomic_fifo_get_read_index(struct simple_atomic_fifo *fifo_ptr)
{
	smp_rmb();
	return fifo_ptr->read_cursor & fifo_ptr->external_mask;
}

static uint atomic_fifo_get_write_index(struct simple_atomic_fifo *fifo_ptr)
{
	smp_rmb();
	return fifo_ptr->write_cursor & fifo_ptr->external_mask;
}

/****************************************************************************/
static void btlesbc_handle_frame_advance(
		struct snd_pcm_substream *substream, uint num_frames)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	btlesbc->frames_in_period += num_frames;
	/* Tell ALSA if we have advanced by one or more periods. */
	if (btlesbc->frames_in_period >= substream->runtime->period_size) {
		snd_pcm_period_elapsed(substream);
		btlesbc->frames_in_period %= substream->runtime->period_size;
	}
}

static uint32_t btlesbc_bump_write_index(
			struct snd_pcm_substream *substream,
			uint32_t num_samples)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	uint32_t pos = btlesbc->write_index;

	/* Advance write position. */
	pos += num_samples;
	/* Wrap around at end of the circular buffer. */
	pos %= btlesbc->frames_per_buffer;
	btlesbc->write_index = pos;

	btlesbc_handle_frame_advance(substream, num_samples);

	return pos;
}

/****************************************************************************/
#define SBC_NUM_FORMATS         (4)
#define SBC_MAX_FORMAT          (SBC_NUM_FORMATS - 1)
#define SBC_SAMPLES_PER_BLOCK   (8)

/*
 * These settings are based on the @Home Remote Protocol document.
 * SBC format parameters arranged from low to high quality.
 * TODO Move to sbc decoder as inline functions.
 */
const uint8_t s_blocks_per_packet_old[SBC_NUM_FORMATS] = { 16, 12,  8,  5 };
const uint8_t s_num_bits_old[SBC_NUM_FORMATS]          = { 10, 14, 21, 33 };

const uint8_t s_blocks_per_packet_new[SBC_NUM_FORMATS] = { 16, 12,  8,  6 };
const uint8_t s_num_bits_new[SBC_NUM_FORMATS]          = { 10, 14, 21, 28 };

/*
 * Decode an SBC packet and write the PCM data into a circular buffer.
 */
static int btlesbc_decode_sbc_packet(
			struct snd_pcm_substream *substream,
			uint packet_type, /* describes SBC format, 0-3 */
			const uint8_t *sbc_input,
			size_t num_bytes,
			bool new_timings
			)
{
	uint num_samples = 0;
	uint remaining;
	uint i;
	uint32_t pos;
	uint read_index = 0;
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);

	/* Get decoder params */
	const uint8_t *s_blocks_per_packet = new_timings ?
			s_blocks_per_packet_new : s_blocks_per_packet_old;
	const uint8_t *s_num_bits = new_timings ?
			s_num_bits_new : s_num_bits_old;

	/* Decode SBC data to PCM. */
	uint blocks_per_packet = s_blocks_per_packet[packet_type];
	uint num_bits = s_num_bits[packet_type];
	num_samples = sbc_decode(blocks_per_packet, num_bits, sbc_input,
		num_bytes, &btlesbc->sbc_output[0]);

	/* Write PCM data to the buffer. */
	pos = btlesbc->write_index;
	if ((pos + num_samples) > btlesbc->frames_per_buffer) {
		for (i = pos; i < btlesbc->frames_per_buffer; i++)
			btlesbc->pcm_buffer[i] =
				btlesbc->sbc_output[read_index++];

		remaining = (pos + num_samples) - btlesbc->frames_per_buffer;
		for (i = 0; i < remaining; i++)
			btlesbc->pcm_buffer[i] =
				btlesbc->sbc_output[read_index++];

	} else {
		for (i = 0; i < num_samples; i++) {
			int16_t sample = btlesbc->sbc_output[read_index++];
			if (sample > btlesbc->peak_level)
				btlesbc->peak_level = sample;

			btlesbc->pcm_buffer[i + pos] = sample;
		}
	}

	btlesbc_bump_write_index(substream, num_samples);

	return num_samples;
}

/***********************************************************************/
/************ Statistics ***********************************************/
/***********************************************************************/
static int sbc_packet_counter;
/*
 * ATHOME_SBC_FORMAT_COUNT is based on the ATHOME Remote protocol for BTLE,
 * which defines a fixed number of audio packet types.
 */
#define ATHOME_SBC_FORMAT_COUNT    4

static int debug_decoder_callback_stats[ATHOME_SBC_FORMAT_COUNT];

#define PERCENT_SBC_PACKETS(format) (100 * \
	debug_decoder_callback_stats[format] / sbc_packet_counter)

/* Print the percentage of each SBC packet type received.
 * This gives us an indirect measure of the BTLE connection quality.
 * When BTLE is working well, we should get all packet format 3.
 */
static void athome_bt_dump_sbc_stats(struct snd_btlesbc *btlesbc)
{
	int i;
	if (sbc_packet_counter > 0) {
		int weighted_average;
		int score_int;
		int score_fraction;
		/* Print average score as a fake floating point value.
		 * This score is used to evaluate SBC compression quality,
		 * which is used to infer the BTLE transmission quality. */
		int sum = 0;
		for (i = 0; i < 4; i++)
			sum += i * debug_decoder_callback_stats[i];
		weighted_average = 100 * sum / sbc_packet_counter;
		score_int = weighted_average / 100;
		score_fraction = weighted_average - (score_int * 100);

		/* The code below is dependent on there being 4 formats.
		 * I could have used a for-loop but I did not want
		 * to spew 5 lines into the log. */
		BUG_ON(ATHOME_SBC_FORMAT_COUNT != 4);
		/* Ignore warning from checkpatch.pl about
		 * string concatenation. */
		btlesbc_log("%4d SBC packets decoded at %5d Hz"
			", formats (bad to good): "
			"%3d%% %3d%% %3d%% %3d%%, quality = %d.%02d\n",
			sbc_packet_counter,
			btlesbc->sample_rate,
			PERCENT_SBC_PACKETS(0),
			PERCENT_SBC_PACKETS(1),
			PERCENT_SBC_PACKETS(2),
			PERCENT_SBC_PACKETS(3),
			score_int, score_fraction
			);
	}
}

#undef PERCENT_SBC_PACKETS
/***********************************************************************/

/*
 * Write packet into the atomic SBC FIFO.
 * This is only called from athome_bt_audio_dec()
 * under the s_substream_lock spinlock.
 */
static bool athome_bt_audio_dec_ll(
			int which,
			int format,
			const uint8_t *sbc_input,
			size_t num_bytes,
			bool new_timings
			)
{
	bool dropped_packet = false;
	struct snd_pcm_substream *substream = s_substream_for_btle;
	if (substream != NULL) {
		struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
		/* Write SBC data to a FIFO for decoding by the timer task. */
		uint writable = atomic_fifo_available_to_write(
			&btlesbc->sbc_fifo_controller);
		if (writable > 0) {
			uint fifo_index = atomic_fifo_get_write_index(
				&btlesbc->sbc_fifo_controller);
			struct sbc_fifo_packet *packet =
				&btlesbc->sbc_fifo_packet_buffer[fifo_index];
			packet->which = (uint8_t)which;
			packet->format = (uint8_t)format;
			packet->new_timings = (uint8_t)(new_timings ? 1 : 0);
			packet->num_bytes = (uint8_t)num_bytes;
			memcpy(packet->sbc_data, sbc_input, num_bytes);
			atomic_fifo_advance_write(
				&btlesbc->sbc_fifo_controller, 1);
		} else {
			dropped_packet = true;
			s_substream_for_btle = NULL; /* Stop decoding. */
		}
	}
	sbc_packet_counter++;
	if ((uint)format < ATHOME_SBC_FORMAT_COUNT)
		debug_decoder_callback_stats[format] += 1;
	return dropped_packet;
}

/**
 * This is called by the Android@Home Bluetooth driver when it gets an
 * SBC packet from Bemote. It writes the packet into a FIFO
 * which is then read and decoded by the timer task.
 * @param which index of Bemote NOT USED
 * @param format AAH SBC format, 0-3
 * @param sbc_input pointer to SBC data to be decoded
 * @param num_bytes how many bytes in sbc_input
 * @param new_timings - set for new packet format in proto 0.1.0.1
 * @return number of samples decoded or negative error.
 */
void athome_bt_audio_dec(
			int which,
			int format,
			const uint8_t *sbc_input,
			size_t num_bytes,
			bool new_timings
			)
{
	bool dropped_packet;

	spin_lock(&s_substream_lock);
	dropped_packet = athome_bt_audio_dec_ll(which, format, sbc_input,
			num_bytes, new_timings);
	spin_unlock(&s_substream_lock);

	if (dropped_packet)
			btlesbc_log("WARNING, SBC packet dropped, FIFO full\n");

#if LOG_PACKET_COUNTS
	if ((sbc_packet_counter & 0x3FF) == 1)
		btlesbc_log("%s called %d times\n",
			__func__, sbc_packet_counter);
#endif
}

/*
 * Note that smp_rmb() is called by btlesbc_timer_callback()
 * before calling this function.
 *
 * Reads:
 *    jiffies
 *    btlesbc->previous_jiffies
 * Writes:
 *    btlesbc->previous_jiffies
 * Returns:
 *    num_frames needed to catch up to the current time
 */
static uint btlesbc_calc_frame_advance(struct snd_btlesbc *btlesbc)
{
	/* Determine how much time passed. */
	uint now_jiffies = jiffies;
	uint elapsed_jiffies = now_jiffies - btlesbc->previous_jiffies;
	/* Convert jiffies to frames. */
	uint frames_by_time = jiffies_to_msecs(elapsed_jiffies)
		* btlesbc->sample_rate / 1000;
	btlesbc->previous_jiffies = now_jiffies;

	/* Don't write more than one buffer full. */
	if (frames_by_time > (btlesbc->frames_per_buffer - 4))
		frames_by_time  = btlesbc->frames_per_buffer - 4;

	return frames_by_time;
}

/* Write zeros into the PCM buffer. */
static uint32_t btlesbc_write_silence(struct snd_btlesbc *btlesbc,
			uint32_t pos,
			int frames_to_advance)
{
	/* Does it wrap? */
	if ((pos + frames_to_advance) > btlesbc->frames_per_buffer) {
		/* Write to end of buffer. */
		int16_t *destination = &btlesbc->pcm_buffer[pos];
		size_t num_frames = btlesbc->frames_per_buffer - pos;
		size_t num_bytes = num_frames * sizeof(int16_t);
		memset(destination, 0, num_bytes);
		/* Write from start of buffer to new pos. */
		destination = &btlesbc->pcm_buffer[0];
		num_frames = frames_to_advance - num_frames;
		num_bytes = num_frames * sizeof(int16_t);
		memset(destination, 0, num_bytes);
	} else {
		/* Write within the buffer. */
		int16_t *destination = &btlesbc->pcm_buffer[pos];
		size_t num_bytes = frames_to_advance * sizeof(int16_t);
		memset(destination, 0, num_bytes);
	}
	/* Advance and wrap write_index */
	pos += frames_to_advance;
	pos %= btlesbc->frames_per_buffer;
	return pos;
}

/*
 * Called by timer task to decode SBC data from the FIFO into the PCM buffer.
 * Returns the number of packets decoded.
 */
static uint btlesbc_decode_from_fifo(struct snd_pcm_substream *substream)
{
	uint i;
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	uint readable = atomic_fifo_available_to_read(
		&btlesbc->sbc_fifo_controller);
	for (i = 0; i < readable; i++) {
		uint fifo_index = atomic_fifo_get_read_index(
			&btlesbc->sbc_fifo_controller);
		struct sbc_fifo_packet *packet =
			&btlesbc->sbc_fifo_packet_buffer[fifo_index];
		btlesbc_decode_sbc_packet(substream,
				packet->format,
				packet->sbc_data,
				packet->num_bytes,
				(packet->new_timings > 0));
		atomic_fifo_advance_read(&btlesbc->sbc_fifo_controller, 1);
	}
	return readable;
}

static int btlesbc_schedule_timer(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	uint msec_to_sleep = (substream->runtime->period_size * 1000)
			/ btlesbc->sample_rate;
	uint jiffies_to_sleep = msecs_to_jiffies(msec_to_sleep);
	if (jiffies_to_sleep < 2)
		jiffies_to_sleep = 2;
	ret = mod_timer(&btlesbc->decoding_timer, jiffies + jiffies_to_sleep);
	if (ret < 0)
		pr_err("%s:%d - ERROR in mod_timer, ret = %d\n",
			   __func__, __LINE__, ret);
	return ret;
}

static void btlesbc_timer_callback(unsigned long data)
{
	uint readable;
	uint packets_read;
	bool need_silence = false;
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);

	/* timer_enabled will be false when stopping a stream. */
	smp_rmb();
	if (!btlesbc->timer_enabled)
		return;
	btlesbc->timer_callback_count++;

	switch (btlesbc->timer_state) {
	case TIMER_STATE_BEFORE_SBC:
		readable = atomic_fifo_available_to_read(
				&btlesbc->sbc_fifo_controller);
		if (readable > 0) {
			btlesbc->timer_state = TIMER_STATE_DURING_SBC;
			/* Fall through into next state. */
		} else {
			need_silence = true;
			break;
		}

	case TIMER_STATE_DURING_SBC:
		packets_read = btlesbc_decode_from_fifo(substream);
		if (packets_read > 0) {
			btlesbc->previous_jiffies = jiffies; /* Defer timeout */
			break;
		} else {
			if (s_substream_for_btle == NULL) {
				btlesbc->timer_state = TIMER_STATE_AFTER_SBC;
				/* Decoder died. Overflowed?
				 * Fall through into next state. */
			} else if ((jiffies - btlesbc->previous_jiffies) >
					btlesbc->timeout_jiffies) {
				btlesbc->timer_state = TIMER_STATE_AFTER_SBC;
				/* Disable BTLE thread. */
				s_substream_for_btle = NULL;
				btlesbc_log("audio UNDERFLOW detected\n");
				/* Fall through into next state. */
			} else
				break;
		}

	case TIMER_STATE_AFTER_SBC:
			need_silence = true;
		break;
	}

	/* Write silence before and after SBC. */
	if (need_silence) {
		uint frames_to_silence = btlesbc_calc_frame_advance(btlesbc);
		btlesbc->write_index = btlesbc_write_silence(
				btlesbc,
				btlesbc->write_index,
				frames_to_silence);
		/* This can cause btlesbc_pcm_trigger() to be called, which
		 * may try to stop the timer. */
		btlesbc_handle_frame_advance(substream, frames_to_silence);
	}

	smp_rmb();
	if (btlesbc->timer_enabled)
		btlesbc_schedule_timer(substream);
}

static void btlesbc_timer_start(struct snd_pcm_substream *substream)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	btlesbc->timer_enabled = true;
	btlesbc->previous_jiffies = jiffies;
	btlesbc->timeout_jiffies =
		msecs_to_jiffies(SND_BTLESBC_RUNNING_TIMEOUT_MSEC);
	btlesbc->timer_callback_count = 0;
	smp_wmb();
	setup_timer(&btlesbc->decoding_timer,
		btlesbc_timer_callback,
		(unsigned long)substream);

	btlesbc_schedule_timer(substream);
}

static void btlesbc_timer_stop(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);

	/* Tell timer function not to reschedule itself if it runs. */
	btlesbc->timer_enabled = false;
	smp_wmb();
	if (!in_interrupt()) {
		/* del_timer_sync will hang if called in the timer callback. */
		ret = del_timer_sync(&btlesbc->decoding_timer);
		if (ret < 0)
			pr_err("%s:%d - ERROR del_timer_sync failed, %d\n",
				__func__, __LINE__, ret);
	}
	/*
	 * Else if we are in an interrupt then we are being called from the
	 * middle of the btlesbc_timer_callback(). The timer will not get
	 * rescheduled because btlesbc->timer_enabled will be false
	 * at the end of btlesbc_timer_callback().
	 * We do not need to "delete" the timer.
	 * The del_timer functions just cancel pending timers.
	 * There are no resources that need to be cleaned up.
	 */
}

/* ===================================================================== */
/*
 * PCM interface
 */

static int btlesbc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		btlesbc_log("%s starting audio\n", __func__);

		memset(debug_decoder_callback_stats, 0,
			sizeof(debug_decoder_callback_stats));
		sbc_decoder_reset();
		btlesbc->peak_level = 0;
		btlesbc->previous_jiffies = jiffies;
		btlesbc->timer_state = TIMER_STATE_BEFORE_SBC;
		sbc_packet_counter = 0;
		btlesbc_timer_start(substream);
		 /* Enables callback from BTLE driver. */
		s_substream_for_btle = substream;
		smp_wmb(); /* so other thread will see s_substream_for_btle */
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		btlesbc_log("%s stopping audio, peak = %d, # packets = %d\n",
			__func__, btlesbc->peak_level, sbc_packet_counter);

		s_substream_for_btle = NULL;
		smp_wmb(); /* so other thread will see s_substream_for_btle */
		btlesbc_timer_stop(substream);
		return 0;
	}
	return -EINVAL;
}

static int btlesbc_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	btlesbc_log("%s, rate = %d, period_size = %d, buffer_size = %d\n",
		__func__, (int) runtime->rate,
		(int) runtime->period_size,
		(int) runtime->buffer_size);

	if (runtime->buffer_size > MAX_FRAMES_PER_BUFFER)
		return -EINVAL;

	btlesbc->sample_rate = runtime->rate;
	btlesbc->frames_per_buffer = runtime->buffer_size;

	return 0; /* TODO - review */
}

static struct snd_pcm_hardware btlesbc_pcm_hardware = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		USE_FORMATS,
	.rates =		USE_RATES_MASK,
	.rate_min =		USE_RATE_MIN,
	.rate_max =		USE_RATE_MAX,
	.channels_min =		USE_CHANNELS_MIN,
	.channels_max =		USE_CHANNELS_MAX,
	.buffer_bytes_max =	MAX_PCM_BUFFER_SIZE,
	.period_bytes_min =	MIN_PERIOD_SIZE,
	.period_bytes_max =	MAX_PERIOD_SIZE,
	.periods_min =		USE_PERIODS_MIN,
	.periods_max =		USE_PERIODS_MAX,
	.fifo_size =		0,
};

static int btlesbc_pcm_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *hw_params)
{
	int ret = 0;
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);

	btlesbc->write_index = 0;
	smp_wmb();

	return ret;
}

static int btlesbc_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static int btlesbc_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	int ret = atomic_fifo_init(&btlesbc->sbc_fifo_controller,
			MAX_SBC_PACKETS_PER_BUFFER);
	if (ret)
		return ret;

	runtime->hw = btlesbc->pcm_hw;
	if (substream->pcm->device & 1) {
		runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
		runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	}
	if (substream->pcm->device & 2)
		runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP
			| SNDRV_PCM_INFO_MMAP_VALID);

	btlesbc_log("%s, built %s %s\n", __func__, __DATE__, __TIME__);

	/*
	 * Allocate the maximum buffer now and then just use part of it when
	 * the substream starts. We don't need DMA because it will just
	 * get written to by the BTLE code.
	 */
	/* We only use this buffer in the kernel and we do not do
	 * DMA so vmalloc should be OK. */
	btlesbc->pcm_buffer = vmalloc(MAX_PCM_BUFFER_SIZE);
	if (btlesbc->pcm_buffer == NULL) {
		pr_err("%s:%d - ERROR PCM buffer allocation failed\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	/* We only use this buffer in the kernel and we do not do
	 * DMA so vmalloc should be OK. */
	btlesbc->sbc_fifo_packet_buffer = vmalloc(MAX_SBC_BUFFER_SIZE);
	if (btlesbc->sbc_fifo_packet_buffer == NULL) {
		pr_err("%s:%d - ERROR SBC buffer allocation failed\n",
			__func__, __LINE__);
		vfree(btlesbc->pcm_buffer);
		btlesbc->pcm_buffer = NULL;
		return -ENOMEM;
	}

	return 0;
}

static int btlesbc_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
#if LOG_PACKET_COUNTS
	athome_bt_dump_sbc_stats(btlesbc);
#endif
	if (btlesbc->timer_callback_count > 0)
		btlesbc_log("processed %d packets in %d timer callbacks\n",
			sbc_packet_counter, btlesbc->timer_callback_count);

	if (btlesbc->pcm_buffer) {
		vfree(btlesbc->pcm_buffer);
		btlesbc->pcm_buffer = NULL;
	}

	/*
	 * Use spinlock so we don't free the SBC FIFO when the
	 * BTLE driver is writing to it.
	 * The s_substream_for_btle should already be NULL by now.
	 */
	spin_lock(&s_substream_lock);
	if (btlesbc->sbc_fifo_packet_buffer) {
		vfree(btlesbc->sbc_fifo_packet_buffer);
		btlesbc->sbc_fifo_packet_buffer = NULL;
	}
	spin_unlock(&s_substream_lock);

	return 0;
}

static snd_pcm_uframes_t btlesbc_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	/* write_index is written by another driver thread */
	smp_rmb();
	return btlesbc->write_index;
}

static int btlesbc_pcm_copy(struct snd_pcm_substream *substream,
			  int channel, snd_pcm_uframes_t pos,
			  void __user *dst, snd_pcm_uframes_t count)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	short *output = (short *)dst;

	/* TODO Needs to be modified if we support more than 1 channel. */
	/*
	 * Copy from PCM buffer to user memory.
	 * Are we reading past the end of the buffer?
	 */
	if ((pos + count) > btlesbc->frames_per_buffer) {
		const int16_t *source = &btlesbc->pcm_buffer[pos];
		int16_t *destination = output;
		size_t num_frames = btlesbc->frames_per_buffer - pos;
		size_t num_bytes = num_frames * sizeof(int16_t);
		memcpy(destination, source, num_bytes);

		source = &btlesbc->pcm_buffer[0];
		destination += num_frames;
		num_frames = count - num_frames;
		num_bytes = num_frames * sizeof(int16_t);
		memcpy(destination, source, num_bytes);
	} else {
		const int16_t *source = &btlesbc->pcm_buffer[pos];
		int16_t *destination = output;
		size_t num_bytes = count * sizeof(int16_t);
		memcpy(destination, source, num_bytes);
	}

	return 0;
}

static int btlesbc_pcm_silence(struct snd_pcm_substream *substream,
			int channel, snd_pcm_uframes_t pos,
			snd_pcm_uframes_t count)
{
	return 0; /* Do nothing. Only used by output? */
}

static struct snd_pcm_ops btlesbc_pcm_ops_no_buf = {
	.open =		btlesbc_pcm_open,
	.close =	btlesbc_pcm_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	btlesbc_pcm_hw_params,
	.hw_free =	btlesbc_pcm_hw_free,
	.prepare =	btlesbc_pcm_prepare,
	.trigger =	btlesbc_pcm_trigger,
	.pointer =	btlesbc_pcm_pointer,
	.copy =		btlesbc_pcm_copy,
	.silence =	btlesbc_pcm_silence,
};

static int __devinit snd_card_btlesbc_pcm(
				struct snd_btlesbc *btlesbc,
				int device,
				int substreams)
{
	struct snd_pcm *pcm;
	struct snd_pcm_ops *ops;
	int err;

	err = snd_pcm_new(btlesbc->card, "BTLESBC PCM", device,
			  0, /* no playback substreams */
			  1, /* 1 capture substream */
			  &pcm);
	if (err < 0)
		return err;
	btlesbc->pcm = pcm;
	ops = &btlesbc_pcm_ops_no_buf;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, ops);
	pcm->private_data = btlesbc;
	pcm->info_flags = 0;
	strcpy(pcm->name, "BTLESBC PCM");

	return 0;
}

#if defined(CONFIG_SND_DEBUG) && defined(CONFIG_PROC_FS)
/*
 * proc interface
 */
static void print_formats(struct snd_btlesbc *btlesbc,
			  struct snd_info_buffer *buffer)
{
	int i;

	for (i = 0; i < SNDRV_PCM_FORMAT_LAST; i++) {
		if (btlesbc->pcm_hw.formats & (1ULL << i))
			snd_iprintf(buffer, " %s", snd_pcm_format_name(i));
	}
}

static void print_rates(struct snd_btlesbc *btlesbc,
			struct snd_info_buffer *buffer)
{
	static int rates[] = USE_RATES_ARRAY;
	int i;

	if (btlesbc->pcm_hw.rates & SNDRV_PCM_RATE_CONTINUOUS)
		snd_iprintf(buffer, " continuous");
	if (btlesbc->pcm_hw.rates & SNDRV_PCM_RATE_KNOT)
		snd_iprintf(buffer, " knot");
	for (i = 0; i < ARRAY_SIZE(rates); i++)
		if (btlesbc->pcm_hw.rates & (1 << i))
			snd_iprintf(buffer, " %d", rates[i]);
}

#define get_btlesbc_int_ptr(btlesbc, ofs) \
	(unsigned int *)((char *)&((btlesbc)->pcm_hw) + (ofs))
#define get_btlesbc_ll_ptr(btlesbc, ofs) \
	(unsigned long long *)((char *)&((btlesbc)->pcm_hw) + (ofs))

struct btlesbc_hw_field {
	const char *name;
	const char *format;
	unsigned int offset;
	unsigned int size;
};
#define FIELD_ENTRY(item, fmt) {		   \
	.name = #item,				   \
	.format = fmt,				   \
	.offset = offsetof(struct snd_pcm_hardware, item), \
	.size = sizeof(btlesbc_pcm_hardware.item) }

static struct btlesbc_hw_field fields[] = {
	FIELD_ENTRY(formats, "%#llx"),
	FIELD_ENTRY(rates, "%#x"),
	FIELD_ENTRY(rate_min, "%d"),
	FIELD_ENTRY(rate_max, "%d"),
	FIELD_ENTRY(channels_min, "%d"),
	FIELD_ENTRY(channels_max, "%d"),
	FIELD_ENTRY(buffer_bytes_max, "%ld"),
	FIELD_ENTRY(period_bytes_min, "%ld"),
	FIELD_ENTRY(period_bytes_max, "%ld"),
	FIELD_ENTRY(periods_min, "%d"),
	FIELD_ENTRY(periods_max, "%d"),
};

static void btlesbc_proc_read(struct snd_info_entry *entry,
			struct snd_info_buffer *buffer)
{
	struct snd_btlesbc *btlesbc = entry->private_data;
	int i;

	btlesbc_log("%s, fill in proc\n", __func__);
	for (i = 0; i < ARRAY_SIZE(fields); i++) {
		snd_iprintf(buffer, "%s ", fields[i].name);
		if (fields[i].size == sizeof(int))
			snd_iprintf(buffer, fields[i].format,
				*get_btlesbc_int_ptr(btlesbc,
					fields[i].offset));
		else
			snd_iprintf(buffer, fields[i].format,
				*get_btlesbc_ll_ptr(btlesbc, fields[i].offset));
		if (!strcmp(fields[i].name, "formats"))
			print_formats(btlesbc, buffer);
		else if (!strcmp(fields[i].name, "rates"))
			print_rates(btlesbc, buffer);
		snd_iprintf(buffer, "\n");
	}
}

static void btlesbc_proc_write(struct snd_info_entry *entry,
			struct snd_info_buffer *buffer)
{
	struct snd_btlesbc *btlesbc = entry->private_data;
	char line[64];

	while (!snd_info_get_line(buffer, line, sizeof(line))) {
		char item[20];
		const char *ptr;
		unsigned long long val;
		int i;

		ptr = snd_info_get_str(item, line, sizeof(item));
		for (i = 0; i < ARRAY_SIZE(fields); i++) {
			if (!strcmp(item, fields[i].name))
				break;
		}
		if (i >= ARRAY_SIZE(fields))
			continue;
		snd_info_get_str(item, ptr, sizeof(item));
		if (kstrtoull(item, 0, &val))
			continue;
		if (fields[i].size == sizeof(int))
			*get_btlesbc_int_ptr(btlesbc, fields[i].offset) = val;
		else
			*get_btlesbc_ll_ptr(btlesbc, fields[i].offset) = val;
	}
}

static void __devinit btlesbc_proc_init(struct snd_btlesbc *chip)
{
	struct snd_info_entry *entry;

	if (!snd_card_proc_new(chip->card, "btlesbc_pcm", &entry)) {
		snd_info_set_text_ops(entry, chip, btlesbc_proc_read);
		entry->c.text.write = btlesbc_proc_write;
		entry->mode |= S_IWUSR;
		entry->private_data = chip;
	}
}
#else
#define btlesbc_proc_init(x)
#endif /* CONFIG_SND_DEBUG && CONFIG_PROC_FS */

static int __devinit snd_btlesbc_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct snd_btlesbc *btlesbc;
	int idx, err;
	int dev = devptr->id;

	err = snd_card_create(index[dev], id[dev], THIS_MODULE,
			sizeof(struct snd_btlesbc), &card);
	if (err < 0)
		return err;
	btlesbc = card->private_data;
	btlesbc->card = card;

	for (idx = 0; idx < MAX_PCM_DEVICES && idx < pcm_devs[dev]; idx++) {
		if (pcm_substreams[dev] < 1)
			pcm_substreams[dev] = 1;
		if (pcm_substreams[dev] > MAX_PCM_SUBSTREAMS)
			pcm_substreams[dev] = MAX_PCM_SUBSTREAMS;
		err = snd_card_btlesbc_pcm(btlesbc, idx, pcm_substreams[dev]);
		if (err < 0)
			goto __nodev;
	}

	btlesbc->pcm_hw = btlesbc_pcm_hardware;

	strcpy(card->driver, "BTLESBC");
	strcpy(card->shortname, "BTLESBC");
	sprintf(card->longname, "BTLESBC %i audio from bemote", dev + 1);

	btlesbc_proc_init(btlesbc);

	snd_card_set_dev(card, &devptr->dev);

	err = snd_card_register(card);
	if (err == 0) {
		platform_set_drvdata(devptr, card);
		return 0;
	}
__nodev:
	snd_card_free(card);
	return err;
}

static int __devexit snd_btlesbc_remove(struct platform_device *devptr)
{
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int snd_btlesbc_suspend(
	struct platform_device *pdev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_btlesbc *btlesbc = card->private_data;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
	snd_pcm_suspend_all(btlesbc->pcm);
	return 0;
}

static int snd_btlesbc_resume(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	return 0;
}
#endif

#define SND_BTLESBC_DRIVER	"snd_btlesbc"

static struct platform_driver snd_btlesbc_driver = {
	.probe		= snd_btlesbc_probe,
	.remove		= __devexit_p(snd_btlesbc_remove),
#ifdef CONFIG_PM
	.suspend	= snd_btlesbc_suspend,
	.resume		= snd_btlesbc_resume,
#endif
	.driver		= {
		.name	= SND_BTLESBC_DRIVER
	},
};

static void snd_btlesbc_unregister_all(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(devices); ++i)
		platform_device_unregister(devices[i]);
	platform_driver_unregister(&snd_btlesbc_driver);
}

static int __init alsa_card_btlesbc_init(void)
{
	int i, cards, err;

	err = platform_driver_register(&snd_btlesbc_driver);
	if (err < 0)
		return err;

	cards = 0;
	for (i = 0; i < SNDRV_CARDS; i++) {
		struct platform_device *device;
		if (!enable[i])
			continue;
		device = platform_device_register_simple(SND_BTLESBC_DRIVER,
							 i, NULL, 0);
		if (IS_ERR(device))
			continue;
		if (!platform_get_drvdata(device)) {
			platform_device_unregister(device);
			continue;
		}
		devices[i] = device;
		cards++;
	}
	if (!cards) {
#ifdef MODULE
		pr_err("%s:%d - ERROR BTLESBC soundcard not found or device busy\n",
			__func__, __LINE__);
#endif
		snd_btlesbc_unregister_all();
		return -ENODEV;
	}
	return 0;
}

static void __exit alsa_card_btlesbc_exit(void)
{
	snd_btlesbc_unregister_all();
}

module_init(alsa_card_btlesbc_init)
module_exit(alsa_card_btlesbc_exit)
