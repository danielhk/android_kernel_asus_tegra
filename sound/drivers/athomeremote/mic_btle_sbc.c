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

MODULE_AUTHOR("Phil Burk <philburk@google.com>");
MODULE_DESCRIPTION("BTLE SBC Mic");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,RemoteMic soundcard}}");

/* If set to 1, generate a sine wave instead of decoding SBC audio. */
#define FAKE_SOUND 0
/* If set to 1, occasionally print how many SBC packets
 * have been sent from BTLE. */
#define DEBUG_COUNT_PACKETS 1

/* defaults */
#define MAX_PCM_DEVICES     1
#define MAX_PCM_SUBSTREAMS  4
#define MAX_MIDI_DEVICES    0

#define SUPPORTED_SAMPLE_RATE   16000

#define MAX_FRAMES_PER_BUFFER  (8192)

#define USE_CHANNELS_MIN   1
#define USE_CHANNELS_MAX   1
#define USE_PERIODS_MIN    1
#define USE_PERIODS_MAX    1024

#define MAX_BUFFER_SIZE     (MAX_FRAMES_PER_BUFFER * sizeof(int16_t))
#define MIN_PERIOD_SIZE      64
#define MAX_PERIOD_SIZE      (MAX_BUFFER_SIZE / 8)
#define USE_FORMATS          (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)

/* Use preprocessor trick to build name. */
#define __PCM_RATE_NAME__(x) SNDRV_PCM_RATE_##x
#define PCM_RATE_NAME(x)     __PCM_RATE_NAME__(x)
#define USE_RATE             PCM_RATE_NAME(SUPPORTED_SAMPLE_RATE)

#define USE_RATE_MIN         SUPPORTED_SAMPLE_RATE
#define USE_RATE_MAX         SUPPORTED_SAMPLE_RATE

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;  /* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;   /* ID for this card */
static int enable[SNDRV_CARDS] = {1, 0};
/* Linux does not like NULL initialization. */
static char *model[SNDRV_CARDS]; /* = {[0 ... (SNDRV_CARDS - 1)] = NULL}; */
static int pcm_devs[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};
static int pcm_substreams[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for btlesbc soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for btlesbc soundcard.");
module_param_array(enable, int, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable this btlesbc soundcard.");
module_param_array(model, charp, NULL, 0444);
MODULE_PARM_DESC(model, "Soundcard model.");
module_param_array(pcm_devs, int, NULL, 0444);
MODULE_PARM_DESC(pcm_devs, "PCM devices # (0-4) for btlesbc driver.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams,
	"PCM substreams # (1-128) for btlesbc driver?");


static struct platform_device *devices[SNDRV_CARDS];

/* TODO Hack so Bluetooth can pass SBC to a running stream.
 * This also serves to enable or disable the decoding of audio in the callback.
 */
struct snd_pcm_substream *s_current_substream;

struct snd_btlesbc {
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;
#if FAKE_SOUND
	uint previous_jiffies; /* Used when faking an audio source. */
	uint sine_phase;
#endif
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
	int16_t *buffer;
	int16_t sbc_output[SBC_MAX_SAMPLES_PER_PACKET];
};

#if FAKE_SOUND
static struct timer_list synth_timer;

static const int16_t s_sine_wave[100] = {
	     0,   2057,   4106,   6139,   8148,  10125,  12062,  13951,
	 15785,  17557,  19259,  20886,  22430,  23886,  25247,  26509,
	 27666,  28713,  29648,  30465,  31163,  31737,  32186,  32508,
	 32702,  32767,  32702,  32508,  32186,  31737,  31163,  30465,
	 29648,  28713,  27666,  26509,  25247,  23886,  22430,  20886,
	 19259,  17557,  15785,  13951,  12062,  10125,   8148,   6139,
	  4106,   2057,      0,  -2057,  -4106,  -6139,  -8148, -10125,
	-12062, -13951, -15785, -17557, -19259, -20886, -22430, -23886,
	-25247, -26509, -27666, -28713, -29648, -30465, -31163, -31737,
	-32186, -32508, -32702, -32767, -32702, -32508, -32186, -31737,
	-31163, -30465, -29648, -28713, -27666, -26509, -25247, -23886,
	-22430, -20886, -19259, -17557, -15785, -13951, -12062, -10125,
	 -8148,  -6139,  -4106,  -2057
};

#endif

static void btlesbc_handle_frame_advance(
		struct snd_pcm_substream *substream, uint num_frames)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	btlesbc->frames_in_period += num_frames;
	/* Tell ALSA if we have advanced by a period. */
	if (btlesbc->frames_in_period > substream->runtime->period_size) {
		snd_pcm_period_elapsed(substream);
		btlesbc->frames_in_period = btlesbc->frames_in_period
			% substream->runtime->period_size;
	}
}

/* ===================================================================== */
#define SBC_NUM_FORMATS         (4)
#define SBC_MAX_FORMAT          (SBC_NUM_FORMATS - 1)
#define SBC_SAMPLES_PER_BLOCK   (8)

/*
 * These settings are based on the @Home Remote Protocol document.
 * SBC format parameters arranged from low to high quality.
 * TODO Move to sbc decoder as inline functions.
 */
const uint8_t s_blocks_per_packet[SBC_NUM_FORMATS] = { 16, 12,  8,  5 };
const uint8_t s_num_bits[SBC_NUM_FORMATS]          = { 10, 14, 21, 33 };

static int btlesbc_decode_sbc_packet(
			struct snd_pcm_substream *substream,
			uint packet_type, /* describes SBC format, 0-3 */
			const uint8_t *sbc_input,
			size_t num_bytes
			)
{
	uint num_samples;
	uint remaining;
	uint i;
	uint32_t pos;
	uint read_index = 0;
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);

	/* Decode SBC data to PCM. */
	uint blocks_per_packet = s_blocks_per_packet[packet_type];
	uint num_bits = s_num_bits[packet_type];
	num_samples = sbc_decode(blocks_per_packet, num_bits, sbc_input,
		num_bytes, &btlesbc->sbc_output[0]);

	/* Write PCM data to the buffer. */
	pos = btlesbc->write_index;
	if ((pos + num_samples) > btlesbc->frames_per_buffer) {
		for (i = pos; i < btlesbc->frames_per_buffer; i++)
			btlesbc->buffer[i] = btlesbc->sbc_output[read_index++];

		remaining = (pos + num_samples) - btlesbc->frames_per_buffer;
		for (i = 0; i < remaining; i++)
			btlesbc->buffer[i] = btlesbc->sbc_output[read_index++];

	} else {
		for (i = 0; i < num_samples; i++) {
			int16_t sample = btlesbc->sbc_output[read_index++];
			if (sample > btlesbc->peak_level)
				btlesbc->peak_level = sample;

			btlesbc->buffer[i + pos] = sample;
		}
	}

	/* Advance write position. */
	pos += num_samples;
	/* Wrap around at end of the circular buffer. */
	if (pos > btlesbc->frames_per_buffer)
		pos -= btlesbc->frames_per_buffer;

	btlesbc->write_index = pos;

	btlesbc_handle_frame_advance(substream, num_samples);

	return num_samples;
}

/**
 * This is called by the Android@Home Bluetooth driver when it gets an
 * SBC packet from Bemote.
 * @param which index of Bemote TODO
 * @param format AAH SBC format, 0-3
 * @param sbc_input pointer to SBC data to be decoded
 * @return number of samples decoded or negative error.
 */
void athome_bt_audio_dec(
			int which,
			int format,
			const uint8_t *sbc_input,
			size_t num_bytes
			)
{
#if DEBUG_COUNT_PACKETS
	static int counter;
#endif
	smp_rmb(); /* for s_current_substream and the write_index */
#if DEBUG_COUNT_PACKETS
	if ((counter++ & 0x3FF) == 0) {
		pr_info("%s called %d times, %s\n", __func__, counter,
			((s_current_substream == NULL) ? "ignored" : "on"));
	}
#endif
	if (s_current_substream != NULL) {
		/* TODO use array of substream pointers for 4 cards. */
		btlesbc_decode_sbc_packet(s_current_substream,
			format, sbc_input, num_bytes);
	}
	smp_wmb(); /* so other thread will see updated position */
}

/* ===================================================================== */
#if FAKE_SOUND
static uint btlesbc_elapsed_frames_by_time(struct snd_pcm_substream *substream)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	/* Determine how much time passed and simulate the advance of
	 * a hardware pointer. */
	uint now_jiffies = jiffies;
	uint elapsed_jiffies = now_jiffies - btlesbc->previous_jiffies;
	btlesbc->previous_jiffies = now_jiffies;
	/* Convert jiffies to frames. */
	return jiffies_to_msecs(elapsed_jiffies) * SUPPORTED_SAMPLE_RATE / 1000;
}

void synth_timer_callback(unsigned long data)
{
	int ret;
	uint32_t frames_to_sleep;
	uint32_t pos;
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	int frames_to_advance = btlesbc_elapsed_frames_by_time(substream);

	/* Simulate microphone by writing sine waves to buffer. DEBUG HACK */
	int frames_left = frames_to_advance;
	smp_rmb();
	uint32_t pos = btlesbc->write_index;
	while (frames_left > 0) {
		btlesbc->buffer[pos] = s_sine_wave[btlesbc->sine_phase];
		/* increment and wrap indices */
		btlesbc->sine_phase = (btlesbc->sine_phase + 1)
			% ARRAY_SIZE(s_sine_wave);
		pos = (pos + 1) % btlesbc->frames_per_buffer;
		frames_left -= 1;
	}
	btlesbc->write_index = pos;
	smp_wmb(); /* so other thread will see updated position */

	btlesbc_handle_frame_advance(substream, frames_to_advance);

	frames_to_sleep = substream->runtime->period_size
				- btlesbc->frames_in_period;
	if (frames_to_sleep > 0) {
		uint msec_to_sleep = (frames_to_sleep * 1000)
			/ SUPPORTED_SAMPLE_RATE;
		uint jiffies_to_sleep = msecs_to_jiffies(msec_to_sleep);

		if (jiffies_to_sleep < 2)
			jiffies_to_sleep  = 2;

		ret = mod_timer(&synth_timer, jiffies + jiffies_to_sleep);
		if (ret) {
			pr_err("%s:%d - ERROR in mod_timer\n",
				__func__, __LINE__);
		}
	}
}

void synth_timer_start(struct snd_pcm_substream *substream)
{
	int ret;
	int period_msec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	/* synth_timer.function, synth_timer.data */
	setup_timer(&synth_timer,
		synth_timer_callback,
		(unsigned long)substream);

	period_msec = runtime->period_size * 1000 / runtime->rate;
	ret = mod_timer(&synth_timer, jiffies + msecs_to_jiffies(period_msec));
	if (ret)
		pr_err("%s:%d - ERROR in mod_timer\n", __func__, __LINE__);
}

void synth_timer_stop(void)
{
	int ret = del_timer(&synth_timer);
	if (ret) {
		pr_err("%s:%d - ERROR the timer is still in use.\n",
			__func__, __LINE__);
	}
}
#endif  /* FAKE_SOUND */

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
		/* TODO use other device ids besides zero */
		pr_info("%s starting audio\n", __func__);
		sbc_decoder_reset();
		btlesbc->peak_level = 0;
		s_current_substream = substream;
		smp_wmb(); /* so other thread will see s_current_substream */
#if FAKE_SOUND
		synth_timer_start(substream);
#endif
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		pr_info("%s stopping audio, peak_level = %d\n",
			__func__, btlesbc->peak_level);
#if FAKE_SOUND
		synth_timer_stop();
#endif
		s_current_substream = NULL;
		smp_wmb(); /* so other thread will see s_current_substream */
		return 0;
	}
	return -EINVAL;
}

static int btlesbc_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
#define PRINT_RUNTIME(member) pr_info("%s, " #member " = %d\n", \
		__func__, (int) runtime->member);
	PRINT_RUNTIME(access);
	PRINT_RUNTIME(format);
	PRINT_RUNTIME(subformat);
	PRINT_RUNTIME(rate);
	PRINT_RUNTIME(channels);
	PRINT_RUNTIME(period_size);
	PRINT_RUNTIME(buffer_size);
	PRINT_RUNTIME(min_align);
	if (runtime->buffer_size > MAX_FRAMES_PER_BUFFER)
		return -EINVAL;

	btlesbc->frames_per_buffer = runtime->buffer_size;
	pr_info("%s, frames_per_buffer = %u\n", __func__,
		btlesbc->frames_per_buffer);

	return 0; /* TODO - review */
}

static struct snd_pcm_hardware btlesbc_pcm_hardware = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		USE_FORMATS,
	.rates =		USE_RATE,
	.rate_min =		USE_RATE_MIN,
	.rate_max =		USE_RATE_MAX,
	.channels_min =		USE_CHANNELS_MIN,
	.channels_max =		USE_CHANNELS_MAX,
	.buffer_bytes_max =	MAX_BUFFER_SIZE,
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

	pr_info("%s, frames_per_buffer = %u\n", __func__,
		btlesbc->frames_per_buffer);

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

#if FAKE_SOUND
	/* Initialize time for simulated signals. */
	btlesbc->previous_jiffies = jiffies;
#endif
	pr_info("mic_btle_sbc: %s, open substream\n", __func__);

	runtime->hw = btlesbc->pcm_hw;
	if (substream->pcm->device & 1) {
		runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
		runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	}
	if (substream->pcm->device & 2)
		runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
				      SNDRV_PCM_INFO_MMAP_VALID);

	/*
	 * Allocate the maximum buffer now and then just use part of it when
	 * the substream starts. We don't need DMA because it will just
	 * get written to by the BTLE code.
	 */
	pr_info("%s, allocating %u bytes, built %s %s\n", __func__,
		MAX_BUFFER_SIZE, __DATE__, __TIME__);
	/* We only use this buffer in this driver and we do not do
	 * DMA so vmalloc should be OK. */
	btlesbc->buffer = vmalloc(MAX_BUFFER_SIZE);
	if (btlesbc->buffer == NULL) {
		pr_err("%s:%d - ERROR buffer allocation failed\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	return 0;
}

static int btlesbc_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_btlesbc *btlesbc = snd_pcm_substream_chip(substream);
	pr_info("%s: CLOSE substream\n", __func__);

	if (btlesbc->buffer) {
		vfree(btlesbc->buffer);
		btlesbc->buffer = NULL;
	}
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
		const int16_t *source = &btlesbc->buffer[pos];
		int16_t *destination = output;
		size_t num_frames = btlesbc->frames_per_buffer - pos;
		size_t num_bytes = num_frames * sizeof(int16_t);
		memcpy(destination, source, num_bytes);

		source = &btlesbc->buffer[0];
		destination += num_frames;
		num_frames = count - num_frames;
		num_bytes = num_frames * sizeof(int16_t);
		memcpy(destination, source, num_bytes);
	} else {
		const int16_t *source = &btlesbc->buffer[pos];
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
	static int rates[] = { SUPPORTED_SAMPLE_RATE };
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

	pr_info("mic_btle_sbc: %s, fill in proc\n", __func__);
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
		if (strict_strtoull(item, 0, &val))
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
