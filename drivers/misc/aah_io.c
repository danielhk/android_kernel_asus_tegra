/* drivers/misc/aah_io.c
 *
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This is a driver that communicates with both a gpio_input
 * and lp5521 led driver.  It borrows some code from leds-lp5521.c
 * which has the following info:
 *    Copyright (C) 2010 Nokia Corporation
 *    Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *
 * It's userland interface for led control is an ioctl rather than
 * sysfs, and instead of 3 channels, it gives just one RGB control
 * since the lp5521 is used to control a single RGB led
 * and having separate sysfs interfaces for each channel is both
 * inefficient and would cause undesirable transition effects.
 *
 * The driver also combines gpio input handling to implement a
 * special factory reboot functionality during early boot until
 * disabled by the first IOCTL from userland.
 */
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/aah_io.h>

#include <linux/delay.h>
#include <linux/reboot.h>

MODULE_LICENSE("GPL v2");

#define LP5521_PROGRAM_LENGTH		32	/* in bytes */

#define LP5521_MAX_LEDS			3	/* Maximum number of LEDs */
#define LP5521_MAX_ENGINES		3	/* Maximum number of engines */

#define LP5521_ENG_MASK_BASE		0x30	/* 00110000 */
#define LP5521_ENG_STATUS_MASK		0x07	/* 00000111 */

#define LP5521_CMD_LOAD			0x15	/* 00010101 */
#define LP5521_CMD_RUN			0x2a	/* 00101010 */
#define LP5521_CMD_DIRECT		0x3f	/* 00111111 */
#define LP5521_CMD_DISABLED		0x00	/* 00000000 */

/* Registers */
#define LP5521_REG_ENABLE		0x00
#define LP5521_REG_OP_MODE		0x01
#define LP5521_REG_R_PWM		0x02
#define LP5521_REG_G_PWM		0x03
#define LP5521_REG_B_PWM		0x04
#define LP5521_REG_R_CURRENT		0x05
#define LP5521_REG_G_CURRENT		0x06
#define LP5521_REG_B_CURRENT		0x07
#define LP5521_REG_CONFIG		0x08
#define LP5521_REG_R_CHANNEL_PC		0x09
#define LP5521_REG_G_CHANNEL_PC		0x0A
#define LP5521_REG_B_CHANNEL_PC		0x0B
#define LP5521_REG_STATUS		0x0C
#define LP5521_REG_RESET		0x0D
#define LP5521_REG_GPO			0x0E
#define LP5521_REG_R_PROG_MEM		0x10
#define LP5521_REG_G_PROG_MEM		0x30
#define LP5521_REG_B_PROG_MEM		0x50

#define LP5521_PROG_MEM_BASE		LP5521_REG_R_PROG_MEM
#define LP5521_PROG_MEM_SIZE		0x20

/* Base register to set LED current */
#define LP5521_REG_LED_CURRENT_BASE	LP5521_REG_R_CURRENT

/* Base register to set the brightness */
#define LP5521_REG_LED_PWM_BASE		LP5521_REG_R_PWM

/* Bits in ENABLE register */
#define LP5521_MASTER_ENABLE		0x40	/* Chip master enable */
#define LP5521_LOGARITHMIC_PWM		0x80	/* Logarithmic PWM adjustment */
#define LP5521_EXEC_RUN			0x2A

/* Bits in CONFIG register */
#define LP5521_PWM_HF			0x40	/* PWM: 0 = 256Hz, 1 = 558Hz */
#define LP5521_PWRSAVE_EN		0x20	/* 1 = Power save mode */
#define LP5521_CP_MODE_OFF		0	/* Charge pump (CP) off */
#define LP5521_CP_MODE_BYPASS		8	/* CP forced to bypass mode */
#define LP5521_CP_MODE_1X5		0x10	/* CP forced to 1.5x mode */
#define LP5521_CP_MODE_AUTO		0x18	/* Automatic mode selection */
#define LP5521_R_TO_BATT		4	/* R out: 0 = CP, 1 = Vbat */
#define LP5521_CLK_SRC_EXT		0	/* Ext-clk source (CLK_32K) */
#define LP5521_CLK_INT			1	/* Internal clock */
#define LP5521_CLK_AUTO			2	/* Automatic clock selection */

/* Status */
#define LP5521_EXT_CLK_USED		0x08

#define WIPE_WORKER_DELAY_MS 100
#define WIPE_TIMEOUT_SECS 10

struct aah_io_driver_state {
	struct aah_io_platform_data *pdata;
	struct mutex lock;

	/* I2C client/device and platform specific data
	 * (BSP and GPIO selection)
	 */
	struct i2c_client *i2c_client;

	/* Device node registation */
	struct miscdevice dev_node;
	int dev_node_registered;

	/* Input device registration */
	struct platform_device *key_dev;

	/* Used to support wipe feature during early boot */
	struct workqueue_struct *workq;
	struct delayed_work dwork;
	unsigned long key_down_start_time;
	const struct led_rgb_vals *color;
	u8 key_down;

	/* animation mode */
	u8 led_mode;
};

/*
 * gpio_input_event() has no good way to get state so
 * we stash in a global
 */
static struct aah_io_driver_state *g_state;

static const struct led_rgb_vals red = {
	.rgb[0] = 128, .rgb[1] = 0, .rgb[2] = 0
};
static const struct led_rgb_vals black = {
	.rgb[0] = 0, .rgb[1] = 0, .rgb[2] = 0
};

static struct gpio_event_direct_entry gpio_keypad_keys_map[] = {
	{
		/* start by using MSC_RAW.  will switch to
		 * KEY_MUTE later once we're past boot stage.
		 */
		.code = MSC_RAW,

		/* .gpio to be filled in from pdata */
	},
};

static inline int lp5521_write(struct i2c_client *client, u8 reg, u8 value);

static int aah_io_led_set_mode(struct aah_io_driver_state *state,
			       const u8 mode)
{
	int rc = 0;
	struct i2c_client *client = state->i2c_client;

	if (mode != state->led_mode) {
		switch (mode) {
		case AAH_LED_MODE_POWER_UP_ANIMATION:
			lp5521_write(client,
				     LP5521_REG_OP_MODE, LP5521_CMD_RUN);
			state->led_mode = mode;
			break;
		case AAH_LED_MODE_DIRECT:
			lp5521_write(client,
				     LP5521_REG_OP_MODE, LP5521_CMD_DIRECT);
			state->led_mode = mode;
			break;
		default:
			pr_err("%s: unknown mode %d\n", __func__, mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

static int aah_io_led_set_rgb(struct aah_io_driver_state *state,
			      const struct led_rgb_vals *rgb_val)
{
	if (state->led_mode != AAH_LED_MODE_DIRECT)
		return -EFAULT;

	return i2c_smbus_write_i2c_block_data(state->i2c_client,
					      LP5521_REG_LED_PWM_BASE,
					      3, &rgb_val->rgb[0]);
}

static int gpio_input_event(struct gpio_event_input_devs *input_devs,
			    struct gpio_event_info *info,
			    void **data, unsigned int dev,
			    unsigned int type, unsigned int code,
			    int value)
{
	/* no clean way to get the state so have to use a global */
	struct aah_io_driver_state *state = g_state;
	if (!state) {
		pr_err("%s: no global state structure\n", __func__);
		return 0;
	}
	if (code != MSC_RAW) {
		pr_warn("%s: unexpected code %d\n", __func__, code);
		return 0;
	}
	if (!state->workq) {
		/* no workq, odd, just return */
		return 0;
	}
	if (value) {
		pr_debug("%s: key down\n", __func__);
		/* start delayed work.  worker will
		 * flash LEDs and eventually cause
		 * wipe if not cancelled.
		 */
		state->key_down = 1;
		state->key_down_start_time = jiffies;
		queue_delayed_work(state->workq, &state->dwork, 0);
	} else {
		pr_debug("%s: key released\n", __func__);
		state->key_down = 0;
	}
	return 0;
}

static struct gpio_event_input_info gpio_keypad_keys_info = {
	.info.func = gpio_event_input_func,
	.info.event = gpio_input_event,
#ifdef DEBUG
	.flags = GPIOEDF_PRINT_KEY_DEBOUNCE | GPIOEDF_PRINT_KEYS,
#endif
	.info.no_suspend = true,
	/* start by generating EV_MSC so our event handler is
	 * called.  later switch to EV_KEY once we're past
	 * boot stage and in normal operating mode.  our
	 * event handler allows implementation of the factory
	 * reset upon key held during early boot.
	 */
	.type = EV_MSC,
	.keymap = gpio_keypad_keys_map,
	.keymap_size = ARRAY_SIZE(gpio_keypad_keys_map),
	.debounce_time.tv64 = 2 * NSEC_PER_MSEC,
};

static struct gpio_event_info *gpio_keypad_info[] = {
	&gpio_keypad_keys_info.info,
};

static struct gpio_event_platform_data gpio_keypad_data = {
	.name = "gpio-keypad",
	.info = gpio_keypad_info,
	.info_count = ARRAY_SIZE(gpio_keypad_info)
};

static void cleanup_driver_state(struct aah_io_driver_state *state)
{
	if (NULL == state)
		return;

	if (state->dev_node_registered) {
		misc_deregister(&state->dev_node);
		state->dev_node_registered = 0;
	}

	if (state->key_dev) {
		platform_device_unregister(state->key_dev);
		state->key_dev = NULL;
	}

	if (state->workq) {
		destroy_workqueue(state->workq);
		state->workq = NULL;
	}

	i2c_set_clientdata(state->i2c_client, NULL);
	kfree(state);
}

static int aah_io_leddev_open(struct inode *inode, struct file *file)
{
	/* nothing special to do here right now. */
	return 0;
}

static long aah_io_leddev_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct aah_io_driver_state *state;
	struct miscdevice *m = file->private_data;
	long rc = 0;

	state = container_of(m, struct aah_io_driver_state, dev_node);

	mutex_lock(&state->lock);

	/* on first call from userspace that's not a read,
	 * cancel the worker that's processing wipe reset, if any
	 */
	if (state->workq && (_IOC_DIR(cmd) == _IOC_WRITE)) {
		pr_info("%s: First write ioctl from userspace, "
			"destroying workq\n", __func__);

		/*
		 * unregister the device first to prevent any
		 * new gpio_input_event() calls from occurring.
		 */
		platform_device_unregister(state->key_dev);

		/* Cancel any pending works and destroy workq */
		cancel_delayed_work_sync(&state->dwork);
		destroy_workqueue(state->workq);
		state->workq = NULL;

		aah_io_led_set_mode(state, AAH_LED_MODE_POWER_UP_ANIMATION);

		/* reregister the gpio_event as a key if a code is provided. */
		if (state->pdata->key_code) {
			gpio_keypad_keys_map[0].code = state->pdata->key_code;
			gpio_keypad_keys_info.type = EV_KEY;
			state->key_dev = platform_device_register_data(NULL,
					       GPIO_EVENT_DEV_NAME,
					       0,
					       &gpio_keypad_data,
					       sizeof(gpio_keypad_data));
		} else {
			state->key_dev = NULL;
		}
	}
	pr_debug("%s: cmd = 0x%x\n", __func__, cmd);

	switch (cmd) {
	case AAH_IO_LED_GET_MODE: {
		u8 val = state->led_mode;
		pr_debug("%s: get mode\n", __func__);
		if (copy_to_user((void __user *)arg,
				&val, sizeof(val)))
			rc = -EFAULT;
	} break;

	case AAH_IO_LED_SET_MODE: {
		u8 val;
		pr_debug("%s: set mode\n", __func__);
		if (copy_from_user(&val, (const void __user *)arg,
					sizeof(val))) {
			rc = -EFAULT;
			break;
		}
		rc = aah_io_led_set_mode(state, val);
	} break;

	case AAH_IO_LED_SET_RGB: {
		struct led_rgb_vals req;

		pr_debug("%s: set rgb\n", __func__);
		if (copy_from_user(&req, (const void __user *)arg,
					sizeof(req))) {
			rc = -EFAULT;
			break;
		}
		rc = aah_io_led_set_rgb(state, &req);
	} break;

	default: {
		pr_err("%s: unknown ioctl 0x%x\n", __func__, cmd);
		rc = -EINVAL;
	} break;

	}

	mutex_unlock(&state->lock);

	return rc;
}

static const struct file_operations aah_io_led_fops = {
	.owner = THIS_MODULE,
	.open = aah_io_leddev_open,
	.unlocked_ioctl = aah_io_leddev_ioctl,
};

static void aah_io_wipe_worker(struct work_struct *work)
{
	struct aah_io_driver_state *state;

	state = container_of(work, typeof(*state), dwork.work);
	if (state->key_down) {
		unsigned long time_down;

		/* since we're running in early boot, we don't
		 * expect any wrapping of jiffies
		 */
		time_down = jiffies - state->key_down_start_time;
		if (time_down >
		    msecs_to_jiffies(WIPE_TIMEOUT_SECS * 1000)) {
			pr_info("%s: key down more than %u seconds,"
				" starting recovery to wipe data\n",
				__func__, WIPE_TIMEOUT_SECS);
			kernel_restart("recovery:wipe_data");
		}
		pr_debug("%s: key still down after %u ms\n",
			__func__, jiffies_to_msecs(time_down));
		/* toggle led red and black while down
		 * to give user some feedback
		 */
		if (state->color == &red)
			state->color = &black;
		else
			state->color = &red;
		aah_io_led_set_mode(state, AAH_LED_MODE_DIRECT);
		aah_io_led_set_rgb(state, state->color);

		/* prepare next poll check */
		queue_delayed_work(state->workq,
				   &state->dwork,
				   msecs_to_jiffies(WIPE_WORKER_DELAY_MS));
	} else {
		/* Switch back to power up animation mode as
		 * device continues to boot.
		 */
		aah_io_led_set_mode(state, AAH_LED_MODE_POWER_UP_ANIMATION);
	}
}

static inline int lp5521_write(struct i2c_client *client, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int lp5521_read(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return -EIO;

	*buf = ret;
	return 0;
}

/* we suppose the chip is already initialized by the bootloader */
static int __devinit lp5521_is_enabled(struct i2c_client *client, u8 *mode)
{
	int ret;
	u8 val;

	/* see if the chip is enabled */
	ret  = lp5521_read(client, LP5521_REG_ENABLE, &val);
	if (ret)
		return 0;
	if (!(val & LP5521_MASTER_ENABLE) || !(val & LP5521_EXEC_RUN))
		return 0;

	/* see if the chip is in run mode */
	ret = lp5521_read(client, LP5521_REG_OP_MODE, &val);
	if (ret)
		return 0;

	if (val == LP5521_CMD_RUN)
		*mode = AAH_LED_MODE_POWER_UP_ANIMATION;
	else if (val == LP5521_CMD_DIRECT)
		*mode = AAH_LED_MODE_DIRECT;
	else
		return 0;

	return 1;
}

static int aah_io_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct aah_io_driver_state *state;
	struct aah_io_platform_data *pdata;
	int rc;

	pr_info("%s\n", __func__);
	state = kzalloc(sizeof(struct aah_io_driver_state),
			GFP_KERNEL);
	if (NULL == state) {
		pr_err("Failed to allocate state structure");
		rc = -ENOMEM;
		goto error;
	}

	mutex_init(&state->lock);

	/* Do this before registering the devices */
	state->workq = create_singlethread_workqueue("aah_io_worker");
	state->color = &black;
	INIT_DELAYED_WORK(&state->dwork, aah_io_wipe_worker);

	/* Hook up all of the state structures to each other. */
	i2c_set_clientdata(client, state);
	state->i2c_client = client;
	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		rc = -ENODEV;
		goto error;
	}
	state->pdata = pdata;

	if (!lp5521_is_enabled(client, &state->led_mode)) {
		dev_err(&client->dev, "not enabled\n");
		rc = -EINVAL;
		goto error;
	}

	dev_info(&client->dev, "lp5521 programmable led running");

	/* allocate the gpio_input device */
	gpio_keypad_keys_map[0].gpio = pdata->key_gpio;
	state->key_dev = platform_device_register_data(NULL,
					       GPIO_EVENT_DEV_NAME,
					       0,
					       &gpio_keypad_data,
					       sizeof(gpio_keypad_data));

	/* register the LED device */
	state->dev_node.minor = MISC_DYNAMIC_MINOR,
	state->dev_node.name  = "led",
	state->dev_node.fops  = &aah_io_led_fops,

	rc = misc_register(&state->dev_node);
	if (rc) {
		pr_err("Failed to register LED device node (rc = %d)", rc);
		goto error;
	}

	pr_info("AAH IO Driver loaded.\n");
	g_state = state;

	return rc;

error:
	cleanup_driver_state(state);
	return rc;
}

static int __devexit aah_io_remove(struct i2c_client *client)
{
	struct aah_io_driver_state *state = i2c_get_clientdata(client);

	/* Switch back to boot animation mode before we clean out our state and
	 * finish unloading the driver.
	 */
	aah_io_led_set_mode(state, AAH_LED_MODE_POWER_UP_ANIMATION);

	/* cleanup our state and get out. */
	cleanup_driver_state(state);

	return 0;
}

static void aah_io_shutdown(struct i2c_client *client)
{
	struct aah_io_driver_state *state = i2c_get_clientdata(client);

	/* Switch back to power up animation mode as device reboots. */
	aah_io_led_set_mode(state, AAH_LED_MODE_POWER_UP_ANIMATION);
}

static struct i2c_device_id aah_io_idtable[] = {
	{ "aah-io", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, aah_io_idtable);

static struct i2c_driver aah_io_driver = {
	.driver = {
		.name = "aah-io",
	},

	.id_table = aah_io_idtable,
	.probe = aah_io_probe,
	.remove = __devexit_p(aah_io_remove),

	/* TODO(johngro) implement these optional power management routines. */
	.shutdown = aah_io_shutdown,
	.suspend = NULL,
	.resume = NULL,
};

static int aah_io_init(void)
{
	return i2c_add_driver(&aah_io_driver);
}

static void aah_io_exit(void)
{
	i2c_del_driver(&aah_io_driver);
}

module_init(aah_io_init);
module_exit(aah_io_exit);
