/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <audio/dmic.h>
#include <drivers/i2s.h>
#include <drivers/gpio.h>
#include <string.h>

#define MAX_SAMPLE_RATE  15625
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

#define INITIAL_BLOCKS   2

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT      4
static K_MEM_SLAB_DEFINE(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

static int16_t echo_block[MAX_BLOCK_SIZE / 2];
static bool echo_enabled = false;
static K_SEM_DEFINE(toggle_transfer, 1, 1);

static enum {
	TOGGLE_ECHO,
	TOGGLE_STEREO
} user_request;
static K_SEM_DEFINE(button_pressed, 0, 1);

static void sw0_handler(const struct device *dev, struct gpio_callback *cb,
			uint32_t pins)
{
	user_request = TOGGLE_ECHO;
	k_sem_give(&button_pressed);
}

static void sw1_handler(const struct device *dev, struct gpio_callback *cb,
			uint32_t pins)
{
	user_request = TOGGLE_STEREO;
	k_sem_give(&button_pressed);
}

static bool init_buttons(void)
{
	static const struct button_spec {
		struct gpio_dt_spec gpio;
		char const *label;
		char const *action;
		gpio_callback_handler_t handler;
	} btn_spec[] = {
		{
			GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
			DT_PROP(DT_ALIAS(sw0), label),
			"toggle echo effect",
			sw0_handler
		},
		{
			GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
			DT_PROP(DT_ALIAS(sw1), label),
			"toggle stereo",
			sw1_handler
		},
	};
	static struct gpio_callback btn_cb_data[ARRAY_SIZE(btn_spec)];

	for (int i = 0; i < ARRAY_SIZE(btn_spec); ++i) {
		const struct button_spec *btn = &btn_spec[i];
		int ret;

		if (!device_is_ready(btn->gpio.port)) {
			printk("%s is not ready\n", btn->gpio.port->name);
			return false;
		}

		ret = gpio_pin_configure_dt(&btn->gpio, GPIO_INPUT);
		if (ret < 0) {
			printk("Failed to configure %s pin %d: %d\n",
				btn->gpio.port->name, btn->gpio.pin, ret);
			return false;
		}

		ret = gpio_pin_interrupt_configure_dt(&btn->gpio,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			printk("Failed to configure interrupt on %s pin %d: %d\n",
				btn->gpio.port->name, btn->gpio.pin, ret);
			return false;
		}

		gpio_init_callback(&btn_cb_data[i],
				   btn->handler, BIT(btn->gpio.pin));
		gpio_add_callback(btn->gpio.port, &btn_cb_data[i]);
		printk("-> press \"%s\" to %s\n", btn->label, btn->action);
	}

	return true;
}

static bool prepare_transfer(const struct device *i2s_dev, uint32_t block_size)
{
	int ret;

	for (int i = 0; i < INITIAL_BLOCKS; ++i) {
		void *mem_block;

		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			printk("Failed to allocate TX block %d: %d\n", i, ret);
			return false;
		}

		memset(mem_block, 0, block_size);

		ret = i2s_write(i2s_dev, mem_block, block_size);
		if (ret < 0) {
			printk("Failed to write block %d: %d\n", i, ret);
			return false;
		}
	}

	return true;
}

void main(void)
{
	const struct device *dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	const struct device *i2s_dev  = DEVICE_DT_GET(DT_NODELABEL(i2s_dev));
	int ret;

	printk("DMIC sample\n");

	if (!device_is_ready(dmic_dev)) {
		printk("%s is not ready\n", dmic_dev->name);
		return;
	}

	if (!device_is_ready(i2s_dev)) {
		printk("%s is not ready\n", dmic_dev->name);
		return;
	}

	struct pcm_stream_cfg stream = {
		.pcm_rate   = MAX_SAMPLE_RATE,
		.pcm_width  = SAMPLE_BIT_WIDTH,
		.mem_slab   = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	struct i2s_config i2s_cfg = {
		.word_size = SAMPLE_BIT_WIDTH,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = MAX_SAMPLE_RATE,
		.mem_slab = &mem_slab,
		.timeout = 0,
	};

	if (!init_buttons()) {
		return;
	}

	bool stereo = true;
	for (;;) {
		printk("Configuring %s stream\n", stereo ? "stereo" : "mono");
		if (stereo) {
			i2s_cfg.channels = 2;
			i2s_cfg.block_size = BLOCK_SIZE(MAX_SAMPLE_RATE, 2);
			stream.block_size = BLOCK_SIZE(MAX_SAMPLE_RATE, 2);
			cfg.channel.req_num_chan = 2;
			cfg.channel.req_chan_map_lo =
				dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
				dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
		} else {
			i2s_cfg.channels = 1;
			i2s_cfg.block_size = BLOCK_SIZE(MAX_SAMPLE_RATE, 1);
			stream.block_size = BLOCK_SIZE(MAX_SAMPLE_RATE, 1);
			cfg.channel.req_num_chan = 1;
			cfg.channel.req_chan_map_lo =
				dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
		}

		ret = dmic_configure(dmic_dev, &cfg);
		if (ret < 0) {
			printk("Failed to configure the driver: %d\n", ret);
			return;
		}

		ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
		if (ret < 0) {
			printk("Failed to configure I2S stream: %d\n", ret);
			return;
		}

		if (!prepare_transfer(i2s_dev, i2s_cfg.block_size)) {
			return;
		}

		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
		if (ret < 0) {
			printk("DMIC START trigger failed: %d\n", ret);
			return;
		}

		ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
		if (ret < 0) {
			printk("I2S START trigger failed: %d\n", ret);
			return;
		}

		for (;;) {
			void *buffer;
			uint32_t size;
			int ret;

			ret = dmic_read(dmic_dev, 0, &buffer, &size,
					READ_TIMEOUT);
			if (ret < 0) {
				printk("Read failed: %d\n", ret);
				return;
			}

			if (echo_enabled) {
				for (int i = 0; i < size/2; ++i) {
					int16_t *sample = &((int16_t *)buffer)[i];
					*sample += echo_block[i];
					echo_block[i] = (*sample) / 2;
				}
			}

			ret = i2s_write(i2s_dev, buffer, size);
			if (ret < 0) {
				printk("Write failed: %d\n", ret);
				return;
			}

			if (k_sem_take(&button_pressed, K_NO_WAIT) == 0) {
				if (user_request == TOGGLE_STEREO) {
					stereo = !stereo;
					break;
				}

				echo_enabled = !echo_enabled;
				memset(echo_block, 0, sizeof(echo_block));
				printk("Echo %sabled\n",
					echo_enabled ? "en" : "dis");
			}
		}

		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
		if (ret < 0) {
			printk("DMIC STOP trigger failed: %d\n", ret);
			return;
		}

		ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
		if (ret < 0) {
			printk("I2S DROP trigger failed: %d\n", ret);
			return;
		}
	}
}
