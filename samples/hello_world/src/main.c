/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/spi.h>
#include <hal/nrf_gpio.h>

#define SPI_NODE  DT_NODELABEL(spi4)

#if IS_ENABLED(CONFIG_SPI)
static struct spi_cs_control spi_cs_ctrl = {
    .gpio_dev = DEVICE_DT_GET(DT_GPIO_CTLR(SPI_NODE, cs_gpios)),
    .delay = 0,
    .gpio_pin = DT_GPIO_PIN(SPI_NODE, cs_gpios),
    .gpio_dt_flags = DT_GPIO_FLAGS(SPI_NODE, cs_gpios),
};

static struct spi_config spi_cfg = {
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .frequency = 2000000,
    .slave = 0,
    .cs = &spi_cs_ctrl,
};
#elif IS_ENABLED(CONFIG_NRFX_SPIM)
#include <nrfx_spim.h>
static const nrfx_spim_t spim = NRFX_SPIM_INSTANCE(4);
static K_SEM_DEFINE(done, 0, 1);
static void event_handler(const nrfx_spim_evt_t *p_event, void *p_context)
{
	if (p_event->type == NRFX_SPIM_EVENT_DONE) {
		k_sem_give(&done);
	}
}
#else
#error
#endif

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	// NRF_TAD_S->CLOCKSTOP = 1;
	// NRF_TAD_S->ENABLE = 0;
	// NRF_TAD_S->PSEL.TRACECLK   = 0xFFFFFFFF;
	// NRF_TAD_S->PSEL.TRACEDATA0 = 0xFFFFFFFF;
	// NRF_TAD_S->PSEL.TRACEDATA1 = 0xFFFFFFFF;
	// NRF_TAD_S->PSEL.TRACEDATA2 = 0xFFFFFFFF;
	// NRF_TAD_S->PSEL.TRACEDATA3 = 0xFFFFFFFF;

	// NRF_TAD_S->PSEL.TRACECLK   = 12;
	// NRF_TAD_S->PSEL.TRACEDATA0 = 11;
	// NRF_TAD_S->PSEL.TRACEDATA1 = 10;
	// NRF_TAD_S->PSEL.TRACEDATA2 = 9;
	// NRF_TAD_S->PSEL.TRACEDATA3 = 8;
	// nrf_gpio_pin_mcu_select(8,  NRF_GPIO_PIN_MCUSEL_TND);
	// nrf_gpio_pin_mcu_select(9,  NRF_GPIO_PIN_MCUSEL_TND);
	// nrf_gpio_pin_mcu_select(10, NRF_GPIO_PIN_MCUSEL_TND);
	// nrf_gpio_pin_mcu_select(11, NRF_GPIO_PIN_MCUSEL_TND);
	// nrf_gpio_pin_mcu_select(12, NRF_GPIO_PIN_MCUSEL_TND);
	// NRF_TAD_S->ENABLE = 1;
	// NRF_TAD_S->CLOCKSTART = 1;

	// nrf_gpio_pin_mcu_select(8,  NRF_GPIO_PIN_MCUSEL_APP);
	// nrf_gpio_pin_mcu_select(9,  NRF_GPIO_PIN_MCUSEL_APP);
	// nrf_gpio_pin_mcu_select(10, NRF_GPIO_PIN_MCUSEL_APP);

	uint8_t address[3] = { 0xAA, 0xBB, 0xCC };
	uint8_t rx_buffer[sizeof(address)];
	uint8_t rx_buffer2[sizeof(address)];

#if IS_ENABLED(CONFIG_SPI)
	const struct spi_buf tx_buf = {
		.buf = address,
		.len = sizeof(address)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
	if (!device_is_ready(spi_dev)) {
		printk("%s is not ready!\n", spi_dev->name);
	}

	// NRF_SPIM4->ENABLE = SPIM_ENABLE_ENABLE_Enabled;
	spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
	rx_buf.buf = rx_buffer2;
	spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
#else
	nrfx_err_t err;
	nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(
		DT_PROP(SPI_NODE, sck_pin),
		DT_PROP(SPI_NODE, mosi_pin),
		DT_PROP(SPI_NODE, miso_pin),
		NRF_DT_GPIOS_TO_PSEL(SPI_NODE, cs_gpios));
	config.frequency = NRF_SPIM_FREQ_2M;
	config.mode = NRF_SPIM_MODE_0;

	IRQ_CONNECT(DT_IRQN(SPI_NODE), DT_IRQ(SPI_NODE, priority),
		    nrfx_spim_4_irq_handler, 0, 0);

	err = nrfx_spim_init(&spim, &config, event_handler, NULL);
	if (err != NRFX_SUCCESS) {
		printk("Failed to initialize SPIM\n");
		return;
	}

	nrfx_spim_xfer_desc_t xfer_desc = {
		.p_tx_buffer = address,
		.tx_length = sizeof(address),
		.p_rx_buffer = rx_buffer,
		.rx_length = sizeof(rx_buffer),
	};
	err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
	if (err != NRFX_SUCCESS) {
		printk("Transfer 1 failed\n");
		return;
	}
	k_sem_take(&done, K_FOREVER);

	xfer_desc.p_rx_buffer = rx_buffer2;
	err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
	if (err != NRFX_SUCCESS) {
		printk("Transfer 2 failed\n");
		return;
	}
	k_sem_take(&done, K_FOREVER);
#endif

	for (int i = 0; i < sizeof(rx_buffer); ++i) {
		printk("%02X %02X ", rx_buffer[i], rx_buffer2[i]);
	}
	printk("\n");
}
