/*
 * Copyright (c) 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr.h>
#include <ztest.h>

#include <spi.h>
#include <gpio.h>

#define SPI_MASTER_DRV_NAME	CONFIG_SPI_TEST_MASTER_DRV_NAME
#define SPI_SLAVE		CONFIG_SPI_TEST_SLAVE_NUMBER
#define SLOW_FREQ		CONFIG_SPI_TEST_SLOW_FREQ
#define FAST_FREQ		CONFIG_SPI_TEST_FAST_FREQ

#define SPI_SLAVE_DRV_NAME	CONFIG_SPI_TEST_SLAVE_DRV_NAME

#if defined(CONFIG_SPI_TEST_CS_GPIO)
#define CS_CTRL_GPIO_DRV_NAME CONFIG_SPI_TEST_CS_CTRL_GPIO_DRV_NAME
struct spi_cs_control spi_cs = {
	.gpio_pin = CONFIG_SPI_TEST_CS_CTRL_GPIO_PIN,
	.delay = 0,
};
#define SPI_CS (&spi_cs)
#else
#define SPI_CS NULL
#define CS_CTRL_GPIO_DRV_NAME ""
#endif

#define BUF_SIZE 17
static u8_t m_master_tx[] = "0123456789abcdef\0";
static u8_t m_slave_tx[]  = "ABCDEFGHIJKLMNOP\0";
static u8_t m_master_rx[BUF_SIZE];
static u8_t m_slave_rx[BUF_SIZE];

static const struct spi_config m_spi_cfg_master_slow = {
	.frequency = SLOW_FREQ,
	.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA |
		     SPI_WORD_SET(8) | SPI_LINES_SINGLE,
	.slave = SPI_SLAVE,
	.cs = SPI_CS,
};

static const struct spi_config m_spi_cfg_master_fast = {
	.frequency = FAST_FREQ,
	.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA |
		     SPI_WORD_SET(8) | SPI_LINES_SINGLE,
	.slave = SPI_SLAVE,
	.cs = SPI_CS,
};

static const struct spi_config m_spi_cfg_slave = {
	.operation = SPI_OP_MODE_SLAVE | SPI_MODE_CPOL | SPI_MODE_CPHA |
		     SPI_WORD_SET(8) | SPI_LINES_SINGLE,
	.cs = NULL,
};

#if defined(CONFIG_SPI_TEST_CS_GPIO)
static int cs_ctrl_gpio_config(void)
{
	spi_cs.gpio_dev = device_get_binding(CS_CTRL_GPIO_DRV_NAME);
	if (!spi_cs.gpio_dev) {
		LOG_ERR("Cannot find %s!\n",
			CS_CTRL_GPIO_DRV_NAME);
		return -1;
	}
	gpio_pin_configure(spi_cs.gpio_dev,
			   spi_cs.gpio_pin,
			   GPIO_DIR_OUT);
	gpio_pin_write(spi_cs.gpio_dev,
		       spi_cs.gpio_pin,
		       1);

	return 0;
}
#endif /* CONFIG_SPI_TEST_CS_GPIO */

static struct k_poll_signal m_async_sig =
	K_POLL_SIGNAL_INITIALIZER(m_async_sig);
static struct k_poll_event m_async_evt =
	K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
				 K_POLL_MODE_NOTIFY_ONLY,
				 &m_async_sig);

static int prepare_slave(struct device *spi_slave)
{
	const struct spi_buf tx_bufs[] = {
		{
			.buf = m_slave_tx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = m_slave_rx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs)
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};
	int ret;

	ret = spi_transceive_async(spi_slave, &m_spi_cfg_slave,
				   &tx, &rx, &m_async_sig);
	if (ret == -ENOTSUP) {
		LOG_DBG("Not supported");
		return 0;
	}
	if (ret) {
		LOG_ERR("Code %d", ret);
		return ret;
	}

	return 0;
}

static int spi_basic_test(struct device *spi_master,
			  const struct spi_config *spi_cfg_master)
{
	const struct spi_buf tx_bufs[] = {
		{
			.buf = m_master_tx,
			.len = 7, //BUF_SIZE,
		},
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = m_master_rx,
			.len = 11, //BUF_SIZE,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = 1
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = 1
	};
	int ret;

	LOG_INF("Start");

	ret = spi_transceive(spi_master, spi_cfg_master, &tx, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		return ret;
	}

	k_poll(&m_async_evt, 1, K_FOREVER);
	LOG_INF("Async call status: %d", m_async_evt.signal->result);

	if (memcmp(m_master_tx, m_slave_rx, tx.buffers[0].len)) {
		LOG_ERR("m_master_tx != m_slave_rx");
		return -1;
	}

	if (memcmp(m_slave_tx, m_master_rx, rx.buffers[0].len)) {
		LOG_ERR("m_slave_tx != m_master_rx");
		return -1;
	}

	LOG_INF("Passed");

	return 0;
}

void testing_spi(void)
{
	struct device *spi_master_slow;
	struct device *spi_master_fast;
	struct device *spi_slave;

#if defined(CONFIG_SPI_TEST_CS_GPIO)
	if (cs_ctrl_gpio_config()) {
		return;
	}
#endif

	spi_slave = device_get_binding(SPI_SLAVE_DRV_NAME);
	if (!spi_slave) {
		LOG_ERR("Cannot find %s!\n", SPI_SLAVE_DRV_NAME);
		return;
	}

	spi_master_slow = device_get_binding(SPI_MASTER_DRV_NAME);
	if (!spi_master_slow) {
		LOG_ERR("Cannot find %s!\n", SPI_MASTER_DRV_NAME);
		return;
	}

	spi_master_fast = spi_master_slow;

	if (prepare_slave(spi_slave) ||
	    spi_basic_test(spi_master_slow, &m_spi_cfg_master_slow)) {
		return;
	}

	if (prepare_slave(spi_slave) ||
	    spi_basic_test(spi_master_fast, &m_spi_cfg_master_fast)) {
		return;
	}

	LOG_INF("All tx/rx passed");
}

/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(test_spi, ztest_unit_test(testing_spi));
	ztest_run_test_suite(test_spi);
}
