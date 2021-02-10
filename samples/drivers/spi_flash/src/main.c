/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/flash.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <string.h>

#if (CONFIG_SPI_NOR - 0) ||				\
	DT_NODE_HAS_STATUS(DT_INST(0, jedec_spi_nor), okay)
#define FLASH_DEVICE DT_LABEL(DT_INST(0, jedec_spi_nor))
#define FLASH_NAME "JEDEC SPI-NOR"
#elif (CONFIG_NORDIC_QSPI_NOR - 0) || \
	DT_NODE_HAS_STATUS(DT_INST(0, nordic_qspi_nor), okay)
#define FLASH_DEVICE DT_LABEL(DT_INST(0, nordic_qspi_nor))
#define FLASH_NAME "JEDEC QSPI-NOR"
#elif DT_NODE_HAS_STATUS(DT_INST(0, st_stm32_qspi_nor), okay)
#define FLASH_DEVICE DT_LABEL(DT_INST(0, st_stm32_qspi_nor))
#define FLASH_NAME "JEDEC QSPI-NOR"
#else
#error Unsupported flash driver
#endif

#if defined(CONFIG_BOARD_ADAFRUIT_FEATHER_STM32F405)
#define FLASH_TEST_REGION_OFFSET 0xf000
#elif defined(CONFIG_BOARD_ARTY_A7_ARM_DESIGNSTART_M1) || \
	defined(CONFIG_BOARD_ARTY_A7_ARM_DESIGNSTART_M3)
/* The FPGA bitstream is stored in the lower 536 sectors of the flash. */
#define FLASH_TEST_REGION_OFFSET \
	DT_REG_SIZE(DT_NODE_BY_FIXED_PARTITION_LABEL(fpga_bitstream))
#else
#define FLASH_TEST_REGION_OFFSET 0xf8000
#endif
#define FLASH_SECTOR_SIZE        4096
#define TEST_REGION_SIZE         (2*FLASH_SECTOR_SIZE)

void main(void)
{
	const struct device *flash_dev;
	int rc;

	printf("\n" FLASH_NAME " SPI flash testing\n");
	printf("==========================\n");

	flash_dev = device_get_binding(FLASH_DEVICE);

	if (!flash_dev) {
		printf("SPI flash driver %s was not found!\n",
		       FLASH_DEVICE);
		return;
	}

	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printf("\nTest 1: Flash erase\n");
	flash_write_protection_set(flash_dev, false);

	rc = flash_erase(flash_dev, FLASH_TEST_REGION_OFFSET,
			 TEST_REGION_SIZE);
	if (rc != 0) {
		printf("Flash erase failed! %d\n", rc);
	} else {
		printf("Flash erase succeeded!\n");
	}

	printf("\nTest 2: Flash write\n");
	flash_write_protection_set(flash_dev, false);

#if 0
	const uint8_t expected[] = { 0x55, 0xaa, 0x66, 0x99 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];

	printf("Attempting to write %u bytes\n", len);
	rc = flash_write(flash_dev, FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printf("Flash write failed! %d\n", rc);
		return;
	}

	memset(buf, 0, len);
	rc = flash_read(flash_dev, FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printf("Flash read failed! %d\n", rc);
		return;
	}

	if (memcmp(expected, buf, len) == 0) {
		printf("Data read matches data written. Good!!\n");
	} else {
		const uint8_t *wp = expected;
		const uint8_t *rp = buf;
		const uint8_t *rpe = rp + len;

		printf("Data read does not match data written!!\n");
		while (rp < rpe) {
			printf("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
	}
#else

#define BLOCK_LEN 512
#define BLOCK_CNT (TEST_REGION_SIZE / BLOCK_LEN)

	for (int i = 0; i < BLOCK_CNT; ++i) {
		static uint8_t buffer[BLOCK_LEN];
		for (int n = 0; n < BLOCK_LEN; ++n) {
			buffer[n] = i + n;
		}

		printf("Writing block %u (%u bytes)\n", i, BLOCK_LEN);
		rc = flash_write(flash_dev, 0x8000 + i*BLOCK_LEN,
				 buffer, BLOCK_LEN);
		if (rc != 0) {
			printf("Flash write failed! %d\n", rc);
			return;
		}
	}

	for (int i = 0; i < BLOCK_CNT; ++i) {
		static uint8_t expected[BLOCK_LEN];
		static uint8_t actual[BLOCK_LEN];
		for (int n = 0; n < BLOCK_LEN; ++n) {
			expected[n] = i + n;
		}

		printf("Reading block %u (%u bytes)\n", i, BLOCK_LEN);
		rc = flash_read(flash_dev, 0x8000 + i*BLOCK_LEN,
				actual, BLOCK_LEN);
		if (rc != 0) {
			printf("Flash read failed! %d\n", rc);
			return;
		}

		if (memcmp(expected, actual, BLOCK_LEN) != 0) {
			printf("Data read does not match data written!\n");
			return;
		}
	}
#endif
}
