/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MSPI_MSPI_DW_VENDOR_SPECIFIC_H_
#define ZEPHYR_DRIVERS_MSPI_MSPI_DW_VENDOR_SPECIFIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_exmif)

#include <nrf.h>

#define	VENDOR_SPECIFIC_CLEAR_IRQ(dev) \
	NRF_EXMIF->EVENTS_CORE = 0

#define	VENDOR_SPECIFIC_INIT(dev) \
	NRF_EXMIF->INTENSET = BIT(EXMIF_INTENSET_CORE_Pos); \
	NRF_EXMIF->TASKS_START = 1

#endif /* DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_exmif) */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_MSPI_MSPI_DW_VENDOR_SPECIFIC_H_ */
