/*
 * Copyright (c) 2024 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/cache.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_hsfll.h>
#include <hal/nrf_lrcconf.h>
#include <soc/nrfx_coredep.h>
LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#if defined(NRF_APPLICATION)
#define HSFLL_NODE DT_NODELABEL(cpuapp_hsfll)
#define FICR_TRIM_BLOCK NRF_FICR->TRIM.APPLICATION
#elif defined(NRF_RADIOCORE)
#define HSFLL_NODE DT_NODELABEL(cpurad_hsfll)
#define FICR_TRIM_BLOCK NRF_FICR->TRIM.RADIOCORE
#else
#error "Unsupported SoC"
#endif

static void power_domain_init(void)
{
#if defined(NRF_LRCCONF010)
	/*
	 * Set:
	 *  - LRCCONF010.POWERON.MAIN: 1
	 *  - LRCCONF010.POWERON.ACT: 1
	 *  - LRCCONF010.RETAIN.MAIN: 1
	 *  - LRCCONF010.RETAIN.ACT: 1
	 *
	 *  This is done here at boot so that when the idle routine will hit
	 *  WFI the power domain will be correctly retained.
	 */

	nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_MAIN, true);
	nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_0, true);

	nrf_lrcconf_retain_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_MAIN, true);
	nrf_lrcconf_retain_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_0, true);
#endif

#if defined(NRF_LRCCONF000) && defined(CONFIG_SOC_NRF54H20_ENG0_CPUAPP)
	nrf_lrcconf_poweron_force_set(NRF_LRCCONF000, NRF_LRCCONF_POWER_DOMAIN_0, true);
#endif
}

#if defined(CONFIG_SOC_TRIM_HSFLL)
static int trim_hsfll(void)
{
	NRF_HSFLL_Type *hsfll = (NRF_HSFLL_Type *)DT_REG_ADDR(HSFLL_NODE);
	const nrf_hsfll_trim_t trim = {
		.vsup = FICR_TRIM_BLOCK.HSFLL.TRIM.VSUP,
		.coarse = FICR_TRIM_BLOCK.HSFLL.TRIM.COARSE[CONFIG_FICR_HSFLL_TRIM_IDX],
		.fine = FICR_TRIM_BLOCK.HSFLL.TRIM.FINE[CONFIG_FICR_HSFLL_TRIM_IDX],
	};

	LOG_DBG("Trim: HSFLL VSUP: 0x%.8x", trim.vsup);
	LOG_DBG("Trim: HSFLL COARSE: 0x%.8x", trim.coarse);
	LOG_DBG("Trim: HSFLL FINE: 0x%.8x", trim.fine);

	if (CONFIG_HSFLL_CLOCK_MULTIPLIER) {
		nrf_hsfll_clkctrl_mult_set(hsfll, CONFIG_HSFLL_CLOCK_MULTIPLIER);
	}
	nrf_hsfll_trim_set(hsfll, &trim);

	nrf_hsfll_task_trigger(hsfll, NRF_HSFLL_TASK_FREQ_CHANGE);
#if defined(CONFIG_SOC_NRF54H20_ENG0_CPUAPP) || defined(CONFIG_SOC_NRF54H20_ENG0_CPURAD)
	/* In this HW revision, HSFLL task frequency change needs to be
	 * triggered additional time to take effect.
	 */
	nrf_hsfll_task_trigger(hsfll, NRF_HSFLL_TASK_FREQ_CHANGE);
#endif

	LOG_DBG("NRF_HSFLL->TRIM.VSUP = %d", hsfll->TRIM.VSUP);
	LOG_DBG("NRF_HSFLL->TRIM.COARSE = %d", hsfll->TRIM.COARSE);
	LOG_DBG("NRF_HSFLL->TRIM.FINE = %d", hsfll->TRIM.FINE);

	return 0;
}
#endif /* CONFIG_SOC_TRIM_HSFLL */

static int nordicsemi_nrf54h_init(void)
{
#if defined(CONFIG_NRF_ENABLE_ICACHE)
	sys_cache_instr_enable();
#endif

	power_domain_init();

#if defined(CONFIG_SOC_TRIM_HSFLL)
	trim_hsfll();
#endif

	return 0;
}

void arch_busy_wait(uint32_t time_us)
{
	nrfx_coredep_delay_us(time_us);
}

SYS_INIT(nordicsemi_nrf54h_init, PRE_KERNEL_1, 0);
