/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "power_states.h"
#include "qm_common.h"
#include "qm_comparator.h"
#include "qm_interrupt.h"
#include "qm_pinmux.h"
#include "qm_isr.h"
#include "qm_rtc.h"
#include "qm_gpio.h"
#include "vreg.h"
#include "clk.h"

/* POWER SOC app example
 *
 * This app requires a comparator to wake the board from deep sleep.
 * RTC will be explicitly disabled in deep sleep.
 * RTC is not required but this is done to showcase comparator
 * wake up when in deep sleep mode.
 *
 * On the Quark SE development platform this pin is found on header J13 PIN 20.
 *
 * Wake up from SoC sleep and SoC deep sleep states result
 * in a processor core reset.
 * In order to retain the previous state, this application
 * makes use of General Purpose Register 2.
 *
 * States executed in this example are:
 * SLEEP: Core voltage rail off, peripherals off except Always On.
 * 	  Wake up from reset vector.
 * DEEP_SLEEP: Core voltage rail off,
 *             1P8V voltage regulator in linear mode,
 *             3P3V voltage regulator in linear mode,
 *             peripherals off except Always On.
 * 	       Wake up from reset vector.
 * ADVANCED SLEEP: Core voltage rail off.
 *               1P8V voltage regulator in shutdown mode,
 *               3P3V voltage regulator in linear mode,
 *               peripherals off except Always On.
 * 	         Wake up from reset vector.
 */

#define WAKEUP_COMPARATOR_PIN (13)
#define QM_AC_COMPARATORS_MASK (0x7FFFF)

#define PIN_CMP_READY (4)

#define PIN_OUT (0)
#define PIN_INTR (3)

#define QM_SCSS_GP_SOC_STATE_SLEEP BIT(0)
#define QM_SCSS_GP_SOC_STATE_DEEP_SLEEP BIT(1)
#define QM_SCSS_GP_SOC_STATE_ADVANCED_SLEEP BIT(2)

#define RTC_SYNC_CLK_COUNT (5)

static void aon_gpio_callback(void *, uint32_t);

static void setup_aon_comparator(void)
{
	qm_ac_config_t ac_cfg;

	QM_PUTS("Setting up Comparator.");

	/* Set up pin muxing and request IRQ*/
	qm_pmux_select(QM_PIN_ID_13, QM_PMUX_FN_1);
	qm_pmux_input_en(QM_PIN_ID_13, true);

	/* Clear all comparator pending interrupts */
	QM_SCSS_CMP->cmp_pwr = 0;
	QM_SCSS_CMP->cmp_stat_clr = QM_AC_COMPARATORS_MASK;

	ac_cfg.reference =
	    BIT(WAKEUP_COMPARATOR_PIN); /* Ref internal voltage */
	ac_cfg.polarity = 0x0; /* Fire if greater than ref (high level) */
	ac_cfg.power = BIT(WAKEUP_COMPARATOR_PIN);  /* Normal operation mode */
	ac_cfg.int_en = BIT(WAKEUP_COMPARATOR_PIN); /* Enable comparator */
	qm_ac_set_config(&ac_cfg);

	qm_irq_request(QM_IRQ_AC, qm_ac_isr);
}

static void wait_for_comparator_to_ground(void)
{
	volatile bool comparator_ready = false;
	qm_gpio_port_config_t cfg;

	/* Request IRQ and write GPIO port config */
	cfg.direction = 0;			/* All pins are input */
	cfg.int_en = BIT(PIN_CMP_READY);	/* Interrupt enabled */
	cfg.int_type = BIT(PIN_CMP_READY);      /* Edge sensitive interrupt */
	cfg.int_polarity = ~BIT(PIN_CMP_READY); /* Falling edge */
	cfg.int_debounce = BIT(PIN_CMP_READY);  /* Debounce enabled */
	cfg.int_bothedge = 0x0;			/* Both edge disabled */
	cfg.callback = aon_gpio_callback;
	cfg.callback_data = (void *)&comparator_ready;

	qm_irq_request(QM_IRQ_AONGPIO_0, qm_aon_gpio_isr_0);

	qm_gpio_set_config(QM_AON_GPIO_0, &cfg);

	QM_PUTS("Set comparator pin to Ground and press PB0 when ready.");

	while (comparator_ready == false) {
	}
}

int main(void)
{

	qm_rtc_config_t rtc_cfg;
	bool start = false;
	bool sleep_wakeup = false;
	bool deep_sleep_wakeup = false;
	uint32_t aonc_start;

	if (QM_SCSS_GP->gps2 & QM_SCSS_GP_SOC_STATE_SLEEP) {
		sleep_wakeup = true;
		QM_PUTS("SoC states example back from sleep.");
	} else if (QM_SCSS_GP->gps2 & QM_SCSS_GP_SOC_STATE_DEEP_SLEEP) {
		deep_sleep_wakeup = true;
		QM_PUTS("SoC states example back from deep sleep.");
	} else if (QM_SCSS_GP->gps2 & QM_SCSS_GP_SOC_STATE_ADVANCED_SLEEP) {
		QM_PUTS("SoC states example back from advanced sleep.");
	} else {
		start = true;
		QM_PUTS("Starting: Power SoC example.");
	}

	/* Reset state in GPS2. */
	QM_SCSS_GP->gps2 = 0;

	if (start) {
		/*  Initialise RTC configuration. */
		rtc_cfg.init_val = 0;
		rtc_cfg.alarm_en = true;
		rtc_cfg.alarm_val = QM_RTC_ALARM_SECOND << 2;
		rtc_cfg.callback = NULL;
		rtc_cfg.callback_data = NULL;
		qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

		qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

		QM_PUTS("Go to sleep.");

		/* Save the state in GPS2 for host wake up information */
		QM_SCSS_GP->gps2 |= QM_SCSS_GP_SOC_STATE_SLEEP;

		/*
		 * The RTC clock resides in a different clock domain
		 * to the system clock.
		 * It takes 3-4 RTC ticks for a system clock write to propagate
		 * to the RTC domain.
		 * If an entry to sleep is initiated without waiting for the
		 * transaction to complete the SOC will not wake from sleep.
		 */
		aonc_start = QM_SCSS_AON[0].aonc_cnt;
		while (QM_SCSS_AON[0].aonc_cnt - aonc_start <
		       RTC_SYNC_CLK_COUNT) {
		}

		/* Go to sleep, RTC will wake me up. */
		power_soc_sleep();
		/* Not reachable. We will wake up from reset. */
		QM_PUTS("Error: reached unreachable code.");
	} else if (sleep_wakeup) {
		wait_for_comparator_to_ground();

		setup_aon_comparator();

		QM_PUTS("Go to deep sleep. Trigger comparator to wake up.");
		/* Save the state in GPS2 for host signal. */
		QM_SCSS_GP->gps2 = QM_SCSS_GP_SOC_STATE_DEEP_SLEEP;
		/* Disable RTC during sleep to be only woken up by external
		 * interrupt. */
		QM_SCSS_PMU->slp_cfg |= QM_SCSS_SLP_CFG_RTC_DIS;
		/* Go to deep sleep, Comparator will wake me up. */
		power_soc_deep_sleep();
		/* Not reachable. We will wake out from reset. */
		QM_PUTS("Error: reached unreachable code.");
	} else if (deep_sleep_wakeup) {
		wait_for_comparator_to_ground();

		setup_aon_comparator();

		QM_PUTS("Go to deep sleep. Trigger comparator to wake up.");
		/* Save the state in GPS2 for host signal. */
		QM_SCSS_GP->gps2 = QM_SCSS_GP_SOC_STATE_ADVANCED_SLEEP;
		/* Disable RTC during sleep to be only woken up by external
		 * interrupt. */
		QM_SCSS_PMU->slp_cfg |= QM_SCSS_SLP_CFG_RTC_DIS;

		/* Configure regulators.
		 *
		 * This configuration enables lower power consumption
		 * than the deep sleep function.
		 *
		 * This takes the assumption that the 1P8 voltage
		 * regulator can be safely turned off.
		 * This may not be true for your board.
		 */
		vreg_plat1p8_set_mode(VREG_MODE_SHUTDOWN);
		vreg_plat3p3_set_mode(VREG_MODE_LINEAR);

		/* Enable low power sleep mode. */
		QM_SCSS_PMU->slp_cfg |= QM_SCSS_SLP_CFG_LPMODE_EN;
		QM_SCSS_PMU->pm1c |= QM_SCSS_PM1C_SLPEN;

		/* Not reachable. We will wake out from reset. */
		QM_PUTS("Error: reached unreachable code.");
	}

	QM_PUTS("Finished: Power SoC example.");

	return 0;
}

void aon_gpio_callback(void *data, uint32_t status)
{
	volatile bool *comparator_ready = (volatile bool *)data;
	*comparator_ready = true;
}
