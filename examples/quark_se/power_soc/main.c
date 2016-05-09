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
#include "qm_gpio.h"
#include "qm_pinmux.h"
#include "qm_isr.h"

/* POWER SOC app example
 *
 * This application must run in conjonction with its Sensor Subsystem
 * counterpart.
 *
 * This app requires a board to be set up with a jumper cable connecting the
 * pins listed below so the output pin can trigger an interrupt on the
 * input pin. PIN_OUT will be configured as an output pin and PIN_INTR will be
 * configured as an input pin with interrupts enabled.
 *
 * On Atlas Hills, PIN_OUT and PIN_INTR are located on header P4 PIN 42 and 40.
 *
 * It also requires a comparator to wake the board from deep sleep.
 * RTC will be explicitly disabled in this state.
 * RTC is not mandatory disabled but this is done to demonstrate comparator
 * wake up when in sleep or deep sleep mode.
 *
 * On Atlas Hills, this pin is located on header P3 PIN 20.
 *
 * This application running on the Sensor Subsystem will trigger the
 * transitions with PIN_INTR on the host core.
 * Sensor collaboration is needed as LPSS can only be achieved
 * as a combination of the two cores states.
 *
 * Wake up from SoC sleep and SoC deep sleep states result
 * in a processor core reset.
 * In order to retain the previous state, this application
 * makes use of General Purpose Register 2 for Host,
 * General Purpose Register 3 for Sensor Subsystem.
 *
 * States executed in this example are:
 * C2: Processor clock gated, gateway to LPSS state
 * C2LP: Processor complex clock gated
 * LPSS: Combination of C2/C2LP and SS2 (Sensor state)
 * SLEEP: Core voltage rail off, peripherals off except Always On.
 * 	      Wake up from reset vector.
 * DEEP_SLEEP: Core voltage rail off,
 *             1P8V voltage regulator in linear mode,
 *             3P3V voltage regulator in linear mode,
 *             peripherals off except Always On.
 * 	           Wake up from reset vector.
 */
#define WAKEUP_COMPARATOR_PIN (13)

#define PIN_OUT (0)
#define PIN_INTR (3)

static void gpio_example_callback(void *, uint32_t);

#define QM_SCSS_GP_SOC_STATE_SLEEP BIT(0)
#define QM_SCSS_GP_SOC_STATE_DEEP_SLEEP BIT(1)
#define QM_SCSS_GP_CORE_STATE_C2 BIT(2)
#define QM_SCSS_GP_CORE_STATE_C2LP BIT(3)

int main(void)
{
	qm_gpio_port_config_t cfg;
	qm_ac_config_t ac_cfg;
	bool sleep_wakeup = false;
	bool deep_sleep_wakeup = false;

	if (QM_SCSS_GP->gps2 & QM_SCSS_GP_SOC_STATE_SLEEP) {
		sleep_wakeup = true;
		QM_SCSS_GP->gps2 &= ~QM_SCSS_GP_SOC_STATE_SLEEP;
		QM_PUTS("SoC states example back from sleep.");
	}

	if (QM_SCSS_GP->gps2 & QM_SCSS_GP_SOC_STATE_DEEP_SLEEP) {
		deep_sleep_wakeup = true;
		QM_SCSS_GP->gps2 &= ~QM_SCSS_GP_SOC_STATE_DEEP_SLEEP;
		QM_PUTS("SoC states example back from deep sleep.");
	}

	if (!sleep_wakeup && !deep_sleep_wakeup) {
		QM_PUTS("Starting: Power SoC example.");
	}

	/*
	 * Demonstrating entering and exiting LPSS mode.
	 * The Sensor Subsystem will wake from LPSS into SS0.
	 * As the Host is in C2 at that moment, the sensor subsystem
	 * will trigger an interrupt into the Host via GPIO.
	 *
	 * Request IRQ and write GPIO port config
	 */
	cfg.direction = BIT(PIN_OUT);
	cfg.int_en = BIT(PIN_INTR);       /* Interrupt enabled */
	cfg.int_type = BIT(PIN_INTR);     /* Edge sensitive interrupt */
	cfg.int_polarity = BIT(PIN_INTR); /* Rising edge */
	cfg.int_debounce = BIT(PIN_INTR); /* Debounce enabled */
	cfg.int_bothedge = 0x0;		  /* Both edge disabled */
	cfg.callback = gpio_example_callback;
	cfg.callback_data = NULL;

	qm_irq_request(QM_IRQ_GPIO_0, qm_gpio_isr_0);

	qm_gpio_set_config(QM_GPIO_0, &cfg);

	QM_PUTS("Go to c2. Sensor in SS2. Enable LPSS");
	/* Enable LPSS */
	power_soc_lpss_enable();
	/* Tell sensor we are ready for LPSS mode */
	QM_SCSS_GP->gps2 |= QM_SCSS_GP_CORE_STATE_C2;
	/* Go to LPSS. Sensor will make me transition to C2 and wake me up */
	power_cpu_c2();
	/* Clear flag */
	QM_SCSS_GP->gps2 &= ~QM_SCSS_GP_CORE_STATE_C2;
	QM_PUTS("Wake up from LPSS.");

	QM_PUTS("Go to c2lp. Sensor in SS2. Enable LPSS");
	/* Enable LPSS */
	power_soc_lpss_enable();
	/* Tell sensor we are ready for LPSS mode */
	QM_SCSS_GP->gps2 |= QM_SCSS_GP_CORE_STATE_C2LP;
	/* Go to C2LP. Sensor will make me transition to LPSS and wake me up */
	power_cpu_c2lp();
	/* Clear flag */
	QM_SCSS_GP->gps2 &= ~QM_SCSS_GP_CORE_STATE_C2LP;
	QM_PUTS("Wake up from LPSS.");

	if (!sleep_wakeup && !deep_sleep_wakeup) {
		QM_PUTS("Go to sleep.");
		/* Save the state in GPS2 for host wake up information */
		QM_SCSS_GP->gps2 |= QM_SCSS_GP_SOC_STATE_SLEEP;
		/* Save the state in GPS3 for sensor wake up information */
		QM_SCSS_GP->gps3 |= QM_SCSS_GP_SOC_STATE_SLEEP;
		/* Go to sleep, RTC will wake me up. */
		power_soc_sleep();
		/* Not reachable. We will wake up from reset */
		QM_PUTS("Error: reached unreachable code.");
	} else if (sleep_wakeup && !deep_sleep_wakeup) {
		QM_PUTS("Setting up Comparator.");
		ac_cfg.reference =
		    BIT(WAKEUP_COMPARATOR_PIN); /* Ref internal voltage */
		ac_cfg.polarity =
		    0x0; /* Fire if greater than ref (high level) */
		ac_cfg.power =
		    BIT(WAKEUP_COMPARATOR_PIN); /* Normal operation mode */
		ac_cfg.int_en =
		    BIT(WAKEUP_COMPARATOR_PIN); /* Enable comparator */
		qm_ac_set_config(&ac_cfg);

		qm_irq_request(QM_IRQ_AC, qm_ac_isr);

		/* Set up pin muxing and request IRQ*/
		qm_pmux_select(QM_PIN_ID_13, QM_PMUX_FN_1);
		qm_pmux_input_en(QM_PIN_ID_13, true);

		QM_PUTS("Go to deep sleep.");
		/* Save the state in GPS2 for host signal*/
		QM_SCSS_GP->gps2 |= QM_SCSS_GP_SOC_STATE_DEEP_SLEEP;
		/* Save the state in GPS3 for sensor signal*/
		QM_SCSS_GP->gps3 |= QM_SCSS_GP_SOC_STATE_DEEP_SLEEP;
		/* Disable RTC during sleep to be only woken up by external
		 * interrupt */
		QM_SCSS_PMU->slp_cfg |= QM_SCSS_SLP_CFG_RTC_DIS;
		/* Go to deep sleep, Comparator will wake me up. */
		power_soc_deep_sleep();
		/* Not reachable. We will wake out from reset */
		QM_PUTS("Error: reached unreachable code.");
	}

	QM_PUTS("Finished: Power SoC example.");

	return 0;
}

void gpio_example_callback(void *data, uint32_t status)
{
	QM_PUTS("GPIO interrupt triggered");
}
