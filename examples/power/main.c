/*
 * Copyright (c) 2015, Intel Corporation
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

#include "qm_common.h"
#include "qm_power.h"
#include "qm_rtc.h"
#include "qm_adc.h"
#include "qm_comparator.h"
#include "qm_interrupt.h"
#include "qm_pinmux.h"

/* On the Quark Microcontroller D2000 Development Platform this pin is marked
 * as "A5"
 */
#define WAKEUP_COMPARATOR_PIN (6)

static void rtc_example_callback();
static void ac_example_callback();

/* Low power app example */
int main(void)
{
	/* Setup the RTC to get out of sleep mode. deep sleep will require an */
	/* analog comparator interrupt to wake up the system. */
	/*  Variables */
	uint32_t pmux_sel_save[2], pmux_in_en_save, pmux_pullup_save;
	qm_ac_config_t ac_cfg;
	qm_rtc_config_t rtc_cfg;

	QM_PUTS("Low power mode example.");

	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	/*  Initialise RTC configuration. */
	rtc_cfg.init_val = 0;
	rtc_cfg.alarm_en = 1;
	rtc_cfg.alarm_val = QM_RTC_ALARM_SECOND;
	rtc_cfg.callback = rtc_example_callback;
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

	QM_PUTS("CPU Halt.");
	/* Halt the CPU, RTC alarm will wake me up. */
	cpu_halt();
	QM_PUTS("CPU Halt wakeup.");

	/* Set another alarm one minute from now. */
	qm_rtc_set_alarm(QM_RTC_0,
			 QM_RTC[QM_RTC_0].rtc_ccvr + QM_RTC_ALARM_SECOND);
	QM_PUTS("Go to sleep.");
	/* Go to sleep, RTC will wake me up. */
	soc_sleep();
	QM_PUTS("Wake up from sleep.");

	/* Physical step at this stage to raise the V on the comparator pin. */
	/* Go to deep sleep, a comparator should wake me up. */
	QM_PUTS("Go to deep sleep.");

	ac_cfg.reference =
	    BIT(WAKEUP_COMPARATOR_PIN); /* Ref internal voltage */
	ac_cfg.polarity = 0x0; /* Fire if greater than ref (high level) */
	ac_cfg.power = BIT(WAKEUP_COMPARATOR_PIN);  /* Normal operation mode */
	ac_cfg.int_en = BIT(WAKEUP_COMPARATOR_PIN); /* Enable comparator */
	ac_cfg.callback = ac_example_callback;
	qm_ac_set_config(&ac_cfg);

	qm_irq_request(QM_IRQ_AC, qm_ac_isr);

	/*
	 * Comparator pin will fire an interrupt when the input voltage is
	 * greater than the reference voltage (0.95V).
	 */

	/*
	 * In order to minimise power, pmux_sel must be set to 0, input enable
	 * must be cleared for any pins not expecting to be used to wake the
	 * SoC from deep sleep mode, in this example we are using AC 6.
	 */
	pmux_sel_save[0] = QM_SCSS_PMUX->pmux_sel[0];
	pmux_sel_save[1] = QM_SCSS_PMUX->pmux_sel[1];
	pmux_in_en_save = QM_SCSS_PMUX->pmux_in_en[0];

	pmux_pullup_save = QM_SCSS_PMUX->pmux_pullup[0];

	QM_SCSS_PMUX->pmux_sel[0] = QM_SCSS_PMUX->pmux_sel[1] = 0;
	QM_SCSS_PMUX->pmux_in_en[0] = BIT(WAKEUP_COMPARATOR_PIN);
	QM_SCSS_PMUX->pmux_pullup[0] = 0;

	/* Mux out comparator */
	qm_pmux_select(QM_PIN_ID_6, QM_PMUX_FN_1);
	qm_pmux_input_en(QM_PIN_ID_6, true);

	soc_deep_sleep();

	/* Restore previous pinmuxing settings. */
	QM_SCSS_PMUX->pmux_sel[0] = pmux_sel_save[0];
	QM_SCSS_PMUX->pmux_sel[1] = pmux_sel_save[1];
	QM_SCSS_PMUX->pmux_in_en[0] = pmux_in_en_save;
	QM_SCSS_PMUX->pmux_pullup[0] = pmux_pullup_save;

	QM_PUTS("Wake up from deep sleep.");
	return 0;
}

void rtc_example_callback()
{
}

void ac_example_callback()
{
	/* The analog comparators use level triggered interrupts so we will get
	 * a constant stream of interrupts if we do not mask them. Comment the
	 * following line if you want to get more than one interrupt. */
	QM_SCSS_INT->int_comparators_host_mask |= BIT(WAKEUP_COMPARATOR_PIN);
}
