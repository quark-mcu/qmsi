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

#include "qm_soc_regs.h"
#include "ss_power_states.h"
#include "qm_ss_timer.h"
#include "qm_ss_interrupt.h"
#include "qm_interrupt.h"
#include "qm_uart.h"
#include "qm_isr.h"
#include "qm_ss_isr.h"
#include "qm_rtc.h"

/* SS POWER CORE app example
 *
 * This example demonstrates the Sensor Subsystem Core States
 *
 * States executed in this example are:
 * SS1: Processor clock gated
 * SS2: Processor clock gated, sensor timers off
 */
#define TWO_SEC_AT_32MHZ (0x04000000)

static void timer0_expired(void *data)
{
	QM_PUTS("Timer0 expired");
}

static void rtc_example_callback()
{
	QM_PUTS("RTC expired");
}

int main(void)
{
	qm_ss_timer_config_t ss_timer_cfg;
	qm_rtc_config_t rtc_cfg;

	QM_PUTS("Starting: SS Power Core example");

	/*  Initialise Timer configuration. */
	qm_ss_int_vector_request(QM_SS_INT_TIMER_0, qm_ss_timer_isr_0);
	qm_ss_irq_unmask(QM_SS_INT_TIMER_0);

	ss_timer_cfg.watchdog_mode = false;
	ss_timer_cfg.inc_run_only = false;
	ss_timer_cfg.int_en = true;
	ss_timer_cfg.count = TWO_SEC_AT_32MHZ;
	ss_timer_cfg.callback = timer0_expired;
	ss_timer_cfg.callback_data = NULL;

	if (qm_ss_timer_set_config(QM_SS_TIMER_0, &ss_timer_cfg) != 0) {
		QM_PUTS("Error: Sensor Timer Set Config for TIMER0 failed");
		return -1;
	}

	if (qm_ss_timer_set(QM_SS_TIMER_0, 0x00000000) != 0) {
		QM_PUTS("Error: Sensor Timer Set Count for TIMER0 failed");
		return -1;
	}

	QM_PUTS("Go to ss1. Core Off Timers On RTC On");
	/* Go to SS1, Timer will wake me up. */
	ss_power_cpu_ss1(SS_POWER_CPU_SS1_TIMER_ON);
	QM_PUTS("Wake up from ss1.");

	/* Disable timer as it won't wake us up from all following states */
	ss_timer_cfg.int_en = false;
	if (qm_ss_timer_set_config(QM_SS_TIMER_0, &ss_timer_cfg) != 0) {
		QM_PUTS("Error: Sensor Timer Set Config for TIMER0 failed");
		return -1;
	}

	/*  Initialise RTC configuration. */
	rtc_cfg.init_val = 0;
	rtc_cfg.alarm_en = 1;
	rtc_cfg.alarm_val = QM_RTC_ALARM_SECOND << 2;
	rtc_cfg.callback = rtc_example_callback;
	rtc_cfg.callback_data = NULL;
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

	QM_PUTS("Go to ss1. Core Off Timers Off RTC On");
	/* Go to SS1, Timer will wake me up. */
	ss_power_cpu_ss1(SS_POWER_CPU_SS1_TIMER_OFF);
	QM_PUTS("Wake up from ss1.");

	/* Set another alarm 4 seconds from now. */
	qm_rtc_set_alarm(QM_RTC_0, QM_RTC[QM_RTC_0].rtc_ccvr +
				       (QM_RTC_ALARM_SECOND << 2));

	QM_PUTS("Go to ss2.");
	/* Go to SS2, Timer will wake me up. */
	ss_power_cpu_ss2();
	QM_PUTS("Wake up from ss2.");

	QM_PUTS("Finished: SS Power Core example.");

	return 0;
}
