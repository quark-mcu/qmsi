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

#include "qm_pwm.h"
#include "qm_interrupt.h"
#include "qm_scss.h"

#define UDELAY (500000)

void timer_example_callback(uint32_t timer_int);

uint32_t interrupt_from;

/* QMSI timer app example */
int main(void)
{
	/* Variables */
	qm_pwm_config_t wr_cfg, rd_cfg;
	uint32_t lo_cnt, hi_cnt;

	/* Initialise timer configuration */
	wr_cfg.lo_count = 0x100000;
	wr_cfg.hi_count = 0;
	wr_cfg.mode = QM_PWM_MODE_TIMER_COUNT;
	wr_cfg.mask_interrupt = false;
	wr_cfg.callback = timer_example_callback;

	/* Enable clocking for the PWM block */
	clk_periph_enable(CLK_PERIPH_PWM_REGISTER | CLK_PERIPH_CLK);

	/* Set the configuration of the Timer */
	qm_pwm_set_config(QM_PWM_0, QM_PWM_ID_1, &wr_cfg);
	/* Register the ISR with the SoC */
	qm_irq_request(QM_IRQ_PWM_0, qm_pwm_isr_0);

	/* Optionally, get config back to see the settings */
	qm_pwm_get_config(QM_PWM_0, QM_PWM_ID_1, &rd_cfg);

	/* Start Timer 2 */
	qm_pwm_start(QM_PWM_0, QM_PWM_ID_1);

	/* Optionally, get the current count values */
	qm_pwm_get(QM_PWM_0, QM_PWM_ID_1, &lo_cnt, &hi_cnt);

	clk_sys_udelay(UDELAY);
	/* Optionally, reload new values into the Timer */
	lo_cnt = 0x40000;
	hi_cnt = 0;
	qm_pwm_set(QM_PWM_0, QM_PWM_ID_1, lo_cnt, hi_cnt);

	clk_sys_udelay(UDELAY);

	/* Stop the Timer from running */
	qm_pwm_stop(QM_PWM_0, QM_PWM_ID_1);
	/* Disable clocking for the PWM block */
	clk_periph_disable(CLK_PERIPH_PWM_REGISTER);

	return 0;
}

void timer_example_callback(uint32_t timer_int)
{
	if (timer_int & BIT(QM_PWM_ID_0)) {
		QM_PUTS("Timer 0 fired.\n");
		interrupt_from = QM_PWM_ID_0;
	}

	if (timer_int & BIT(QM_PWM_ID_1)) {
		QM_PUTS("Timer 1 fired.\n");
		interrupt_from = QM_PWM_ID_1;
	}

#if (HAS_4_TIMERS)
	if (timer_int & BIT(QM_PWM_ID_2)) {
		QM_PUTS("Timer 2 fired.\n");
		interrupt_from = QM_PWM_ID_2;
	}

	if (timer_int & BIT(QM_PWM_ID_3)) {
		QM_PUTS("Timer 3 fired.\n");
		interrupt_from = QM_PWM_ID_3;
	}
#endif
}
