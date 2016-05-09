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
#include "qm_ss_interrupt.h"
#include "qm_ss_timer.h"
#include "qm_gpio.h"
#include "qm_uart.h"
#include "qm_ss_isr.h"

#define LED_BIT (25) /* On-board LED on Atlas Hills board */

#define ONE_SEC_AT_32MHZ (0x02000000)

#define TIMER_FIRED_MAX (5)
static volatile uint32_t timer_fired = 0;

static void timer0_expired(void *data)
{
	qm_gpio_state_t pin;
	timer_fired++;
	qm_gpio_read_pin(QM_GPIO_0, LED_BIT, &pin);

	if (pin) {
		qm_gpio_clear_pin(QM_GPIO_0, LED_BIT);
	} else {
		qm_gpio_set_pin(QM_GPIO_0, LED_BIT);
	}

	QM_PUTS("Timer fired\n");
	qm_ss_timer_set(QM_SS_TIMER_0, 0x00000000);
}

/* This example uses the Sensor Subsystem timer.
 * The timer is set with interrupts enabled.
 *
 * Interrupt is triggered every TWO_SEC_AT_32MHZ.
 *
 * On Atlas Hills, this examples blinks the on-board LED.
 */
int main(void)
{
	qm_ss_timer_config_t conf;
	qm_gpio_port_config_t gpio_cfg = {0};

	QM_PUTS("Starting: Sensor Timer\n");

	/* Timer is core internals, it falls in the exception's category,
	 * not a peripheral's IRQ.  */
	qm_ss_int_vector_request(QM_SS_INT_TIMER_0, qm_ss_timer_isr_0);
	qm_ss_irq_unmask(QM_SS_INT_TIMER_0);

	/* Request IRQ and write GPIO port config */
	gpio_cfg.direction = BIT(LED_BIT);

	qm_gpio_set_config(QM_GPIO_0, &gpio_cfg);

	conf.watchdog_mode = false;
	conf.inc_run_only = false;
	conf.int_en = true;
	conf.count = ONE_SEC_AT_32MHZ;
	conf.callback = timer0_expired;
	conf.callback_data = NULL;

	if (qm_ss_timer_set_config(QM_SS_TIMER_0, &conf) != 0) {
		QM_PUTS("Error: Set Config for TIMER0 failed\n");
		return -1;
	}

	if (qm_ss_timer_set(QM_SS_TIMER_0, 0x00000000) != 0) {
		QM_PUTS("Error: Set Count for TIMER0 failed\n");
		return -1;
	}

	while (TIMER_FIRED_MAX > timer_fired) {
	}

	/*
	 * Disable the interrupts, we can never stop the timer once it is has
	 * started running.
	 */
	conf.int_en = false;
	if (qm_ss_timer_set_config(QM_SS_TIMER_0, &conf) != 0) {
		QM_PUTS("Error: Set Config for TIMER0 failed\n");
		return -1;
	}

	QM_PUTS("Finished: Sensor Timer\n");
	return 0;
}
