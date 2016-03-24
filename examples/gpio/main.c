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

#include "qm_gpio.h"
#include "qm_interrupt.h"

/* QMSI GPIO app example
 *
 * This app requires a board to be set up with a jumper cable connecting the
 * pins listed below so the output pin can can trigger an interrupt on the
 * input pin. PIN_OUT will be configured as an output pin and PIN_INTR will be
 * configured as an input pin with interrupts
 * enabled.
 *
 * On the Intel(R) Quark(TM) Microcontroller D2000 Development Platform PIN_OUT
 * and PIN_INTR are marked "SSO 10" and "A0".
 * On Atlas Hills, PIN_OUT and PIN_INTR are located on header P4 PIN 42 and 40.
 */
#define PIN_OUT 0
#define PIN_INTR 3

/* Example callback function */
static void gpio_example_callback(uint32_t);

volatile bool callback_invoked;

int main(void)
{
	qm_gpio_port_config_t cfg;

	QM_PUTS("GPIO example app\n");

	/* Request IRQ and write GPIO port config */
	cfg.direction = BIT(PIN_OUT);
	cfg.int_en = BIT(PIN_INTR);       /* Interrupt enabled */
	cfg.int_type = BIT(PIN_INTR);     /* Edge sensitive interrupt */
	cfg.int_polarity = BIT(PIN_INTR); /* Rising edge */
	cfg.int_debounce = BIT(PIN_INTR); /* Debounce enabled */
	cfg.int_bothedge = 0x0;		  /* Both edge disabled */
	cfg.callback = gpio_example_callback;
	callback_invoked = false;

	qm_irq_request(QM_IRQ_GPIO_0, qm_gpio_isr_0);

	qm_gpio_set_config(QM_GPIO_0, &cfg);

	/* Set PIN_OUT to trigger PIN_INTR interrupt */
	qm_gpio_clear_pin(QM_GPIO_0, PIN_OUT);
	qm_gpio_set_pin(QM_GPIO_0, PIN_OUT);

	while (!callback_invoked) {
	}

	if (false != qm_gpio_read_pin(QM_GPIO_0, PIN_OUT)) {
		QM_PUTS("Error: pin comparison failed\n");
		return 1;
	}
	qm_gpio_clear_pin(QM_GPIO_0, PIN_OUT);

	QM_PUTS("GPIO example app complete\n");
	return 0;
}

void gpio_example_callback(uint32_t status)
{
	callback_invoked = true;
	QM_PRINTF("GPIO callback - status register = 0x%u\n", status);
	qm_gpio_clear_pin(QM_GPIO_0, PIN_OUT);
}
