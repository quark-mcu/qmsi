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
#include "qm_gpio.h"
#include "qm_pinmux.h"

#define UDELAY (500000)

#if (QUARK_D2000)
#define QM_PWM_CH_0_PIN (19)
#define QM_PWM_CH_0_FN_GPIO (1)
#define QM_PWM_CH_0_FN_PWM (2)
#define QM_PWM_CH_1_PIN (24)
#define QM_PWM_CH_1_FN_GPIO (0)
#define QM_PWM_CH_1_FN_PWM (2)
#elif(QUARK_SE)
/*
 * These values will not work from the Lakemont core as the PWM pins are muxed
 * with the sensor subsystem GPIO pins.
 */
#define QM_PWM_CH_0_PIN (10)
#define QM_PWM_CH_0_FN_GPIO (0)
#define QM_PWM_CH_0_FN_PWM (1)
#define QM_PWM_CH_1_PIN (11)
#define QM_PWM_CH_1_FN_GPIO (0)
#define QM_PWM_CH_1_FN_PWM (1)
#else
#error("Unsupported / unspecified processor type")
#endif

void pwm_example_callback(uint32_t pwm_int);
void set_pwm_as_gpio(const qm_pwm_t pwm, const qm_pwm_id_t id, bool high);

uint32_t interrupt_from;

/* QMSI pwm app example */
int main(void)
{
	/* Variables */
	qm_pwm_config_t wr_cfg, rd_cfg;
	uint32_t lo_cnt, hi_cnt;

	/* Initialise pwm configuration */
	wr_cfg.lo_count = 0x100000;
	wr_cfg.hi_count = 0x100000;
	wr_cfg.mode = QM_PWM_MODE_PWM;
	wr_cfg.mask_interrupt = false;
	wr_cfg.callback = pwm_example_callback;

	/* Enable clocking for the PWM block */
	clk_periph_enable(CLK_PERIPH_PWM_REGISTER | CLK_PERIPH_CLK);

	/* Set the configuration of the PWM */
	qm_pwm_set_config(QM_PWM_0, QM_PWM_ID_1, &wr_cfg);
	/* Register the ISR with the SoC */
	qm_irq_request(QM_IRQ_PWM_0, qm_pwm_isr_0);

	/* Optionally, get config back to see the settings */
	qm_pwm_get_config(QM_PWM_0, QM_PWM_ID_1, &rd_cfg);

	qm_pmux_select(QM_PWM_CH_1_PIN, QM_PWM_CH_1_FN_PWM);
	/* Start PWM 2 */
	qm_pwm_start(QM_PWM_0, QM_PWM_ID_1);

	/* Optionally, get the current count values */
	qm_pwm_get(QM_PWM_0, QM_PWM_ID_1, &lo_cnt, &hi_cnt);

	clk_sys_udelay(UDELAY);
	/* Optionally, reload new values into the PWM */
	lo_cnt = hi_cnt = 0x40000;
	qm_pwm_set(QM_PWM_0, QM_PWM_ID_1, lo_cnt, hi_cnt);

	clk_sys_udelay(UDELAY);

	/* Set PWM with 0% duty cycle. */
	set_pwm_as_gpio(QM_PWM_0, QM_PWM_ID_1, false);
	clk_sys_udelay(UDELAY);
	/* Set PWM with 100% duty cycle. */
	set_pwm_as_gpio(QM_PWM_0, QM_PWM_ID_1, true);
	clk_sys_udelay(UDELAY);
	/* Stop the PWM from running */
	qm_pwm_stop(QM_PWM_0, QM_PWM_ID_1);
	/* Disable clocking for the PWM block */
	clk_periph_disable(CLK_PERIPH_PWM_REGISTER);

	return 0;
}

/*
 * Duty cycles of 0% and 100% are not handled by the PWM block. Instead, these
 * are essentially a constant low, or constant high value on the pin.
 */
void set_pwm_as_gpio(const qm_pwm_t pwm, const qm_pwm_id_t id, bool high)
{
#if (QUARK_D2000)
	/* Set pin as output. */
	qm_gpio_port_config_t cfg;
	qm_gpio_get_config(QM_GPIO_0, &cfg);
#elif(QUARK_SE)
/*
 * Warning: Quark SE pins are on sensor subsystem, need to set sensor subsystem
 * port. This can not be done from the Lakemont core.
 */
#else
#error("Unsupported / unspecified processor type")
#endif
	uint32_t pin = 0;
	qm_pmux_fn_t fn = 0;

	switch (id) {
	case QM_PWM_ID_0:
		pin = QM_PWM_CH_0_PIN;
		fn = QM_PWM_CH_0_FN_GPIO;
		break;
	case QM_PWM_ID_1:
		pin = QM_PWM_CH_1_PIN;
		fn = QM_PWM_CH_1_FN_GPIO;
		break;
	default:
		break;
	}
	/* Perform pin muxing. */
	qm_pmux_select(pin, fn);

#if (QUARK_D2000)
	cfg.direction |= BIT(pin);
	qm_gpio_set_config(QM_GPIO_0, &cfg);
	if (true == high) {
		qm_gpio_set_pin(QM_GPIO_0, pin);
	} else {
		qm_gpio_clear_pin(QM_GPIO_0, pin);
	}
#elif(QUARK_SE)
/*
 * Warning: Quark SE pins are on sensor subsystem, need to set sensor subsystem
 * port. This can not be done from the Lakemont core.
 */
#else
#error("Unsupported / unspecified processor type")
#endif
}

void pwm_example_callback(uint32_t pwm_int)
{
	if (pwm_int & BIT(QM_PWM_ID_0)) {
		QM_PUTS("PWM 0 fired.\n");
		interrupt_from = QM_PWM_ID_0;
	}

	if (pwm_int & BIT(QM_PWM_ID_1)) {
		QM_PUTS("PWM 1 fired.\n");
		interrupt_from = QM_PWM_ID_1;
	}

#if (HAS_4_TIMERS)
	if (pwm_int & BIT(QM_PWM_ID_2)) {
		QM_PUTS("PWM 2 fired.\n");
		interrupt_from = QM_PWM_ID_2;
	}

	if (pwm_int & BIT(QM_PWM_ID_3)) {
		QM_PUTS("PWM 3 fired.\n");
		interrupt_from = QM_PWM_ID_3;
	}
#endif
}
