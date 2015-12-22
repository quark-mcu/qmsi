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

static void (*callback[QM_GPIO_NUM])(uint32_t);

void qm_gpio_isr_0(void)
{
	uint32_t int_status = QM_GPIO[QM_GPIO_0].gpio_intstatus;

	if (callback[QM_GPIO_0]) {
		(*callback[QM_GPIO_0])(int_status);
	}

	/* This will clear all pending interrupts flags in status */
	QM_GPIO[QM_GPIO_0].gpio_porta_eoi = int_status;
}

qm_rc_t qm_gpio_set_config(const qm_gpio_t gpio,
			   const qm_gpio_port_config_t *const cfg)
{
	QM_CHECK(gpio < QM_GPIO_NUM, QM_RC_EINVAL);
	QM_CHECK(cfg != NULL, QM_RC_EINVAL);

	uint32_t mask = QM_GPIO[gpio].gpio_intmask;
	QM_GPIO[gpio].gpio_intmask = 0xffffffff;

	QM_GPIO[gpio].gpio_swporta_ddr = cfg->direction;
	QM_GPIO[gpio].gpio_inttype_level = cfg->int_type;
	QM_GPIO[gpio].gpio_int_polarity = cfg->int_polarity;
	QM_GPIO[gpio].gpio_debounce = cfg->int_debounce;
	QM_GPIO[gpio].gpio_int_bothedge = cfg->int_bothedge;
	callback[gpio] = cfg->callback;
	QM_GPIO[gpio].gpio_inten = cfg->int_en;

	QM_GPIO[gpio].gpio_intmask = mask;

	return QM_RC_OK;
}

qm_rc_t qm_gpio_get_config(const qm_gpio_t gpio,
			   qm_gpio_port_config_t *const cfg)
{
	QM_CHECK(gpio < QM_GPIO_NUM, QM_RC_EINVAL);
	QM_CHECK(cfg != NULL, QM_RC_EINVAL);

	cfg->direction = QM_GPIO[gpio].gpio_swporta_ddr;
	cfg->int_en = QM_GPIO[gpio].gpio_inten;
	cfg->int_type = QM_GPIO[gpio].gpio_inttype_level;
	cfg->int_polarity = QM_GPIO[gpio].gpio_int_polarity;
	cfg->int_debounce = QM_GPIO[gpio].gpio_debounce;
	cfg->int_bothedge = QM_GPIO[gpio].gpio_int_bothedge;
	cfg->callback = callback[gpio];

	return QM_RC_OK;
}

bool qm_gpio_read_pin(const qm_gpio_t gpio, const uint8_t pin)
{
	return (((QM_GPIO[gpio].gpio_ext_porta) >> pin) & 1);
}

qm_rc_t qm_gpio_set_pin(const qm_gpio_t gpio, const uint8_t pin)
{
	QM_CHECK(gpio < QM_GPIO_NUM, QM_RC_EINVAL);
	QM_CHECK(pin <= QM_NUM_GPIO_PINS, QM_RC_EINVAL);

	QM_GPIO[gpio].gpio_swporta_dr |= (1 << pin);

	return QM_RC_OK;
}

qm_rc_t qm_gpio_clear_pin(const qm_gpio_t gpio, const uint8_t pin)
{
	QM_CHECK(gpio < QM_GPIO_NUM, QM_RC_EINVAL);
	QM_CHECK(pin <= QM_NUM_GPIO_PINS, QM_RC_EINVAL);

	QM_GPIO[gpio].gpio_swporta_dr &= ~(1 << pin);

	return QM_RC_OK;
}

uint32_t qm_gpio_read_port(const qm_gpio_t gpio)
{
	return (QM_GPIO[gpio].gpio_ext_porta);
}

qm_rc_t qm_gpio_write_port(const qm_gpio_t gpio, const uint32_t val)
{
	QM_CHECK(gpio < QM_GPIO_NUM, QM_RC_EINVAL);

	QM_GPIO[gpio].gpio_swporta_dr = val;

	return QM_RC_OK;
}
