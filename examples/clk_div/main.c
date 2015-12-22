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

#include "qm_scss.h"
#include "qm_uart.h"
#include "qm_pinmux.h"

#define BANNER ("Hello, world!  QMSI clk div example\n\n")

/* QMSI clock divisor app example */
int main(void)
{
	qm_uart_config_t cfg;

	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_1);

	cfg.baud_divisor = QM_UART_CFG_BAUD_DL_PACK(0, 17, 6);
	cfg.line_control = QM_UART_LC_8N1;
	cfg.hw_fc = false;

#if (QUARK_SE)
	qm_pmux_select(QM_PIN_ID_18, QM_PMUX_FN_0);
	qm_pmux_select(QM_PIN_ID_19, QM_PMUX_FN_0);
	qm_pmux_input_en(QM_PIN_ID_18, true);
#else
	qm_pmux_select(QM_PIN_ID_12, QM_PMUX_FN_2);
	qm_pmux_select(QM_PIN_ID_13, QM_PMUX_FN_2);
	qm_pmux_input_en(QM_PIN_ID_13, true);
#endif

	clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_UARTA_REGISTER);
	qm_uart_set_config(QM_UART_0, &cfg);

	qm_uart_write_buffer(QM_UART_0, (uint8_t *)BANNER, sizeof(BANNER));
	/* Change clock speed to 16MHz, and reconf we still get 115200 BAUD. */
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_2);
	cfg.baud_divisor = QM_UART_CFG_BAUD_DL_PACK(0, 8, 11);
	qm_uart_set_config(QM_UART_0, &cfg);
	qm_uart_write_buffer(QM_UART_0, (uint8_t *)BANNER, sizeof(BANNER));

	/* Change clock speed to 4 MHz. Div to 1. */
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_8);
	cfg.baud_divisor = QM_UART_CFG_BAUD_DL_PACK(0, 2, 3);
	qm_uart_set_config(QM_UART_0, &cfg);
	qm_uart_write_buffer(QM_UART_0, (uint8_t *)BANNER, sizeof(BANNER));

	clk_gpio_db_set_div(CLK_GPIO_DB_DIV_2);
	clk_ext_set_div(CLK_EXT_DIV_4);
	clk_rtc_set_div(CLK_RTC_DIV_8);

	return 0;
}
