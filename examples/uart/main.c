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

#include "qm_pinmux.h"
#include "qm_uart.h"
#include "qm_interrupt.h"
#include "qm_scss.h"

#define BANNER_STR ("Hello, world!  QMSI echo application, Pol'd message.\n\n")

#define BANNER_IRQ_ID (1)
#define BANNER_IRQ ("Hello, world!  QMSI echo application, IRQ'd message.\n\n")
#define BANNER_IRQ_COMPLETE ("IRQ Transfer completed.\n")

#define RX_STR ("Data received.\n")
#define ERR_STR ("Transmission error.\n")

#define CLR_SCREEN ("\033[2J\033[;H")

static void uart_0_example_tx_callback(uint32_t id, uint32_t len);
static void uart_0_example_rx_callback(uint32_t id, uint32_t len);
static void uart_0_example_error_callback(uint32_t id, qm_uart_status_t status);

#define BIG_NUMBER_RX (50)
static uint8_t rx_buffer[BIG_NUMBER_RX];

/* Sample UART0 QMSI application. */
int main(void)
{
	qm_uart_config_t cfg, rd_cfg;
	qm_uart_transfer_t xfer;
	qm_uart_status_t ret __attribute__((unused));

	/* Set divisors to yield 115200bps baud rate. */
	/* Sysclk is set by boot ROM to hybrid osc in crystal mode (32MHz),
	 * peripheral clock divisor set to 1.
	 */
	cfg.baud_divisor = QM_UART_CFG_BAUD_DL_PACK(0, 17, 6);

	cfg.line_control = QM_UART_LC_8N1;
	cfg.hw_fc = false;

/* Mux out UART0 tx/rx pins and enable input for rx. */
#if (QUARK_SE)
	qm_pmux_select(QM_PIN_ID_18, QM_PMUX_FN_0);
	qm_pmux_select(QM_PIN_ID_19, QM_PMUX_FN_0);
	qm_pmux_input_en(QM_PIN_ID_18, true);

#elif(QUARK_D2000)
	qm_pmux_select(QM_PIN_ID_12, QM_PMUX_FN_2);
	qm_pmux_select(QM_PIN_ID_13, QM_PMUX_FN_2);
	qm_pmux_input_en(QM_PIN_ID_13, true);

#else
#error("Unsupported / unspecified processor detected.")
#endif

	clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_UARTA_REGISTER);
	qm_uart_set_config(QM_UART_0, &cfg);
	qm_uart_get_config(QM_UART_0, &rd_cfg);

	qm_uart_write_buffer(QM_UART_0, (uint8_t *)CLR_SCREEN,
			     sizeof(CLR_SCREEN));
	qm_uart_write_buffer(QM_UART_0, (uint8_t *)BANNER_STR,
			     sizeof(BANNER_STR));

	/* Setup xfer for IRQ transfer. */
	xfer.data = (uint8_t *)BANNER_IRQ;
	xfer.data_len = sizeof(BANNER_IRQ);
	xfer.fin_callback = uart_0_example_tx_callback;
	xfer.err_callback = uart_0_example_error_callback;
	xfer.id = BANNER_IRQ_ID;

	qm_irq_request(QM_IRQ_UART_0, qm_uart_0_isr);

	/* Transmit IRQ hello world string. */
	ret = qm_uart_irq_write(QM_UART_0, &xfer);
	QM_ASSERT(QM_UART_OK == ret);

	/* Setup a non-blocking IRQ RX transfer. */
	xfer.data = rx_buffer;
	xfer.data_len = BIG_NUMBER_RX;
	xfer.fin_callback = uart_0_example_rx_callback;
	xfer.err_callback = uart_0_example_error_callback;
	ret = qm_uart_irq_read(QM_UART_0, &xfer);
	QM_ASSERT(QM_UART_OK == ret);

	return 0;
}

void uart_0_example_tx_callback(uint32_t id, uint32_t len)
{
	switch (id) {

	case BANNER_IRQ_ID:
		qm_uart_write_buffer(QM_UART_0, (uint8_t *)BANNER_IRQ_COMPLETE,
				     sizeof(BANNER_IRQ_COMPLETE));
		break;

	default:
		break;
	}
}

void uart_0_example_rx_callback(uint32_t id, uint32_t len)
{
	switch (id) {

	case BANNER_IRQ_ID:
		qm_uart_write_buffer(QM_UART_0, (uint8_t *)RX_STR,
				     sizeof(RX_STR));
		qm_uart_write_buffer(QM_UART_0, rx_buffer, BIG_NUMBER_RX);
		break;
	default:
		break;
	}
}

void uart_0_example_error_callback(uint32_t id, qm_uart_status_t status)
{
	qm_uart_write_buffer(QM_UART_0, (uint8_t *)ERR_STR, sizeof(ERR_STR));
}
