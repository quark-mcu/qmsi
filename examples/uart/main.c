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

#include "qm_pinmux.h"
#include "qm_uart.h"
#include "qm_interrupt.h"
#include "qm_isr.h"
#include "clk.h"

#define BANNER_STR ("Hello, world!  QMSI echo application, Pol'd message.\n")

#define BANNER_IRQ_ID (1)
#define BANNER_IRQ                                                             \
	("\n\nHello, world!  QMSI echo application, IRQ'd message.\n")
#define BANNER_IRQ_COMPLETE ("IRQ Transfer completed.\n")

#define BANNER_DMA_ID (2)
#define BANNER_DMA ("\n\nHello, world!  QMSI echo application, DMA message.\n")
#define BANNER_DMA_COMPLETE ("DMA Transfer completed.\n")

#define RX_STR ("Data received: ")
#define ERR_STR ("Error: Transmission incomplete.\n")

/* Read callback status polling period (us). */
#define WAIT_READ_CB_PERIOD_1MS (1000)
#define TIMEOUT_10SEC (10 * 1000000 / WAIT_READ_CB_PERIOD_1MS)
/* Wait time (us). */
#define WAIT_1SEC (1000000)
#define WAIT_5SEC (5000000)

static void wait_rx_callback_timeout(uint32_t timeout);
static void uart_example_tx_callback(void *data, int error,
				     qm_uart_status_t status, uint32_t len);
static void uart_example_rx_callback(void *data, int error,
				     qm_uart_status_t status, uint32_t len);

#define BIG_NUMBER_RX (50)
static uint8_t rx_buffer[BIG_NUMBER_RX];
static volatile bool rx_callback_invoked = false;

/* Request data for UART asynchronous operation (transfer descriptor) needs to
 * be kept alive during request processing. It is safer when it is globally
 * accessible within the file - we are sure then that they will be always in the
 * scope when IRQ will be triggered*/
static qm_uart_transfer_t async_xfer_desc = {0};

/* Sample UART0 QMSI application. */
int main(void)
{
	qm_uart_config_t cfg = {0};
	qm_uart_status_t uart_status __attribute__((unused)) = 0;
	int ret __attribute__((unused));
	const uint32_t xfer_irq_data = BANNER_IRQ_ID;
	const uint32_t xfer_dma_data = BANNER_DMA_ID;

	/* Set divisors to yield 115200bps baud rate. */
	/* Sysclk is set by boot ROM to hybrid osc in crystal mode (32MHz),
	 * peripheral clock divisor set to 1.
	 */
	cfg.baud_divisor = QM_UART_CFG_BAUD_DL_PACK(0, 17, 6);

	cfg.line_control = QM_UART_LC_8N1;
	cfg.hw_fc = false;

/* Mux out STDOUT_UART tx/rx pins and enable input for rx. */
#if (QUARK_SE)
	if (STDOUT_UART == QM_UART_0) {
		qm_pmux_select(QM_PIN_ID_18, QM_PMUX_FN_0);
		qm_pmux_select(QM_PIN_ID_19, QM_PMUX_FN_0);
		qm_pmux_input_en(QM_PIN_ID_18, true);
	} else {
		qm_pmux_select(QM_PIN_ID_16, QM_PMUX_FN_2);
		qm_pmux_select(QM_PIN_ID_17, QM_PMUX_FN_2);
		qm_pmux_input_en(QM_PIN_ID_17, true);
	}

#elif(QUARK_D2000)
	if (STDOUT_UART == QM_UART_0) {
		qm_pmux_select(QM_PIN_ID_12, QM_PMUX_FN_2);
		qm_pmux_select(QM_PIN_ID_13, QM_PMUX_FN_2);
		qm_pmux_input_en(QM_PIN_ID_13, true);
	} else {
		qm_pmux_select(QM_PIN_ID_20, QM_PMUX_FN_2);
		qm_pmux_select(QM_PIN_ID_21, QM_PMUX_FN_2);
		qm_pmux_input_en(QM_PIN_ID_21, true);
	}

#else
#error("Unsupported / unspecified processor detected.")
#endif

	clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_UARTA_REGISTER);
	qm_uart_set_config(STDOUT_UART, &cfg);

	QM_PRINTF("Starting: UART\n");

	/* Synchronous TX. */
	ret = qm_uart_write_buffer(STDOUT_UART, (uint8_t *)BANNER_STR,
				   sizeof(BANNER_STR));
	QM_ASSERT(0 == ret);

/* Register the UART interrupts. */
#if (STDOUT_UART_0)
	qm_irq_request(QM_IRQ_UART_0, qm_uart_0_isr);
#elif(STDOUT_UART_1)
	qm_irq_request(QM_IRQ_UART_1, qm_uart_1_isr);
#endif

	/* Used on both TX and RX. */
	async_xfer_desc.callback_data = (void *)&xfer_irq_data;

	/* IRQ based TX. */
	async_xfer_desc.data = (uint8_t *)BANNER_IRQ;
	async_xfer_desc.data_len = sizeof(BANNER_IRQ);
	async_xfer_desc.callback = uart_example_tx_callback;
	ret = qm_uart_irq_write(STDOUT_UART, &async_xfer_desc);
	QM_ASSERT(0 == ret);

	clk_sys_udelay(WAIT_1SEC);

	/* IRQ based RX. */
	rx_callback_invoked = false;

	async_xfer_desc.data = rx_buffer;
	async_xfer_desc.data_len = BIG_NUMBER_RX;
	async_xfer_desc.callback = uart_example_rx_callback;
	ret = qm_uart_irq_read(STDOUT_UART, &async_xfer_desc);
	QM_ASSERT(0 == ret);
	QM_PRINTF("\nWaiting for you to type %d characters... ?\n",
		  BIG_NUMBER_RX);

	wait_rx_callback_timeout(TIMEOUT_10SEC);

	if (!rx_callback_invoked) {
		/* RX complete callback was not invoked, we need to terminate
		 * the transfer in order to grab whatever is available in the RX
		 * buffer. */
		ret = qm_uart_irq_read_terminate(STDOUT_UART);
		QM_ASSERT(0 == ret);
	} else {
		/* RX complete callback was invoked and RX buffer was read, we
		 * wait in case the user does not stop typing after entering the
		 * exact amount of data that fits the RX buffer, i.e. there may
		 * be additional bytes in the RX FIFO that need to be read
		 * before continuing. */
		clk_sys_udelay(WAIT_5SEC);

		qm_uart_get_status(STDOUT_UART, &uart_status);
		if (QM_UART_RX_BUSY & uart_status) {
			/* There is some data in the RX FIFO, let's fetch it. */
			ret = qm_uart_irq_read(STDOUT_UART, &async_xfer_desc);
			QM_ASSERT(0 == ret);

			ret = qm_uart_irq_read_terminate(STDOUT_UART);
			QM_ASSERT(0 == ret);
		}
	}

	/* Register the DMA interrupts. */
	qm_irq_request(QM_IRQ_DMA_0, qm_dma_0_isr_0);
	qm_irq_request(QM_IRQ_DMA_ERR, qm_dma_0_isr_err);

	/* DMA controller initialization. */
	ret = qm_dma_init(QM_DMA_0);
	QM_ASSERT(0 == ret);

	/* Used on both TX and RX. */
	async_xfer_desc.callback_data = (void *)&xfer_dma_data;

	/* DMA based TX. */
	ret =
	    qm_uart_dma_channel_config(STDOUT_UART, QM_DMA_0, QM_DMA_CHANNEL_0,
				       QM_DMA_MEMORY_TO_PERIPHERAL);
	QM_ASSERT(0 == ret);

	async_xfer_desc.data = (uint8_t *)BANNER_DMA;
	async_xfer_desc.data_len = sizeof(BANNER_DMA);
	async_xfer_desc.callback = uart_example_tx_callback;
	ret = qm_uart_dma_write(STDOUT_UART, &async_xfer_desc);
	QM_ASSERT(0 == ret);

	clk_sys_udelay(WAIT_1SEC);

	/* DMA based RX. */
	rx_callback_invoked = false;

	ret =
	    qm_uart_dma_channel_config(STDOUT_UART, QM_DMA_0, QM_DMA_CHANNEL_0,
				       QM_DMA_PERIPHERAL_TO_MEMORY);
	QM_ASSERT(0 == ret);

	QM_PUTS("Waiting for data on STDOUT_UART (DMA mode) ...");
	async_xfer_desc.data = (uint8_t *)rx_buffer;
	async_xfer_desc.data_len = BIG_NUMBER_RX;
	async_xfer_desc.callback = uart_example_rx_callback;
	ret = qm_uart_dma_read(STDOUT_UART, &async_xfer_desc);
	QM_ASSERT(0 == ret);

	wait_rx_callback_timeout(TIMEOUT_10SEC);

	if (!rx_callback_invoked) {
		/* RX complete callback was not invoked, we need to terminate
		 * the transfer in order to grab whatever was written in the RX
		 * buffer. */
		ret = qm_uart_dma_read_terminate(STDOUT_UART);
		QM_ASSERT(0 == ret);
	}

	QM_PRINTF("\nFinished: UART\n");
	return 0;
}

static void wait_rx_callback_timeout(uint32_t timeout)
{
	while (!rx_callback_invoked && timeout) {
		clk_sys_udelay(WAIT_READ_CB_PERIOD_1MS);
		timeout--;
	};
}

void uart_example_tx_callback(void *data, int error, qm_uart_status_t status,
			      uint32_t len)
{
	uint32_t id = *(uint32_t *)data;

	switch (id) {

	case BANNER_IRQ_ID:
		qm_uart_write_buffer(STDOUT_UART,
				     (uint8_t *)BANNER_IRQ_COMPLETE,
				     sizeof(BANNER_IRQ_COMPLETE));
		break;

	case BANNER_DMA_ID:
		qm_uart_write_buffer(STDOUT_UART,
				     (uint8_t *)BANNER_DMA_COMPLETE,
				     sizeof(BANNER_DMA_COMPLETE));
		break;

	default:
		break;
	}

	QM_PUTS("\n");
}

void uart_example_rx_callback(void *data, int error, qm_uart_status_t status,
			      uint32_t len)
{
	uint32_t id = *(uint32_t *)data;

	if (!error || error == -ECANCELED) {
		/* Transfer successful or RX terminated. */
		switch (id) {
		case BANNER_IRQ_ID:
		case BANNER_DMA_ID:
			qm_uart_write_buffer(STDOUT_UART, (uint8_t *)RX_STR,
					     sizeof(RX_STR));
			qm_uart_write_buffer(STDOUT_UART, rx_buffer, len);
			rx_callback_invoked = true;
			break;
		default:
			break;
		}
	} else {
		qm_uart_write_buffer(STDOUT_UART, (uint8_t *)ERR_STR,
				     sizeof(ERR_STR));
	}
}
