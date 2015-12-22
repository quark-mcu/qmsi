/*
 *  Copyright (c) 2015, Intel Corporation
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. Neither the name of the Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "qm_common.h"
#include "qm_spi.h"
#include "qm_pinmux.h"
#include "qm_interrupt.h"
#include "qm_scss.h"

/* With a system clock of 32MHz this value will result in 125khz SPI clk. */
#define SPI_CLOCK_DIV (256)
#define BUFFERSIZE (128)
#define SPI_EXAMPLE_IRQ_ID (1)

/* QMSI SPI loopback example.
 *
 * This example uses the SPI loopback mode to demonstrate setting up and
 * executing both a polled and interrupt based transfer on SPI master 0.
 */

static void spi_tx_example_cb(uint32_t id, uint32_t len);
static void spi_rx_example_cb(uint32_t id, uint32_t len);
static void spi_err_example_cb(uint32_t id, qm_rc_t status);

static uint8_t tx_buff[BUFFERSIZE];
static uint8_t rx_buff[BUFFERSIZE];

/*  QMSI SPI app example */
int main(void)
{
	/*  Variables */
	qm_spi_config_t cfg;
	qm_spi_transfer_t polled_xfer_desc;
	qm_spi_async_transfer_t irq_xfer_desc;
	unsigned int i;

	for (i = 0; i < BUFFERSIZE; i++) {
		tx_buff[i] = i;
	}

/* Mux out SPI tx/rx pins and enable input for rx. */
#if (QUARK_SE)
	qm_pmux_select(QM_PIN_ID_55, QM_PMUX_FN_1); /* SPI0_M SCK */
	qm_pmux_select(QM_PIN_ID_56, QM_PMUX_FN_1); /* SPI0_M MISO */
	qm_pmux_select(QM_PIN_ID_57, QM_PMUX_FN_1); /* SPI0_M MOSI */
	qm_pmux_select(QM_PIN_ID_58, QM_PMUX_FN_1); /* SPI0_M SS0 */
	qm_pmux_select(QM_PIN_ID_59, QM_PMUX_FN_1); /* SPI0_M SS1 */
	qm_pmux_select(QM_PIN_ID_60, QM_PMUX_FN_1); /* SPI0_M SS2 */
	qm_pmux_select(QM_PIN_ID_61, QM_PMUX_FN_1); /* SPI0_M SS3 */
	qm_pmux_input_en(QM_PIN_ID_56, true);

#elif(QUARK_D2000)
	qm_pmux_select(QM_PIN_ID_0, QM_PMUX_FN_2);  /* SS0 */
	qm_pmux_select(QM_PIN_ID_1, QM_PMUX_FN_2);  /* SS1 */
	qm_pmux_select(QM_PIN_ID_2, QM_PMUX_FN_2);  /* SS2 */
	qm_pmux_select(QM_PIN_ID_3, QM_PMUX_FN_2);  /* SS3 */
	qm_pmux_select(QM_PIN_ID_16, QM_PMUX_FN_2); /* SCK */
	qm_pmux_select(QM_PIN_ID_17, QM_PMUX_FN_2); /* TXD */
	qm_pmux_select(QM_PIN_ID_18, QM_PMUX_FN_2); /* RXD */
	qm_pmux_input_en(QM_PIN_ID_18, true);       /* RXD input */
#else
#error("Unsupported / unspecified processor detected.")
#endif

	/* Enable the clock to the controller. */
	clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_SPI_M0_REGISTER);

	/*  Initialise SPI configuration */
	cfg.frame_size = QM_SPI_FRAME_SIZE_8_BIT;
	cfg.transfer_mode = QM_SPI_TMOD_TX_RX;
	cfg.bus_mode = QM_SPI_BMODE_0;
	cfg.clk_divider = SPI_CLOCK_DIV;

	qm_spi_set_config(QM_SPI_MST_0, &cfg);

	/* Set up loopback mode by RMW directly to the ctrlr0 register. */
	QM_SPI[QM_SPI_MST_0]->ctrlr0 |= BIT(11);

	qm_spi_slave_select(QM_SPI_MST_0, QM_SPI_SS_0);

	QM_PRINTF("\nStatus = 0x%08x", qm_spi_get_status(QM_SPI_MST_0));

	/* Set up the sync transfer struct and call a polled transfer. */
	polled_xfer_desc.tx = tx_buff;
	polled_xfer_desc.rx = rx_buff;
	polled_xfer_desc.tx_len = BUFFERSIZE;
	polled_xfer_desc.rx_len = BUFFERSIZE;

	qm_spi_transfer(QM_SPI_MST_0, &polled_xfer_desc);

	/* Compare RX and TX buffers */
	/* Also reset the buffers for the IRQ example */
	for (i = 0; i < BUFFERSIZE; i++) {
		QM_CHECK(tx_buff[i] == rx_buff[i], QM_RC_EINVAL);
		tx_buff[i] = i;
		rx_buff[i] = 0;
	}

	/* Set up the async transfer struct. */
	irq_xfer_desc.tx = tx_buff;
	irq_xfer_desc.rx = rx_buff;
	irq_xfer_desc.tx_len = BUFFERSIZE;
	irq_xfer_desc.rx_len = BUFFERSIZE;
	irq_xfer_desc.id = SPI_EXAMPLE_IRQ_ID;
	irq_xfer_desc.rx_callback = spi_rx_example_cb;
	irq_xfer_desc.tx_callback = spi_tx_example_cb;
	irq_xfer_desc.err_callback = spi_err_example_cb;

	/* Register driver IRQ. */
	qm_irq_request(QM_IRQ_SPI_MASTER_0, qm_spi_master_0_isr);

	/* Start the async (irq-based) transfer. */
	qm_spi_irq_transfer(QM_SPI_MST_0, &irq_xfer_desc);

	while (1)
		;

	return 0;
}

void spi_tx_example_cb(uint32_t id, uint32_t len)
{
	QM_PUTS("\nSPI TX DONE\n");
}

void spi_rx_example_cb(uint32_t id, uint32_t len)
{
	int i;
	QM_PUTS("\nSPI RX DONE\n");
	/* Compare RX and TX buffers */
	for (i = 0; i < BUFFERSIZE; i++) {
		if (tx_buff[i] != rx_buff[i]) {
			QM_PUTS("\nERROR: RX/TX Buffers mismatch!\n");
		} else if (i == BUFFERSIZE - 1) {
			QM_PUTS("\nSUCCESS: Buffers match!\n");
		}
	}
}

void spi_err_example_cb(uint32_t id, qm_rc_t status)
{
	QM_PUTS("\nSPI ERROR\n");
}
