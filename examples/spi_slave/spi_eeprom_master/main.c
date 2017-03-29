/*
 *  Copyright (c) 2017, Intel Corporation
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

/*
 * QMSI SPI eeprom master
 *
 * This example should be used together with spi_eeprom_slave, which has to be
 * flashed on 2 differents board and connected via SPI port.
 * spi_eeprom_slave simulate a spi eeprom to demonstrate how to use spi slave
 * API.
 * This application will send some data to the eeprom, and then will read them
 * back.
 *
 * Instruction:
 * - flash one board with examples/spi_eeprom_slave.
 * - flash an another board with this application.
 * - connect together ground of both boards.
 * - connect SPI0_M_SCK, SPI0_M_MISO, SPI0_M_MOSI and SPI0_M_CS0 of this board
 *   to respectivly SPI_S_SCK, SPI_S_MISO, SPI_S_MOSI and SPI_S_CS of the slave
 *   board.
 */

#include "clk.h"
#include "qm_common.h"
#include "qm_interrupt.h"
#include "qm_interrupt_router.h"
#include "qm_isr.h"
#include "qm_pinmux.h"
#include "qm_spi.h"

/* The SPI clock divider is calculated in reference to a 32MHz system clock */
#define SPI_CLOCK_125KHZ_DIV (256)
#define SPI_CLOCK_DIV SPI_CLOCK_125KHZ_DIV
#define QM_SPI_BMODE QM_SPI_BMODE_1
#define BUFFERSIZE (128)
#define HDR_SIZE (2)

/*
 * Our eeprom protocol start by sending 2 bytes.
 * first byte gie the mode RX or TX, the second byte the slot of the eeprom we
 * want to read/write.
 */
#define XFER_MODE_RX (BIT(0))
#define XFER_MODE_TX (BIT(1))

/*
 * This example uses the SPI IRQ API to communicate with spi_eeprom_slave, an
 * enulated eeprom.
 */
static uint8_t tx_buff[BUFFERSIZE];
static uint8_t rx_buff[BUFFERSIZE];

/*
 * Request data for SPI asynchronous operation (transfer descriptor) needs to
 * be kept alive during request processing. It is safer when it is globally
 * accessible within the file - we are sure then that they will be always in
 * scope when IRQ will be triggered.
 */
static volatile qm_spi_async_transfer_t async_irq_xfer;
static volatile bool xfer_done = false;

/* IRQ based transfer callback */
static void spi_example_cb(void *data, int err, qm_spi_status_t status,
			   uint16_t len)
{
	xfer_done = true;

	if (!err && len == BUFFERSIZE) {
		/* Transfer completed! */
	} else if (status == QM_SPI_RX_OVERFLOW) {
		QM_PUTS("SPI XFER ERROR -- RX Overflow");
	}
}

static void spi_config(const qm_spi_config_t *cfg_p)
{
	int ret __attribute__((unused));

	ret = qm_spi_set_config(QM_SPI_MST_0, cfg_p);
	QM_ASSERT(0 == ret);

	ret = qm_spi_slave_select(QM_SPI_MST_0, QM_SPI_SS_0);
	QM_ASSERT(0 == ret);
}

static void spi_irq_mode(uint16_t rx_len, uint16_t tx_len)
{
	qm_spi_config_t cfg;
	int ret __attribute__((unused));

	/* Initialise SPI configuration. */
	cfg.frame_size = QM_SPI_FRAME_SIZE_8_BIT;
	if (rx_len && (tx_len == 0)) {
		cfg.transfer_mode = QM_SPI_TMOD_RX;
	} else {
		cfg.transfer_mode = QM_SPI_TMOD_TX;
		QM_ASSERT(tx_len && (rx_len == 0));
	}
	cfg.frame_format = QM_SPI_FRAME_FORMAT_STANDARD;
	cfg.bus_mode = QM_SPI_BMODE;
	cfg.clk_divider = SPI_CLOCK_DIV;
	spi_config(&cfg);

	/* Set up the async transfer struct. */
	async_irq_xfer.tx = tx_buff;
	async_irq_xfer.tx_len = tx_len;
	async_irq_xfer.rx = rx_buff;
	async_irq_xfer.rx_len = rx_len;
	async_irq_xfer.callback = spi_example_cb;
	async_irq_xfer.keep_enabled = false;
	async_irq_xfer.callback_data = NULL;
	ret = qm_spi_irq_transfer(QM_SPI_MST_0, &async_irq_xfer);
	QM_ASSERT(0 == ret);
}

static void spi_buff_reset(void)
{
	int i;
	for (i = 0; i < BUFFERSIZE; i++) {
		tx_buff[i] = i % 0xff;
		rx_buff[i] = 0xf0;
	}
}

static void spi_irq_eeprom_write(void)
{
	xfer_done = false;

	/* Prepare the command. */
	tx_buff[0] = XFER_MODE_TX;
	tx_buff[1] = 0;

	/* Send it. */
	spi_irq_mode(0, HDR_SIZE);
	while (false == xfer_done) {
	}

	/* Wait for the slave to apply new config. */
	clk_sys_udelay(500);

	/* Prepare data to send. */
	spi_buff_reset();
	xfer_done = false;

	/* Send it. */
	spi_irq_mode(0, BUFFERSIZE);
	while (false == xfer_done) {
	}

	/* Wait for the slave to apply new config. */
	clk_sys_udelay(500);
}

static void spi_irq_eeprom_read(void)
{
	int i;
	int error = 0;

	xfer_done = false;

	/* Prepare the command. */
	tx_buff[0] = XFER_MODE_RX;
	tx_buff[1] = 0;

	/* Send command. */
	spi_irq_mode(0, HDR_SIZE);
	while (false == xfer_done) {
	}

	/* Wait for the slave to apply new config. */
	clk_sys_udelay(500);

	xfer_done = false;
	spi_buff_reset();

	/* Read data back. */
	spi_irq_mode(BUFFERSIZE, 0);
	while (false == xfer_done) {
	}

	/* Compare RX and TX buffers. */
	for (i = 0; i < BUFFERSIZE; i++) {
		if (tx_buff[i] != rx_buff[i]) {
			error = 1;
			break;
		}
	}

	if (error == 0) {
		QM_PUTS("SUCCESS: Buffers match!");
	} else {
		QM_PUTS("ERROR: Buffers doesnt match!");
	}
}

/*  QMSI SPI app example */
int main(void)
{
	QM_PRINTF("Starting: SPI\n\n");

/* Mux out SPI TX/RX pins and enable input for RX. */
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

	/* Register driver IRQ. */
	QM_IR_UNMASK_INT(QM_IRQ_SPI_MASTER_0_INT);
	QM_IRQ_REQUEST(QM_IRQ_SPI_MASTER_0_INT, qm_spi_master_0_isr);

	/* Write to eeprom. */
	QM_PRINTF("Write into the eeprom\n");
	spi_irq_eeprom_write();

	/* Read back the data, and compare. */
	QM_PRINTF("Read back from eeprom\n");
	spi_irq_eeprom_read();

	QM_PRINTF("Finished: SPI\n");

	return 0;
}
