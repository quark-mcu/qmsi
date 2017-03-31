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
 * QMSI SPI eeprom emulation.
 *
 * This example simulates R/W access to a SPI EEPROM,
 *
 * Instructions:
 * - flash one board with examples/spi_eeprom_master.
 * - flash another board with this application.
 * - connect ground of both boards together.
 * - connect SPI0_M_SCK, SPI0_M_MISO, SPI0_M_MOSI and SPI0_M_CS0 of master board
 *   to SPI_S_SCK, SPI_S_MISO, SPI_S_MOSI and SPI_S_CS of the slave board
 *   respectively.
 */
#include "clk.h"
#include "qm_common.h"
#include "qm_interrupt.h"
#include "qm_interrupt_router.h"
#include "qm_isr.h"
#include "qm_pinmux.h"
#include "qm_spi.h"

/*
 * Note: QM_SPI_BMODE_0 is not working on master mode because of a hardware bug.
 */
#define QM_SPI_BMODE QM_SPI_BMODE_1
#define EEPROM_NB_PAGES (1)
#define EEPROM_PAGE_SIZE (128)
#define REQUEST_SIZE (2)

/* Possible value for xfer_mode. */
#define XFER_MODE_RX (BIT(0))
#define XFER_MODE_TX (BIT(1))

/* Maximum number of SPI transaction before stopping. */
#define NB_SPI_TRANSFER (2)

static uint8_t eeprom_pages[EEPROM_NB_PAGES][EEPROM_PAGE_SIZE];
static uint8_t buff[EEPROM_PAGE_SIZE];
static uint8_t addressed_page = 0;
/* Remaining transfer before stopping. */
static volatile int nb_spi_transfer = NB_SPI_TRANSFER;

/*
 * Request data for SPI asynchronous operation (transfer descriptor) needs to
 * be kept alive during request processing. It is safer when it is globally
 * accessible within the file - we are sure then that it will be always in
 * scope when the IRQ is triggered.
 */
static volatile qm_spi_async_transfer_t async_irq_xfer;

static void spi_eeprom_hdr_cb(void *data, int error, qm_spi_status_t status,
			      uint16_t len);

static void spi_irq_cfg(const qm_spi_tmode_t tmode)
{
	int ret __attribute__((unused));
	qm_spi_config_t cfg;

	/*  Initialise SPI configuration. */
	cfg.frame_size = QM_SPI_FRAME_SIZE_8_BIT;
	cfg.transfer_mode = tmode;
	cfg.bus_mode = QM_SPI_BMODE;
	cfg.clk_divider = 1; /* Dummy value. */
	cfg.frame_format = QM_SPI_FRAME_FORMAT_STANDARD;
	/* Configure. */
	ret = qm_spi_set_config(QM_SPI_SLV_0, &cfg);
	QM_ASSERT(0 == ret);
}

static void spi_eeprom_prepare_new_request(void)
{
	async_irq_xfer.rx = buff;
	async_irq_xfer.tx = 0;
	async_irq_xfer.rx_len = REQUEST_SIZE;
	async_irq_xfer.tx_len = 0;
	async_irq_xfer.callback = spi_eeprom_hdr_cb;
	async_irq_xfer.keep_enabled = false;
	async_irq_xfer.callback_data = NULL;
}

/* IRQ based transfer callback, called when processing data. */
static void spi_eeprom_data_cb(void *data, int error, qm_spi_status_t status,
			       uint16_t len)
{
	int ret __attribute__((unused));
	QM_ASSERT(status != QM_SPI_RX_OVERFLOW);

	if (status == QM_SPI_RX_FULL) {
		QM_ASSERT(len == EEPROM_PAGE_SIZE);

	} else if (status == QM_SPI_IDLE) {
		if (nb_spi_transfer) {
			nb_spi_transfer--;
		}
	}
}

/* IRQ based transfer callback, called when processing header. */
static void spi_eeprom_hdr_cb(void *data, int error, qm_spi_status_t status,
			      uint16_t len)
{
	int ret __attribute__((unused));

	if (status == QM_SPI_RX_FULL) {
		QM_ASSERT(len == REQUEST_SIZE);
		/*
		 * 2nd byte is the slot number of the eeprom we want to read
		 * from/write to.
		 */
		addressed_page = buff[1];
		QM_ASSERT(addressed_page < EEPROM_NB_PAGES);
	} else if (status == QM_SPI_IDLE) {
		/*
		 * First byte, buff[0] is the transfer mode the master wants
		 * to do (ie: transmission or reception mode).
		 */
		if (buff[0] & XFER_MODE_RX) {
			spi_irq_cfg(QM_SPI_TMOD_TX);
			async_irq_xfer.rx = 0;
			async_irq_xfer.rx_len = 0;
			async_irq_xfer.tx = eeprom_pages[addressed_page];
			async_irq_xfer.tx_len = EEPROM_PAGE_SIZE;
		} else if (buff[0] & XFER_MODE_TX) {
			spi_irq_cfg(QM_SPI_TMOD_RX);
			async_irq_xfer.rx = eeprom_pages[addressed_page];
			async_irq_xfer.rx_len = EEPROM_PAGE_SIZE;
			async_irq_xfer.tx = 0;
			async_irq_xfer.tx_len = 0;
		} else {
			QM_ASSERT(0);
		}

		async_irq_xfer.callback = spi_eeprom_data_cb;
		async_irq_xfer.callback_data = NULL;
		async_irq_xfer.keep_enabled = false;
		ret = qm_spi_irq_transfer(QM_SPI_SLV_0, &async_irq_xfer);
	}
}

/*  QMSI SPI app example */
int main(void)
{
	int ret = 0;
	int old_nb_spi_transfer = 0;
	int i = 0;

	QM_PRINTF("Starting: SPI Slave\n\n");

/* Mux out SPI TX/RX pins and enable input for RX. */
#if (QUARK_SE)
	qm_pmux_select(QM_PIN_ID_2, QM_PMUX_FN_2); /* SPI0_S SCK  */
	qm_pmux_select(QM_PIN_ID_1, QM_PMUX_FN_2); /* SPI0_S MISO */
	qm_pmux_select(QM_PIN_ID_3, QM_PMUX_FN_2); /* SPI0_S MOSI */
	qm_pmux_select(QM_PIN_ID_0, QM_PMUX_FN_2); /* SPI0_S CS */
	qm_pmux_input_en(QM_PIN_ID_2, true);
	qm_pmux_input_en(QM_PIN_ID_3, true);
	qm_pmux_input_en(QM_PIN_ID_0, true);
#elif(QUARK_D2000)
	qm_pmux_select(QM_PIN_ID_8, QM_PMUX_FN_2);  /* SPI0_S SCK */
	qm_pmux_select(QM_PIN_ID_10, QM_PMUX_FN_2); /* SPI0_S MISO */
	qm_pmux_select(QM_PIN_ID_9, QM_PMUX_FN_2);  /* SPI0_S MOSI */
	qm_pmux_select(QM_PIN_ID_11, QM_PMUX_FN_2); /* SPI0_S CS */
	qm_pmux_input_en(QM_PIN_ID_8, true);
	qm_pmux_input_en(QM_PIN_ID_9, true);
	qm_pmux_input_en(QM_PIN_ID_11, true);
#else
#error("Unsupported / unspecified processor detected.")
#endif

	/* Register driver IRQ. */
	QM_IR_UNMASK_INT(QM_IRQ_SPI_SLAVE_0_INT);
	QM_IRQ_REQUEST(QM_IRQ_SPI_SLAVE_0_INT, qm_spi_slave_0_isr);

	for (i = 0; i < NB_SPI_TRANSFER; ++i) {
		old_nb_spi_transfer = nb_spi_transfer;

		/* Prepare to receive a header packet. */
		spi_irq_cfg(QM_SPI_TMOD_RX);
		spi_eeprom_prepare_new_request();
		ret = qm_spi_irq_transfer(QM_SPI_SLV_0, &async_irq_xfer);
		QM_ASSERT(0 == ret);

		while (nb_spi_transfer == old_nb_spi_transfer) {
		}
	}

	QM_PRINTF("Finished: SPI Slave\n\n");

	return ret;
}
