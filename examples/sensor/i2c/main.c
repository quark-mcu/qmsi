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

#include "qm_ss_i2c.h"
#include "qm_ss_interrupt.h"
#include "qm_uart.h"
#include "qm_ss_isr.h"
#include "ss_clk.h"
#include "clk.h"

#define EEPROM_PAGE_SIZE_BYTES (64)
#define EEPROM_ADDR_SIZE_BYTES (2)
#define EEPROM_SLAVE_ADDR (0x51)
#define EEPROM_DATA_ADDR (0x0000)
#define EEPROM_WRITE_WAIT_TIME_US (50000)
#define EEPROM_ADDR_FIRST_PAGE_LO (0)
#define EEPROM_ADDR_FIRST_PAGE_HI (0)
#define NULL_TERMINATOR (1)

/*
 * QMSI SS I2C Example
 *
 * This example uses a Microchip 24FC256-I/P I2C EEPROM to demonstrate
 * both polled and interrupt based master I2C transfers on I2C_SS_0.
 *
 * EEPROM pin 2, 3, 4 and 7 are connected to ground
 * EEPROM pin 1, 8 is connected to 3.3V
 * EEPROM pin 5 is connected to I2C_0 SDA pin with pull-up resistor
 * EEPROM pin 6 is connected to I2C_0 SCL pin with pull-up resistor
 */

void eeprom_compare_page(uint8_t *write_data, uint8_t *read_data);
static void i2c_0_cb(void *data, int rc, qm_ss_i2c_status_t status,
		     uint32_t len);

volatile bool i2c_0_complete = false;
static uint8_t id_write = 0;
static uint8_t id_read = 1;

/* Reserve two bytes for eeprom address */
uint8_t eeprom_pio_write_data[EEPROM_PAGE_SIZE_BYTES + EEPROM_ADDR_SIZE_BYTES] =
    "  ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKL";
uint8_t eeprom_irq_write_data[EEPROM_PAGE_SIZE_BYTES + EEPROM_ADDR_SIZE_BYTES] =
    "  1234567890123456789012345678901234567890123456789012345678901234";
uint8_t eeprom_read_addr[EEPROM_ADDR_SIZE_BYTES] = {EEPROM_ADDR_FIRST_PAGE_LO,
						    EEPROM_ADDR_FIRST_PAGE_HI};
uint8_t eeprom_irq_read_buf[EEPROM_PAGE_SIZE_BYTES + NULL_TERMINATOR];
uint8_t eeprom_pio_read_buf[EEPROM_PAGE_SIZE_BYTES + NULL_TERMINATOR];

int main(void)
{
	/*  Variables */
	qm_ss_i2c_config_t cfg;
	qm_ss_i2c_transfer_t xfer_write, xfer_read;
	qm_ss_i2c_status_t status;

	QM_PUTS("Starting: I2C SS EEPROM");

	/*  Enable I2C 0 */
	ss_clk_i2c_enable(QM_SS_I2C_0);

	qm_ss_irq_request(QM_SS_IRQ_I2C_0_ERR, qm_ss_i2c_isr_0);
	qm_ss_irq_request(QM_SS_IRQ_I2C_0_RX_AVAIL, qm_ss_i2c_isr_0);
	qm_ss_irq_request(QM_SS_IRQ_I2C_0_TX_REQ, qm_ss_i2c_isr_0);
	qm_ss_irq_request(QM_SS_IRQ_I2C_0_STOP_DET, qm_ss_i2c_isr_0);

	/* Configure I2C */
	cfg.address_mode = QM_SS_I2C_7_BIT;
	cfg.speed = QM_SS_I2C_SPEED_STD;

	if (qm_ss_i2c_set_config(QM_SS_I2C_0, &cfg)) {
		QM_PUTS("Error: I2C_SS_0 config\n");
	}

	/* Add eeprom data address to the beginning of each message */
	eeprom_pio_write_data[0] = EEPROM_ADDR_FIRST_PAGE_LO;
	eeprom_pio_write_data[1] = EEPROM_ADDR_FIRST_PAGE_HI;
	eeprom_irq_write_data[0] = EEPROM_ADDR_FIRST_PAGE_LO;
	eeprom_irq_write_data[1] = EEPROM_ADDR_FIRST_PAGE_HI;

	QM_PUTS("PIO write");
	if (qm_ss_i2c_master_write(
		QM_SS_I2C_0, EEPROM_SLAVE_ADDR, eeprom_pio_write_data,
		sizeof(eeprom_pio_write_data), true, &status)) {
		QM_PUTS("Error: PIO write");
	} else {

		QM_PUTS("SS I2C PIO TX Transfer complete");
	}

	clk_sys_udelay(EEPROM_WRITE_WAIT_TIME_US);

	QM_PUTS("PIO combined write + read");
	if (qm_ss_i2c_master_write(QM_SS_I2C_0, EEPROM_SLAVE_ADDR,
				   eeprom_read_addr, EEPROM_ADDR_SIZE_BYTES,
				   false, &status)) {
		QM_PUTS("Error: PIO write");
	} else {

		QM_PUTS("SS I2C PIO TX Transfer complete");
	}

	if (qm_ss_i2c_master_read(QM_SS_I2C_0, EEPROM_SLAVE_ADDR,
				  eeprom_pio_read_buf, EEPROM_PAGE_SIZE_BYTES,
				  true, &status)) {
		QM_PUTS("Error: PIO read");
	} else {
		QM_PUTS("SS I2C PIO RX Transfer complete");
	}

	eeprom_compare_page(eeprom_pio_write_data, eeprom_pio_read_buf);

	QM_PUTS("IRQ write");
	xfer_write.tx = eeprom_irq_write_data;
	xfer_write.tx_len = EEPROM_PAGE_SIZE_BYTES + EEPROM_ADDR_SIZE_BYTES;
	xfer_write.callback = i2c_0_cb;
	xfer_write.callback_data = &id_write;
	xfer_write.rx = NULL;
	xfer_write.rx_len = 0;
	xfer_write.stop = true;

	if (qm_ss_i2c_master_irq_transfer(QM_SS_I2C_0, &xfer_write,
					  EEPROM_SLAVE_ADDR)) {
		QM_PUTS("Error: IRQ write");
	}

	while (!i2c_0_complete) {
	}
	clk_sys_udelay(EEPROM_WRITE_WAIT_TIME_US);

	QM_PUTS("IRQ combined write + read");
	xfer_read.tx = eeprom_read_addr;
	xfer_read.tx_len = EEPROM_ADDR_SIZE_BYTES;
	xfer_read.callback = i2c_0_cb;
	xfer_read.callback_data = &id_read;
	xfer_read.rx = eeprom_irq_read_buf;
	xfer_read.rx_len = EEPROM_PAGE_SIZE_BYTES;
	xfer_read.stop = true;

	i2c_0_complete = false;
	if (qm_ss_i2c_master_irq_transfer(QM_I2C_0, &xfer_read,
					  EEPROM_SLAVE_ADDR)) {
		QM_PUTS("Error: IRQ read");
	}

	while (!i2c_0_complete) {
	}

	eeprom_compare_page(eeprom_irq_write_data, eeprom_irq_read_buf);

	QM_PUTS("Finished: I2C SS EEPROM");

	return 0;
}

void eeprom_compare_page(uint8_t *write_data, uint8_t *read_data)
{
	uint32_t i;
	for (i = 0; i < EEPROM_PAGE_SIZE_BYTES; i++) {
		/*write_data contains the address in first 2 bytes, so offset
		 comparison by 2 bytes*/
		if (write_data[i + 2] != read_data[i]) {
			QM_PUTS("Error: Data compare");
			return;
		}
	}

	QM_PUTS("Data compare OK:");
	QM_PUTS((const char *)read_data);
}

static void i2c_0_cb(void *data, int rc, qm_ss_i2c_status_t status,
		     uint32_t len)
{
	QM_PUTS("SS I2C IRQ Transfer complete");
	i2c_0_complete = true;
}
