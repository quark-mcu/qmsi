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
#include "qm_interrupt.h"
#include "qm_i2c.h"
#include "qm_pinmux.h"
#include "qm_scss.h"

#define SLAVE_ADDR 0x08
#define READ_LEN 23
#define MAX_RETRY_COUNT 5

static void i2c_0_example_tx_callback(uint32_t id, uint32_t len);
static void i2c_0_example_rx_callback(uint32_t id, uint32_t len);
static void i2c_0_error_callback(uint32_t id, qm_i2c_status_t status);

uint8_t irq_data_read[READ_LEN];
uint8_t irq_data[] = "helloworldwhatdayisit?";

/*
 * NOTE: This example app uses lab equipment(aardvark I2C test device)
 * For an example using a real I2C device look at the accel and magneto
 * examples
 */

/*
 * QMSI I2C app example.
 * This example uses an aardvark I2C Host device.
 * Using the aardvark "Control Center Serial"
 * 1. Set to Slave mode with address 0x08
 * 2. Set response to ("helloworldwhatdayisit?"):
 *    68 65 6C 6C 6F 77 6F 72 6C 64 77 68 61 74 64 61 79 69 73 69 74 3F 00
 * 3. Enable
 */
int main(void)
{
	/*  Variables */
	int retry_count = 0;
	uint8_t data[] = "helloworldwhatdayisit?";
	uint8_t data_read[READ_LEN];
	qm_i2c_config_t cfg;
	qm_i2c_transfer_t xfer;

	qm_irq_request(QM_IRQ_I2C_0, qm_i2c_0_isr);

	/*  Enable I2C 0 */
	clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_I2C_M0_REGISTER);
/*  Setup pin mux */
#if (QUARK_SE)
	qm_pmux_select(QM_PIN_ID_20, QM_PMUX_FN_0);
	qm_pmux_select(QM_PIN_ID_21, QM_PMUX_FN_0);
#elif(QUARK_D2000)
	qm_pmux_select(QM_PIN_ID_6, QM_PMUX_FN_2);
	qm_pmux_select(QM_PIN_ID_7, QM_PMUX_FN_2);
#endif

	/* Configure I2C */
	cfg.address_mode = QM_I2C_7_BIT;
	cfg.mode = QM_I2C_MASTER;
	cfg.speed = QM_I2C_SPEED_STD;

	qm_i2c_set_config(QM_I2C_0, &cfg);

	/* Master write */
	while ((qm_i2c_master_write(QM_I2C_0, SLAVE_ADDR, data, sizeof(data),
				    true) != QM_RC_OK) &&
	       (retry_count < MAX_RETRY_COUNT)) {
		retry_count++;
	}

	retry_count = 0;

	/* Master read */
	while ((qm_i2c_master_read(QM_I2C_0, SLAVE_ADDR, data_read, READ_LEN,
				   true) != QM_RC_OK) &&
	       (retry_count < MAX_RETRY_COUNT)) {
		retry_count++;
	}

	/* Combined IRQ mode transfer */
	QM_PUTS("Master IRQ mode combined transfer");
	xfer.tx = irq_data;
	xfer.tx_len = sizeof(irq_data);
	xfer.tx_callback = i2c_0_example_tx_callback;
	xfer.rx = irq_data_read;
	xfer.rx_len = READ_LEN;
	xfer.rx_callback = i2c_0_example_rx_callback;
	xfer.err_callback = i2c_0_error_callback;
	xfer.id = 1;
	xfer.stop = true;

	if (qm_i2c_master_irq_transfer(QM_I2C_0, &xfer, SLAVE_ADDR)) {
		QM_PUTS("IRQ Transfer: Error");
	}

	return 0;
}

static void i2c_0_example_tx_callback(uint32_t id, uint32_t len)
{
	QM_PUTS("I2C TX Transfer complete");
}
static void i2c_0_example_rx_callback(uint32_t id, uint32_t len)
{
	int i = 0;
	QM_PUTS("I2C RX Transfer complete");
	for (i = 0; i < READ_LEN; i++) {
		if (irq_data_read[i] != irq_data[i]) {
			QM_PUTS("Data Received: FAIL");
			return;
		}
	}
	QM_PUTS("Data Received: OK");
}

static void i2c_0_error_callback(uint32_t id, qm_i2c_status_t status)
{
	QM_PUTS("I2C ERROR");
}
