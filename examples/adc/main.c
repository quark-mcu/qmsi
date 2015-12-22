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

#include "qm_adc.h"
#include "qm_interrupt.h"
#include "qm_scss.h"
#include "qm_pinmux.h"

/* QMSI ADC app example
 *
 */

#define NUM_CHANNELS (2)
#define NUM_SAMPLES_POLLED (10)
#define NUM_SAMPLES_INTERRUPT (50)

#if (QUARK_D2000)
static void complete_callback(void);
static void error_callback(void);
#endif /* QUARK_D2000 */

static volatile bool complete = false;

int main(void)
{
#if (QUARK_D2000)
	int i;
	qm_adc_config_t cfg;
	qm_adc_xfer_t xfer;
	qm_adc_channel_t channels[] = {QM_ADC_CH_8, QM_ADC_CH_9};
	uint32_t samples_polled[NUM_SAMPLES_POLLED] = {0};
	uint32_t samples_interrupt[NUM_SAMPLES_INTERRUPT] = {0};

	QM_PUTS("\nADC example app start");

	/* Enable the ADC and set the clock divisor */
	clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_ADC |
			  CLK_PERIPH_ADC_REGISTER);
	clk_adc_set_div(100); /* ADC clock is 320KHz @ 32MHz */

	/* Set up pinmux */
	qm_pmux_select(QM_PIN_ID_8, QM_PMUX_FN_1);
	qm_pmux_select(QM_PIN_ID_9, QM_PMUX_FN_1);
	qm_pmux_input_en(QM_PIN_ID_8, true);
	qm_pmux_input_en(QM_PIN_ID_9, true);

	/* Set the mode and calibrate */
	qm_adc_set_mode(QM_ADC_0, QM_ADC_MODE_NORM_CAL);
	qm_adc_calibrate(QM_ADC_0);

	/* Set up config */
	cfg.window = 20; /* Clock cycles between the start of each sample */
	cfg.resolution = QM_ADC_RES_12_BITS;
	qm_adc_set_config(QM_ADC_0, &cfg);

	/*******************************
	 * ADC polled mode example
	 ******************************/
	QM_PUTS("ADC polled mode");

	/* Set up xfer */
	xfer.ch = channels;
	xfer.ch_len = NUM_CHANNELS;
	xfer.samples = samples_polled;
	xfer.samples_len = NUM_SAMPLES_POLLED;

	/* Run the conversion */
	if (qm_adc_convert(QM_ADC_0, &xfer)) {
		QM_PUTS("ERROR: qm_adc_convert failed");
		return 1;
	}

	/* Print the values of the samples */
	for (i = 0; i < NUM_SAMPLES_POLLED; i++) {
		QM_PRINTF("%d:%x ", i, (unsigned int)samples_polled[i]);
	}

	/*******************************
	 * ADC interrupt mode example
	 ******************************/
	QM_PUTS("\nADC interrupt mode");

	qm_irq_request(QM_IRQ_ADC_0, qm_adc_0_isr);

	/* Set up xfer */
	xfer.ch = channels;
	xfer.ch_len = NUM_CHANNELS;
	xfer.samples = samples_interrupt;
	xfer.samples_len = NUM_SAMPLES_INTERRUPT;
	xfer.complete_callback = complete_callback;
	xfer.error_callback = error_callback;

	if (qm_adc_irq_convert(QM_ADC_0, &xfer)) {
		QM_PUTS("ERROR: qm_adc_irq_convert failed");
		return 1;
	}

	/* Wait for the callback */
	while (false == complete)
		;

	for (i = 0; i < NUM_SAMPLES_INTERRUPT; i++) {
		QM_PRINTF("%d:%x ", i, (unsigned int)samples_interrupt[i]);
	}

	QM_PUTS("\nADC example app complete");

#endif /* QUARK_D2000 */
	return 0;
}

void complete_callback(void)
{
	QM_PUTS("---COMPLETE CALLBACK---");
	complete = true;
}

void error_callback(void)
{
	QM_PUTS("---ERROR CALLBACK---");
}
