/*
 *  Copyright (c) 2016, Intel Corporation
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

#include "clk.h"
#include "qm_dma.h"
#include "qm_interrupt.h"
#include "qm_isr.h"

#include <inttypes.h>
#include <string.h>

#define WAIT_1SEC (1000000)
#define TRANSFER_LOOPS (5)
#define RX_BUFF_SIZE (100)

typedef struct {
	/** Controller ID */
	qm_dma_t controller_id;

	/** Channel ID */
	qm_dma_channel_id_t channel_id;
} dma_channel_desc_t;

static char tx_data[] = "The DMA example application shows how to initiate a "
			"memory to memory transfer.";
static char rx_data[TRANSFER_LOOPS][RX_BUFF_SIZE] = {{0}};

volatile uint8_t transfer_count = 0;

static void transfer_callback(void *callback_context, uint32_t len,
			      int error_code)
{
	qm_dma_transfer_t dma_trans = {0};
	dma_channel_desc_t *chan_desc = (dma_channel_desc_t *)callback_context;
	int return_code;

	if (error_code) {
		QM_PRINTF("Transfer Error with Error Code: %u\n", error_code);
		return;
	}

	QM_PRINTF("Transfer Loop %d Complete with Data Length: %u\n",
		  transfer_count, len);

	transfer_count++;

	/* Restart the transfer TRANSFER_LOOPS times with new RX buffer. */
	if (transfer_count < TRANSFER_LOOPS) {
		dma_trans.block_size = strlen(tx_data);
		dma_trans.source_address = (uint32_t *)tx_data;
		dma_trans.destination_address =
		    (uint32_t *)rx_data[transfer_count];

		return_code = qm_dma_transfer_mem_to_mem(
		    chan_desc->controller_id, chan_desc->channel_id,
		    &dma_trans);
		if (return_code) {
			QM_PRINTF("ERROR: qm_dma_mem_to_mem_transfer\n");
		}
	}
}

/*
 * DMA Memory to Memory app example.
 */
int main(void)
{
	qm_dma_channel_config_t dma_chan_cfg = {0};
	qm_dma_transfer_t dma_trans = {0};
	static dma_channel_desc_t chan_desc;
	int return_code;
	int i;

	/*
	 * Request the required interrupts. Depending on the channel used a
	 * different isr is set:
	 *     qm_irq_request(QM_IRQ_DMA_<channel>, qm_dma_0_isr_<channel>)
	 */
	qm_irq_request(QM_IRQ_DMA_0, qm_dma_0_isr_0);
	qm_irq_request(QM_IRQ_DMA_ERR, qm_dma_0_isr_err);

	QM_PRINTF("Starting: DMA\n");

	/* Set the controller and channel IDs. */
	chan_desc.controller_id = QM_DMA_0;
	chan_desc.channel_id = QM_DMA_CHANNEL_0;

	QM_PRINTF("Initialising DMA Controller\n");
	return_code = qm_dma_init(chan_desc.controller_id);
	if (return_code) {
		QM_PRINTF("ERROR: qm_dma_init\n");
	}

	QM_PRINTF("Configuring the channel\n");
	dma_chan_cfg.channel_direction = QM_DMA_MEMORY_TO_MEMORY;
	dma_chan_cfg.source_transfer_width = QM_DMA_TRANS_WIDTH_8;
	dma_chan_cfg.destination_transfer_width = QM_DMA_TRANS_WIDTH_8;
	dma_chan_cfg.source_burst_length = QM_DMA_BURST_TRANS_LENGTH_1;
	dma_chan_cfg.destination_burst_length = QM_DMA_BURST_TRANS_LENGTH_1;
	dma_chan_cfg.client_callback = transfer_callback;

	/*
	 * Set the context as the channel descriptor. This will allow the
	 * descriptor to be available in the callback.
	 */
	dma_chan_cfg.callback_context = (void *)&chan_desc;

	return_code = qm_dma_channel_set_config(
	    chan_desc.controller_id, chan_desc.channel_id, &dma_chan_cfg);
	if (return_code) {
		QM_PRINTF("ERROR: qm_dma_channel_set_config\n");
	}

	QM_PRINTF("Starting the transfer and waiting for 1 second\n");
	dma_trans.block_size = strlen(tx_data);
	dma_trans.source_address = (uint32_t *)tx_data;
	dma_trans.destination_address = (uint32_t *)rx_data[transfer_count];

	return_code = qm_dma_transfer_mem_to_mem(
	    chan_desc.controller_id, chan_desc.channel_id, &dma_trans);
	if (return_code) {
		QM_PRINTF("ERROR: qm_dma_mem_to_mem_transfer\n");
	}

	clk_sys_udelay(WAIT_1SEC);

	/* If the transfer has not finished then we need to stop the channel */
	if (transfer_count < TRANSFER_LOOPS) {
		transfer_count = TRANSFER_LOOPS;
		return_code = qm_dma_transfer_terminate(chan_desc.controller_id,
							chan_desc.channel_id);
		if (return_code) {
			QM_PRINTF("ERROR: qm_dma_transfer_stop\n");
		}
	}

	QM_PRINTF("Each RX buffer should contain the full TX buffer string.\n");
	printf("TX data: %s\n", tx_data);

	for (i = 0; i < TRANSFER_LOOPS; i++) {
		printf("RX data Loop %d: %s\n", i, rx_data[i]);
	}

	QM_PRINTF("Finished: DMA\n");

	return 0;
}
