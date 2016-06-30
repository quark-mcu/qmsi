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

#include "qm_mailbox.h"
#include "qm_interrupt.h"
#include "qm_uart.h"
#include "clk.h"
#include "qm_isr.h"

#define TEST_CASES_NUM 12
#define MBOX_READ_TIMEOUT 1000UL /* timeout 1sec */
#define DELAY 1000

#if (QM_SENSOR)
#define QM_MBOX_PRINTF(...)
#else
#define QM_MBOX_PRINTF(...) printf(__VA_ARGS__)
#endif

int loopback_one(int pair);
int read_and_verify(qm_mbox_ch_t mbox, int loop);
void mbox_cb(void *data);

/*
 * Declaration of the flags used to know when a mailbox message has been
 * received.
 */
volatile bool mb_got_message[QM_MBOX_CH_NUM] = {false, false, false, false,
						false, false, false, false};

qm_mbox_msg_t tx_data, rx_data;
qm_mbox_ch_t mbox_pairs[TEST_CASES_NUM][2] = {
    {QM_MBOX_CH_0, QM_MBOX_CH_1}, {QM_MBOX_CH_2, QM_MBOX_CH_3},
    {QM_MBOX_CH_4, QM_MBOX_CH_5}, {QM_MBOX_CH_6, QM_MBOX_CH_7},
    {QM_MBOX_CH_0, QM_MBOX_CH_4}, {QM_MBOX_CH_1, QM_MBOX_CH_5},
    {QM_MBOX_CH_2, QM_MBOX_CH_6}, {QM_MBOX_CH_3, QM_MBOX_CH_7},
    {QM_MBOX_CH_4, QM_MBOX_CH_0}, {QM_MBOX_CH_5, QM_MBOX_CH_1},
    {QM_MBOX_CH_6, QM_MBOX_CH_2}, {QM_MBOX_CH_7, QM_MBOX_CH_3}};
int ofset = 0x100;

/*
 * QMSI MAILBOX app example
 *
 * This app is example of data loopbacked from LMT to SS and back over to LMT.
 * The loopback mailbox channels are defined in mbox_pairs[][].
  * mailbox channel. Test case scenario is:
 * 1. data sent from LMT over the mbox channel defined in mbox_pairs
 * 2. SS handles an interrupt request for data on the same channel.
 * 3. send data from SS over the paired mbox channel defined in mbox_pairs
 * 4. LMT will handle an interrupt request for data on the same channel.
 * 5. repeat steps 1-4 for the every channel pair defined in mbox_pairs
 */
int main(void)
{
	int i;
	QM_MBOX_PRINTF("Starting: Mailbox Loop-back example - Demonstrating "
		       "MAILBOX interrupt functionality\n");

	for (i = 0; i < TEST_CASES_NUM; i++) {
		if (-EIO == loopback_one(i)) {
			return 0;
		}
	}
	QM_MBOX_PRINTF("Finished: Mailbox Loop-back example\n");
	return 0;
}

int loopback_one(int pair)
{
	int i;
	qm_mbox_ch_t mbox_tx, mbox_rx;

#if (QM_SENSOR)
	mbox_rx = mbox_pairs[pair][0];
	mbox_tx = mbox_pairs[pair][1];
#else
	mbox_rx = mbox_pairs[pair][1];
	mbox_tx = mbox_pairs[pair][0];
	QM_MBOX_PRINTF("Mailbox: TX channel %d -> RX channel %d:\n", mbox_tx,
		       mbox_rx);
#endif

	/* Configure the mailbox for this channel */
	qm_mbox_ch_data_ack(mbox_tx); /* clean DATA status bit on TX */
	/* Register the interrupt handler. */
	qm_irq_request(QM_IRQ_MBOX, qm_mbox_isr);
	qm_mbox_ch_set_config(mbox_rx, mbox_cb, (void *)&mbox_rx,
			      true); /* configure RX channel */

	rx_data.ctrl = 0;
	rx_data.data[0] = 0;
	rx_data.data[1] = 0;
	rx_data.data[2] = 0;
	rx_data.data[3] = 0;

	for (i = 0; i < 10; i++) {
		tx_data.ctrl = ofset + i + 1;
		tx_data.data[0] = ofset + i + 2;
		tx_data.data[1] = ofset + i + 3;
		tx_data.data[2] = ofset + i + 4;
		tx_data.data[3] = ofset + i + 5;

#if (QM_SENSOR)
		if (0 == read_and_verify(mbox_rx, i)) {
		} else {
			QM_MBOX_PRINTF("Error: mbox %d read %d\n", mbox_rx, i);
			return -EIO;
		}
#endif /* Run the test for all test pairs. */

		if (0 != (unsigned int)qm_mbox_ch_write(mbox_tx, &tx_data)) {
			QM_MBOX_PRINTF("Error: mbox %d write %d\n", mbox_tx, i);
			return -EIO;
		}

#if !defined(QM_SENSOR)
		if (0 == read_and_verify(mbox_rx, i)) {
			QM_MBOX_PRINTF("ch-%d loop-%d -> OK;\n", mbox_rx, i);
		} else {
			QM_MBOX_PRINTF("Error: mbox %d read %d\n", mbox_rx, i);
			return -EIO;
		}
#endif
	}

	/* DeConfigure the mailbox channel 0 */
	qm_mbox_ch_set_config(mbox_rx, NULL, NULL, false);

	return 0;
}

int read_and_verify(qm_mbox_ch_t mbox, int loop)
{
	int i;

/* Loop here, waiting mailbox IRQ */
#if DEBUG
	/* In DEBUG build we wait forever.
	 * To allow breakpoints on the other core */
	while (false == mb_got_message[mbox]) {
#else
	int wait_loop = MBOX_READ_TIMEOUT;
	while ((false == mb_got_message[mbox]) && (wait_loop--)) {
#endif
		clk_sys_udelay(DELAY);
	}

	if (true != mb_got_message[mbox]) {
		QM_MBOX_PRINTF("Error: Reading failed on timeout\n");
		return -EIO;
	}
	mb_got_message[mbox] = false;
	/* Verify the Mailbox context */
	if (rx_data.ctrl != tx_data.ctrl) {
		QM_MBOX_PRINTF("Error: Reading failed on ctrl tx-%d, rx-%d\n",
			       (int)tx_data.ctrl, (int)rx_data.ctrl);
		return -EIO;
	}
	for (i = 0; i < 4; i++) {
		if (rx_data.data[i] != tx_data.data[i]) {
			QM_MBOX_PRINTF(
			    "Error: Reading failed on data[%d] tx-%d,"
			    " rx-%d\n",
			    i, (int)tx_data.data[i], (int)rx_data.data[i]);
			return -EIO;
		}
	}
	return 0;
}

void mbox_cb(void *mbox)
{
	qm_mbox_ch_t *ch = (qm_mbox_ch_t *)mbox;
	if (0 != qm_mbox_ch_read(*ch, &rx_data)) {
		QM_MBOX_PRINTF("Error: Reading failed on mbox=%d, ctrl=%d.\n",
			       *ch, (int)rx_data.ctrl);
	}
	mb_got_message[*ch] = true;
}
