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
#include "qm_mpr.h"
#include "qm_interrupt.h"
#include "qm_common.h"
#include "qm_isr.h"
#include "clk.h"

extern char __heap;

static void mpr_example_callback(void *);
static volatile bool callback_invoked = false;

#define MPR_PAGE_SIZE (0x400)

#if (QUARK_D2000)
#define SRAM_BASE (0x00280000)
#elif(QUARK_SE)
#define SRAM_BASE (0xA8000000)
#endif

/* QMSI MPR sample application: this example uses an MPR to pretect a 1kB page
   of SRAM, and then triggers an MPR violation interrupt. */
int main(void)
{
	qm_mpr_config_t cfg;
	uint8_t lower_bound;
	uint32_t heap_offset, mpr_base;

	QM_PRINTF("Starting: MPR\n");

	/* we're going to put this MPR in the heap, to ensure it doesn't clash
	 * with anything else, so we need to figure out the page number that
	 * corresponds to the start of the heap. It is important to note that
	 * the stack lives at the other end of the heap, and grows down towards
	 * heap_offset. In a small application like this there is no danger of
	 * the stack growing into our MPR, but it is something to keep in mind
	 * when enabling MPRs on the heap. */
	heap_offset = (uint32_t)&__heap;
	lower_bound = (uint8_t)((heap_offset - SRAM_BASE) / MPR_PAGE_SIZE) + 1;

	/* get the physical address of the start of the MPR */
	mpr_base = SRAM_BASE + (lower_bound * MPR_PAGE_SIZE);

	/* Set the violation policy to trigger an interrupt */
	qm_irq_request(QM_IRQ_SRAM, qm_mpr_isr);
	qm_mpr_set_violation_policy(MPR_VIOL_MODE_INTERRUPT,
				    mpr_example_callback, NULL);

	/* Configure MPR to allow R/W from DMA agent only */
	cfg.en_lock_mask = QM_SRAM_MPR_EN_MASK_ENABLE;
	cfg.agent_read_en_mask = QM_SRAM_MPR_AGENT_MASK_DMA;
	cfg.agent_write_en_mask = QM_SRAM_MPR_AGENT_MASK_DMA;
	cfg.up_bound = lower_bound;
	cfg.low_bound = lower_bound;

	qm_mpr_set_config(QM_MPR_0, &cfg);

	/* trigger a violation event by attempting a write inside the MPR */
	REG_VAL(mpr_base + 1) = 0xff;

	while (false == callback_invoked) {
	}

	QM_PRINTF("MPR Violation!\n");
	QM_PRINTF("Finished: MPR\n");
	return 0;
}

static void mpr_example_callback(void *data)
{
	callback_invoked = true;
}
