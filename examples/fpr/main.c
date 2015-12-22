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
#include "qm_fpr.h"
#include "qm_interrupt.h"
#include "qm_common.h"

static void fpr_example_cb()
{
	QM_PUTS("FPR Violation!");
}

/**
 * QMSI FPR sample application: this example uses an FPR to lock read
 * on the FLASH and the try read. Before, the violation policy is configured
 * with the example callback.
 */
int main(void)
{
	uint32_t value;
	qm_fpr_config_t cfg;
	/**
	 * Lower and up bound must be lower than 36
	 */
	uint8_t lower_bound = 0x1A;
	int address =
	    QM_FLASH_REGION_SYS_0_BASE + ((uint32_t)lower_bound * 0x400) + 1;

	/* Set the violation policy to trigger an interrupt */
	qm_irq_request(QM_IRQ_FLASH_0, qm_fpr_isr_0);
	qm_fpr_set_violation_policy(FPR_VIOL_MODE_INTERRUPT, QM_FLASH_0,
				    fpr_example_cb);

	/* Configure MPR to allow R/W from DMA agent that will lock host agent
	 */
	cfg.en_mask = QM_FPR_ENABLE;
	cfg.allow_agents = QM_FPR_DMA;
	cfg.up_bound = lower_bound;
	cfg.low_bound = lower_bound;

	qm_fpr_set_config(QM_FLASH_0, QM_FPR_0, &cfg, QM_MAIN_FLASH_SYSTEM);

	/**
	 * Trigger a violation event by attempting to read in the FLASH
	 */
	value = REG_VAL(address);
	QM_PUTS("End of sample application");
	value = value;
	return 0;
}
