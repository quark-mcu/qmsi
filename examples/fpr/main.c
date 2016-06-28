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
#include "qm_fpr.h"
#include "qm_interrupt.h"
#include "qm_common.h"
#include "qm_isr.h"

#define FPR_SIZE (0x400)

#if (QUARK_D2000)
#define FLASH_END (0x00200000)
#elif(QUARK_SE)
#define FLASH_END (0x40060000)
#endif

static volatile bool callback_invoked = false;

static void fpr_example_cb(void *data)
{
	callback_invoked = true;
	QM_PUTS("FPR Violation!");
}

/**
 * QMSI FPR sample application: this example uses an FPR to lock read
 * on the FLASH and then try to read. Before, the violation policy is
 * configured with the example callback.
 */

extern uint32_t __data_lma[];
extern uint32_t __data_size[];

int main(void)
{
	uint32_t flash_base;
	uint32_t fpr_flash;
	uint32_t value;
	uint32_t app_end;
	uint32_t address;
	uint8_t low_bound;
	qm_fpr_config_t cfg = {0};

#if (QUARK_D2000)
	flash_base = QM_FLASH_REGION_SYS_0_BASE;
	fpr_flash = QM_FLASH_0;
#elif(QUARK_SE)
	flash_base = QM_FLASH_REGION_SYS_1_BASE;
	fpr_flash = QM_FLASH_1;
#endif

	QM_PRINTF("Starting: FPR\n");

	/* Calculate how much space the application code occupies, so we can
	 * ensure our FPR does not overlap with it */
	app_end = (uint32_t)__data_lma + (uint32_t)__data_size;

	if ((app_end + FPR_SIZE + 1) > FLASH_END) {
		QM_PRINTF("No free pages. Quitting.\n");
		return 0;
	}

	/* Calculate 1k-aligned physical flash address for FPR start */
	low_bound = ((app_end - flash_base) / FPR_SIZE) + 1;

	/* Calculate MMIO address of a location inside the FPR */
	address = (flash_base + (FPR_SIZE * low_bound)) + 4;

/* Set the violation policy to trigger an interrupt */
#if (QUARK_D2000)
	qm_irq_request(QM_IRQ_FLASH_0, qm_fpr_isr_0);
#elif(QUARK_SE)
	qm_irq_request(QM_IRQ_FLASH_1, qm_fpr_isr_1);
#endif

	qm_fpr_set_violation_policy(FPR_VIOL_MODE_INTERRUPT, fpr_flash,
				    fpr_example_cb, NULL);

	/* Configure MPR to allow R/W from DMA agent only */
	cfg.en_mask = QM_FPR_ENABLE;
	cfg.allow_agents = QM_FPR_DMA;
	cfg.up_bound = low_bound + 1;
	cfg.low_bound = low_bound;

	qm_fpr_set_config(fpr_flash, QM_FPR_0, &cfg, QM_MAIN_FLASH_SYSTEM);

	/* Trigger a violation event by attempting to read in the FLASH */
	value = REG_VAL(address);

	while (false == callback_invoked) {
	}
	QM_PRINTF("Finished: FPR\n");
	value = value;
	return 0;
}
