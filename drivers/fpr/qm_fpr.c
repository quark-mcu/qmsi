/*
 * Copyright (c) 2017, Intel Corporation
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

#include "qm_fpr.h"
#include "qm_interrupt.h"
#include "qm_interrupt_router.h"

static void (*callback[QM_FLASH_NUM])(void *);
static void *callback_data[QM_FLASH_NUM];

QM_ISR_DECLARE(qm_flash_mpr_0_isr)
{
	if (callback[QM_FLASH_0]) {
		(*callback[QM_FLASH_0])(callback_data[QM_FLASH_0]);
	}
	QM_FLASH[QM_FLASH_0]->mpr_vsts = QM_FPR_MPR_VSTS_VALID;

	QM_ISR_EOI(QM_IRQ_FLASH_MPR_0_INT_VECTOR);
}

#if (QUARK_SE)
QM_ISR_DECLARE(qm_flash_mpr_1_isr)
{
	if (callback[QM_FLASH_1]) {
		(*callback[QM_FLASH_1])(callback_data[QM_FLASH_1]);
	}
	QM_FLASH[QM_FLASH_1]->mpr_vsts = QM_FPR_MPR_VSTS_VALID;

	QM_ISR_EOI(QM_IRQ_FLASH_MPR_1_INT_VECTOR);
}
#endif

int qm_fpr_set_config(const qm_flash_t flash, const qm_fpr_id_t id,
		      const qm_fpr_config_t *const cfg,
		      const qm_flash_region_type_t region)
{
	QM_CHECK(flash < QM_FLASH_NUM, -EINVAL);
	QM_CHECK(id < QM_FPR_NUM, -EINVAL);
	QM_CHECK(region < QM_MAIN_FLASH_NUM, -EINVAL);
	QM_CHECK(cfg != NULL, -EINVAL);
	QM_CHECK(cfg->low_bound <= cfg->up_bound, -EINVAL);

	qm_flash_reg_t *const controller = QM_FLASH[flash];

	controller->fpr_rd_cfg[id] &= ~QM_FPR_LOCK;

	if (region == QM_MAIN_FLASH_SYSTEM) {
		controller->fpr_rd_cfg[id] =
		    (cfg->allow_agents << QM_FPR_RD_ALLOW_OFFSET) |
		    ((cfg->up_bound + QM_FLASH_REGION_DATA_BASE_OFFSET)
		     << QM_FPR_UPPER_BOUND_OFFSET) |
		    (cfg->low_bound + QM_FLASH_REGION_DATA_BASE_OFFSET);
	}
#if (QUARK_D2000)
	else if (region == QM_MAIN_FLASH_DATA) {
		controller->fpr_rd_cfg[id] =
		    (cfg->allow_agents << QM_FPR_RD_ALLOW_OFFSET) |
		    (cfg->up_bound << QM_FPR_UPPER_BOUND_OFFSET) |
		    cfg->low_bound;
	}
#endif
	/* qm_fpr_id_t enable/lock */
	controller->fpr_rd_cfg[id] |= (cfg->en_mask << QM_FPR_ENABLE_OFFSET);

	return 0;
}

#if (QM_SENSOR)
int qm_fpr_set_violation_policy(const qm_fpr_viol_mode_t mode,
				const qm_flash_t flash,
				qm_fpr_callback_t callback_fn, void *data)
{
	QM_CHECK(mode <= FPR_VIOL_MODE_PROBE, -EINVAL);
	QM_CHECK(flash < QM_FLASH_NUM, -EINVAL);
	volatile uint32_t *int_flash_controller_mask =
	    &QM_INTERRUPT_ROUTER->flash_mpr_0_int_mask;

	/* interrupt mode */
	if (FPR_VIOL_MODE_INTERRUPT == mode) {

		callback[flash] = callback_fn;
		callback_data[flash] = data;

		QM_IR_UNMASK_INTERRUPTS(int_flash_controller_mask[flash]);

		QM_IR_MASK_HALTS(int_flash_controller_mask[flash]);

		QM_SCSS_SS->ss_cfg &= ~QM_SS_STS_HALT_INTERRUPT_REDIRECTION;
	}

	/* probe or reset mode */
	else {
		QM_IR_MASK_INTERRUPTS(int_flash_controller_mask[flash]);

		QM_IR_UNMASK_HALTS(int_flash_controller_mask[flash]);

		if (FPR_VIOL_MODE_PROBE == mode) {

			/* When an enabled host halt interrupt occurs, this bit
			* determines if the interrupt event triggers a warm
			* reset
			* or an entry into Probe Mode.
			* 0b : Warm Reset
			* 1b : Probe Mode Entry
			*/
			QM_SCSS_SS->ss_cfg |=
			    QM_SS_STS_HALT_INTERRUPT_REDIRECTION;
		} else {
			QM_SCSS_SS->ss_cfg &=
			    ~QM_SS_STS_HALT_INTERRUPT_REDIRECTION;
		}
	}
	return 0;
}

#else /* QM_SENSOR */

int qm_fpr_set_violation_policy(const qm_fpr_viol_mode_t mode,
				const qm_flash_t flash,
				qm_fpr_callback_t callback_fn, void *data)
{
	QM_CHECK(mode <= FPR_VIOL_MODE_PROBE, -EINVAL);
	QM_CHECK(flash < QM_FLASH_NUM, -EINVAL);
	volatile uint32_t *int_flash_controller_mask =
	    &QM_INTERRUPT_ROUTER->flash_mpr_0_int_mask;

	/* interrupt mode */
	if (FPR_VIOL_MODE_INTERRUPT == mode) {

		callback[flash] = callback_fn;
		callback_data[flash] = data;

		/* unmask interrupt */
		if (flash == QM_FLASH_0) {
			QM_IR_UNMASK_INT(QM_IRQ_FLASH_MPR_0_INT);
#if (QUARK_SE)
		} else {
			QM_IR_UNMASK_INT(QM_IRQ_FLASH_MPR_1_INT);
#endif
		}

		QM_IR_MASK_HALTS(int_flash_controller_mask[flash]);

		QM_SCSS_PMU->p_sts &= ~QM_P_STS_HALT_INTERRUPT_REDIRECTION;
	}

	/* probe or reset mode */
	else {
		/* mask interrupt */
		if (flash == QM_FLASH_0) {
			QM_IR_MASK_INT(QM_IRQ_FLASH_MPR_0_INT);
#if (QUARK_SE)
		} else {
			QM_IR_MASK_INT(QM_IRQ_FLASH_MPR_1_INT);
#endif
		}

		QM_IR_UNMASK_HALTS(int_flash_controller_mask[flash]);

		if (FPR_VIOL_MODE_PROBE == mode) {

			/* When an enabled host halt interrupt occurs, this bit
			* determines if the interrupt event triggers a warm
			* reset
			* or an entry into Probe Mode.
			* 0b : Warm Reset
			* 1b : Probe Mode Entry
			*/
			QM_SCSS_PMU->p_sts |=
			    QM_P_STS_HALT_INTERRUPT_REDIRECTION;
		} else {
			QM_SCSS_PMU->p_sts &=
			    ~QM_P_STS_HALT_INTERRUPT_REDIRECTION;
		}
	}
	return 0;
}
#endif /* QM_SENSOR */

#if (ENABLE_RESTORE_CONTEXT)
int qm_fpr_save_context(const qm_flash_t flash, qm_fpr_context_t *const ctx)
{
	QM_CHECK(flash < QM_FLASH_NUM, -EINVAL);
	QM_CHECK(ctx != NULL, -EINVAL);
	uint8_t i;

	qm_flash_reg_t *const controller = QM_FLASH[flash];

	for (i = 0; i < QM_FPR_NUM; i++) {
		ctx->fpr_rd_cfg[i] = controller->fpr_rd_cfg[i];
	}

	return 0;
}

int qm_fpr_restore_context(const qm_flash_t flash,
			   const qm_fpr_context_t *const ctx)
{
	QM_CHECK(flash < QM_FLASH_NUM, -EINVAL);
	QM_CHECK(ctx != NULL, -EINVAL);
	uint8_t i;

	qm_flash_reg_t *const controller = QM_FLASH[flash];

	for (i = 0; i < QM_FPR_NUM; i++) {
		controller->fpr_rd_cfg[i] = ctx->fpr_rd_cfg[i];
	}

	return 0;
}
#else
int qm_fpr_save_context(const qm_flash_t flash, qm_fpr_context_t *const ctx)
{
	(void)flash;
	(void)ctx;

	return 0;
}

int qm_fpr_restore_context(const qm_flash_t flash,
			   const qm_fpr_context_t *const ctx)
{
	(void)flash;
	(void)ctx;

	return 0;
}
#endif
