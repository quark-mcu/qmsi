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

#ifndef __QM_AON_COUNTERS_H__
#define __QM_AON_COUNTERS_H__

#include "qm_common.h"
#include "qm_soc_regs.h"

/**
 * Always-on Counters.
 *
 * @note The always on counters are in the 32kHz clock domain. Some register
 * operations take a minimum of a 32kHz clock cycle to complete. If the Always
 * on timer interrupt is not configured to be edge triggered, multiple
 * interrupts will occur.
 *
 * @defgroup groupAONC Always-on Counters
 * @{
 */

/**
 * Always on counter status.
 */
typedef enum {
	/**
	 * Default Timer Status
	 */
	QM_AONPT_READY = 0,
	/**
	 * Timer expired. Status must be cleared with qm_aonpt_clear().
	 */
	QM_AONPT_EXPIRED,
} qm_aonpt_status_t;

/**
 * QM Always-on Periodic Timer configuration type.
 */
typedef struct {
	uint32_t count; /**< Time to count down from in clock cycles.*/
	bool int_en;    /**< Enable/disable the interrupts. */

	/**
	 * User callback.
	 *
	 * @param[in] data User defined data.
	 */
	void (*callback)(void *data);
	void *callback_data; /**< Callback data. */
} qm_aonpt_config_t;

/**
 * Enable the Always-on Counter.
 *
 * @param[in] aonc Always-on counter to read.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
*/
int qm_aonc_enable(const qm_aonc_t aonc);

/**
 * Disable the Always-on Counter.
 *
 * @param[in] aonc Always-on counter to read.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonc_disable(const qm_aonc_t aonc);

/**
 * Get the current value of the Always-on Counter.
 *
 * Returns a 32-bit value which represents the number of clock cycles
 * since the counter was first enabled.
 *
 * @param[in] aonc Always-on counter to read.
 * @param[out] val Value of the counter. This must not be NULL.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonc_get_value(const qm_aonc_t aonc, uint32_t *const val);

/**
 * Set the Always-on Periodic Timer configuration.
 *
 * This includes the initial value of the Always-on Periodic Timer,
 * the interrupt enable and the callback function that will be run
 * when the timer expiers and an interrupt is triggered.
 * The Periodic Timer is disabled if the counter is set to 0.
 *
 * @param[in] aonc Always-on counter to read.
 * @param[in] cfg New configuration for the Always-on Periodic Timer.
 * This must not be NULL.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonpt_set_config(const qm_aonc_t aonc,
			const qm_aonpt_config_t *const cfg);

/**
 * Get the current value of the Always-on Periodic Timer.
 *
 * Returns a 32-bit value which represents the number of clock cycles
 * remaining before the timer fires.
 * This is the initial configured number minus the number of cycles that have
 * passed.
 *
 * @param[in] aonc Always-on counter to read.
 * @param[out] val Value of the Always-on Periodic Timer.
 * This must not be NULL.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonpt_get_value(const qm_aonc_t aonc, uint32_t *const val);

/**
 * Get the current status of an Always-on Periodic Timer.
 *
 * @param[in] aonc Always-on counter to read.
 * @param[out] status Status of the Always-on Periodic Timer.
 * This must not be NULL.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonpt_get_status(const qm_aonc_t aonc, qm_aonpt_status_t *const status);

/**
 * Clear the status of the Always-on Periodic Timer.
 *
 * The status must be clear before the Always-on Periodic Timer can trigger
 * another interrupt.
 *
 * @param[in] aonc Always-on counter to read.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonpt_clear(const qm_aonc_t aonc);

/**
 * Reset the Always-on Periodic Timer back to the configured value.
 *
 * @param[in] aonc Always-on counter to read.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonpt_reset(const qm_aonc_t aonc);

/**
 * Save the Always-on Periodic Timer context.
 *
 * Save the configuration of the specified AONC peripheral
 * before entering sleep.
 *
 * @param[in] aonc AONC index.
 * @param[out] ctx AONC context structure. This must not be NULL.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonpt_save_context(const qm_aonc_t aonc, qm_aonc_context_t *const ctx);

/**
 * Restore the Always-on Periodic Timer context.
 *
 * Restore the configuration of the specified AONC peripheral
 * after exiting sleep.
 *
 * @param[in] aonc AONC index.
 * @param[in] ctx AONC context structure. This must not be NULL.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int qm_aonpt_restore_context(const qm_aonc_t aonc,
			     const qm_aonc_context_t *const ctx);

/**
 * @}
 */

#endif /* __QM_AON_COUNTERS_H__ */
