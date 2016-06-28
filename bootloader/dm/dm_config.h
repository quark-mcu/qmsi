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

#ifndef __DM_CONFIG_H__
#define __DM_CONFIG_H__

/** Dual-bank configuration flag. */
/* NOTE: consider moving this to a bl_config.h file or rename this file */
#define BL_CONFIG_DUAL_BANK (false)

/*
 * DM UART comm parameters.
 */

#if (QUARK_SE)
#define DM_CONFIG_UART (1)
#elif(QUARK_D2000)
#define DM_CONFIG_UART (0)
#endif
#define DM_CONFIG_UART_BAUD_DIV (BOOTROM_UART_115200)

/*
 * Low-Power Comparator (LPC) for DM requests.
 */

/* NOTE: the LPC selection is yet to be agreed, the below may change. */

#if (QUARK_SE)
/* Mount Atlas pin J14.47*/
#define DM_CONFIG_LPC (7)
#define DM_CONFIG_LPC_PIN_ID (QM_PIN_ID_7)
#define DM_CONFIG_LPC_PIN_FN (QM_PMUX_FN_1)
#elif(QUARK_D2000)
/* Intel(R) Quark(TM) Microcontroller D2000 Development Platform pin IO8
 * (J4.8). */
#define DM_CONFIG_LPC (9)
#define DM_CONFIG_LPC_PIN_ID (QM_PIN_ID_9)
#define DM_CONFIG_LPC_PIN_FN (QM_PMUX_FN_1)
#endif

#endif /* __DM_CONFIG_H__ */
