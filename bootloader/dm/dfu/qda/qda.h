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

#ifndef __QDA_H__
#define __QDA_H__

#include <stddef.h>

#include "../dfu.h"

/**
 * Quark DFU Adaptation (QDA) layer.
 *
 * @defgroup groupQDA Quark DFU Adaptation (QDA)
 * @{
 */

/**
 * Initialize QDA module.
 *
 * Initialize the Quark DFU Adaptation (QDA) module.
 *
 * @param[in] send_func The function to be used by the module to send packet.
 * @param[in] cfg	The DFU configuration.
 */
void qda_init(int (*send_func)(uint8_t *pkt, size_t len), const dfu_cfg_t *cfg);

/**
 * Process QDA packet.
 *
 * Parse, process, and reply to an incoming QDA packet.
 *
 * @param[in] data The buffer containing the packet.
 * @param[in] len  The length of packet or its upper bound (since XMODEM may
 * 		   add some garbage bytes).
 */
void qda_process_pkt(const uint8_t *data, size_t len);

/**
 * @}
 */

#endif /* __QDA_H__ */
