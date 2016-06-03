/**
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

#ifndef __XMODEM_H__
#define __XMODEM_H__

#include "qm_common.h"

/**
 * @defgroup groupXMODEM XMODEM
 * @{
 *
 **/

/**
 * Switch XMODEM to receive mode.
 *
 * XMODEM start to send 'C' (NAK-CRC) messages to the sender and waits for
 * incoming transmissions.
 * The received data is copied into the package pointer. If more data are
 * received then the len, an error is returned. The final data length is
 * returned on success.
 *
 * The transmission is completed or terminated as soon as the function returns.
 *
 * @param[in] package Data pointer of the received package.
 * @param[in] len Maximum allowed package length.
 *
 * @return Number of Received bytes or -1.
 * @retval -1 Error
 * @retval >0 Receive successful.
 */
int xmodem_receive_package(uint8_t *package, size_t len);

/**
 * Switch XMODEM to transmit mode.
 *
 * XMODEM waits for 'C' (NAK-CRC) messages until the transmission begins.
 * The package content is sent in 128 bytes frames. The transmission is
 * completed or terminated as soon as the function returns. Extra data is added
 * to the last frame if the package size is not aligned to 128 bytes.
 *
 * @param[in] package Data pointer of the package to send.
 * @param[in] len Length of package to send.
 *
 * @return Actual written bytes or -1.
 * @retval -1 Transmission failed.
 * @retval >0 Transmission successful.
 */
int xmodem_transmit_package(uint8_t *package, size_t len);

/** @} */

#endif
