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

#include <string.h>

#include "qm_common.h"

#include "qfu_format.h"
#include "../dfu/dfu.h"

/* NOTE: move to the bootloader config file when such a file is defined */
#define DFU_BLOCK_SIZE_MAX (2048)

/*-----------------------------------------------------------------------*/
/* FORWARD DECLARATIONS                                                  */
/*-----------------------------------------------------------------------*/
static void qfu_init();
static void qfu_get_status(dfu_dev_status_t *status, uint32_t *poll_timeout_ms);
static void qfu_clear_status();
static void qfu_dnl_process_block(uint32_t block_num, const uint8_t *data,
				  uint16_t len);
static int qfu_dnl_finalize_transfer();
static void qfu_upl_fill_block(uint32_t block_num, uint8_t *data,
			       uint16_t max_len, uint16_t *len);
static void qfu_abort_transfer();

/*-----------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                      */
/*-----------------------------------------------------------------------*/
/**
 * QFU request handler variable (used by DFU core when an alternate setting
 * different from 0 is selected).
 */
dfu_request_handler_t qfu_dfu_rh = {
    &qfu_init,
    &qfu_get_status,
    &qfu_clear_status,
    &qfu_dnl_process_block,
    &qfu_dnl_finalize_transfer,
    &qfu_upl_fill_block,
    &qfu_abort_transfer,
};

/**
 * The QFU block buffer.
 */
static struct blk_buf {
	unsigned int len;		  /**< The length of the block */
	uint8_t data[DFU_BLOCK_SIZE_MAX]; /**< The actual buffer */
} qfu_block;

/**
 * The DFU status of this DFU request handler.
 */
static dfu_dev_status_t qfu_err_status;

/*-----------------------------------------------------------------------*/
/* STATIC FUNCTIONS (DFU Request Handler implementation)                 */
/*-----------------------------------------------------------------------*/

/**
 * Initialize the DFU Request Handler.
 *
 * This function is called when a DFU alt setting associated with this handler
 * is selected.
 */
static void qfu_init()
{
	qfu_err_status = DFU_STATUS_OK;
	qfu_block.len = 0;
}

/**
 * Get the status and state of the handler.
 *
 * NOTE: in case of the QFU handler, this function triggers the writing into the
 * flash of the QFU block previously transferred in a DNLOAD request (if any).
 * The pool timeout returned is the expected time to write the block.
 *
 * @param[out] status The status of the processing (OK or error code)
 * @param[out] poll_timeout_ms The poll_timeout to be returned to the host.
 * 			       Must be zero if the processing is over; otherwise
 * 			       it should be the expected remaining time.
 */
static void qfu_get_status(dfu_dev_status_t *status, uint32_t *poll_timeout_ms)
{
	/* NOTE: stub; just return 'done' (poll_timeout_ms = 0) for now */
	*status = qfu_err_status;
	*poll_timeout_ms = 0;

	qfu_block.len = 0;
}

/**
 * Clear the status and state of the handler.
 *
 * This function is used to reset the handler state machine after an error. It
 * is called by DFU core when a DFU_CLRSTATUS request is received.
 */
static void qfu_clear_status()
{
	qfu_err_status = DFU_STATUS_OK;
	qfu_block.len = 0;
}

/**
 * Process a DFU_DNLOAD block.
 *
 * @param[in] block_num The block number.
 * @param[in] data      The buffer containing the block data.
 * @param[in] len       The length of the block.
 */
static void qfu_dnl_process_block(uint32_t block_num, const uint8_t *data,
				  uint16_t len)
{
	/* NOTE: stub; for now we just copy the block into the QFU buffer */
	memcpy(qfu_block.data, data, len);
	qfu_block.len = len;
}

/**
 * Finalize the current DFU_DNLOAD transfer.
 *
 * This function is called by DFU Core when an empty DFU_DNLOAD request
 * (signaling the end of the current DFU_DNLOAD transfer) is received.
 *
 * NOTE: in case of the QFU handler, this is where bootloader data (e.g.,
 * application version, SVN, image selector, etc.) get updated with information
 * about the new application firmware.
 *
 * @return O on success, negative errno otherwise.
 */
static int qfu_dnl_finalize_transfer()
{
	/* NOTE: stub */

	return 0;
}

/**
 * Fill up a DFU_UPLOAD block.
 *
 * This function is called by the DFU logic when a request for an
 * UPLOAD block is received. The handler is in charge of filling the
 * payload of the block.
 *
 * NOTE: when the QFU handler is active (i.e., the selected alternate setting
 * is different from 0), DFU_UPLOAD requests are not allowed and therefore an
 * empty payload is always returned.
 *
 * @param[in]  blk_num The block number (first block is always block 0).
 * @param[out] data    The buffer where the payload will be put.
 * @param[in]  req_len The amount of data requested by the host.
 * @param[out] len     A pointer to the variable where to store the actual
 * 		       amount of data provided by the handler (len < req_len
 * 		       means that there is no more data to send, i.e., this
 * 		       is the last block).
 */
static void qfu_upl_fill_block(uint32_t blk_num, uint8_t *data,
			       uint16_t req_len, uint16_t *len)
{
	/* Firmware extraction is not allowed: upload nothing */
	*len = 0;
}

/**
 * Abort current DNLOAD/UPLOAD transfer and go back to handler's initial state.
 *
 * This function is called by DFU core when a DFU_ABORT request is received.
 */
static void qfu_abort_transfer()
{
	qfu_block.len = 0;
}
