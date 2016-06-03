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

#include <string.h>

#include "qm_common.h"

#include "qdm_packets.h"
#include "../dfu/dfu.h"
/* qfu_format.h included because of authentication enum (qdm_auth_type_t) */
#include "../qfu/qfu_format.h"

/**
 * The size of the buffer used by the QDM module. It is equal to the size of
 * the biggest QDM packet, which is QDM_SYS_INFO_RSP.
 */
#define QDM_BUF_SIZE                                                           \
	(sizeof(qdm_generic_pkt_t) + sizeof(qdm_sys_info_rsp_payload_t))

/*-----------------------------------------------------------------------*/
/* FORWARD DECLARATIONS                                                  */
/*-----------------------------------------------------------------------*/
static void qdm_init();
static void qdm_get_processing_status(dfu_dev_status_t *status,
				      uint32_t *poll_timeout_ms);
static void qdm_clear_status();
static void qdm_dnl_process_block(uint32_t block_num, const uint8_t *data,
				  uint16_t len);
static int qdm_dnl_finalize_transfer();
static void qdm_upl_fill_block(uint32_t block_num, uint8_t *data,
			       uint16_t max_len, uint16_t *len);
static void qdm_abort_transfer();

/*-----------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                      */
/*-----------------------------------------------------------------------*/
/**
 * QDM request handler variable (used by DFU core when alternate setting zero
 * is selected).
 */
dfu_request_handler_t qdm_dfu_rh = {
    &qdm_init,
    &qdm_get_processing_status,
    &qdm_clear_status,
    &qdm_dnl_process_block,
    &qdm_dnl_finalize_transfer,
    &qdm_upl_fill_block,
    &qdm_abort_transfer,
};

/**
 * The buffer where we store incoming/outgoing QDM packets.
 */
static struct qdm_buf {
	unsigned int len;	   /**< The length of the data in the buffer */
	uint8_t data[QDM_BUF_SIZE]; /**< The actual buffer */
} qdm_buf;

/**
 * The DFU status of this DFU request handler.
 */
static dfu_dev_status_t dfu_status;

/**
 * The remaining time for processing of the latest QDM request. Currently used
 * only for the QDM_APP_ERASE request.
 */
static unsigned int remaining_time_ms;

/*-----------------------------------------------------------------------*/
/* STATIC FUNCTIONS (QDM functions)                                      */
/*-----------------------------------------------------------------------*/
/**
 * Prepare a QDM System Information response (QDM_SYS_INFO_RSP) packet.
 *
 * This function is called when a QDM System Information request
 * (QDM_SYS_INFO_REQ) is received.
 */
static void prepare_sys_info_rsp()
{
	qdm_generic_pkt_t *pkt;
	qdm_sys_info_rsp_payload_t *pl;

	/* NOTE: optimization: use a pre-compiled response packet */
	pkt = (qdm_generic_pkt_t *)qdm_buf.data;
	pkt->type = QDM_SYS_INFO_RSP;
	pl = (qdm_sys_info_rsp_payload_t *)pkt->payload;
	pl->sysupd_version = 0x0100;
	pl->soc_type = 0x00;
	pl->auth_type = QFU_AUTH_NONE;

	qdm_buf.len = sizeof(*pkt) + sizeof(*pl);
}

/**
 * Application Erase.
 *
 * Erase all application code from the flash. The function is expected to run
 * in parallel with the DFU state machine.
 */
static void app_erase()
{
	/* NOTE: we should set remaining_time_ms to the expected completion
	 * time for the application erase and update it periodically with the
	 * expected remaining time; the value is used to set the host
	 * poll_timeout */
	remaining_time_ms = 100;
	/* NOTE: to do: implement application erase; if we fail, we should set
	 * dfu_status to the proper error status */
	remaining_time_ms = 0;
}

/**
 * Parse and process the incoming QDM request.
 *
 * This function is called every time a DFU_DNLOAD transfer is finalized (since
 * in QDM mode DFU_DNLOAD transfers carry QDM requests).
 * This function has no parameter since the request is expected to be in the
 * QDM buffer (qdm_buf global variable).
 */
static void process_qdm_req()
{
	qdm_generic_pkt_t *pkt;

	pkt = (qdm_generic_pkt_t *)qdm_buf.data;
	switch (pkt->type) {
	case QDM_SYS_INFO_REQ:
		prepare_sys_info_rsp();
		break;
	case QDM_APP_ERASE:
		/* NOTE: fix me: call app_erase() in a different thread */
		app_erase();
		/* No QDM response for this request; set len to zero */
		qdm_buf.len = 0;
		break;
	case QDM_UPDATE_KEY:
	/* Not yet supported, handle it as an invalid message */
	default:
		dfu_status = DFU_STATUS_ERR_VENDOR;
		/* No QDM response in this case; set len to zero */
		qdm_buf.len = 0;
	}
}

/*-----------------------------------------------------------------------*/
/* STATIC FUNCTIONS (DFU Request Handler implementation)                 */
/*-----------------------------------------------------------------------*/

/**
 * Initialize the DFU Request Handler.
 *
 * This function must be called when a DFU alt setting associated with this
 * handler is selected.
 */
static void qdm_init()
{
	dfu_status = DFU_STATUS_OK;
}

/**
 * Get the status and state of the handler.
 *
 * @param[out] status The status of the processing (OK or error code)
 * @param[out] poll_timeout The poll_timeout to be returned to the host.
 * 			     Must be zero if the processing is over; otherwise
 * 			     it	should be the expected remaining time.
 */
static void qdm_get_processing_status(dfu_dev_status_t *status,
				      uint32_t *poll_timeout_ms)
{
	*status = dfu_status;
	*poll_timeout_ms = remaining_time_ms;
}

/**
 * Clear the status and state of the handler.
 *
 * This function is used to reset the handler state machine after an error. It
 * is called by DFU core when a DFU_CLRSTATUS request is recevied.
 */
static void qdm_clear_status()
{
	dfu_status = DFU_STATUS_OK;
	qdm_buf.len = 0;
}

/**
 * Process a DFU_DNLOAD block.
 *
 * @param[in] block_num The block number.
 * @param[in] data      The buffer containing the block data.
 * @param[in] len       The length of the block.
 */
static void qdm_dnl_process_block(uint32_t block_num, const uint8_t *data,
				  uint16_t len)
{
	if (block_num == 0) {
		qdm_buf.len = 0;
	}
	if (len > (sizeof(qdm_buf.data) - qdm_buf.len)) {
		dfu_status = DFU_STATUS_ERR_VENDOR;
		return;
	}
	memcpy(&qdm_buf.data[qdm_buf.len], data, len);
	qdm_buf.len += len;
}

/**
 * Finalize the current DFU_DNLOAD transfer.
 *
 * This function is called by DFU Core when an empty DFU_DNLOAD request
 * (signaling the end of the current DFU_DNLOAD transfer) is received.
 *
 * @return QMSI return code.
 * @retval 0    if handler agrees with the end of the transfer.
 * @retval -EIO if handler was actually expecting more data.
 */
static int qdm_dnl_finalize_transfer()
{
	process_qdm_req();

	return 0;
}

/**
 * Fill up a DFU_UPLOAD block.
 *
 * This function is called by the DFU logic when a request for an
 * UPLOAD block is received. The handler is in charge of filling the
 * payload of the block.
 *
 * When QDM mode (i.e., alternate setting 0) is active, the host sends a
 * DFU_UPLOAD request to retrieve the response to the QDM Request previously
 * sent in DFU_DNLOAD transfer. Note, however, that not every QDM request
 * produces a QDM response (for instance, the QDM Application Erase request).
 *
 * @param[in]  blk_num The block number (first block is always block 0).
 * @param[out] data    The buffer where the payload will be put.
 * @param[in]  req_len The amount of data requested by the host.
 * @param[out] len     A pointer to the variable where to store the actual
 * 		       amount of data provided by the handler (len < req_len
 * 		       means that there is no more data to send, i.e., this
 * 		       is the last block).
 */
static void qdm_upl_fill_block(uint32_t blk_num, uint8_t *data,
			       uint16_t req_len, uint16_t *len)
{
	/*
	 * For the sake of code-size minimization, we require the host to use a
	 * block size (i.e., req_len) greater than the response length.  In
	 * other words, the response must fit in a single UPLOAD block.  This
	 * is not a huge limitation, since there is no reason for the host to
	 * use a block size smaller than the device's maximum block size
	 * (typically a few kB).
	 */
	if ((qdm_buf.len > 0) && (req_len >= qdm_buf.len)) {
		memcpy(data, qdm_buf.data, qdm_buf.len);
		*len = qdm_buf.len;
	} else {
		/* We upload nothing */
		*len = 0;
	}
}

/**
 * Abort current DNLOAD/UPLOAD transfer and go back to handler's initial state.
 *
 * This function is called by DFU core when a DFU_ABORT request is received.
 */
static void qdm_abort_transfer()
{
	qdm_buf.len = 0;
}
