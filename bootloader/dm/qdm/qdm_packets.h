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

#ifndef __QDM_PACKETS_H__
#define __QDM_PACKETS_H__

#include <stdint.h>

/*
 * NOTE: different compilers use different directives for packing structs;
 * therefore we define a macro whose value will depend on the specific compiler
 * used.
 * For now, we set it to the GCC directive.
 */
#define __QM_ATTR_PACKED__ __attribute__((__packed__))

/**
 * The enumeration of QDM packet types.
 */
typedef enum {
	/* Requests */
	QDM_SYS_INFO_REQ = 0x444D0000, /**< System Information request. */
	QDM_APP_ERASE = 0x444D0001,    /**< Application Erase request. */
	QDM_UPDATE_KEY = 0x444D0002,   /**< Crypto Key Update request. */
	/* Responses */
	QDM_SYS_INFO_RSP = 0x444D8000, /**< System Information response. */
} qdm_pkt_type_t;

/**
 * The generic structure of a QDM packet.
 */
typedef struct __QM_ATTR_PACKED__ {
	uint32_t type;     /**< The packet type, see qdm_pkt_type_t enum. */
	uint8_t payload[]; /**< The type-specific payload/structure. */
} qdm_generic_pkt_t;

/**
 * Type-specific structure for the QDM System Information response packet.
 */
/*
 * NOTE: this is a first stub to be completed with other fields when
 * defined.
 */
typedef struct __QM_ATTR_PACKED__ {
	uint16_t sysupd_version; /**< The bootloader version. */
	uint8_t soc_type;	/**< The SOC type. */
	uint8_t auth_type;       /**< The active authentication type. */
} qdm_sys_info_rsp_payload_t;

#endif /* __QDM_PACKETS_H__ */
