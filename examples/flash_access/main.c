/*
 * Copyright (c) 2015, Intel Corporation
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

#include "qm_flash.h"

/*
 * The flash controller segments its memory into pages of 2KB in size
 * (multiples of 0x800). Writes don't cross over to other pages.
 *
 * For Quark Microcontroller D2000, there is 1 flash controller.
 * Controller 0:
 * |  Component                 | Size          | System start address
 * |  System ROM		| 8KB	 	| 0x00000000
 * |  System Flash		| 32KB		| 0x00180000
 * |  Data region		| 4KB		| 0x00200000
 *
 * For Quark SE SOC, there are 2 flash controllers.
 * Controller 0:
 * |  Component                 | Size          | System start address
 * |  System Flash		| 192KB		| 0x40000000
 * |  System ROM		| 8KB           | 0xFFFFE000
 *
 * Controller 1:
 * |  Component                 | Size          | System start address
 * |  System Flash		| 192KB		| 0x40030000
 */

#define NUM_DATA_WORDS (0x03)
#define WR_FLASH_ADDR (0x101C)
#define PAGE_TO_ERASE (0x02)
#define PAGE_TO_WRITE (0x02)
#define MASS_ERASE_INCLUDE_ROM (0x00)
#define US_COUNT (0x20)
#define WAIT_STATES (0x01)

/* This buffer must be at least QM_FLASH_PAGE_SIZE. Practically, this buffer */
/* may be shared with other buffers to save space. */
static uint32_t flash_page_buffer[QM_FLASH_PAGE_SIZE];

/* QMSI flash access app example */
int main(void)
{
	qm_flash_config_t cfg_wr, cfg_rd;
	uint32_t flash_data[NUM_DATA_WORDS] = {0x00010203, 0x04050607,
					       0x08090A0B};

	cfg_wr.us_count = US_COUNT;
	cfg_wr.wait_states = WAIT_STATES;
	cfg_wr.write_disable = QM_FLASH_WRITE_ENABLE;

	qm_flash_set_config(QM_FLASH_0, &cfg_wr);
	qm_flash_get_config(QM_FLASH_0, &cfg_rd);

	/* Requires a 2KB buffer to store flash page. */
	qm_flash_page_update(QM_FLASH_0, QM_FLASH_REGION_SYS, WR_FLASH_ADDR,
			     flash_page_buffer, flash_data, NUM_DATA_WORDS);

	qm_flash_page_erase(QM_FLASH_0, QM_FLASH_REGION_SYS, PAGE_TO_ERASE);

	qm_flash_page_write(QM_FLASH_0, QM_FLASH_REGION_SYS, PAGE_TO_WRITE,
			    flash_data, NUM_DATA_WORDS);

	/*qm_flash_mass_erase(QM_FLASH_0, MASS_ERASE_INCLUDE_ROM);*/
	return QM_RC_OK;
}
