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

MEMORY
{
	/* Flash controllers */
	flash0 (r)	: ORIGIN = 0x40000000, LENGTH = 192K
	flash1 (r)	: ORIGIN = 0x40030000, LENGTH = 192K

	/* x86 RAM */
	x86_esram (rw)  : ORIGIN = 0xA800A000, LENGTH = 40K - 1K - 0x220
	x86_stack (rw)  : ORIGIN = 0xA80139E0, LENGTH = 1K - 0x4 - 0x20
	/* Place IDT at after the stack on the x86 SRAM, 68 gate wide */
	x86_esram_idt (rw)  : ORIGIN = 0xA8013DBC, LENGTH = 0x220

	/* Shared RAM (beeween the app and the bootloader) */
	esram_restore_info (rw) : ORIGIN = 0xA8013FDC, LENGTH = 0x4
	esram_gdt (rw)  : ORIGIN = 0xA8013FE0, LENGTH = 0x20

	/* Sensor RAM */
	/* Place Vector table at the bottom of SRAM, 68 gate wide */
	sensor_ivt(rw)  : ORIGIN = 0xA8000000, LENGTH = 0x110
	/*
	 * Add padding to allow for IVT memory protection.
	 *
	 * Since our MPR granularity is 1kB, we need some padding to prevent
	 * the IVT MPR from protecting part of SRAM used for static variables.
	 */
	sensor_ivt_padding(rw)	: ORIGIN = 0xA8000110, LENGTH = 1K - 0x110
	sensor_esram (rw)  : ORIGIN = 0xA8000400, LENGTH = 40K - 1K - 1K
	sensor_stack (rw)  : ORIGIN = 0xA8009C00, LENGTH = 1K
}

 /* IDT definition */
__idt_start = ORIGIN(x86_esram_idt);
__idt_end = __idt_start + LENGTH(x86_esram_idt);

/* Shared RAM definition */
__x86_restore_info = ORIGIN(esram_restore_info);
__x86_restore_info_end = __x86_restore_info + LENGTH(esram_restore_info);

/* GDT definition */
__gdt_start = ORIGIN(esram_gdt);
__gdt_end = __gdt_start + LENGTH(esram_gdt);
