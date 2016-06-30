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

#include "qm_common.h"
#include <stdlib.h>

#define SIZE 100
/*
 * Very simple application demonstrating the usage of
 * QM_PUTS/QM_PRINTF/QM_ASSERT and malloc.
 * You can enable/modify these settings via CFLAGS.
 */

int main(void)
{
	unsigned int cnt;
	int *p;
	int i;

	QM_PUTS("Demonstrating QM_PUTS/QM_PRINTF functionality");

	for (cnt = 0; cnt < 10; cnt++) {
		QM_PRINTF("%d\n", cnt);
	}

	/* pico_printf only supports %d, %u, %x, %X and %s */
	pico_printf("pico_printf demo: %d\n", 5);

	/* Full printf has a bigger footprint than pico_printf */
	printf("printf demo: %.3f\n", 3.14159);

	QM_PUTS("Demonstrating QM_ASSERT functionality");

	QM_ASSERT(1 == 0);

	p = malloc(SIZE * sizeof(int));

	for (i = 0; i < SIZE; i++) {
		p[i] = i;
	}

	free(p);

	return 0;
}
