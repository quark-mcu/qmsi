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
#include "spinlock.h"

#if (QUARK_SE)
/*
 * Spinlock (Peterson algorithm).
 * are used in synchronization between cores that don't have any other means of
 * hardware-based synchronization.
 * NOTE: This Spinlock implementation doesn't solve all system synchronization
 * issues, it has limited applicability and should be used with care. Can only
 * be used to protect critical sections used by both cores.
 */

extern spinlock_t __esram_lock_start;

#if (QM_SENSOR)

void spinlock_lock(spinlock_t *lock)
{
	lock->flag[1] = 1;
	lock->turn = 0;
	while (lock->flag[0] == 1 && lock->turn == 0) {
		/* busy wait */
	}
}
void spinlock_unlock(spinlock_t *lock)
{
	lock->flag[1] = 0;
}

#else

void spinlock_lock(spinlock_t *lock)
{
	lock->flag[0] = 1;
	lock->turn = 1;
	while (lock->flag[1] == 1 && lock->turn == 1) {
		/* busy wait */
	}
}
void spinlock_unlock(spinlock_t *lock)
{
	lock->flag[0] = 0;
}

#endif /* QM_SENSOR */

#endif /* QUARK_SE */
