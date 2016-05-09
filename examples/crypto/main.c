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

#include <x86intrin.h>
#include <qm_common.h>
#include <qm_rtc.h>
#include <ctr_mode.h>
#include <hmac.h>
#include <string.h>

static const uint8_t hmac_sha256_key[32] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30,
    0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31};
static const uint8_t hmac_sha256_data[28] = {
    0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x31, 0x31, 0x31,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x31, 0x31, 0x31,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x31};

static uint8_t hmac_sha256_digest[32];

/* pre-computed digest */
static const uint8_t hmac_sha256_expected[32] = {
    0xd6, 0xa3, 0x86, 0x9b, 0x03, 0x57, 0x83, 0xca, 0x68, 0x91, 0x06,
    0xfc, 0xf2, 0x66, 0x83, 0xba, 0xe0, 0x0a, 0x92, 0xa3, 0xb0, 0x3b,
    0x5e, 0xc3, 0xdc, 0x8a, 0xfd, 0xc8, 0x5c, 0x40, 0x3d, 0x6d};

/* example of HMAC digest calculation */
static void do_hmac_sha256(void)
{
	uint64_t t_init, t_end;
	uint64_t t_op_usec;
	struct tc_hmac_state_struct h;

	QM_PRINTF("Computing HMAC-SHA256 digest...\n");

	memset(&h, 0x00, sizeof(h));
	tc_hmac_set_key(&h, hmac_sha256_key, sizeof(hmac_sha256_key));

	t_init = _rdtsc();

	tc_hmac_init(&h);
	tc_hmac_update(&h, hmac_sha256_data, sizeof(hmac_sha256_data));
	tc_hmac_final(hmac_sha256_digest, TC_SHA256_DIGEST_SIZE, &h);

	t_end = _rdtsc();
	t_op_usec = (t_end - t_init) / SYS_TICKS_PER_US_32MHZ;

	if (memcmp(hmac_sha256_expected, hmac_sha256_digest,
		   sizeof(hmac_sha256_digest)) == 0) {
		QM_PRINTF("HMAC-SHA256 digest value OK (%lu usecs)\n",
			  (unsigned long)t_op_usec);
	} else {
		QM_PRINTF("Error: HMAC-SHA256 mismatch in digest value\n");
	}
}

static const uint8_t aes_ctr_key[16] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
                                        0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
                                        0x32, 0x33, 0x34, 0x35};
static uint8_t aes_ctr_ctr[16] = {0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5,
                                  0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb,
                                  0xec, 0xed, 0xee, 0xef};
static const uint8_t aes_ctr_plaintext[64] = {
    0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x31, 0x31, 0x31, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x31, 0x31, 0x31, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x31, 0x31, 0x31, 0x31, 0x31, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x31, 0x31, 0x31, 0x31, 0x31, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x31, 0x31, 0x31, 0x31, 0x31, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x31, 0x31, 0x31, 0x31, 0x31, 0x30, 0x30, 0x30, 0x30};

static uint8_t aes_ctr_out[80];
static uint8_t aes_ctr_decrypted[64];

/* pre-computed cipher text */
static const uint8_t aes_ctr_ciphertext[80] = {
    0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb,
    0xec, 0xed, 0xee, 0xef, 0x43, 0xa6, 0x54, 0xcd, 0xfb, 0x0f, 0xdc, 0x9b,
    0x9c, 0x48, 0x42, 0x88, 0x31, 0x85, 0x19, 0xbc, 0x02, 0xc0, 0xaa, 0xc2,
    0x3c, 0x78, 0x7b, 0x17, 0x5b, 0x9e, 0xc0, 0xa2, 0x74, 0x87, 0xe8, 0xb1,
    0x45, 0xee, 0xf8, 0x76, 0xfe, 0xa7, 0x2e, 0xdb, 0xcf, 0x7d, 0x83, 0xc2,
    0x58, 0x7b, 0x37, 0xe1, 0x17, 0x5f, 0x12, 0x54, 0xe1, 0x87, 0xfa, 0xe0,
    0x94, 0x61, 0x28, 0xea, 0x0e, 0xf0, 0xec, 0x1d};

/* example of AES-CTR encryption and decryption */
static void do_aes_ctr(void)
{
	uint64_t t_init, t_end;
	uint64_t t_op_usec;

	struct tc_aes_key_sched_struct sched;

	QM_PRINTF("AES-CTR Encrypting plain text...\n");

	memset(&sched, 0x00, sizeof(sched));
	tc_aes128_set_encrypt_key(&sched, aes_ctr_key);

	memcpy(aes_ctr_out, aes_ctr_ctr, sizeof(aes_ctr_ctr));

	t_init = _rdtsc();

	tc_ctr_mode(&aes_ctr_out[TC_AES_BLOCK_SIZE], sizeof(aes_ctr_plaintext),
		    aes_ctr_plaintext, sizeof(aes_ctr_plaintext), aes_ctr_ctr,
		    &sched);

	t_end = _rdtsc();
	t_op_usec = (t_end - t_init) / SYS_TICKS_PER_US_32MHZ;

	if (memcmp(aes_ctr_ciphertext, aes_ctr_out, sizeof(aes_ctr_out)) == 0) {
		QM_PRINTF("AES-CTR encryption success (%lu usecs)\n",
			  (unsigned long)t_op_usec);
	} else {
		QM_PRINTF("Error: AES-CTR mismatch in data encrypted\n");
		return;
	}

	QM_PRINTF("AES-CTR Decrypting cipher text...\n");
	memcpy(aes_ctr_ctr, aes_ctr_out, sizeof(aes_ctr_ctr));

	t_init = _rdtsc();

	tc_ctr_mode(aes_ctr_decrypted, sizeof(aes_ctr_decrypted),
		    &aes_ctr_out[TC_AES_BLOCK_SIZE], sizeof(aes_ctr_decrypted),
		    aes_ctr_ctr, &sched);

	t_end = _rdtsc();
	t_op_usec = (t_end - t_init) / SYS_TICKS_PER_US_32MHZ;

	if (memcmp(aes_ctr_plaintext, aes_ctr_decrypted,
		   sizeof(aes_ctr_plaintext)) == 0) {
		QM_PRINTF("AES-CTR decryption success (%lu usecs)\n",
			  (unsigned long)t_op_usec);
	} else {
		QM_PRINTF("Error: AES-CTR mismatch in data decrypted\n");
	}
}

/*  Tinycrypt crypto app example */
int main(void)
{
	QM_PRINTF("Starting: Crypto\n");
	do_aes_ctr();
	do_hmac_sha256();
	QM_PRINTF("Finished: Crypto\n");

	return 0;
}
