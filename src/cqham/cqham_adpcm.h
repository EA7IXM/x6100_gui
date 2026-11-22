/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 *
 *  IMA ADPCM, one self-contained block per 20 ms frame, byte-compatible with
 *  CQHAM's shared/adpcm.js:
 *    [0..1] predictor (int16 LE, the first sample)
 *    [2]    step index 0..88
 *    [3]    0
 *    [4..]  nibbles, low nibble first; sample 0 is coded in nibble 0 too.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#define CQHAM_ADPCM_BYTES(n)    (4 + ((n) + 1) / 2)

/* Encodes n samples into out (CQHAM_ADPCM_BYTES(n) bytes). Returns the size. */
size_t cqham_adpcm_encode(const int16_t *pcm, size_t n, uint8_t *out);

/* Decodes a block of len bytes into at most max samples. Returns the count. */
size_t cqham_adpcm_decode(const uint8_t *in, size_t len, int16_t *pcm, size_t max);
