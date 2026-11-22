/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 */

#include "cqham_adpcm.h"

#include <stdlib.h>

static const int16_t step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230,
    253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
    1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327,
    3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
    11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
    32767
};

static const int8_t index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8
};

static inline int clamp_index(int i) {
    return i < 0 ? 0 : (i > 88 ? 88 : i);
}

static inline int clamp16(int v) {
    return v < -32768 ? -32768 : (v > 32767 ? 32767 : v);
}

/* Starting step from the mean absolute difference between samples, as
 * adpcm.js pickIndex() does: a good start keeps the first nibbles clean. */
static int pick_index(const int16_t *pcm, size_t n) {
    if (n < 2) {
        return 0;
    }
    long sum = 0;
    for (size_t i = 1; i < n; i++) {
        sum += labs((long)pcm[i] - pcm[i - 1]);
    }
    long mean = sum / (long)(n - 1);
    int  idx  = 0;
    while (idx < 88 && step_table[idx + 1] <= mean) {
        idx++;
    }
    return idx;
}

size_t cqham_adpcm_encode(const int16_t *pcm, size_t n, uint8_t *out) {
    int predictor = n ? pcm[0] : 0;
    int index     = pick_index(pcm, n);

    out[0] = (uint8_t)(predictor & 0xFF);
    out[1] = (uint8_t)((predictor >> 8) & 0xFF);
    out[2] = (uint8_t)index;
    out[3] = 0;

    size_t bytes = CQHAM_ADPCM_BYTES(n);
    for (size_t i = 4; i < bytes; i++) {
        out[i] = 0;
    }

    for (size_t i = 0; i < n; i++) {
        int step = step_table[index];
        int diff = pcm[i] - predictor;
        int code = 0;

        if (diff < 0) {
            code = 8;
            diff = -diff;
        }

        int delta = step >> 3;
        if (diff >= step) {
            code |= 4;
            diff -= step;
            delta += step;
        }
        if (diff >= (step >> 1)) {
            code |= 2;
            diff -= step >> 1;
            delta += step >> 1;
        }
        if (diff >= (step >> 2)) {
            code |= 1;
            delta += step >> 2;
        }

        predictor = clamp16((code & 8) ? predictor - delta : predictor + delta);
        index     = clamp_index(index + index_table[code]);

        size_t at = 4 + i / 2;
        if (i & 1) {
            out[at] |= (uint8_t)(code << 4);
        } else {
            out[at] |= (uint8_t)code;
        }
    }
    return bytes;
}

size_t cqham_adpcm_decode(const uint8_t *in, size_t len, int16_t *pcm, size_t max) {
    if (len < 4) {
        return 0;
    }
    int predictor = (int16_t)(in[0] | (in[1] << 8));
    int index     = clamp_index(in[2]);
    size_t n      = (len - 4) * 2;
    if (n > max) {
        n = max;
    }

    for (size_t i = 0; i < n; i++) {
        uint8_t byte = in[4 + i / 2];
        int     code = (i & 1) ? (byte >> 4) : (byte & 0x0F);
        int     step = step_table[index];

        int diff = step >> 3;
        if (code & 4) diff += step;
        if (code & 2) diff += step >> 1;
        if (code & 1) diff += step >> 2;

        predictor = clamp16((code & 8) ? predictor - diff : predictor + diff);
        index     = clamp_index(index + index_table[code]);
        pcm[i]    = (int16_t)predictor;
    }
    return n;
}
