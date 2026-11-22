/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 */

#include "cqham_bands.h"

#include <stdlib.h>
#include <string.h>

/* Order as the band selector walks them (low to high), SWL first. */
static const cqham_band_t bands[] = {
    {.name = "SWL",  .start = 499000,    .end = 546000,    .mode = CQHAM_FM,  .swl = true, .fm_only = true, .rx_only = true},
    {.name = "160m", .start = 1800000,   .end = 2000000,   .mode = CQHAM_LSB},
    {.name = "80m",  .start = 3500000,   .end = 4000000,   .mode = CQHAM_LSB},
    {.name = "60m",  .start = 5330500,   .end = 5406400,   .mode = CQHAM_USB},
    {.name = "40m",  .start = 7000000,   .end = 7300000,   .mode = CQHAM_LSB},
    {.name = "30m",  .start = 10100000,  .end = 10150000,  .mode = CQHAM_CW},
    {.name = "20m",  .start = 14000000,  .end = 14350000,  .mode = CQHAM_USB},
    {.name = "17m",  .start = 18068000,  .end = 18168000,  .mode = CQHAM_USB},
    {.name = "15m",  .start = 21000000,  .end = 21450000,  .mode = CQHAM_USB},
    {.name = "12m",  .start = 24890000,  .end = 24990000,  .mode = CQHAM_USB},
    {.name = "CB",   .start = 26965000,  .end = 27405000,  .mode = CQHAM_AM,  .cb = true, .free_band = true},
    {.name = "11m",  .start = 27405000,  .end = 27800000,  .mode = CQHAM_USB, .rooms = true, .free_band = true},
    {.name = "10m",  .start = 28000000,  .end = 29700000,  .mode = CQHAM_USB},
    {.name = "6m",   .start = 50000000,  .end = 54000000,  .mode = CQHAM_USB},
    {.name = "2m",   .start = 144000000, .end = 148000000, .mode = CQHAM_FM},
    {.name = "70cm", .start = 420000000, .end = 450000000, .mode = CQHAM_FM},
    {.name = "PMR",  .start = 446000000, .end = 446200000, .mode = CQHAM_FM,  .pmr = true, .fm_only = true, .free_band = true},
};

#define NBANDS ((int)(sizeof(bands) / sizeof(bands[0])))

/* CB: channels 23/24/25 are out of order, as on a real CB. */
static const int64_t cb_ch[40] = {
    26965000, 26975000, 26985000, 27005000, 27015000, 27025000, 27035000, 27055000,
    27065000, 27075000, 27085000, 27105000, 27115000, 27125000, 27135000, 27155000,
    27165000, 27175000, 27185000, 27205000, 27215000, 27225000, 27255000, 27235000,
    27245000, 27265000, 27275000, 27285000, 27295000, 27305000, 27315000, 27325000,
    27335000, 27345000, 27355000, 27365000, 27375000, 27385000, 27395000, 27405000,
};

static const int64_t pmr_ch[16] = {
    446006250, 446018750, 446031250, 446043750, 446056250, 446068750, 446081250, 446093750,
    446106250, 446118750, 446131250, 446143750, 446156250, 446168750, 446181250, 446193750,
};

static const int64_t swl_ch[10] = {
    500000, 505000, 510000, 515000, 520000, 525000, 530000, 535000, 540000, 545000,
};

static const char *mode_names[CQHAM_MODES] = {"LSB", "USB", "AM", "FM", "CW", "DIGI"};

int cqham_bands_count(void) {
    return NBANDS;
}

const cqham_band_t *cqham_band_at(int i) {
    return (i >= 0 && i < NBANDS) ? &bands[i] : NULL;
}

int cqham_band_index_for(int64_t hz) {
    for (int i = 0; i < NBANDS; i++) {
        const cqham_band_t *b = &bands[i];
        if ((b->pmr || b->cb || b->swl) && hz >= b->start && hz <= b->end) return i;
    }
    for (int i = 0; i < NBANDS; i++) {
        if (hz >= bands[i].start && hz <= bands[i].end) return i;
    }
    return -1;
}

int64_t cqham_band_default_freq(int i) {
    const cqham_band_t *b = cqham_band_at(i);
    if (!b) return 14250000;
    if (b->cb) return 27185000;          /* channel 19 */
    if (b->pmr) return pmr_ch[0];
    if (b->swl) return swl_ch[0];
    if (strcmp(b->name, "11m") == 0) return 27555000;
    int64_t mid = (b->start + b->end) / 2;
    return (mid / 1000) * 1000;
}

const char *cqham_mode_name(cqham_mode_t m) {
    return (m >= 0 && m < CQHAM_MODES) ? mode_names[m] : "USB";
}

cqham_mode_t cqham_mode_from_name(const char *s) {
    for (int i = 0; i < CQHAM_MODES; i++) {
        if (s && strcmp(s, mode_names[i]) == 0) return (cqham_mode_t)i;
    }
    return CQHAM_USB;
}

int cqham_channels(int band, const int64_t **list) {
    const cqham_band_t *b = cqham_band_at(band);
    *list = NULL;
    if (!b) return 0;
    if (b->cb)  { *list = cb_ch;  return 40; }
    if (b->pmr) { *list = pmr_ch; return 16; }
    if (b->swl) { *list = swl_ch; return 10; }
    return 0;
}

int cqham_channel_nearest(int band, int64_t hz) {
    const int64_t *list;
    int            n    = cqham_channels(band, &list);
    int            best = 0;
    int64_t        bd   = INT64_MAX;
    for (int i = 0; i < n; i++) {
        int64_t d = llabs(list[i] - hz);
        if (d < bd) {
            bd   = d;
            best = i;
        }
    }
    return best;
}
