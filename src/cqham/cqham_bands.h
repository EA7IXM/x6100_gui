/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 *
 *  CQHAM's band plan (shared/bands.js), frequencies in Hz.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CQHAM_LSB,
    CQHAM_USB,
    CQHAM_AM,
    CQHAM_FM,
    CQHAM_CW,
    CQHAM_DIGI,
    CQHAM_MODES
} cqham_mode_t;

typedef struct {
    const char   *name;
    int64_t       start, end;     /* Hz */
    cqham_mode_t  mode;           /* default mode */
    bool          cb, pmr, swl;   /* channelised bands */
    bool          fm_only, rx_only;
    bool          rooms;          /* 11 m: rooms a / b */
    bool          free_band;      /* no licence needed to talk */
} cqham_band_t;

int                 cqham_bands_count(void);
const cqham_band_t *cqham_band_at(int i);
/* Special segments (PMR, CB, SWL) first, as bandForFreq() */
int                 cqham_band_index_for(int64_t hz);
int64_t             cqham_band_default_freq(int i);

const char         *cqham_mode_name(cqham_mode_t m);
cqham_mode_t        cqham_mode_from_name(const char *s);

/* Channelised bands: number of channels and channel n (0-based) in Hz, the
 * nearest channel to hz, and its number shown to the user (1-based). */
int                 cqham_channels(int band, const int64_t **list);
int                 cqham_channel_nearest(int band, int64_t hz);
