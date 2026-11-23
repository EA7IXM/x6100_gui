/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 *
 *  Audio: the network's voices on the radio's speaker (the base's play
 *  mode, the receiver is muted meanwhile) and the hand mic to the network
 *  (the base's voice-record mode). Nothing is transmitted on RF.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void  cqham_audio_start(void);
void  cqham_audio_stop(void);

/* A binary frame from the relay (14-byte header + PCM or ADPCM). */
void  cqham_audio_rx(const uint8_t *frame, size_t len);
/* PCM from a remote's receiver (scope_audio), 8 kHz mono. */
void  cqham_audio_rx_pcm(const int16_t *pcm, size_t n);

/* Mic to the network on / off. */
void  cqham_audio_tx(bool on);

float cqham_audio_rx_level(void);    /* 0..1, for the S-meter */
float cqham_audio_mic_level(void);   /* 0..1, for the MIC bar */

/* Listening-mode test variants (see cqham_audio.c) */
#define CQHAM_AUDIO_VARIANTS 11
#define CQHAM_AUDIO_DEFAULT  9   /* REMOTO FLUIDO (squelch) */
void        cqham_audio_set_variant(int v);
int         cqham_audio_get_variant(void);
const char *cqham_audio_variant_name(int v);

/* true while the base plays CQHAM and so ignores the hand mic PTT */
bool        cqham_audio_mic_ptt_blind(void);
