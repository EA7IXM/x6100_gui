/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 *
 *  RX: one jitter buffer per talker (as the desktop: start at 60 ms, grow
 *  50 ms after late frames up to 500 ms, drop what runs 300 ms ahead), a
 *  300-2800 Hz voice filter, "duck" sound when off frequency in SSB, and a
 *  player at 8 kHz on the radio's speaker (base play mode).
 *
 *  TX: the hand mic at 48 kHz (base voice-record mode) -> 3.4 kHz low-pass
 *  -> 8 kHz -> mic gain -> 160-sample ADPCM frames to the relay.
 */

#include "cqham_audio.h"
#include "cqham_adpcm.h"
#include "cqham_client.h"

#include "../audio.h"
#include "../radio.h"
#include "../dsp.h"
#include "../cfg/cfg_api.h"

#include <aether_radio/x6100_control/low/control.h>

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define RATE            8000
#define FRAME           160             /* 20 ms */
#define HDR             14
#define SLOTS           4
#define RING            8192            /* ~1 s at 8 kHz */
#define LEAD_START      480             /* 60 ms */
#define LEAD_STEP       400             /* +50 ms */
#define LEAD_MAX        4000            /* 500 ms */
#define LEAD_AHEAD      2400            /* 300 ms over the target: drop */
#define SLOT_IDLE_MS    2000

/***** Biquads (RBJ) *****/

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquad_t;

static void bq_lowpass(biquad_t *f, float fs, float fc, float q) {
    float w = 2.0f * (float)M_PI * fc / fs, c = cosf(w), s = sinf(w), al = s / (2.0f * q);
    float a0 = 1.0f + al;
    f->b0 = (1.0f - c) / 2.0f / a0;
    f->b1 = (1.0f - c) / a0;
    f->b2 = f->b0;
    f->a1 = -2.0f * c / a0;
    f->a2 = (1.0f - al) / a0;
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

static void bq_highpass(biquad_t *f, float fs, float fc, float q) {
    float w = 2.0f * (float)M_PI * fc / fs, c = cosf(w), s = sinf(w), al = s / (2.0f * q);
    float a0 = 1.0f + al;
    f->b0 = (1.0f + c) / 2.0f / a0;
    f->b1 = -(1.0f + c) / a0;
    f->b2 = f->b0;
    f->a1 = -2.0f * c / a0;
    f->a2 = (1.0f - al) / a0;
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

static inline float bq_run(biquad_t *f, float x) {
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;
    return y;
}

/***** RX *****/

typedef struct {
    bool     used;
    uint32_t id;
    int16_t  ring[RING];
    int      rd, wr, count;
    bool     playing;
    int      lead;
    int      stable;
    int64_t  last_ms;
    float    gain;          /* by the talker's power */
    float    shift_hz;      /* 0: on frequency */
    float    detune_att;
    double   phase;
    biquad_t hp, lp;
} slot_t;

static pthread_mutex_t  rx_lock = PTHREAD_MUTEX_INITIALIZER;
static slot_t           slots[SLOTS];
static slot_t           remote_slot;    /* scope_audio from a remote's receiver */

static pthread_t        play_thread;
static atomic_bool      running;
static atomic_bool      tx_on;
static atomic_bool      tx_mode_applied;
static audio_player_t  *player;

static _Atomic float    rx_level;
static _Atomic float    mic_level;

/* diagnostics */
static atomic_int       stat_rx_frames;
static atomic_int       stat_tx_frames;
static atomic_int       stat_mic_cb;
static _Atomic float    stat_mic_sum;
static int64_t          stat_last_ms;

/* base audio mode (see apply_mode) */
typedef enum { MODE_NONE = -1, MODE_IDLE, MODE_PLAY, MODE_REC } base_mode_t;

static base_mode_t    mode_applied = MODE_NONE;
static atomic_llong   last_rx_ms;
static atomic_llong   last_voice_ms;   /* a voice of the network (relay frames) */
static atomic_llong   last_remote_ms;  /* a remote's receiver (scope_audio) */
static bool           mic_ptt_armed;
#define PLAY_HOLD_MS  1500

static int64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void slot_reset(slot_t *s, uint32_t id) {
    memset(s, 0, sizeof(*s));
    s->used = true;
    s->id   = id;
    s->lead = LEAD_START;
    bq_highpass(&s->hp, RATE, 300.0f, 0.707f);
    bq_lowpass(&s->lp, RATE, 2800.0f, 0.707f);
}

static slot_t *slot_for(uint32_t id) {
    int64_t now    = now_ms();
    slot_t *free_s = NULL, *oldest = &slots[0];
    for (int i = 0; i < SLOTS; i++) {
        slot_t *s = &slots[i];
        if (s->used && now - s->last_ms > SLOT_IDLE_MS) s->used = false;
        if (s->used && s->id == id) return s;
        if (!s->used && !free_s) free_s = s;
        if (s->last_ms < oldest->last_ms) oldest = s;
    }
    slot_t *s = free_s ? free_s : oldest;
    slot_reset(s, id);
    return s;
}

static void slot_write(slot_t *s, const int16_t *pcm, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (s->count == RING) {             /* full: drop the oldest */
            s->rd = (s->rd + 1) % RING;
            s->count--;
        }
        s->ring[s->wr] = pcm[i];
        s->wr          = (s->wr + 1) % RING;
        s->count++;
    }
    /* too far ahead of the target: drop down to it */
    if (s->playing && s->count > s->lead + LEAD_AHEAD) {
        int drop = s->count - s->lead;
        s->rd    = (s->rd + drop) % RING;
        s->count -= drop;
    }
    s->last_ms = now_ms();
}

static inline double be_double(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v = (v << 8) | p[i];
    double d;
    memcpy(&d, &v, sizeof(d));
    return d;
}

void cqham_audio_rx(const uint8_t *frame, size_t len) {
    if (!atomic_load(&running) || len <= HDR || atomic_load(&tx_on)) {
        return;     /* semi-duplex: nothing plays while I transmit */
    }

    double   f_mhz = be_double(frame);
    uint32_t id    = ((uint32_t)frame[8] << 24) | ((uint32_t)frame[9] << 16) | ((uint32_t)frame[10] << 8) | frame[11];
    uint16_t pw    = (uint16_t)((frame[12] << 8) | frame[13]);

    if (pw & 0x8000) {
        return;     /* encrypted PMR: not supported */
    }

    int16_t pcm[1024];
    size_t  n;
    if (pw & 0x4000) {
        n = cqham_adpcm_decode(frame + HDR, len - HDR, pcm, 1024);
    } else {
        n = (len - HDR) / 2;
        if (n > 1024) n = 1024;
        for (size_t i = 0; i < n; i++) {
            pcm[i] = (int16_t)(frame[HDR + 2 * i] | (frame[HDR + 2 * i + 1] << 8));
        }
    }

    int watts = pw & 0x1FFF;
    if (watts == 0) watts = 100;
    atomic_fetch_add(&stat_rx_frames, 1);
    {
        /* a remote station (link bit 0x2000) transmits all the time: it is
         * peeked over like scope_audio, or I could never key it. A "voice"
         * with no pause for 15 s is taken as a remote as well. */
        static int64_t voice_start;
        int64_t        t = now_ms();
        if (t - atomic_load(&last_voice_ms) > 600 && t - atomic_load(&last_remote_ms) > 600) voice_start = t;
        if ((pw & 0x2000) || t - voice_start > 15000) {
            atomic_store(&last_remote_ms, t);
        } else {
            atomic_store(&last_voice_ms, t);
        }
    }
    atomic_store(&last_rx_ms, now_ms());

    int64_t      my_freq;
    cqham_mode_t mode;
    cqham_client_tuning(&my_freq, &mode, NULL);
    float shift = (float)((double)my_freq - f_mhz * 1e6);
    float clean = (mode == CQHAM_FM) ? 1200.0f : (mode == CQHAM_AM ? 1000.0f : 400.0f);

    pthread_mutex_lock(&rx_lock);
    slot_t *s = slot_for(id);
    s->gain   = watts >= 100 ? 1.0f + 0.6f * fminf((watts - 100) / 600.0f, 1.0f)
                             : 0.35f + 0.65f * watts / 100.0f;
    if (fabsf(shift) < clean) {
        s->shift_hz   = 0.0f;
        s->detune_att = 1.0f;
    } else {
        s->shift_hz   = (mode == CQHAM_FM || mode == CQHAM_AM) ? 0.0f : shift;
        float att     = 1.0f - (fabsf(shift) - clean) / (3500.0f - clean);
        s->detune_att = att < 0.0f ? 0.0f : att;
    }
    slot_write(s, pcm, n);
    pthread_mutex_unlock(&rx_lock);
}

void cqham_audio_rx_pcm(const int16_t *pcm, size_t n) {
    if (!atomic_load(&running) || atomic_load(&tx_on)) return;
    atomic_store(&last_remote_ms, now_ms());
    atomic_store(&last_rx_ms, now_ms());
    pthread_mutex_lock(&rx_lock);
    if (!remote_slot.used) slot_reset(&remote_slot, 0xFFFFFFFFu);
    remote_slot.gain       = 1.0f;
    remote_slot.detune_att = 1.0f;
    slot_write(&remote_slot, pcm, n);
    pthread_mutex_unlock(&rx_lock);
}

/* One 20 ms block of a slot, filtered and detuned, added into mix. */
static void slot_read(slot_t *s, float *mix) {
    if (!s->used) return;

    if (!s->playing) {
        if (s->count >= s->lead) {
            s->playing = true;
            s->stable  = 0;
        } else {
            return;
        }
    }
    if (s->count < FRAME) {             /* underrun: wait for a longer lead */
        s->playing = false;
        if (s->lead < LEAD_MAX) s->lead += LEAD_STEP;
        return;
    }
    if (++s->stable >= 500 && s->lead > LEAD_START) {
        s->lead -= 160;
        s->stable = 0;
    }

    double step = 2.0 * M_PI * s->shift_hz / RATE;
    for (int i = 0; i < FRAME; i++) {
        float x = s->ring[s->rd] / 32768.0f;
        s->rd   = (s->rd + 1) % RING;
        x       = bq_run(&s->lp, bq_run(&s->hp, x));
        if (s->shift_hz != 0.0f) {      /* SSB off frequency: the "duck" */
            x *= (float)cos(s->phase) * 1.4f;
            s->phase += step;
            if (s->phase > 2.0 * M_PI) s->phase -= 2.0 * M_PI;
        }
        mix[i] += x * s->gain * s->detune_att;
    }
    s->count -= FRAME;
}

/* The base's audio modes:
 *   PLAY  - our audio on the speaker, the receiver muted. The base then
 *           ignores the hand mic PTT (seen on the radio: no BASE TX at all).
 *   IDLE  - the base's normal mode, the hand mic PTT armed; the speaker is
 *           muted (rxvol 0) so the receiver isn't heard.
 *   REC   - the hand mic streamed to us while we talk.
 * With the hand mic PTT in use, the window idles in IDLE and goes to PLAY
 * only while voices come in (and 1.5 s after). */

/* Test variants of the listening mode (AUDIO button, page 4), to find the
 * one where the BASE still keys on the hand mic PTT while it plays CQHAM:
 *   0 AUTO     normal mode while quiet, play while voices come in
 *   1 PLAY     play mode (as v3)
 *   2 PLAY+MIC play mode, hand mic gain back
 *   3 REC      voice-record mode as the listening mode
 *   4 SIN-REC  play mode with the voice-record flag cleared
 *   5 PTTM 1   play mode, pttmode bits (reg 21, bits 2-3) = 1
 *   6 PTTM 2   play mode, pttmode bits = 2
 *   7 TX DIRECTO  as SIN-REC, and the mode is NOT changed on PTT: the
 *              voice-record flag drops the base out of TX (log of v5),
 *              so the mic is taken from the capture stream as it is
 *   8 AUTO PTT    the base keys on the mic PTT only with the voice-record
 *              flag OFF, and plays CQHAM only with it ON (v6 tests). So:
 *              flag off while nobody talks (PTT at once); while CQHAM
 *              plays, flag off for PROBE_MS every PROBE_PERIOD to see
 *              whether the base keys; while the base is in TX (my mic
 *              PTT) the flag stays off: no CQHAM audio, no own voice.
 * pttmode is put back as it was when leaving 5/6 and on close. */
static atomic_int listen_variant;
static uint32_t   pttmode_saved;
static bool       pttmode_touched;

const char *cqham_audio_variant_name(int v) {
    static const char *n[CQHAM_AUDIO_VARIANTS] = {"AUTO", "PLAY", "PLAY+MIC", "REC", "SIN-REC", "PTTM 1", "PTTM 2", "TX DIRECTO", "AUTO PTT", "REMOTO FLUIDO", "BUSCAR PTT", "PRUEBA MIC"};
    return (v >= 0 && v < CQHAM_AUDIO_VARIANTS) ? n[v] : "?";
}

static void pttmode_restore(void) {
    if (!pttmode_touched) return;
    uint32_t r = x6100_control_get(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr) & ~(3u << 2);
    x6100_control_cmd(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr, r | (pttmode_saved << 2));
    pttmode_touched = false;
    cqham_log("AUDIO pttmode restaurado a %u", (unsigned)pttmode_saved);
}

static void pttmode_set(uint32_t v) {
    uint32_t r = x6100_control_get(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr);
    if (!pttmode_touched) {
        pttmode_saved   = (r >> 2) & 3u;
        pttmode_touched = true;
    }
    x6100_control_cmd(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr, (r & ~(3u << 2)) | ((v & 3u) << 2));
    cqham_log("AUDIO pttmode %u (antes %u, registro 0x%08x)", (unsigned)v, (unsigned)pttmode_saved, (unsigned)r);
}

static void apply_play(int variant) {
    if (variant != 5 && variant != 6) pttmode_restore();
    switch (variant) {
        case 2:
        case 11:
            audio_set_play_mode(AUDIO_PLAY_ON);
            x6100_control_hmic_set(param_i_get(cfg.hmic()));
            break;
        case 3:
            audio_set_play_mode(AUDIO_PLAY_VOICE_REC);
            break;
        case 8:
        case 9:
        case 10:
            audio_set_play_mode(AUDIO_PLAY_ON);
            x6100_control_hmic_set(param_i_get(cfg.hmic()));   /* the base keys only with it */
            break;
        case 4:
        case 7:
            audio_set_play_mode(AUDIO_PLAY_ON);
            x6100_control_record_set(false);
            x6100_control_hmic_set(param_i_get(cfg.hmic()));
            break;
        case 5:
        case 6:
            audio_set_play_mode(AUDIO_PLAY_ON);
            pttmode_set(variant == 5 ? 1 : 2);
            break;
        default:
            audio_set_play_mode(AUDIO_PLAY_ON);
            break;
    }
}

void cqham_audio_set_variant(int v) {
    if (v < 0 || v >= CQHAM_AUDIO_VARIANTS) v = 0;
    atomic_store(&listen_variant, v);
    mode_applied = MODE_NONE;   /* the play thread applies it again */
    cqham_log("VARIANTE AUDIO %d %s", v, cqham_audio_variant_name(v));
}

int cqham_audio_get_variant(void) {
    return atomic_load(&listen_variant);
}

static void apply_mode(base_mode_t m) {
    if (m == mode_applied) return;
    switch (m) {
        case MODE_IDLE:
            pttmode_restore();
            audio_set_play_mode(AUDIO_PLAY_OFF);
            x6100_control_rxvol_set(0);
            break;
        case MODE_PLAY:
            apply_play(atomic_load(&listen_variant));
            x6100_control_rxvol_set(param_i_get(cfg.volume()));
            break;
        case MODE_REC:
            audio_set_play_mode(AUDIO_PLAY_VOICE_REC);
            break;
        default:
            break;
    }
    mode_applied = m;
    atomic_store(&tx_mode_applied, m == MODE_REC);
    cqham_log("AUDIO modo %s", m == MODE_IDLE ? "NORMAL (PTT del micro armado, receptor silenciado)"
                             : m == MODE_PLAY ? "PLAY_ON (CQHAM al altavoz)"
                                              : "VOICE_REC (micro a CQHAM)");
}

/* AUTO PTT (variant 8): the voice-record flag, see the list above */
#define PROBE_PERIOD  700
#define PROBE_MS       95
#define TX_TAIL_MS    300
static int      rec_state = -1;     /* -1 unknown, 0 off, 1 on */
static _Atomic int listen_rec;      /* the base plays CQHAM (flag on): capture = mic */
static int64_t  tx_tail_until;
static bool     probe_logged;

/* flag on = CQHAM on the speaker: hand mic gain 0 then, or the base sends
 * the mic to the speaker (the monitor-like howl of v7). Flag off = the base
 * listens to the mic PTT: hand mic gain back, for my voice to CQHAM. */
static void rec_set(bool on) {
    if (rec_state == (int)on) return;
    x6100_control_record_set(on);
    rec_state = on;
    atomic_store(&listen_rec, on);
}

bool cqham_audio_mic_ptt_blind(void) {
    int v = atomic_load(&listen_variant);
    return (v == 8 || v == 9) && atomic_load(&listen_rec) && !atomic_load(&tx_on);
}

/* true while CQHAM audio can be heard (flag on) */
/* last mixed level and its slow average, for peeking in a pause */
static float   mix_rms, mix_avg;
static int64_t probe_until, probe_last;
#define PROBE_MAX_GAP   800   /* a peek at least this often over a remote */
#define PROBE_MIN_GAP   600

static bool auto_ptt_step(bool tx, bool busy) {
    (void)busy;
    bool    base_tx = radio_get_state() == RADIO_TX;
    int64_t now     = now_ms();

    if (tx && !base_tx) {
        /* PTT from the screen / key / F1: the base is not keyed, the
         * mic comes through voice-record as always */
        if (mode_applied != MODE_REC) rec_state = -1;
        apply_mode(MODE_REC);
        tx_tail_until = now + TX_TAIL_MS;
        return false;
    }
    if (mode_applied != MODE_PLAY) rec_state = -1;   /* PLAY_ON sets the flag again */
    apply_mode(MODE_PLAY);

    bool want;
    if (tx || base_tx) {
        want = false;                                /* my over: nothing on the speaker */
        tx_tail_until = now + TX_TAIL_MS;
        if (!probe_logged && base_tx) {
            cqham_log("AUTO PTT: la base entro en TX (PTT del micro)");
            probe_logged = true;
        }
    } else if (now < tx_tail_until) {
        want = false;
    } else if (now < probe_until) {
        want = false;                                /* peeking: the base may key */
    } else if (now - atomic_load(&last_voice_ms) < 600) {
        /* a voice of the network: never cut it (CQHAM gives me the channel
         * only when it ends anyway; then the PTT keys at once) */
        want = true;
        probe_logged = false;
    } else if (now - atomic_load(&last_remote_ms) < 500 && atomic_load(&listen_variant) == 9) {
        /* REMOTO FLUIDO = a squelch on the remote's receiver. A station on
         * it: played whole, never cut. Only its noise: muted, and the base
         * is left listening to the mic PTT, which keys at once (the base
         * ignores the mic PTT while anything plays: BUSCAR PTT, 13:27). */
        static float   nf = 0.05f;          /* the remote's noise floor */
        static int64_t sig_until;
        /* the floor learns from the noise only, never from a station */
        if (tx_tail_until + 400 > now) {
            /* just after my over: keep the floor */
        } else if (mix_rms < nf) {
            nf = nf * 0.90f + mix_rms * 0.10f;
        } else if (mix_rms < nf * 1.8f) {
            nf = nf * 0.995f + mix_rms * 0.005f;
        }
        {
            static int64_t sq_log;
            if (now - sq_log > 2000) {
                sq_log = now;
                cqham_log("SQL remoto: nivel %.3f ruido %.3f -> %s", mix_rms, nf,
                          now < sig_until ? "ABIERTO" : "cerrado");
            }
        }
        if (mix_rms > nf * 2.5f && mix_rms > 0.03f) sig_until = now + 900;   /* hang time */
        want = now < sig_until;
        probe_logged = false;
    } else if (now - atomic_load(&last_remote_ms) < 500) {
        /* a remote's receiver, endless: peek in a pause of its audio, or
         * at least every PROBE_MAX_GAP */
        bool pause = mix_rms < 0.02f || mix_rms < mix_avg * 0.45f;
        if (now - probe_last >= PROBE_MAX_GAP || (pause && now - probe_last >= PROBE_MIN_GAP)) {
            probe_until = now + PROBE_MS;
            probe_last  = now;
            want        = false;
        } else {
            want = true;
        }
        probe_logged = false;
    } else {
        want = false;                                /* quiet: the mic PTT keys at once */
        probe_logged = false;
    }
    rec_set(want);
    return want;
}

static bool moni_muted;

/* The monitor level is also what reaches the capture stream while the base
 * is keyed (v9: monitor 0 -> my voice at 0.08 instead of 0.5). So: monitor 0
 * while listening (no howl), the user's monitor while the base is in TX
 * (my voice to CQHAM) with the speaker volume 0 (I don't hear myself). */
static int moni_tx = -1;   /* -1 unknown, 0 listening, 1 base in TX */

static void moni_follow(void) {
    int base_tx = radio_get_state() == RADIO_TX;
    if (base_tx == moni_tx) return;
    moni_tx    = base_tx;
    moni_muted = true;
    if (base_tx) {
        x6100_control_rxvol_set(0);
        x6100_control_moni_set(param_i_get(cfg.moni()));
        cqham_log("AUDIO TX: monitor %d hacia CQHAM, altavoz a 0", param_i_get(cfg.moni()));
    } else {
        x6100_control_moni_set(0);
        x6100_control_rxvol_set(param_i_get(cfg.volume()));
        cqham_log("AUDIO RX: monitor a 0, altavoz %d", param_i_get(cfg.volume()));
    }
}

/* BUSCAR PTT (variant 10): with CQHAM playing (voice-record flag on), walk
 * every pttmode x mic-select setting and the modem bit, 3 s each, while the
 * operator holds the mic PTT; log which ones key the base. */
#define SCAN_STEP_MS 3000
#define SCAN_STEPS   16
static int      scan_step = -1;
static int64_t  scan_t;
static uint32_t scan_reg_saved;
static int      scan_hits[SCAN_STEPS];

static void scan_apply(int i) {
    uint32_t r = scan_reg_saved & ~0x0Fu;
    bool modem = false;
    if (i < 12) {
        r |= (uint32_t)(i / 3) << 2;            /* pttmode 0..3 */
        r |= (uint32_t)(i % 3);                 /* micsel 0 int, 1 hand, 2 auto */
    } else {
        r |= scan_reg_saved & 0x0Fu;
        if (i == 12) modem = true;
        if (i == 13) r ^= 1u << 4;              /* chge */
        if (i == 14) r ^= 1u << 6;              /* auxiqgen */
        if (i == 15) { modem = true; r = (r & ~0x0Fu) | 1u; }
    }
    x6100_control_cmd(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr, r);
    x6100_control_modem_set(modem);
    x6100_control_record_set(true);
    cqham_log("BUSCAR PTT paso %d: registro 0x%08x modem=%d", i, (unsigned)r, modem);
}

static void scan_run(void) {
    int64_t now = now_ms();
    if (scan_step < 0) {
        scan_reg_saved = x6100_control_get(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr);
        memset(scan_hits, 0, sizeof(scan_hits));
        scan_step = 0;
        scan_t    = now;
        cqham_log("BUSCAR PTT: empieza (registro 0x%08x). Manten el PTT del micro", (unsigned)scan_reg_saved);
        scan_apply(0);
        return;
    }
    if (scan_step >= SCAN_STEPS) return;
    if (radio_get_state() == RADIO_TX) scan_hits[scan_step]++;
    if (now - scan_t >= SCAN_STEP_MS) {
        cqham_log("BUSCAR PTT paso %d -> base en TX %d veces", scan_step, scan_hits[scan_step]);
        scan_step++;
        scan_t = now;
        if (scan_step < SCAN_STEPS) {
            scan_apply(scan_step);
        } else {
            x6100_control_cmd(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr, scan_reg_saved);
            x6100_control_modem_set(false);
            x6100_control_record_set(true);
            cqham_log("BUSCAR PTT: fin, registro restaurado");
        }
    }
}

static void *play_fn(void *arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "cqham-play");

    int16_t out[FRAME];
    float   mix[FRAME];
    float   lvl = 0.0f;

    mode_applied = MODE_NONE;
    apply_mode(mic_ptt_armed ? MODE_IDLE : MODE_PLAY);

    while (atomic_load(&running)) {
        bool        tx   = atomic_load(&tx_on);
        bool        busy = now_ms() - atomic_load(&last_rx_ms) < PLAY_HOLD_MS;
        int         var   = atomic_load(&listen_variant);
        bool        autom = var == 0;
        moni_follow();
        bool        audible = true;
        if (var != 10 && scan_step >= 0 && scan_step < SCAN_STEPS) {
            x6100_control_cmd(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr, scan_reg_saved);
            x6100_control_modem_set(false);
            scan_step = SCAN_STEPS;
            cqham_log("BUSCAR PTT: cortado, registro restaurado");
        }
        if (var == 10) {
            if (mode_applied != MODE_PLAY) { apply_mode(MODE_PLAY); scan_step = -1; }
            scan_run();
            audible = !tx;
        } else if ((var == 8 || var == 9) && mic_ptt_armed) {
            audible = auto_ptt_step(tx, busy);
        } else {
            rec_state = -1;
            atomic_store(&listen_rec, 0);
            base_mode_t want  = tx ? (var == 7 ? MODE_PLAY : MODE_REC)
                             : ((busy || !mic_ptt_armed || !autom) ? MODE_PLAY : MODE_IDLE);
            apply_mode(want);
        }

        memset(mix, 0, sizeof(mix));
        if (!tx) {
            pthread_mutex_lock(&rx_lock);
            for (int i = 0; i < SLOTS; i++) slot_read(&slots[i], mix);
            slot_read(&remote_slot, mix);
            pthread_mutex_unlock(&rx_lock);
        }

        int vol;
        cqham_client_levels(&vol, NULL);
        float g   = (vol / 100.0f) * 1.2f;
        float sum = 0.0f;
        if (!audible) g = 0.0f;
        for (int i = 0; i < FRAME; i++) {
            float v = mix[i] * g;
            sum += mix[i] * mix[i];
            if (v > 1.0f) v = 1.0f;
            if (v < -1.0f) v = -1.0f;
            out[i] = (int16_t)(v * 32767.0f);
        }
        float rms = sqrtf(sum / FRAME) * 3.0f;
        mix_rms = rms;
        mix_avg = mix_avg * 0.98f + rms * 0.02f;
        lvl       = lvl * 0.6f + (rms > 1.0f ? 1.0f : rms) * 0.4f;
        atomic_store(&rx_level, lvl);

        int64_t now = now_ms();
        if (now - stat_last_ms >= 15000) {
            stat_last_ms = now;
            int rxf = atomic_exchange(&stat_rx_frames, 0);
            if (rxf) cqham_log("AUDIO RX %d tramas en 15 s, nivel %.2f", rxf, lvl);
        }

        if (player) {
            audio_player_send(player, out, FRAME);   /* blocks: paces the loop */
        } else {
            usleep(20000);
        }
    }
    return NULL;
}

/***** TX *****/

static uint32_t  mic_sub = AUDIO_SUB_INVALID;
static biquad_t  tx_lp1, tx_lp2;
static int       dec_phase;
static int16_t   tx_buf[FRAME];
static int       tx_n;

static void send_frame(void) {
    int64_t freq;
    int     power;
    int     mic;
    cqham_client_tuning(&freq, NULL, &power);
    cqham_client_levels(NULL, &mic);

    uint8_t frame[HDR + CQHAM_ADPCM_BYTES(FRAME)];
    double  f_mhz = (double)freq / 1e6;
    uint64_t v;
    memcpy(&v, &f_mhz, sizeof(v));
    for (int i = 0; i < 8; i++) frame[i] = (uint8_t)(v >> (56 - 8 * i));

    uint32_t id = cqham_client_my_id();
    frame[8]  = (uint8_t)(id >> 24);
    frame[9]  = (uint8_t)(id >> 16);
    frame[10] = (uint8_t)(id >> 8);
    frame[11] = (uint8_t)id;

    uint16_t pw = (uint16_t)(((power > 1000 ? 1000 : power) & 0x1FFF) | 0x4000);
    frame[12] = (uint8_t)(pw >> 8);
    frame[13] = (uint8_t)pw;

    size_t n = cqham_adpcm_encode(tx_buf, FRAME, frame + HDR);
    cqham_client_send_audio(frame, HDR + n);
}

static _Atomic float mt_rms, mt_peak, mt_dc;

void cqham_audio_mic_stats(float *rms, float *peak, float *dc) {
    if (rms) *rms = atomic_load(&mt_rms);
    if (peak) *peak = atomic_load(&mt_peak);
    if (dc) *dc = atomic_load(&mt_dc);
}

/* PRUEBA MIC: 100 ms windows of the raw capture while the base plays (it
 * ignores the mic PTT then): does the PTT change the level, the noise, the
 * DC, or click? */
static void mic_test(size_t n, const int16_t *x) {
    static double  sq, sm;
    static float   pk;
    static size_t  cnt;
    static FILE   *f;
    for (size_t i = 0; i < n; i++) {
        float v = x[i] / 32768.0f;
        sq += (double)v * v;
        sm += v;
        if (fabsf(v) > pk) pk = fabsf(v);
    }
    cnt += n;
    if (cnt < AUDIO_CAPTURE_RATE / 10) return;
    float mean = (float)(sm / cnt);
    float rms  = (float)sqrt(sq / cnt - (double)mean * mean);
    atomic_store(&mt_rms, rms);
    atomic_store(&mt_peak, pk);
    atomic_store(&mt_dc, mean);
    if (!f) f = fopen("/mnt/mic_ptt.log", "a");
    if (f) {
        struct timespec ts;
        struct tm       tm;
        clock_gettime(CLOCK_REALTIME, &ts);
        localtime_r(&ts.tv_sec, &tm);
        fprintf(f, "%02d:%02d:%02d.%03ld rms=%.5f pico=%.5f dc=%+.5f\n", tm.tm_hour, tm.tm_min, tm.tm_sec,
                ts.tv_nsec / 1000000, rms, pk, mean);
        fflush(f);
    }
    sq = sm = 0.0;
    pk = 0.0f;
    cnt = 0;
}

static void mic_cb(size_t n, int16_t *samples) {
    if (!atomic_load(&tx_on) && atomic_load(&listen_variant) == 11) {
        mic_test(n, samples);
        return;
    }
    if (!atomic_load(&tx_on)) {
        /* listening: is the hand mic heard? level for the MIC bar and the
         * log, nothing is sent */
        static float   lv;
        static int64_t lg;
        if (!atomic_load(&listen_rec)) {
            atomic_store(&mic_level, 0.0f);
            return;
        }
        float sum = 0.0f;
        for (size_t i = 0; i < n; i++) {
            float x = samples[i] / 32768.0f;
            sum += x * x;
        }
        float rms = n ? sqrtf(sum / n) * 3.0f : 0.0f;
        lv = lv * 0.7f + (rms > 1.0f ? 1.0f : rms) * 0.3f;
        atomic_store(&mic_level, lv);
        int64_t t = now_ms();
        if (t - lg > 2000) {
            lg = t;
            cqham_log("MIC escuchando (sin PTT): nivel %.3f", lv);
        }
        return;
    }
    atomic_fetch_add(&stat_mic_cb, 1);

    int mic;
    cqham_client_levels(NULL, &mic);
    float gain = mic / 50.0f;

    for (size_t i = 0; i < n; i++) {
        float x = bq_run(&tx_lp2, bq_run(&tx_lp1, samples[i] / 32768.0f));
        if (++dec_phase < 6) continue;     /* 48 kHz -> 8 kHz */
        dec_phase = 0;

        float y = x * gain;
        if (y > 1.0f) y = 1.0f;
        if (y < -1.0f) y = -1.0f;
        tx_buf[tx_n++] = (int16_t)(y * 32767.0f);

        if (tx_n == FRAME) {
            float sum = 0.0f;
            for (int k = 0; k < FRAME; k++) {
                float s = tx_buf[k] / 32768.0f;
                sum += s * s;
            }
            float rms = sqrtf(sum / FRAME) * 3.0f;
            atomic_store(&mic_level, rms > 1.0f ? 1.0f : rms);
            atomic_fetch_add(&stat_tx_frames, 1);
            atomic_store(&stat_mic_sum, atomic_load(&stat_mic_sum) + (rms > 1.0f ? 1.0f : rms));
            send_frame();
            tx_n = 0;
        }
    }
}

void cqham_audio_tx(bool on) {
    if (atomic_load(&tx_on) == on) return;
    if (on) {
        bq_lowpass(&tx_lp1, AUDIO_CAPTURE_RATE, 3400.0f, 0.707f);
        bq_lowpass(&tx_lp2, AUDIO_CAPTURE_RATE, 3400.0f, 0.707f);
        dec_phase = 0;
        tx_n      = 0;
        /* fresh start for the voices after my over */
        pthread_mutex_lock(&rx_lock);
        for (int i = 0; i < SLOTS; i++) slots[i].used = false;
        remote_slot.used = false;
        pthread_mutex_unlock(&rx_lock);
        atomic_store(&stat_tx_frames, 0);
        atomic_store(&stat_mic_cb, 0);
        atomic_store(&stat_mic_sum, 0.0f);
    } else {
        atomic_store(&mic_level, 0.0f);
        int frames = atomic_load(&stat_tx_frames);
        cqham_log("AUDIO TX fin (variante %s): %d bloques del micro, %d tramas enviadas, nivel medio %.3f",
                  cqham_audio_variant_name(atomic_load(&listen_variant)), atomic_load(&stat_mic_cb), frames,
                  frames ? atomic_load(&stat_mic_sum) / frames : 0.0f);
    }
    atomic_store(&tx_on, on);
    if (mic_sub != AUDIO_SUB_INVALID) {
        dsp_audio_set_active(mic_sub, true);   /* also listening: the MIC bar */
    }
}

/***** Start / stop *****/

void cqham_audio_start(void) {
    if (atomic_load(&running)) return;

    memset(slots, 0, sizeof(slots));
    memset(&remote_slot, 0, sizeof(remote_slot));
    atomic_store(&tx_on, false);
    atomic_store(&tx_mode_applied, true);   /* forces the first apply */
    atomic_store(&last_rx_ms, 0);
    atomic_store(&last_voice_ms, 0);
    atomic_store(&last_remote_ms, 0);
    {
        cqham_config_t c;
        cqham_client_config(&c);
        mic_ptt_armed = c.ptt_micro != 0;
        atomic_store(&listen_variant,
                     (c.audio_variant >= 0 && c.audio_variant < CQHAM_AUDIO_VARIANTS) ? c.audio_variant : 0);
        cqham_log("VARIANTE AUDIO %d %s", atomic_load(&listen_variant),
                  cqham_audio_variant_name(atomic_load(&listen_variant)));
    }

    if (!player) {
        player = audio_create_player(RATE, 1);
    }
    if (mic_sub == AUDIO_SUB_INVALID) {
        mic_sub = dsp_audio_subscribe_raw(mic_cb, true);
    }
    dsp_audio_set_active(mic_sub, true);

    atomic_store(&running, true);
    if (pthread_create(&play_thread, NULL, play_fn, NULL) != 0) {
        atomic_store(&running, false);
    }
}

void cqham_audio_stop(void) {
    if (!atomic_load(&running)) return;
    cqham_audio_tx(false);
    atomic_store(&running, false);
    pthread_join(play_thread, NULL);

    if (mic_sub != AUDIO_SUB_INVALID) {
        dsp_audio_unsubscribe(mic_sub);
        mic_sub = AUDIO_SUB_INVALID;
    }
    if (player) {
        audio_player_release(player);
        player = NULL;
    }
    if (scan_step >= 0 && scan_step < SCAN_STEPS) {
        x6100_control_cmd(x6100_micsel_pttmode_chge_spmode_auxiqgen_sqlthr, scan_reg_saved);
        x6100_control_modem_set(false);
    }
    scan_step = -1;
    pttmode_restore();
    if (moni_muted) {
        x6100_control_moni_set(param_i_get(cfg.moni()));
        x6100_control_rxvol_set(param_i_get(cfg.volume()));
        moni_muted = false;
    }
    moni_tx = -1;
    audio_set_play_mode(AUDIO_PLAY_OFF);
    x6100_control_rxvol_set(param_i_get(cfg.volume()));   /* the speaker as it was */
    mode_applied = MODE_NONE;
    atomic_store(&rx_level, 0.0f);
}

float cqham_audio_rx_level(void) {
    return atomic_load(&rx_level);
}

float cqham_audio_mic_level(void) {
    return atomic_load(&mic_level);
}
