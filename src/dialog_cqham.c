/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM window (EA7IXM)
 *
 *  The CQHAM desktop look on the radio: signal meter with a needle on the
 *  left, the blue LCD with the frequency and the CQHAM scope (or a remote
 *  station's real scope) in the middle, the tuning dial on the right.
 *
 *  Controls while it is open:
 *    main dial ........ frequency (channels in CB / PMR / SWL; the remote's
 *                       VFO in remote mode)
 *    BAND up / down ... CQHAM band
 *    SSB .............. LSB <-> USB      AM/FM ... AM <-> FM      CW ... CW
 *    front PTT key .... talk on CQHAM (hold)   hand mic F1: talk (hold)
 *    hand mic UP/DOWN . one step           VOL knob: CQHAM volume
 *  Nothing is transmitted on RF: the radio's own TX power is set to 0 while
 *  the window is open, the hand mic PTT included.
 */

#include "dialog_cqham.h"

#include "cqham/cqham_audio.h"
#include "cqham/cqham_client.h"

#include "buttons.h"
#include "keyboard.h"
#include "msg.h"
#include "pubsub_ids.h"
#include "radio.h"
#include "spectrum.h"
#include "styles.h"
#include "textarea_window.h"
#include "voice.h"
#include "waterfall.h"
#include "cfg/cfg_api.h"

#include <aether_radio/x6100_control/control.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Geometry inside the dialog (796 x 345) */
#define PANEL_W     176
#define LCD_X       (PANEL_W + 8)
#define LCD_W       (DIALOG_WIDTH - 2 * PANEL_W - 16)
#define SC_W        (LCD_W - 8)
#define SC_H        196
#define SP_H        92          /* spectrum */
#define AX_H        14          /* frequency axis */
#define WF_Y        (SP_H + AX_H)
#define WF_H        (SC_H - WF_Y)

#define C_BG1       0x1c2128
#define C_BG2       0x0b0e12
#define C_PANEL     0x161b21
#define C_BORDER    0x2c3540
#define C_ACCENT    0x5fb4ff
#define C_TITLE     0x7fb6e6
#define C_LCD1      0x0d2236
#define C_LCD2      0x081624

static void construct_cb(lv_obj_t *parent);
static void destruct_cb(void);
static void key_cb(lv_event_t *e);

static dialog_t dialog = {
    .run          = false,
    .construct_cb = construct_cb,
    .destruct_cb  = destruct_cb,
    .key_cb       = key_cb,
};

dialog_t *dialog_cqham = &dialog;

/***** Widgets *****/

static lv_obj_t   *meter_obj, *knob_obj;
static lv_obj_t   *lbl_tags, *lbl_freq, *lbl_status, *lbl_notice, *lbl_users, *lbl_conn;
static lv_obj_t   *lbl_pwr, *lbl_step, *lbl_vol, *lbl_mic, *lbl_hint, *mic_bar;
static lv_obj_t   *led_tx, *led_rx, *led_sq;
static lv_obj_t   *canvas;
static lv_color_t *canvas_buf;
static lv_timer_t *timer;
static void       *tx_sub, *rx_sub;

static cqham_state_t s;
static cqham_config_t conf;

/* animation state */
static float      s_meter;          /* 0..1 */
static float      noise_floor = 0.12f;
static float      lvl[SC_W];
static float      rem_cur[SC_W];
static float      knob_angle;
static uint32_t   tick_n;
static int64_t    last_wf_ms;
static uint32_t   notice_seen;
static int64_t    notice_until;

/* remote dial: one request per 85 ms */
static int64_t    rq_freq;
static int64_t    rq_last_ms;
static bool       ptt_by_mic;       /* the hand mic PTT keyed the base */
static bool       ptt_by_key;

static const int  steps[] = {100, 1000, 5000, 10000, 25000};
static const int  powers[] = {5, 10, 25, 50, 100};

static int64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static inline uint32_t rgb(int r, int g, int b) {
    r = r < 0 ? 0 : (r > 255 ? 255 : r);
    g = g < 0 ? 0 : (g > 255 ? 255 : g);
    b = b < 0 ? 0 : (b > 255 ? 255 : b);
    return 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

static inline uint32_t blend(uint32_t dst, uint32_t src, float a) {
    if (a <= 0.0f) return dst;
    if (a >= 1.0f) return src;
    int dr = (dst >> 16) & 255, dg = (dst >> 8) & 255, db = dst & 255;
    int sr = (src >> 16) & 255, sg = (src >> 8) & 255, sb = src & 255;
    return rgb((int)(dr + (sr - dr) * a), (int)(dg + (sg - dg) * a), (int)(db + (sb - db) * a));
}

/* CQHAM's waterfall colours (wfColor) */
static uint32_t wf_color(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    float a;
    if (v < 0.15f) { a = v / 0.15f;           return rgb(0, (int)(20 * a), (int)(20 + 40 * a)); }
    if (v < 0.40f) { a = (v - 0.15f) / 0.25f; return rgb(0, (int)(20 + 80 * a), (int)(60 + 150 * a)); }
    if (v < 0.60f) { a = (v - 0.40f) / 0.20f; return rgb(0, (int)(100 + 155 * a), (int)(210 - 100 * a)); }
    if (v < 0.80f) { a = (v - 0.60f) / 0.20f; return rgb((int)(220 * a), 255, (int)(110 - 110 * a)); }
    a = (v - 0.80f) / 0.20f;
    return rgb((int)(220 + 35 * a), (int)(255 - 200 * a), 0);
}

/***** Helpers *****/

static int band_idx(void) {
    return cqham_band_index_for(s.freq);
}

static const cqham_band_t *band(void) {
    return cqham_band_at(band_idx());
}

static void fmt_freq(int64_t hz, char *buf, size_t n) {
    long long mhz = hz / 1000000, khz = (hz / 1000) % 1000, rest = hz % 1000;
    snprintf(buf, n, "%lld.%03lld.%03lld", mhz, khz, rest);
}

static void show_notice(const char *txt, bool alert) {
    lv_label_set_text(lbl_notice, txt);
    lv_obj_set_style_text_color(lbl_notice, lv_color_hex(alert ? 0xff6b6b : 0xffcf3f), 0);
    notice_until = now_ms() + 6000;
}

static lv_obj_t *mk_panel(lv_obj_t *parent, lv_coord_t x, lv_coord_t w) {
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_remove_style_all(p);
    lv_obj_set_pos(p, x, 2);
    lv_obj_set_size(p, w, DIALOG_HEIGHT - 4);
    lv_obj_set_style_radius(p, 10, 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(p, lv_color_hex(0x20262e), 0);
    lv_obj_set_style_bg_grad_color(p, lv_color_hex(0x12161b), 0);
    lv_obj_set_style_bg_grad_dir(p, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(p, 1, 0);
    lv_obj_set_style_border_color(p, lv_color_hex(C_BORDER), 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return p;
}

static lv_obj_t *mk_label(lv_obj_t *parent, const lv_font_t *font, uint32_t color, lv_coord_t x, lv_coord_t y,
                          lv_coord_t w, lv_text_align_t align, const char *txt) {
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_obj_set_style_text_align(l, align, 0);
    lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_width(l, w);
    lv_label_set_text(l, txt);
    return l;
}

static lv_obj_t *mk_led(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, const char *txt) {
    lv_obj_t *led = lv_obj_create(parent);
    lv_obj_remove_style_all(led);
    lv_obj_set_pos(led, x, y);
    lv_obj_set_size(led, 12, 12);
    lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(led, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(led, lv_color_hex(0x2a3038), 0);
    mk_label(parent, &sony_16, 0x9aa8b4, x + 16, y - 3, 40, LV_TEXT_ALIGN_LEFT, txt);
    return led;
}

static void led_set(lv_obj_t *led, bool on, uint32_t color) {
    lv_obj_set_style_bg_color(led, lv_color_hex(on ? color : 0x2a3038), 0);
    lv_obj_set_style_shadow_width(led, on ? 10 : 0, 0);
    lv_obj_set_style_shadow_color(led, lv_color_hex(color), 0);
}

/***** S-meter (needle) *****/

#define MTR_W 160
#define MTR_H 130

static void meter_draw_cb(lv_event_t *e) {
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t      c;
    lv_obj_get_coords(lv_event_get_target(e), &c);

    lv_draw_rect_dsc_t r;
    lv_draw_rect_dsc_init(&r);
    r.radius       = 8;
    r.bg_color     = lv_color_hex(0x0f1a24);
    r.border_color = lv_color_hex(0x2b4357);
    r.border_width = 2;
    lv_draw_rect(ctx, &r, &c);

    lv_point_t ctr = {c.x1 + MTR_W / 2, c.y1 + 112};
    int        R   = 78;
    /* arc from 205 deg to 335 deg (LVGL: 0 = 3 o'clock, clockwise) */
    const int  a0 = 205, a1 = 335;

    lv_draw_arc_dsc_t arc;
    lv_draw_arc_dsc_init(&arc);
    arc.width = 3;
    struct { float f0, f1; uint32_t col; } seg[] = {
        {0.00f, 0.60f, 0x2ecc71}, {0.60f, 0.64f, 0xf1c40f}, {0.64f, 1.0f, 0xe74c3c},
    };
    for (int i = 0; i < 3; i++) {
        arc.color = lv_color_hex(seg[i].col);
        lv_draw_arc(ctx, &arc, &ctr, R, (uint16_t)(a0 + (a1 - a0) * seg[i].f0),
                    (uint16_t)(a0 + (a1 - a0) * seg[i].f1));
    }

    static const struct { const char *t; float f; } marks[] = {
        {"1", 0.0f}, {"3", 0.15f}, {"5", 0.30f}, {"7", 0.45f}, {"9", 0.60f},
        {"+20", 0.733f}, {"+40", 0.866f}, {"+60", 1.0f},
    };
    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.width = 1;
    ld.color = lv_color_hex(0xcfd8e0);
    lv_draw_label_dsc_t lb;
    lv_draw_label_dsc_init(&lb);
    lb.font  = &sony_14;
    lb.align = LV_TEXT_ALIGN_CENTER;
    for (size_t i = 0; i < sizeof(marks) / sizeof(marks[0]); i++) {
        float      a  = (a0 + (a1 - a0) * marks[i].f) * (float)M_PI / 180.0f;
        lv_point_t p1 = {ctr.x + (lv_coord_t)(R * cosf(a)), ctr.y + (lv_coord_t)(R * sinf(a))};
        lv_point_t p2 = {ctr.x + (lv_coord_t)((R + 6) * cosf(a)), ctr.y + (lv_coord_t)((R + 6) * sinf(a))};
        lv_draw_line(ctx, &ld, &p1, &p2);
        lv_coord_t lx = ctr.x + (lv_coord_t)((R + 17) * cosf(a));
        lv_coord_t ly = ctr.y + (lv_coord_t)((R + 17) * sinf(a));
        lv_area_t  ta = {lx - 16, ly - 8, lx + 16, ly + 8};
        lb.color      = lv_color_hex(marks[i].f > 0.6f ? 0xff7b6b : 0xe8eef3);
        lv_draw_label(ctx, &lb, &ta, marks[i].t, NULL);
    }

    lb.color      = lv_color_hex(C_ACCENT);
    lv_area_t sa  = {c.x1, ctr.y - 34, c.x2, ctr.y - 18};
    lv_draw_label(ctx, &lb, &sa, "S-METER", NULL);

    /* needle */
    float      a  = (a0 + (a1 - a0) * s_meter) * (float)M_PI / 180.0f;
    lv_point_t p1 = ctr;
    lv_point_t p2 = {ctr.x + (lv_coord_t)((R - 4) * cosf(a)), ctr.y + (lv_coord_t)((R - 4) * sinf(a))};
    ld.width      = 2;
    ld.color      = lv_color_hex(0xffc040);
    lv_draw_line(ctx, &ld, &p1, &p2);

    r.radius       = LV_RADIUS_CIRCLE;
    r.bg_color     = lv_color_hex(C_ACCENT);
    r.border_width = 2;
    r.border_color = lv_color_hex(0x33485a);
    lv_area_t pv   = {ctr.x - 5, ctr.y - 5, ctr.x + 5, ctr.y + 5};
    lv_draw_rect(ctx, &r, &pv);
}

/***** Tuning knob *****/

#define KNOB_D 116

static void knob_draw_cb(lv_event_t *e) {
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t      c;
    lv_obj_get_coords(lv_event_get_target(e), &c);
    lv_point_t ctr = {(c.x1 + c.x2) / 2, (c.y1 + c.y2) / 2};
    int        R   = KNOB_D / 2 - 6;

    lv_draw_rect_dsc_t r;
    lv_draw_rect_dsc_init(&r);
    r.radius       = LV_RADIUS_CIRCLE;
    r.bg_color     = lv_color_hex(0x0d1014);
    r.border_color = lv_color_hex(0x4a9be0);
    r.border_width = 2;
    lv_draw_rect(ctx, &r, &c);

    /* brushed metal: light top-left to dark bottom-right */
    lv_area_t k = {ctr.x - R, ctr.y - R, ctr.x + R, ctr.y + R};
    r.border_width        = 0;
    r.shadow_width        = 0;
    r.bg_color            = lv_color_hex(0xc9ced3);
    r.bg_grad.dir         = LV_GRAD_DIR_VER;
    r.bg_grad.stops_count = 3;
    r.bg_grad.stops[0].color = lv_color_hex(0xe6e9ec);
    r.bg_grad.stops[0].frac  = 0;
    r.bg_grad.stops[1].color = lv_color_hex(0x7d858c);
    r.bg_grad.stops[1].frac  = 130;
    r.bg_grad.stops[2].color = lv_color_hex(0x23282e);
    r.bg_grad.stops[2].frac  = 255;
    lv_draw_rect(ctx, &r, &k);

    /* knurling */
    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.width = 1;
    ld.color = lv_color_hex(0x000000);
    ld.opa   = LV_OPA_30;
    for (int i = 0; i < 36; i++) {
        float      a  = i * 2.0f * (float)M_PI / 36.0f + knob_angle;
        lv_point_t p1 = {ctr.x + (lv_coord_t)((R - 12) * cosf(a)), ctr.y + (lv_coord_t)((R - 12) * sinf(a))};
        lv_point_t p2 = {ctr.x + (lv_coord_t)((R - 2) * cosf(a)), ctr.y + (lv_coord_t)((R - 2) * sinf(a))};
        lv_draw_line(ctx, &ld, &p1, &p2);
    }

    /* finger dimple + blue marker */
    float      a = knob_angle - (float)M_PI / 2.0f;
    lv_point_t m = {ctr.x + (lv_coord_t)((R - 22) * cosf(a)), ctr.y + (lv_coord_t)((R - 22) * sinf(a))};
    r.bg_grad.stops_count = 0;
    r.bg_grad.dir         = LV_GRAD_DIR_NONE;
    r.bg_color            = lv_color_hex(C_ACCENT);
    r.shadow_color        = lv_color_hex(C_ACCENT);
    r.shadow_width        = 10;
    lv_area_t ma          = {m.x - 5, m.y - 5, m.x + 5, m.y + 5};
    lv_draw_rect(ctx, &r, &ma);
}

/***** Scope *****/

static void canvas_px(int x, int y, uint32_t col) {
    if (x < 0 || x >= SC_W || y < 0 || y >= SC_H) return;
    canvas_buf[y * SC_W + x].full = col;
}

static void scope_compute_cqham(float t) {
    const cqham_band_t *b = band();
    if (!b) return;
    double span  = (double)(b->end - b->start);
    bool   conn  = s.status == CQHAM_ST_ONLINE;

    noise_floor += ((float)rand() / RAND_MAX - 0.5f) * 0.006f;
    if (noise_floor < 0.06f) noise_floor = 0.06f;
    if (noise_floor > 0.22f) noise_floor = 0.22f;
    float nf = conn ? noise_floor : 0.04f;

    for (int x = 0; x < SC_W; x++) {
        float undu = 0.5f + 0.5f * sinf(x * 0.012f + t * 0.5f) * sinf(x * 0.003f - t * 0.3f);
        lvl[x]     = nf * (0.6f + 0.6f * undu) + ((float)rand() / RAND_MAX) * nf * 0.5f;
    }

    float half = (float)(2400.0 / span) * SC_W / 2.0f;
    if (half < 2.0f) half = 2.0f;

    for (int i = 0; i < s.stations_n; i++) {
        const cqham_station_t *st = &s.stations[i];
        if (!st->tx) continue;
        float xc = (float)((st->freq - b->start) / span * SC_W);
        float pf = 0.4f + 0.6f * (st->power > 100 ? 1.0f : st->power / 100.0f);
        bool  talker = s.talker[0] && !strcmp(st->call, s.talker);
        float h      = talker ? 0.04f + s_meter * 0.94f + cqham_audio_rx_level() * 0.04f
                              : (0.6f + 0.2f * sinf(t * 8.0f + xc)) * pf;
        int   x0 = (int)(xc - 3 * half), x1 = (int)(xc + 3 * half);
        for (int x = x0 < 0 ? 0 : x0; x <= x1 && x < SC_W; x++) {
            float d = (x - xc) / half;
            float g = h * expf(-d * d);
            if (g > lvl[x]) {
                lvl[x] = lvl[x] * 0.4f + g;
                if (lvl[x] > 0.98f) lvl[x] = 0.98f;
            }
        }
    }
}

static void scope_compute_remote(void) {
    float max = s.scope_max > 0 ? (float)s.scope_max : 160.0f;
    for (int x = 0; x < SC_W; x++) {
        float obj = 0.0f;
        if (s.scope_n > 1) {
            float pos = (float)x * (s.scope_n - 1) / (SC_W - 1);
            int   i   = (int)pos;
            float fr  = pos - i;
            int   j   = i + 1 < s.scope_n ? i + 1 : i;
            obj       = (s.scope[i] * (1.0f - fr) + s.scope[j] * fr) / max;
        }
        if (obj > 1.0f) obj = 1.0f;
        rem_cur[x] += (obj - rem_cur[x]) * 0.35f;
        lvl[x]      = rem_cur[x];
    }
}

static void scope_render(void) {
    bool remote = s.remote && s.scope_on && s.scope_fe > s.scope_fs;
    float t     = tick_n * 0.05f;

    if (remote) scope_compute_remote();
    else scope_compute_cqham(t);

    /* spectrum background */
    for (int y = 0; y < SP_H; y++) {
        uint32_t bg = remote ? blend(rgb(0x0a, 0x10, 0x20), rgb(0x06, 0x09, 0x12), (float)y / SP_H)
                             : rgb(0x02, 0x06, 0x0a);
        for (int x = 0; x < SC_W; x++) canvas_buf[y * SC_W + x].full = bg;
    }

    /* grid */
    int64_t lo, hi;
    if (remote) {
        lo = s.scope_fs;
        hi = s.scope_fe;
    } else {
        const cqham_band_t *b = band();
        lo = b ? b->start : s.freq - 100000;
        hi = b ? b->end : s.freq + 100000;
    }
    double  span = (double)(hi - lo);
    int64_t grid = span > 800000 ? 100000 : (span > 200000 ? 20000 : 5000);
    const int64_t *ch_list = NULL;
    int            ch_n = remote ? 0 : cqham_channels(band_idx(), &ch_list);

    for (int k = 1; k < 5; k++) {
        int y = SP_H * k / 5;
        for (int x = 0; x < SC_W; x++)
            canvas_buf[y * SC_W + x].full = blend(canvas_buf[y * SC_W + x].full, rgb(42, 168, 255), 0.10f);
    }
    if (ch_n) {
        int cur = cqham_channel_nearest(band_idx(), s.freq);
        for (int i = 0; i < ch_n; i++) {
            int x = (int)((ch_list[i] - lo) / span * SC_W);
            for (int y = 0; y < SP_H; y++) canvas_px(x, y, blend(canvas_buf[(y < SC_H ? y : 0) * SC_W + (x < 0 ? 0 : (x >= SC_W ? SC_W - 1 : x))].full,
                                                               i == cur ? rgb(52, 208, 88) : rgb(42, 168, 255),
                                                               i == cur ? 0.6f : 0.12f));
        }
    } else {
        for (int64_t f = ((lo + grid - 1) / grid) * grid; f < hi; f += grid) {
            int x = (int)((f - lo) / span * SC_W);
            for (int y = 0; y < SP_H; y++)
                if (x >= 0 && x < SC_W)
                    canvas_buf[y * SC_W + x].full = blend(canvas_buf[y * SC_W + x].full, rgb(42, 168, 255), 0.10f);
        }
    }

    /* passband */
    int64_t vfo = remote ? s.scope_txf : s.freq;
    int     bw  = s.mode == CQHAM_FM ? 8000 : (s.mode == CQHAM_AM ? 6000 : (s.mode == CQHAM_CW ? 500 : 2600));
    int64_t pb0 = vfo, pb1 = vfo;
    if (s.mode == CQHAM_LSB) pb0 = vfo - bw;
    else if (s.mode == CQHAM_USB || s.mode == CQHAM_CW || s.mode == CQHAM_DIGI) pb1 = vfo + bw;
    else { pb0 = vfo - bw / 2; pb1 = vfo + bw / 2; }
    int px0 = (int)((pb0 - lo) / span * SC_W), px1 = (int)((pb1 - lo) / span * SC_W);
    if (px1 <= px0) px1 = px0 + 1;
    for (int x = px0 < 0 ? 0 : px0; x <= px1 && x < SC_W; x++)
        for (int y = 0; y < SP_H; y++)
            canvas_buf[y * SC_W + x].full = blend(canvas_buf[y * SC_W + x].full, rgb(42, 168, 255), 0.12f);

    /* filled trace */
    int prev_y = -1;
    for (int x = 0; x < SC_W; x++) {
        int top = (int)(SP_H - 2 - lvl[x] * (SP_H - 6));
        if (top < 0) top = 0;
        for (int y = top; y < SP_H; y++) {
            float    k   = (float)(y - top) / (SP_H - top + 1);
            uint32_t col = remote ? blend(rgb(22, 224, 255), rgb(10, 40, 120), k)
                                  : blend(rgb(120, 230, 255), rgb(20, 120, 180), k);
            float    a   = remote ? 0.55f - 0.45f * k : 0.55f - 0.40f * k;
            canvas_buf[y * SC_W + x].full = blend(canvas_buf[y * SC_W + x].full, col, a);
        }
        uint32_t line = remote ? rgb(0x9b, 0xec, 0xff) : rgb(0x8f, 0xe8, 0xff);
        if (prev_y >= 0) {
            int y0 = prev_y < top ? prev_y : top, y1 = prev_y < top ? top : prev_y;
            for (int y = y0; y <= y1; y++) canvas_px(x, y, line);
        } else {
            canvas_px(x, top, line);
        }
        prev_y = top;
    }

    /* VFO / TX marker */
    int vx = (int)((vfo - lo) / span * SC_W);
    uint32_t vcol = remote ? (s.scope_tx ? rgb(255, 80, 100) : rgb(90, 255, 160)) : rgb(0xff, 0xcf, 0x3f);
    for (int y = 0; y < SC_H; y++) canvas_px(vx, y, y < SP_H ? vcol : blend(canvas_buf[y * SC_W + (vx < 0 ? 0 : (vx >= SC_W ? SC_W - 1 : vx))].full, vcol, 0.7f));

    /* axis strip */
    for (int y = SP_H; y < WF_Y; y++)
        for (int x = 0; x < SC_W; x++) canvas_buf[y * SC_W + x].full = rgb(0x0a, 0x18, 0x26);

    /* waterfall: one row every 90 ms */
    int64_t now = now_ms();
    if (now - last_wf_ms >= 90) {
        last_wf_ms = now;
        memmove(&canvas_buf[(WF_Y + 1) * SC_W], &canvas_buf[WF_Y * SC_W], (size_t)(WF_H - 1) * SC_W * sizeof(lv_color_t));
        for (int x = 0; x < SC_W; x++) canvas_buf[WF_Y * SC_W + x].full = wf_color(lvl[x]);
    }
    lv_obj_invalidate(canvas);
}

/* Axis labels are LVGL labels over the strip; refreshed when the span moves. */
static lv_obj_t *axis_lbl[5];
static int64_t   axis_lo, axis_hi;

static void axis_update(void) {
    bool    remote = s.remote && s.scope_on && s.scope_fe > s.scope_fs;
    int64_t lo, hi;
    if (remote) {
        lo = s.scope_fs;
        hi = s.scope_fe;
    } else {
        const cqham_band_t *b = band();
        if (!b) return;
        lo = b->start;
        hi = b->end;
    }
    if (lo == axis_lo && hi == axis_hi) return;
    axis_lo = lo;
    axis_hi = hi;
    for (int i = 0; i < 5; i++) {
        int64_t f = lo + (hi - lo) * (2 * i + 1) / 10;
        char    buf[16];
        snprintf(buf, sizeof(buf), "%.3f", f / 1e6);
        lv_label_set_text(axis_lbl[i], buf);
    }
}

/***** Periodic update *****/

static void update_smeter(void) {
    bool  conn    = s.status == CQHAM_ST_ONLINE;
    bool  carrier = s.mode == CQHAM_FM || s.mode == CQHAM_AM;
    float rx      = cqham_audio_rx_level();
    float pf      = 0.4f + 0.6f * (s.talker_power > 100 ? 1.0f : s.talker_power / 100.0f);
    bool  present = s.talker[0] && !s.tx_on;
    float target;

    if (!conn || s.tx_on) {
        target = 0.0f;
    } else if (present && s.talker_sig >= 0.0f) {
        target = s.talker_sig;
    } else if (carrier) {
        target = present ? fminf(1.0f, 0.55f + 0.45f * s.talker_power / 100.0f) : noise_floor * 0.5f;
    } else {
        target = fminf(1.0f, noise_floor * 1.3f + rx * 2.0f * pf);
    }
    float k = target > s_meter ? (carrier ? 0.3f : 0.5f) : 0.06f;
    s_meter += (target - s_meter) * k;
}

static void update_texts(void) {
    char buf[160];
    const cqham_band_t *b = band();

    /* tags */
    char step[16];
    if (b && (b->cb || b->pmr || b->swl)) snprintf(step, sizeof(step), "CANAL");
    else if (conf.step >= 1000) snprintf(step, sizeof(step), "%d kHz", conf.step / 1000);
    else snprintf(step, sizeof(step), "%d Hz", conf.step);

    char extra[32] = "";
    if (b && b->rooms) snprintf(extra, sizeof(extra), "   SALA %c", s.room == 'b' ? 'B' : 'A');
    if (b && (b->cb || b->pmr || b->swl)) {
        char ch[16];
        snprintf(ch, sizeof(ch), "   CH %d", cqham_channel_nearest(band_idx(), s.freq) + 1);
        strncat(extra, ch, sizeof(extra) - strlen(extra) - 1);
    }
    snprintf(buf, sizeof(buf), "#a8d8ff %s . %s#   #ffd27a PWR %dW#   #ffd27a PASO %s#%s%s",
             b ? b->name : "--", cqham_mode_name(s.mode), s.power, step, extra,
             s.remote ? "   #5affa0 REMOTO#" : "");
    if (strcmp(lv_label_get_text(lbl_tags), buf)) lv_label_set_text(lbl_tags, buf);

    /* frequency */
    fmt_freq(s.freq, buf, sizeof(buf));
    if (strcmp(lv_label_get_text(lbl_freq), buf)) lv_label_set_text(lbl_freq, buf);

    /* status line */
    uint32_t col = 0x9fc7e8;
    if (s.tx_on) {
        snprintf(buf, sizeof(buf), "TRANSMITIENDO %dW", s.power);
        col = 0xff4040;
    } else if (s.ptt) {
        snprintf(buf, sizeof(buf), "PTT... esperando canal");
        col = 0xffcf3f;
    } else if (s.status == CQHAM_ST_ONLINE && s.talker[0]) {
        snprintf(buf, sizeof(buf), "%s  %s %s", s.talker, s.talker_name, s.talker_country);
        col = 0xff3b30;
    } else if (s.status == CQHAM_ST_ONLINE) {
        if (s.remote && s.link[0]) snprintf(buf, sizeof(buf), "CONECTADO  .  REMOTO %s%s%s", s.link,
                                            s.holder[0] ? "  control: " : "", s.holder);
        else snprintf(buf, sizeof(buf), "CONECTADO");
    } else {
        snprintf(buf, sizeof(buf), "%s", s.status_text[0] ? s.status_text : "Desconectado");
        col = s.status == CQHAM_ST_STOPPED || s.status == CQHAM_ST_NEED_LOGIN ? 0xff6b6b : 0xffcf3f;
    }
    if (strcmp(lv_label_get_text(lbl_status), buf)) {
        lv_label_set_text(lbl_status, buf);
        lv_obj_set_style_text_color(lbl_status, lv_color_hex(col), 0);
    }

    /* left: connection + users on the channel */
    if (s.status == CQHAM_ST_ONLINE) snprintf(buf, sizeof(buf), "#4cd964 ON# %s  .  %d en red", s.dispcall, s.online);
    else snprintf(buf, sizeof(buf), "#ff6b6b OFF#  CQHAM");
    if (strcmp(lv_label_get_text(lbl_conn), buf)) lv_label_set_text(lbl_conn, buf);

    buf[0] = '\0';
    for (int i = 0; i < s.users_n && i < 6; i++) {
        bool talking = s.talker[0] && !strcmp(s.users[i], s.talker);
        char line[40];
        snprintf(line, sizeof(line), "%s%s%s\n", talking ? "#4cd964 " : "", s.users[i], talking ? "#" : "");
        strncat(buf, line, sizeof(buf) - strlen(buf) - 1);
    }
    if (!s.users_n) snprintf(buf, sizeof(buf), "(nadie en el canal)");
    if (strcmp(lv_label_get_text(lbl_users), buf)) lv_label_set_text(lbl_users, buf);

    /* right panel values */
    snprintf(buf, sizeof(buf), "%d W", s.power);
    if (strcmp(lv_label_get_text(lbl_pwr), buf)) lv_label_set_text(lbl_pwr, buf);
    if (strcmp(lv_label_get_text(lbl_step), step)) lv_label_set_text(lbl_step, step);
    snprintf(buf, sizeof(buf), "VOL %d   MIC %d", conf.volume, conf.mic_gain);
    if (strcmp(lv_label_get_text(lbl_vol), buf)) lv_label_set_text(lbl_vol, buf);
    lv_bar_set_value(mic_bar, (int)(cqham_audio_mic_level() * 100), LV_ANIM_OFF);

    led_set(led_tx, s.tx_on, 0xff453a);
    led_set(led_rx, !s.tx_on && s.talker[0], 0x4cd964);
    led_set(led_sq, s.status == CQHAM_ST_ONLINE, 0xffd60a);

    /* notices */
    if (s.notice_seq != notice_seen) {
        notice_seen = s.notice_seq;
        show_notice(s.notice, s.notice_alert);
    } else if (notice_until && now_ms() > notice_until) {
        notice_until = 0;
        lv_label_set_text(lbl_notice, "");
    }
}

static void timer_cb(lv_timer_t *t) {
    (void)t;
    cqham_client_get(&s);
    cqham_client_config(&conf);
    tick_n++;

    float prev = s_meter;
    update_smeter();
    if (fabsf(prev - s_meter) > 0.002f) lv_obj_invalidate(meter_obj);

    update_texts();
    axis_update();
    scope_render();
}

/***** Actions *****/

static void tune_to(int64_t f, cqham_mode_t m) {
    if (s.remote && s.link[0]) {
        rq_freq = f;   /* sent by the rate limiter */
        s.freq  = f;
        return;
    }
    cqham_client_tune(f, m);
    s.freq = f;
    s.mode = m;
}

static void band_step(int dir) {
    int n  = cqham_bands_count();
    int bi = band_idx();
    bi     = ((bi < 0 ? 0 : bi) + dir + n) % n;
    const cqham_band_t *b = cqham_band_at(bi);
    if (s.remote) cqham_client_remote(false);
    tune_to(cqham_band_default_freq(bi), b->mode);
    voice_say_text_fmt("%s", b->name);
}

static void mode_key(keypad_key_t key) {
    const cqham_band_t *b = band();
    if (b && b->fm_only) return;
    cqham_mode_t m = s.mode;
    switch (key) {
        case KEYPAD_MODE_SSB: m = (s.mode == CQHAM_LSB) ? CQHAM_USB : CQHAM_LSB; break;
        case KEYPAD_MODE_AM:  m = (s.mode == CQHAM_AM) ? CQHAM_FM : CQHAM_AM; break;
        case KEYPAD_MODE_CW:  m = CQHAM_CW; break;
        default: return;
    }
    cqham_client_tune(s.freq, m);
    s.mode = m;
    voice_say_text_fmt("%s", cqham_mode_name(m));
}

static void dial(int32_t diff) {
    const cqham_band_t *b = band();
    if (!b || diff == 0) return;

    knob_angle += diff * 0.12f;
    lv_obj_invalidate(knob_obj);

    const int64_t *list;
    int            n = cqham_channels(band_idx(), &list);
    int64_t        f;
    if (n && !s.remote) {
        int ch = cqham_channel_nearest(band_idx(), s.freq) + (diff > 0 ? 1 : -1);
        if (ch < 0) ch = 0;
        if (ch >= n) ch = n - 1;
        f = list[ch];
    } else {
        int step = conf.step > 0 ? conf.step : 1000;
        f        = (s.freq / step + diff) * step;
        if (f < b->start) f = b->start;
        if (f > b->end) f = b->end;
    }
    tune_to(f, s.mode);
}

/* Remote dial: at most one request per 85 ms */
static void remote_rate_cb(lv_timer_t *t) {
    (void)t;
    if (!rq_freq) return;
    int64_t now = now_ms();
    if (now - rq_last_ms < 85) return;
    rq_last_ms = now;
    cqham_client_remote_qsy(rq_freq);
    rq_freq = 0;
}
static lv_timer_t *rate_timer;

static void ptt(bool on) {
    cqham_client_ptt(on);
}

/***** Login (two text windows: callsign, then password) *****/

static char login_call[CQHAM_CALL_LEN];
static void reopen_cb(lv_timer_t *t);

static bool pass_ok(void) {
    cqham_client_set_login(login_call, textarea_window_get());
    textarea_window_close();
    show_notice("Entrando en CQHAM...", false);
    return true;
}

static bool login_cancel(void) {
    textarea_window_close();
    return true;
}

static bool call_ok(void) {
    snprintf(login_call, sizeof(login_call), "%s", textarea_window_get());
    textarea_window_close();
    if (!login_call[0]) return true;
    /* the second window after this one has gone */
    lv_timer_t *t = lv_timer_create(reopen_cb, 50, NULL);
    lv_timer_set_repeat_count(t, 1);
    return true;
}

static void reopen_cb(lv_timer_t *t) {
    (void)t;
    textarea_window_open_w_label(pass_ok, login_cancel, "Contrasena de CQHAM");
    lv_obj_t *text = textarea_window_text();
    lv_textarea_set_max_length(text, 60);
    lv_textarea_set_password_mode(text, true);
}

static void login_start(void) {
    textarea_window_open_w_label(call_ok, login_cancel, "Indicativo de CQHAM (cuenta propia de la radio)");
    lv_obj_t *text = textarea_window_text();
    lv_textarea_set_accepted_chars(text, "0123456789/ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    lv_textarea_set_max_length(text, CQHAM_CALL_LEN - 1);
    textarea_window_set(conf.callsign);
}

/***** Buttons *****/

static buttons_page_t page1, page2, page3;

static const char *step_label(void) {
    static char b[24];
    if (conf.step >= 1000) snprintf(b, sizeof(b), "PASO\n%d kHz", conf.step / 1000);
    else snprintf(b, sizeof(b), "PASO\n%d Hz", conf.step);
    return b;
}

static const char *pwr_label(void) {
    static char b[24];
    snprintf(b, sizeof(b), "POT\n%d W", s.power);
    return b;
}

static const char *mic_label(void) {
    static char b[24];
    snprintf(b, sizeof(b), "MIC\n%d", conf.mic_gain);
    return b;
}

static const char *ptt_label(void) {
    return s.ptt ? "PTT\nON" : "PTT\n(pulsar)";
}

static const char *room_label(void) {
    return s.room == 'b' ? "SALA\nB (11m)" : "SALA\nA (11m)";
}

static const char *remote_label(void) {
    return s.remote ? "REMOTO\nON" : "REMOTO\nOFF";
}

static void btn_ptt_cb(button_data_t *d);
static void btn_step_cb(button_data_t *d);
static void btn_pwr_cb(button_data_t *d);
static void btn_exit_cb(button_data_t *d);
static void btn_login_cb(button_data_t *d);
static void btn_mic_cb(button_data_t *d);
static void btn_room_cb(button_data_t *d);
static void btn_remote_cb(button_data_t *d);
static void btn_grant_cb(button_data_t *d);
static void btn_deny_cb(button_data_t *d);
static void btn_probe_cb(button_data_t *d);
static void btn_go_remote_cb(button_data_t *d);

static button_data_t b_p1     = {.type = BTN_TEXT, .label = "(CQHAM 1:3)", .press = button_next_page_cb,
                                 .hold = button_prev_page_cb, .next = &page2, .prev = &page3};
static button_data_t b_p2     = {.type = BTN_TEXT, .label = "(CQHAM 2:3)", .press = button_next_page_cb,
                                 .hold = button_prev_page_cb, .next = &page3, .prev = &page1};
static button_data_t b_p3     = {.type = BTN_TEXT, .label = "(CQHAM 3:3)", .press = button_next_page_cb,
                                 .hold = button_prev_page_cb, .next = &page1, .prev = &page2};
static button_data_t b_ptt    = {.type = BTN_TEXT_FN, .label_fn = ptt_label, .press = btn_ptt_cb};
static button_data_t b_step   = {.type = BTN_TEXT_FN, .label_fn = step_label, .press = btn_step_cb};
static button_data_t b_pwr    = {.type = BTN_TEXT_FN, .label_fn = pwr_label, .press = btn_pwr_cb};
static button_data_t b_exit   = {.type = BTN_TEXT, .label = "SALIR", .press = btn_exit_cb};
static button_data_t b_login  = {.type = BTN_TEXT, .label = "CUENTA", .press = btn_login_cb};
static button_data_t b_mic    = {.type = BTN_TEXT_FN, .label_fn = mic_label, .press = btn_mic_cb};
static button_data_t b_room   = {.type = BTN_TEXT_FN, .label_fn = room_label, .press = btn_room_cb};
static button_data_t b_remote = {.type = BTN_TEXT_FN, .label_fn = remote_label, .press = btn_remote_cb};
static button_data_t b_probe  = {.type = BTN_TEXT, .label = "BUSCAR\nREMOTO", .press = btn_probe_cb};
static button_data_t b_go     = {.type = BTN_TEXT, .label = "IR AL\nREMOTO", .press = btn_go_remote_cb};
static button_data_t b_grant  = {.type = BTN_TEXT, .label = "CEDER\nCONTROL", .press = btn_grant_cb};
static button_data_t b_deny   = {.type = BTN_TEXT, .label = "NO\nCEDER", .press = btn_deny_cb};

static buttons_page_t page1 = {{&b_p1, &b_ptt, &b_step, &b_pwr, &b_exit}};
static buttons_page_t page2 = {{&b_p2, &b_login, &b_mic, &b_room, &b_remote}};
static buttons_page_t page3 = {{&b_p3, &b_probe, &b_go, &b_grant, &b_deny}};

static void btn_ptt_cb(button_data_t *d) {
    ptt_by_key = !s.ptt;
    ptt(!s.ptt);
    s.ptt = !s.ptt;
    buttons_refresh(d);
}

static void btn_step_cb(button_data_t *d) {
    int n = sizeof(steps) / sizeof(steps[0]), i = 0;
    while (i < n && steps[i] != conf.step) i++;
    conf.step = steps[(i + 1) % n];
    cqham_client_set_step(conf.step);
    buttons_refresh(d);
}

static void btn_pwr_cb(button_data_t *d) {
    int n = sizeof(powers) / sizeof(powers[0]), i = 0;
    while (i < n && powers[i] != s.power) i++;
    s.power = powers[(i + 1) % n];
    cqham_client_set_power(s.power);
    buttons_refresh(d);
}

static void btn_exit_cb(button_data_t *d) {
    (void)d;
    dialog_destruct(&dialog);
}

static void btn_login_cb(button_data_t *d) {
    (void)d;
    login_start();
}

static void btn_mic_cb(button_data_t *d) {
    conf.mic_gain = conf.mic_gain >= 100 ? 20 : conf.mic_gain + 10;
    cqham_client_set_mic_gain(conf.mic_gain);
    buttons_refresh(d);
}

static void btn_room_cb(button_data_t *d) {
    s.room = s.room == 'b' ? 'a' : 'b';
    cqham_client_set_room(s.room);
    buttons_refresh(d);
}

static void btn_remote_cb(button_data_t *d) {
    s.remote = !s.remote;
    cqham_client_remote(s.remote);
    memset(rem_cur, 0, sizeof(rem_cur));
    show_notice(s.remote ? "Scope remoto: sintoniza la frecuencia del remoto" : "Scope de CQHAM", false);
    buttons_refresh(d);
}

static void btn_probe_cb(button_data_t *d) {
    (void)d;
    cqham_client_remote_probe();
    if (s.link[0]) {
        char buf[96], f[24];
        fmt_freq(s.link_freq, f, sizeof(f));
        snprintf(buf, sizeof(buf), "Remoto %s en %s", s.link, f);
        show_notice(buf, false);
    } else {
        show_notice("Buscando remoto en esta banda...", false);
    }
}

static void btn_go_remote_cb(button_data_t *d) {
    (void)d;
    if (!s.link[0] || !s.link_freq) {
        show_notice("No hay remoto en esta banda: pulsa BUSCAR", true);
        return;
    }
    cqham_client_tune(s.link_freq, s.mode);
    s.remote = true;
    cqham_client_remote(true);
    buttons_refresh(&b_remote);
}

static void btn_grant_cb(button_data_t *d) {
    (void)d;
    cqham_client_ctrl_answer(true);
}

static void btn_deny_cb(button_data_t *d) {
    (void)d;
    cqham_client_ctrl_answer(false);
}

/***** The base's own TX (hand mic PTT) *****/

static void radio_tx_cb(void *sub, lv_msg_t *m) {
    (void)sub;
    (void)m;
    /* The hand mic PTT is wired to the base: it keys the radio (at 0 W, the
     * power is down while this window is open). Use it as CQHAM's PTT. */
    if (!s.ptt) {
        ptt_by_mic = true;
        ptt(true);
    }
}

static void radio_rx_cb(void *sub, lv_msg_t *m) {
    (void)sub;
    (void)m;
    if (ptt_by_mic) {
        ptt_by_mic = false;
        ptt(false);
    }
}

/***** Construct / destruct *****/

static void construct_cb(lv_obj_t *parent) {
    dialog.obj = dialog_init(parent);
    lv_obj_t *o = dialog.obj;

    lv_obj_set_style_bg_img_src(o, NULL, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(o, lv_color_hex(C_BG1), 0);
    lv_obj_set_style_bg_grad_color(o, lv_color_hex(C_BG2), 0);
    lv_obj_set_style_bg_grad_dir(o, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_radius(o, 8, 0);

    lv_group_add_obj(keyboard_group, o);
    lv_obj_add_event_cb(o, key_cb, LV_EVENT_KEY, NULL);

    /* Left panel: signal level */
    lv_obj_t *lp = mk_panel(o, 2, PANEL_W);
    mk_label(lp, &sony_14, C_TITLE, 0, 4, PANEL_W, LV_TEXT_ALIGN_CENTER, "SIGNAL LEVEL CQHAM");
    meter_obj = lv_obj_create(lp);
    lv_obj_remove_style_all(meter_obj);
    lv_obj_set_pos(meter_obj, (PANEL_W - MTR_W) / 2, 24);
    lv_obj_set_size(meter_obj, MTR_W, MTR_H);
    lv_obj_add_event_cb(meter_obj, meter_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    led_tx = mk_led(lp, 14, 164, "TX");
    led_rx = mk_led(lp, 66, 164, "RX");
    led_sq = mk_led(lp, 118, 164, "ON");
    lbl_conn = mk_label(lp, &sony_16, 0xdfe6ee, 8, 188, PANEL_W - 12, LV_TEXT_ALIGN_LEFT, "");
    lv_label_set_recolor(lbl_conn, true);
    mk_label(lp, &sony_14, C_TITLE, 8, 210, PANEL_W - 12, LV_TEXT_ALIGN_LEFT, "EN EL CANAL");
    lbl_users = mk_label(lp, &sony_16, 0xcdd6dd, 8, 228, PANEL_W - 12, LV_TEXT_ALIGN_LEFT, "");
    lv_label_set_recolor(lbl_users, true);
    lv_label_set_long_mode(lbl_users, LV_LABEL_LONG_CLIP);
    lv_obj_set_height(lbl_users, 108);

    /* Centre: the blue LCD */
    lv_obj_t *lcd = lv_obj_create(o);
    lv_obj_remove_style_all(lcd);
    lv_obj_set_pos(lcd, LCD_X, 2);
    lv_obj_set_size(lcd, LCD_W, DIALOG_HEIGHT - 4);
    lv_obj_set_style_radius(lcd, 8, 0);
    lv_obj_set_style_bg_opa(lcd, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(lcd, lv_color_hex(C_LCD1), 0);
    lv_obj_set_style_bg_grad_color(lcd, lv_color_hex(C_LCD2), 0);
    lv_obj_set_style_bg_grad_dir(lcd, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(lcd, 2, 0);
    lv_obj_set_style_border_color(lcd, lv_color_hex(0x3d6a8e), 0);
    lv_obj_set_style_shadow_width(lcd, 12, 0);
    lv_obj_set_style_shadow_color(lcd, lv_color_hex(0x2a6aa0), 0);
    lv_obj_clear_flag(lcd, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lbl_tags = mk_label(lcd, &sony_16, 0xa8d8ff, 8, 4, LCD_W - 16, LV_TEXT_ALIGN_LEFT, "");
    lv_label_set_recolor(lbl_tags, true);
    lbl_freq = mk_label(lcd, &mono_48, 0xd8ecff, 0, 24, LCD_W, LV_TEXT_ALIGN_CENTER, "");
    lv_obj_set_style_text_letter_space(lbl_freq, 1, 0);
    lbl_status = mk_label(lcd, &sony_18, 0x9fc7e8, 0, 76, LCD_W, LV_TEXT_ALIGN_CENTER, "");
    lbl_notice = mk_label(lcd, &sony_16, 0xffcf3f, 0, 98, LCD_W, LV_TEXT_ALIGN_CENTER, "");

    canvas_buf = lv_mem_alloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(SC_W, SC_H));
    canvas     = lv_canvas_create(lcd);
    if (canvas_buf) {
        lv_canvas_set_buffer(canvas, canvas_buf, SC_W, SC_H, LV_IMG_CF_TRUE_COLOR);
        lv_canvas_fill_bg(canvas, lv_color_hex(0x02060a), LV_OPA_COVER);
    }
    lv_obj_set_pos(canvas, 2, DIALOG_HEIGHT - 4 - SC_H - 6);
    for (int i = 0; i < 5; i++) {
        axis_lbl[i] = mk_label(canvas, &sony_14, 0x7d9ab3, SC_W * (2 * i + 1) / 10 - 30, SP_H - 1, 60,
                               LV_TEXT_ALIGN_CENTER, "");
    }
    axis_lo = axis_hi = 0;

    /* Right panel: tuning */
    lv_obj_t *rp = mk_panel(o, DIALOG_WIDTH - PANEL_W - 2, PANEL_W);
    mk_label(rp, &sony_14, C_TITLE, 0, 4, PANEL_W, LV_TEXT_ALIGN_CENTER, "TUNING");
    knob_obj = lv_obj_create(rp);
    lv_obj_remove_style_all(knob_obj);
    lv_obj_set_pos(knob_obj, (PANEL_W - KNOB_D) / 2, 26);
    lv_obj_set_size(knob_obj, KNOB_D, KNOB_D);
    lv_obj_add_event_cb(knob_obj, knob_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_set_style_pad_all(knob_obj, 0, 0);
    mk_label(rp, &sony_14, 0x7d9ab3, 0, 146, PANEL_W, LV_TEXT_ALIGN_CENTER, "dial del X6100");

    lbl_pwr = mk_label(rp, &sony_22, 0xffd27a, 0, 166, PANEL_W, LV_TEXT_ALIGN_CENTER, "");
    mk_label(rp, &sony_14, C_TITLE, 0, 192, PANEL_W, LV_TEXT_ALIGN_CENTER, "PWR");
    lbl_step = mk_label(rp, &sony_22, 0xffd27a, 0, 208, PANEL_W, LV_TEXT_ALIGN_CENTER, "");
    mk_label(rp, &sony_14, C_TITLE, 0, 234, PANEL_W, LV_TEXT_ALIGN_CENTER, "STEP");
    lbl_vol = mk_label(rp, &sony_16, 0xdfe6ee, 0, 254, PANEL_W, LV_TEXT_ALIGN_CENTER, "");
    mic_bar = lv_bar_create(rp);
    lv_obj_set_pos(mic_bar, 14, 278);
    lv_obj_set_size(mic_bar, PANEL_W - 28, 8);
    lv_bar_set_range(mic_bar, 0, 100);
    lv_obj_set_style_bg_color(mic_bar, lv_color_hex(0x0a0f14), LV_PART_MAIN);
    lv_obj_set_style_bg_color(mic_bar, lv_color_hex(0x2f8ae0), LV_PART_INDICATOR);
    lbl_mic  = mk_label(rp, &sony_14, C_TITLE, 0, 290, PANEL_W, LV_TEXT_ALIGN_CENTER, "MIC");
    lbl_hint = mk_label(rp, &sony_14, 0x7d9ab3, 4, 306, PANEL_W - 8, LV_TEXT_ALIGN_CENTER, "PTT: tecla PTT o F1 micro");

    /* the radio's own spectrum and waterfall are drawn straight to the
     * screen: off while this window covers them */
    waterfall_set_enabled(false);
    spectrum_set_enabled(false);

    /* never on RF: radio TX power to 0 while CQHAM is open */
    x6100_control_txpwr_set(0.0f);

    tx_sub = lv_msg_subscribe(MSG_RADIO_TX, radio_tx_cb, NULL);
    rx_sub = lv_msg_subscribe(MSG_RADIO_RX, radio_rx_cb, NULL);

    cqham_client_config(&conf);
    cqham_client_start();
    cqham_audio_start();
    cqham_client_get(&s);

    memset(lvl, 0, sizeof(lvl));
    memset(rem_cur, 0, sizeof(rem_cur));
    s_meter     = 0.0f;
    notice_seen = s.notice_seq;
    ptt_by_mic  = false;
    ptt_by_key  = false;

    buttons_unload_page();
    buttons_load_page(&page1);

    timer      = lv_timer_create(timer_cb, 50, NULL);
    rate_timer = lv_timer_create(remote_rate_cb, 20, NULL);

    if (!conf.callsign[0] && !conf.token[0]) {
        show_notice("Pulsa CUENTA (pagina 2) o edita cqham.txt en la tarjeta", true);
    }
}

static void destruct_cb(void) {
    if (timer) {
        lv_timer_del(timer);
        timer = NULL;
    }
    if (rate_timer) {
        lv_timer_del(rate_timer);
        rate_timer = NULL;
    }
    if (tx_sub) {
        lv_msg_unsubscribe(tx_sub);
        tx_sub = NULL;
    }
    if (rx_sub) {
        lv_msg_unsubscribe(rx_sub);
        rx_sub = NULL;
    }

    cqham_client_stop();
    cqham_audio_stop();

    /* the radio as it was */
    x6100_control_txpwr_set(param_f_get(cfg.pwr()));
    waterfall_set_enabled(true);
    spectrum_set_enabled(true);

    if (canvas_buf) {
        lv_mem_free(canvas_buf);
        canvas_buf = NULL;
    }
    canvas = NULL;
}

static void key_cb(lv_event_t *e) {
    uint32_t key = *((uint32_t *)lv_event_get_param(e));

    switch (key) {
        case LV_KEY_ESC:
            dialog_destruct(&dialog);
            break;

        case KEY_VOL_LEFT_EDIT:
        case KEY_VOL_LEFT_SELECT:
            cqham_client_set_volume(conf.volume - 5);
            break;

        case KEY_VOL_RIGHT_EDIT:
        case KEY_VOL_RIGHT_SELECT:
            cqham_client_set_volume(conf.volume + 5);
            break;
    }
}

/***** Hooks from main_screen.c *****/

bool dialog_cqham_rotary(int32_t diff, uint16_t dt) {
    (void)dt;
    if (!dialog.run) return false;
    dial(diff);
    return true;
}

bool dialog_cqham_keypad(const event_keypad_t *kp) {
    if (!dialog.run) return false;

    switch (kp->key) {
        case KEYPAD_BAND_UP:
        case KEYPAD_BAND_DOWN:
            if (kp->state == KEYPAD_RELEASE) band_step(kp->key == KEYPAD_BAND_UP ? 1 : -1);
            return true;

        case KEYPAD_MODE_SSB:
        case KEYPAD_MODE_AM:
        case KEYPAD_MODE_CW:
            if (kp->state == KEYPAD_RELEASE) mode_key(kp->key);
            return true;

        case KEYPAD_PTT:
            /* the front PTT key: software, so CQHAM only, never RF */
            if (kp->state == KEYPAD_PRESS) {
                ptt_by_key = true;
                ptt(true);
            } else if (kp->state == KEYPAD_RELEASE || kp->state == KEYPAD_LONG_RELEASE) {
                ptt_by_key = false;
                ptt(false);
            }
            return true;

        default:
            return false;
    }
}

bool dialog_cqham_hkey(const event_hkey_t *hk) {
    if (!dialog.run) return false;

    switch (hk->key) {
        case HKEY_F1:   /* hand mic F1: push to talk */
            if (hk->state == HKEY_PRESS) {
                ptt_by_key = true;
                ptt(true);
            } else if (hk->state == HKEY_RELEASE || hk->state == HKEY_LONG_RELEASE) {
                ptt_by_key = false;
                ptt(false);
            }
            return true;

        case HKEY_UP:
        case HKEY_DOWN:
            if (hk->state == HKEY_RELEASE || hk->state == HKEY_PRESS) {
                if (hk->state == HKEY_PRESS) dial(hk->key == HKEY_UP ? 1 : -1);
            }
            return true;

        default:
            return false;
    }
}
