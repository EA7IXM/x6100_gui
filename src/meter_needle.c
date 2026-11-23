/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Analog (needle) S-meter, EA7IXM.
 *
 *  A backlit amber face like a classic transceiver meter: S1..S9 on the
 *  left 60 % of the arc, +20/+40/+60 in red on the rest, and a needle that
 *  swings up fast and falls back slowly (mechanical ballistics), with a thin
 *  peak marker.
 *
 *  The pivot sits far below the meter, so only the top of a wide arc shows,
 *  as on a real rig's flat meter window. The static face is drawn once into
 *  a canvas; every frame only the needle is drawn on top of it.
 */

#include "meter_needle.h"

#include "meter.h"
#include "styles.h"

#include <math.h>
#include <stdlib.h>

/* Scale: S1..S9 over the first 60 %, S9..S9+60 over the last 40 %. */
#define SCALE_S9_FRAC   0.6f
#define SCALE_TOP_DB    (S9 + 60)

/* Ballistics, per 20 ms tick: fraction of the gap closed. */
#define TICK_MS         20
#define K_ATTACK        0.45f
#define K_RELEASE       0.10f
#define K_PEAK          0.25f

/* Face colours */
#define C_FACE_TOP      0xFFF2D8
#define C_FACE_BOTTOM   0xF5BE62
#define C_FACE_BORDER   0x5A4A2A
#define C_SCALE         0x1A1A1A
#define C_RED           0xC8102E
#define C_NEEDLE        0x101010
#define C_PEAK          0xD03020

static lv_obj_t     *canvas;
static lv_obj_t     *needle_obj;
static lv_color_t   *canvas_buf;
static lv_coord_t    W, H;

/* Arc geometry, relative to the object's top-left corner */
static float         cx, cy, R, half_angle;

static float         target_frac, pos_frac;
static float         peak_target_frac, peak_frac;
static float         drawn_frac = -1.0f, drawn_peak = -1.0f;
static float         db_now = S_MIN;

static lv_timer_t   *timer;
static lv_obj_t     *parent_obj;

static float db_to_frac(float db) {
    if (db <= S1) {
        return 0.0f;
    }
    if (db <= S9) {
        return SCALE_S9_FRAC * (db - S1) / (float)(S9 - S1);
    }
    float f = SCALE_S9_FRAC + (1.0f - SCALE_S9_FRAC) * (db - S9) / (float)(SCALE_TOP_DB - S9);
    return f > 1.03f ? 1.03f : f;   /* pins a little past the end, like a real meter */
}

static float frac_to_angle(float frac) {
    return -half_angle + frac * 2.0f * half_angle;
}

static void polar(float angle, float r, float *x, float *y) {
    *x = cx + r * sinf(angle);
    *y = cy - r * cosf(angle);
}

/***** Static face (drawn once) *****/

static void face_line(float x0, float y0, float x1, float y1, lv_coord_t width, uint32_t color) {
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color       = lv_color_hex(color);
    dsc.width       = width;
    dsc.round_start = 0;
    dsc.round_end   = 0;

    lv_point_t p[2] = {
        {(lv_coord_t)lroundf(x0), (lv_coord_t)lroundf(y0)},
        {(lv_coord_t)lroundf(x1), (lv_coord_t)lroundf(y1)},
    };
    lv_canvas_draw_line(canvas, p, 2, &dsc);
}

static void face_arc(float f0, float f1, lv_coord_t width, uint32_t color) {
    const int steps = 24;
    float     px, py;
    polar(frac_to_angle(f0), R, &px, &py);
    for (int i = 1; i <= steps; i++) {
        float x, y;
        polar(frac_to_angle(f0 + (f1 - f0) * i / steps), R, &x, &y);
        face_line(px, py, x, y, width, color);
        px = x;
        py = y;
    }
}

static void face_tick(float db, float len, lv_coord_t width, uint32_t color) {
    float a = frac_to_angle(db_to_frac(db));
    float x0, y0, x1, y1;
    polar(a, R, &x0, &y0);
    polar(a, R + len, &x1, &y1);
    face_line(x0, y0, x1, y1, width, color);
}

static void face_label(float db, const char *txt, uint32_t color) {
    float a = frac_to_angle(db_to_frac(db));
    float x, y;
    polar(a, R + 17.0f, &x, &y);

    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);
    dsc.color = lv_color_hex(color);
    dsc.font  = &sony_16;
    dsc.align = LV_TEXT_ALIGN_CENTER;

    lv_canvas_draw_text(canvas, (lv_coord_t)lroundf(x) - 20, (lv_coord_t)lroundf(y) - 9, 40, &dsc, txt);
}

static void draw_face(void) {
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_TRANSP);

    /* Backlit face */
    lv_draw_rect_dsc_t rect;
    lv_draw_rect_dsc_init(&rect);
    rect.radius        = 7;
    rect.bg_color      = lv_color_hex(C_FACE_TOP);
    rect.bg_grad.dir   = LV_GRAD_DIR_VER;
    rect.bg_grad.stops_count     = 2;
    rect.bg_grad.stops[0].color  = lv_color_hex(C_FACE_TOP);
    rect.bg_grad.stops[0].frac   = 0;
    rect.bg_grad.stops[1].color  = lv_color_hex(C_FACE_BOTTOM);
    rect.bg_grad.stops[1].frac   = 255;
    rect.border_color  = lv_color_hex(C_FACE_BORDER);
    rect.border_width  = 2;
    rect.border_opa    = LV_OPA_COVER;
    lv_canvas_draw_rect(canvas, 2, 2, W - 4, H - 4, &rect);

    /* Scale arc: black S-units, red over S9 */
    face_arc(0.0f, SCALE_S9_FRAC, 2, C_SCALE);
    face_arc(SCALE_S9_FRAC, 1.0f, 4, C_RED);

    /* S-unit ticks: major on odd units, minor between */
    static const struct { int db; const char *txt; } s_major[] = {
        {S1, "1"}, {S3, "3"}, {S5, "5"}, {S7, "7"}, {S9, "9"},
    };
    for (int db = S1; db <= S9; db += 6) {
        face_tick((float)db, 4.0f, 1, C_SCALE);
    }
    for (size_t i = 0; i < sizeof(s_major) / sizeof(s_major[0]); i++) {
        face_tick((float)s_major[i].db, 9.0f, 2, C_SCALE);
        face_label((float)s_major[i].db, s_major[i].txt, C_SCALE);
    }

    /* Over S9, in red */
    for (int over = 10; over <= 60; over += 10) {
        bool major = (over % 20) == 0;
        face_tick((float)(S9 + over), major ? 9.0f : 4.0f, major ? 2 : 1, C_RED);
    }
    face_label((float)(S9 + 20), "+20", C_RED);
    face_label((float)(S9 + 40), "+40", C_RED);
    face_label((float)(S9 + 60), "+60", C_RED);

    /* Corner caption */
    lv_draw_label_dsc_t lbl;
    lv_draw_label_dsc_init(&lbl);
    lbl.color = lv_color_hex(C_SCALE);
    lbl.font  = &sony_18;
    lv_canvas_draw_text(canvas, 10, H - 24, 20, &lbl, "S");
}

/***** Needle (drawn every frame) *****/

static void draw_needle_line(lv_draw_ctx_t *draw_ctx, const lv_area_t *c, float frac,
                             float r_from_bottom_pad, float r_extra, lv_coord_t width,
                             uint32_t color, lv_opa_t opa) {
    float a      = frac_to_angle(frac);
    float y_base = (float)(H - r_from_bottom_pad);
    float r0     = (cy - y_base) / cosf(a);
    float x0, y0, x1, y1;
    polar(a, r0, &x0, &y0);
    polar(a, R + r_extra, &x1, &y1);

    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(color);
    dsc.width = width;
    dsc.opa   = opa;

    lv_point_t p0 = {c->x1 + (lv_coord_t)lroundf(x0), c->y1 + (lv_coord_t)lroundf(y0)};
    lv_point_t p1 = {c->x1 + (lv_coord_t)lroundf(x1), c->y1 + (lv_coord_t)lroundf(y1)};
    lv_draw_line(draw_ctx, &dsc, &p0, &p1);
}

static void needle_draw_cb(lv_event_t *e) {
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
    lv_area_t      c;
    lv_obj_get_coords(lv_event_get_target(e), &c);

    /* Peak: a short thin red mark crossing the arc */
    if (peak_frac > pos_frac + 0.01f) {
        float a = frac_to_angle(peak_frac);
        float x0, y0, x1, y1;
        polar(a, R - 6.0f, &x0, &y0);
        polar(a, R + 9.0f, &x1, &y1);

        lv_draw_line_dsc_t dsc;
        lv_draw_line_dsc_init(&dsc);
        dsc.color = lv_color_hex(C_PEAK);
        dsc.width = 2;
        dsc.opa   = LV_OPA_70;
        lv_point_t p0 = {c.x1 + (lv_coord_t)lroundf(x0), c.y1 + (lv_coord_t)lroundf(y0)};
        lv_point_t p1 = {c.x1 + (lv_coord_t)lroundf(x1), c.y1 + (lv_coord_t)lroundf(y1)};
        lv_draw_line(draw_ctx, &dsc, &p0, &p1);
    }

    /* Soft shadow, then the needle itself */
    draw_needle_line(draw_ctx, &c, pos_frac + 0.004f, 4.0f, 6.0f, 3, 0x000000, LV_OPA_20);
    draw_needle_line(draw_ctx, &c, pos_frac, 4.0f, 8.0f, 2, C_NEEDLE, LV_OPA_COVER);

    /* Reading, bottom right */
    char buf[16];
    if (db_now <= S9) {
        int s = (int)lroundf((db_now - S1) / 6.0f) + 1;
        if (s < 0) {
            s = 0;
        }
        lv_snprintf(buf, sizeof(buf), "S%d", s);
    } else {
        lv_snprintf(buf, sizeof(buf), "S9+%d", (int)lroundf(db_now - S9));
    }

    lv_draw_label_dsc_t lbl;
    lv_draw_label_dsc_init(&lbl);
    lbl.color = lv_color_hex(db_now > S9 ? C_RED : C_SCALE);
    lbl.font  = &sony_18;
    lbl.align = LV_TEXT_ALIGN_RIGHT;
    lv_area_t ta = {c.x2 - 90, c.y2 - 24, c.x2 - 10, c.y2 - 4};
    lv_draw_label(draw_ctx, &lbl, &ta, buf, NULL);
}

static void tick_cb(lv_timer_t *t) {
    (void)t;
    float k = target_frac > pos_frac ? K_ATTACK : K_RELEASE;
    pos_frac += (target_frac - pos_frac) * k;
    peak_frac += (peak_target_frac - peak_frac) * K_PEAK;

    if (!needle_obj || lv_obj_has_flag(needle_obj, LV_OBJ_FLAG_HIDDEN)) {
        return;
    }
    if (fabsf(pos_frac - drawn_frac) > 0.0008f || fabsf(peak_frac - drawn_peak) > 0.002f) {
        drawn_frac = pos_frac;
        drawn_peak = peak_frac;
        lv_obj_invalidate(needle_obj);
    }
}

/***** API *****/

lv_obj_t *meter_needle_create(lv_obj_t *parent) {
    parent_obj = parent;
    lv_obj_update_layout(parent);
    W = lv_obj_get_width(parent);
    H = lv_obj_get_height(parent);
    lv_coord_t pad_l = lv_obj_get_style_pad_left(parent, LV_PART_MAIN);
    lv_coord_t pad_t = lv_obj_get_style_pad_top(parent, LV_PART_MAIN);

    /* Arc: apex 26 px from the top, edges 40 px in from the sides. Flat
     * enough that its ends stay above the two corner texts (on the radio
     * the first version's ends ran into "S" and the reading). */
    R          = 760.0f;
    cx         = W / 2.0f;
    cy         = 26.0f + R;
    half_angle = asinf((W / 2.0f - 40.0f) / R);

    canvas_buf = malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(W, H));
    if (!canvas_buf) {
        LV_LOG_ERROR("meter_needle: no memory for the face");
        return NULL;
    }

    canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(canvas, canvas_buf, W, H, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_add_flag(canvas, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_pos(canvas, -pad_l, -pad_t);
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
    draw_face();

    needle_obj = lv_obj_create(canvas);
    lv_obj_remove_style_all(needle_obj);
    lv_obj_set_size(needle_obj, W, H);
    lv_obj_set_pos(needle_obj, 0, 0);
    lv_obj_clear_flag(needle_obj, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(needle_obj, needle_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    timer = lv_timer_create(tick_cb, TICK_MS, NULL);

    return canvas;
}

void meter_needle_set(float db, float peak_db) {
    db_now           = db;
    target_frac      = db_to_frac(db);
    peak_target_frac = db_to_frac(peak_db);
}

void meter_needle_relayout(int apex_extra) {
    if (!canvas || !parent_obj) return;
    lv_obj_update_layout(parent_obj);
    lv_coord_t nw = lv_obj_get_width(parent_obj);
    lv_coord_t nh = lv_obj_get_height(parent_obj);
    if (nw != W || nh != H) {
        lv_color_t *nb = malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(nw, nh));
        if (!nb) return;
        W = nw;
        H = nh;
        lv_canvas_set_buffer(canvas, nb, W, H, LV_IMG_CF_TRUE_COLOR_ALPHA);
        free(canvas_buf);
        canvas_buf = nb;
        lv_obj_set_size(needle_obj, W, H);
    }
    cx         = W / 2.0f;
    cy         = 26.0f + apex_extra + R;
    half_angle = asinf((W / 2.0f - 40.0f) / R);
    draw_face();
    drawn_frac = drawn_peak = -1.0f;
    lv_obj_invalidate(canvas);
}

void meter_needle_show(bool show) {
    if (!canvas) {
        return;
    }
    if (show) {
        lv_obj_clear_flag(canvas, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(needle_obj, LV_OBJ_FLAG_HIDDEN);
        drawn_frac = -1.0f;
        lv_obj_invalidate(canvas);
    } else {
        lv_obj_add_flag(canvas, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(needle_obj, LV_OBJ_FLAG_HIDDEN);
    }
}
