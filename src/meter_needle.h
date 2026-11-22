/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Analog (needle) S-meter, EA7IXM.
 */

#pragma once

#include <stdbool.h>
#include "lvgl/lvgl.h"

/* Creates the needle meter filling the whole of `parent` (padding included). */
lv_obj_t *meter_needle_create(lv_obj_t *parent);

/* Target level in dBm and its peak; the needle eases toward it on its own. */
void meter_needle_set(float db, float peak_db);

void meter_needle_show(bool show);
