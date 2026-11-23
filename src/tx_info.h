/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2022-2023 Belousov Oleg aka R1CBU
 */

#pragma once

#include <unistd.h>
#include <stdint.h>

#include "lvgl/lvgl.h"

lv_obj_t * tx_info_init(lv_obj_t *parent);

void tx_info_update(float pwr, float vswr, float alc);

bool tx_info_refresh(uint8_t * prev_msg_id, float * alc_p, float * pwr_p, float * vswr_p);

/* EA7IXM: CQHAM open: the PWR row is MIC (the hand mic level, the gain as its number) */
void tx_info_cqham(bool on);
void tx_info_cqham_mic(float level, int gain);
