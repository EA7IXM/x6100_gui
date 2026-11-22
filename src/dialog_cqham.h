/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM window (EA7IXM)
 */

#pragma once

#include "dialog.h"
#include "events.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern dialog_t *dialog_cqham;

/* While the window is open it takes the main dial, BAND, MODE, the front
 * PTT key and some hand-mic keys. Each returns true when it used the event. */
bool dialog_cqham_rotary(int32_t diff, uint16_t dt);
bool dialog_cqham_keypad(const event_keypad_t *kp);
bool dialog_cqham_hkey(const event_hkey_t *hk);

#ifdef __cplusplus
}
#endif
