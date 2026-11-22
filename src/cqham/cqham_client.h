/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 *
 *  The CQHAM protocol (see X6100-CQHAM/PROTOCOLO-CLIENTE-CQHAM-X6100-OFICIAL.md):
 *  login, the WebSocket session (hello first, nothing before "accepted"),
 *  tuning, PTT, the band scope and the remote stations' scope. Runs its own
 *  thread; the window reads a snapshot.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cqham_bands.h"

#define CQHAM_CALL_LEN      16
#define CQHAM_MAX_STATIONS  48
#define CQHAM_MAX_NET       48
#define CQHAM_MAX_USERS     12
#define CQHAM_SCOPE_POINTS  2048

typedef enum {
    CQHAM_ST_IDLE,          /* not started */
    CQHAM_ST_NEED_LOGIN,    /* no callsign / password */
    CQHAM_ST_LOGIN,         /* POST /api/login */
    CQHAM_ST_CONNECTING,    /* TCP + TLS + WebSocket */
    CQHAM_ST_HELLO,         /* waiting for "accepted" */
    CQHAM_ST_ONLINE,
    CQHAM_ST_RETRY,         /* lost, reconnecting in a few seconds */
    CQHAM_ST_STOPPED,       /* kicked, banned, refused: no reconnect */
} cqham_status_t;

typedef struct {
    int64_t freq;           /* Hz */
    int     power;
    bool    tx;
    char    call[CQHAM_CALL_LEN];
} cqham_station_t;

/* One user of the whole network (the "cluster" message, every 1.5 s) */
typedef struct {
    char    call[CQHAM_CALL_LEN];
    char    name[24];
    char    band[8];
    char    mode[6];
    char    country[4];
    int64_t freq;           /* Hz */
    char    room;           /* 'a' / 'b' / 0 */
    bool    tx;
    bool    bot;
} cqham_user_t;

typedef struct {
    cqham_status_t  status;
    char            status_text[96];

    /* account */
    char            callsign[CQHAM_CALL_LEN];
    char            dispcall[CQHAM_CALL_LEN];
    bool            licensed;
    int             online;

    /* tuning */
    int64_t         freq;
    cqham_mode_t    mode;
    char            room;           /* 'a' / 'b' in 11 m */
    int             power;

    /* who talks on my channel */
    char            talker[CQHAM_CALL_LEN];
    char            talker_name[40];
    char            talker_country[8];
    int             talker_power;
    float           talker_sig;     /* <0: none */
    bool            talker_sreal;
    bool            tx_on;          /* my PTT accepted (I'm the talker) */
    bool            ptt;            /* my PTT pressed */

    int             users_n;
    char            users[CQHAM_MAX_USERS][CQHAM_CALL_LEN];
    int             channel_count;

    /* band activity (the CQHAM scope) */
    int             stations_n;
    cqham_station_t stations[CQHAM_MAX_STATIONS];

    /* network activity: everybody connected */
    int             net_n;
    cqham_user_t    net[CQHAM_MAX_NET];
    uint32_t        net_seq;

    /* last notice for the operator */
    char            notice[128];
    uint32_t        notice_seq;
    bool            notice_alert;   /* red: denied, muted... */

    /* remote station */
    char            link[24];       /* "" = no link on my channel */
    int64_t         link_freq;
    char            holder[CQHAM_CALL_LEN];
    char            ctrl_req_by[CQHAM_CALL_LEN];   /* someone asks me for the control */
    bool            remote;         /* the window is in remote-scope mode */
    bool            scope_on;       /* subscribed */
    bool            atu;
    uint32_t        scope_seq;
    int             scope_n;
    uint8_t         scope[CQHAM_SCOPE_POINTS];
    int             scope_max;      /* 160 (IC-7610) or 255 (SunSDR/Hermes) */
    int64_t         scope_fs, scope_fe, scope_txf;
    bool            scope_tx;
} cqham_state_t;

typedef struct {
    char    server[64];
    int     port;
    bool    tls;
    bool    verify;
    char    callsign[CQHAM_CALL_LEN];
    char    password[64];
    char    token[80];
    int64_t freq;
    int     mode;
    char    room;
    int     power;
    int     step;           /* Hz */
    int     mic_gain;       /* 50 = unity, 0..100 */
    int     volume;         /* 0..100 */
    int     ptt_micro;      /* hand mic PTT: 0 off, 1 CW (no carrier), 2 voice */
} cqham_config_t;

/* Settings file on the DATA partition (editable on a PC). */
#define CQHAM_CONFIG_PATH   "/mnt/cqham.txt"

void cqham_config_load(cqham_config_t *cfg);
void cqham_config_save(const cqham_config_t *cfg);

/* Start / stop the session (the window opens / closes). */
void cqham_client_start(void);
void cqham_client_stop(void);

void cqham_client_get(cqham_state_t *out);         /* snapshot */
void cqham_client_config(cqham_config_t *out);     /* current settings */

void cqham_client_set_login(const char *call, const char *password);
void cqham_client_tune(int64_t freq, cqham_mode_t mode);
void cqham_client_set_room(char room);
void cqham_client_set_power(int watts);
void cqham_client_set_step(int hz);
void cqham_client_set_mic_gain(int g);
void cqham_client_set_volume(int v);
void cqham_client_ptt(bool on);

/* Remote stations */
void cqham_client_remote(bool on);            /* scope_sub + remote-scope mode */
void cqham_client_remote_probe(void);         /* hf_bands_req */
void cqham_client_remote_qsy(int64_t freq);   /* hf_qsy_req / qsy_req (11 m) */
void cqham_client_ctrl_answer(bool grant);    /* answer qsy_ctrl_req */

/* Cheap reads for the audio threads */
void cqham_client_tuning(int64_t *freq, cqham_mode_t *mode, int *power);
void cqham_client_levels(int *volume, int *mic_gain);

/* Audio: a complete frame (14-byte header + payload) to send. */
void cqham_client_send_audio(const uint8_t *frame, size_t len);
uint32_t cqham_client_my_id(void);

/* Diagnostic log on the DATA partition (/mnt/cqham_log.txt) */
void cqham_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
