/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 */

#include "cqham_client.h"
#include "cqham_audio.h"
#include "cqham_net.h"

#include "../cJSON.h"

#include <ctype.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define RECONNECT_MS        5000
#define HELLO_TIMEOUT_MS    14000
#define OUT_MAX_BYTES       8000        /* as the desktop's bufferedAmount limit */
#define OUT_MAX_ITEMS       256

/***** Shared state *****/

static pthread_mutex_t  lock = PTHREAD_MUTEX_INITIALIZER;
static cqham_state_t    st;
static cqham_config_t   cfg;
static bool             cfg_loaded;

static pthread_t        thread;
static volatile bool    running;
static volatile bool    login_changed;
static volatile uint32_t my_id;

/* Outgoing queue, drained by the session thread once identified. */
typedef struct out_item {
    struct out_item *next;
    int              opcode;
    size_t           len;
    uint8_t          data[];
} out_item_t;

static out_item_t      *out_head, *out_tail;
static size_t           out_bytes;
static int              out_items;

static int64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* With lock held */
static void notice_l(bool alert, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void notice_l(bool alert, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(st.notice, sizeof(st.notice), fmt, ap);
    va_end(ap);
    st.notice_alert = alert;
    st.notice_seq++;
}

static void status_l(cqham_status_t s, const char *text) {
    st.status = s;
    snprintf(st.status_text, sizeof(st.status_text), "%s", text ? text : "");
}

/***** Config file *****/

static void config_defaults(cqham_config_t *c) {
    memset(c, 0, sizeof(*c));
    snprintf(c->server, sizeof(c->server), "radio.cqdxworld.es");
    c->port     = 443;
    c->tls      = true;
    c->verify   = true;
    c->freq     = 27555000;
    c->mode     = CQHAM_USB;
    c->room     = 'a';
    c->power    = 100;
    c->step     = 1000;
    c->mic_gain = 50;
    c->volume   = 70;
}

static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = '\0';
    return s;
}

void cqham_config_load(cqham_config_t *c) {
    config_defaults(c);
    FILE *f = fopen(CQHAM_CONFIG_PATH, "r");
    if (!f) {
        return;
    }
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *s = trim(line);
        if (!*s || *s == '#') continue;
        char *eq = strchr(s, '=');
        if (!eq) continue;
        *eq = '\0';
        char *k = trim(s), *v = trim(eq + 1);

        if (!strcmp(k, "servidor"))        snprintf(c->server, sizeof(c->server), "%s", v);
        else if (!strcmp(k, "puerto"))     c->port = atoi(v);
        else if (!strcmp(k, "tls"))        c->tls = atoi(v) != 0;
        else if (!strcmp(k, "verificar"))  c->verify = atoi(v) != 0;
        else if (!strcmp(k, "indicativo")) {
            snprintf(c->callsign, sizeof(c->callsign), "%s", v);
            for (char *p = c->callsign; *p; p++) *p = (char)toupper((unsigned char)*p);
        }
        else if (!strcmp(k, "password"))   snprintf(c->password, sizeof(c->password), "%s", v);
        else if (!strcmp(k, "token"))      snprintf(c->token, sizeof(c->token), "%s", v);
        else if (!strcmp(k, "frecuencia")) c->freq = atoll(v);
        else if (!strcmp(k, "modo"))       c->mode = cqham_mode_from_name(v);
        else if (!strcmp(k, "sala"))       c->room = (v[0] == 'b' || v[0] == 'B') ? 'b' : 'a';
        else if (!strcmp(k, "potencia"))   c->power = atoi(v);
        else if (!strcmp(k, "paso"))       c->step = atoi(v);
        else if (!strcmp(k, "mic"))        c->mic_gain = atoi(v);
        else if (!strcmp(k, "volumen"))    c->volume = atoi(v);
    }
    fclose(f);

    if (c->port <= 0) c->port = c->tls ? 443 : 8080;
    if (c->power < 1 || c->power > 100) c->power = 100;
    if (c->step < 10) c->step = 1000;
    if (c->mic_gain < 0 || c->mic_gain > 100) c->mic_gain = 50;
    if (c->volume < 0 || c->volume > 100) c->volume = 70;
    if (cqham_band_index_for(c->freq) < 0) c->freq = 27555000;
}

void cqham_config_save(const cqham_config_t *c) {
    FILE *f = fopen(CQHAM_CONFIG_PATH ".tmp", "w");
    if (!f) {
        return;
    }
    fprintf(f,
            "# CQHAM en el Xiegu X6100 (EA7IXM)\n"
            "# Puedes editar este fichero desde el PC (particion DATA de la tarjeta).\n"
            "# La radio necesita SU PROPIA cuenta de CQHAM (si usa la misma que el PC, se echan).\n"
            "indicativo=%s\n"
            "password=%s\n"
            "servidor=%s\n"
            "puerto=%d\n"
            "tls=%d\n"
            "verificar=%d\n"
            "\n# Lo guarda la radio sola:\n"
            "token=%s\n"
            "frecuencia=%lld\n"
            "modo=%s\n"
            "sala=%c\n"
            "potencia=%d\n"
            "paso=%d\n"
            "mic=%d\n"
            "volumen=%d\n",
            c->callsign, c->password, c->server, c->port, c->tls, c->verify, c->token,
            (long long)c->freq, cqham_mode_name((cqham_mode_t)c->mode), c->room, c->power,
            c->step, c->mic_gain, c->volume);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    rename(CQHAM_CONFIG_PATH ".tmp", CQHAM_CONFIG_PATH);
}

/***** Outgoing *****/

static void out_clear_l(void) {
    while (out_head) {
        out_item_t *n = out_head->next;
        free(out_head);
        out_head = n;
    }
    out_tail  = NULL;
    out_bytes = 0;
    out_items = 0;
}

static void out_push(int opcode, const void *data, size_t len) {
    pthread_mutex_lock(&lock);
    bool online = st.status == CQHAM_ST_ONLINE;
    if (opcode == CQHAM_WS_BIN && (!online || out_bytes + len > OUT_MAX_BYTES)) {
        pthread_mutex_unlock(&lock);   /* slow uplink: drop a frame rather than lag */
        return;
    }
    if (out_items >= OUT_MAX_ITEMS) {
        pthread_mutex_unlock(&lock);
        return;
    }
    out_item_t *it = malloc(sizeof(*it) + len);
    if (it) {
        it->next   = NULL;
        it->opcode = opcode;
        it->len    = len;
        memcpy(it->data, data, len);
        if (out_tail) out_tail->next = it;
        else out_head = it;
        out_tail = it;
        out_bytes += len;
        out_items++;
    }
    pthread_mutex_unlock(&lock);
}

static void send_json(cJSON *obj) {
    char *s = cJSON_PrintUnformatted(obj);
    if (s) {
        out_push(CQHAM_WS_TEXT, s, strlen(s));
        free(s);
    }
    cJSON_Delete(obj);
}

static void send_simple(const char *t) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "t", t);
    send_json(o);
}

/* With lock NOT held */
static void send_tune(void) {
    pthread_mutex_lock(&lock);
    int64_t      f    = st.freq;
    cqham_mode_t m    = st.mode;
    int          pwr  = st.power;
    char         room = st.room;
    bool         sub  = st.remote;
    pthread_mutex_unlock(&lock);

    int                 bi = cqham_band_index_for(f);
    const cqham_band_t *b  = cqham_band_at(bi);
    if (b && b->fm_only) m = CQHAM_FM;

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "t", "tune");
    cJSON_AddNumberToObject(o, "freq", (double)f / 1e6);
    cJSON_AddStringToObject(o, "mode", cqham_mode_name(m));
    cJSON_AddNumberToObject(o, "power", pwr);
    cJSON_AddNumberToObject(o, "ctcss", 0);
    cJSON_AddBoolToObject(o, "enc", false);
    cJSON_AddStringToObject(o, "keyId", "");
    char rs[2] = {(b && b->rooms) ? room : '\0', '\0'};
    cJSON_AddStringToObject(o, "room", rs);
    send_json(o);

    /* the scope subscription follows the channel: renew it after each tune */
    if (sub) {
        cJSON *s = cJSON_CreateObject();
        cJSON_AddStringToObject(s, "t", "scope_sub");
        cJSON_AddBoolToObject(s, "on", true);
        send_json(s);
    }
    send_simple("hf_bands_req");
}

/***** Login *****/

static bool do_login(void) {
    char call[CQHAM_CALL_LEN], pass[64], server[64];
    int  port;
    bool tls, verify;

    pthread_mutex_lock(&lock);
    snprintf(call, sizeof(call), "%s", cfg.callsign);
    snprintf(pass, sizeof(pass), "%s", cfg.password);
    snprintf(server, sizeof(server), "%s", cfg.server);
    port   = cfg.port;
    tls    = cfg.tls;
    verify = cfg.verify;
    status_l(CQHAM_ST_LOGIN, "Iniciando sesion...");
    pthread_mutex_unlock(&lock);

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "callsign", call);
    cJSON_AddStringToObject(o, "password", pass);
    cJSON_AddBoolToObject(o, "remember", true);
    char *body_req = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    if (!body_req) return false;

    char  err[128] = "";
    int   status   = 0;
    char *body     = NULL;
    int   rc = cqham_http_post_json(server, port, tls, verify, "/api/login", body_req, &status, &body, err,
                                    sizeof(err));
    free(body_req);

    if (rc < 0) {
        pthread_mutex_lock(&lock);
        status_l(CQHAM_ST_RETRY, err[0] ? err : "Sin conexion con CQHAM");
        pthread_mutex_unlock(&lock);
        free(body);
        return false;
    }

    cJSON *r     = cJSON_Parse(body);
    free(body);
    cJSON *ok    = cJSON_GetObjectItemCaseSensitive(r, "ok");
    cJSON *token = cJSON_GetObjectItemCaseSensitive(r, "token");
    cJSON *error = cJSON_GetObjectItemCaseSensitive(r, "error");

    bool good = cJSON_IsTrue(ok) && cJSON_IsString(token) && token->valuestring;
    pthread_mutex_lock(&lock);
    if (good) {
        snprintf(cfg.token, sizeof(cfg.token), "%s", token->valuestring);
        cqham_config_save(&cfg);
        status_l(CQHAM_ST_CONNECTING, "Sesion iniciada");
    } else {
        const char *why = cJSON_IsString(error) ? error->valuestring : NULL;
        char        txt[96];
        if (status == 429) snprintf(txt, sizeof(txt), "Demasiados intentos: espera 15 min");
        else snprintf(txt, sizeof(txt), "%s", why ? why : "Login rechazado");
        status_l(CQHAM_ST_NEED_LOGIN, txt);
        notice_l(true, "%s", txt);
    }
    pthread_mutex_unlock(&lock);
    cJSON_Delete(r);
    return good;
}

/***** Incoming *****/

static const char *jstr(cJSON *o, const char *k) {
    cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
    return cJSON_IsString(v) && v->valuestring ? v->valuestring : NULL;
}

static double jnum(cJSON *o, const char *k, double def) {
    cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
    return cJSON_IsNumber(v) ? v->valuedouble : def;
}

static bool jtrue(cJSON *o, const char *k) {
    return cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(o, k));
}

static void copy_call(char *dst, const char *src) {
    snprintf(dst, CQHAM_CALL_LEN, "%s", src ? src : "");
}

static const char *denied_text(const char *reason) {
    if (!reason) return "TX denegada";
    if (!strcmp(reason, "muted"))     return "Estas muteado";
    if (!strcmp(reason, "rxonly"))    return "Banda solo escucha";
    if (!strcmp(reason, "license"))   return "Sin licencia: solo CB, 11m y PMR";
    if (!strcmp(reason, "busy"))      return "Canal ocupado";
    if (!strcmp(reason, "wait"))      return "Espera antes de volver a hablar";
    if (!strcmp(reason, "break"))     return "PARAR DE HABLAR en curso";
    if (!strcmp(reason, "tuning"))    return "El remoto esta acoplando la antena";
    if (!strcmp(reason, "dme"))       return "DME activo en el remoto";
    if (!strcmp(reason, "blockall"))  return "Frecuencia cerrada por un moderador";
    if (!strcmp(reason, "private"))   return "QSO privado en el enlace";
    if (!strcmp(reason, "outofrange"))return "Remoto fuera de su rango";
    if (!strcmp(reason, "turno555"))  return "27.555: el primero conserva la frecuencia";
    if (!strncmp(reason, "ft8", 3))   return "FT8 no disponible";
    if (!strcmp(reason, "unico"))     return "Remoto en uso exclusivo";
    return "TX denegada";
}

static void handle_text(const char *txt) {
    cJSON *m = cJSON_Parse(txt);
    if (!m) return;
    const char *t = jstr(m, "t");
    if (!t) {
        cJSON_Delete(m);
        return;
    }

    bool resend_tune = false;
    bool stop_tx     = false;

    pthread_mutex_lock(&lock);

    if (!strcmp(t, "welcome")) {
        my_id     = (uint32_t)jnum(m, "id", 0);
        st.online = (int)jnum(m, "online", st.online);
    } else if (!strcmp(t, "online")) {
        st.online = (int)jnum(m, "count", st.online);
    } else if (!strcmp(t, "accepted")) {
        copy_call(st.callsign, jstr(m, "callsign"));
        copy_call(st.dispcall, jstr(m, "dispcall") ? jstr(m, "dispcall") : jstr(m, "callsign"));
        st.licensed = jtrue(m, "licensed");
        status_l(CQHAM_ST_ONLINE, "CONECTADO");
        const char *muted = jstr(m, "muted");
        if (muted) notice_l(true, "Muteado: %s", muted);
        resend_tune = true;
    } else if (!strcmp(t, "youcall")) {
        copy_call(st.dispcall, jstr(m, "callsign"));
    } else if (!strcmp(t, "reject")) {
        const char *r = jstr(m, "reason");
        if (r && !strcmp(r, "auth")) {
            cfg.token[0] = '\0';
            cqham_config_save(&cfg);
            status_l(CQHAM_ST_RETRY, "Sesion caducada: entrando de nuevo");
        } else if (r && !strcmp(r, "yaconectado")) {
            status_l(CQHAM_ST_STOPPED, "Esta cuenta ya esta conectada en CQHAM 2.0");
        } else {
            status_l(CQHAM_ST_STOPPED, "CQHAM rechazo la conexion");
        }
    } else if (!strcmp(t, "banned")) {
        status_l(CQHAM_ST_STOPPED, "Baneado de CQHAM");
        notice_l(true, "Baneado: %s", jstr(m, "reason") ? jstr(m, "reason") : "");
    } else if (!strcmp(t, "kicked")) {
        cfg.token[0] = '\0';
        cqham_config_save(&cfg);
        status_l(CQHAM_ST_STOPPED, "Expulsado por un moderador");
        notice_l(true, "Kick de %s: %s", jstr(m, "by") ? jstr(m, "by") : "?",
                 jstr(m, "reason") ? jstr(m, "reason") : "");
        stop_tx = true;
    } else if (!strcmp(t, "presence")) {
        st.channel_count = (int)jnum(m, "count", 0);
        cJSON *u         = cJSON_GetObjectItemCaseSensitive(m, "users");
        st.users_n       = 0;
        cJSON *it;
        cJSON_ArrayForEach(it, u) {
            if (st.users_n >= CQHAM_MAX_USERS) break;
            if (cJSON_IsString(it)) copy_call(st.users[st.users_n++], it->valuestring);
        }
    } else if (!strcmp(t, "talker")) {
        const char *c = jstr(m, "callsign");
        copy_call(st.talker, c);
        snprintf(st.talker_name, sizeof(st.talker_name), "%s", c && jstr(m, "name") ? jstr(m, "name") : "");
        snprintf(st.talker_country, sizeof(st.talker_country), "%s",
                 c && jstr(m, "country") ? jstr(m, "country") : "");
        st.talker_power = (int)jnum(m, "power", 100);
        st.talker_sig   = c ? (float)jnum(m, "sig", -1) : -1.0f;
        st.talker_sreal = jtrue(m, "sreal");
        st.tx_on = c && st.ptt && (!strcmp(c, st.dispcall) || !strcmp(c, st.callsign));
    } else if (!strcmp(t, "spectrum")) {
        cJSON *a      = cJSON_GetObjectItemCaseSensitive(m, "stations");
        st.stations_n = 0;
        cJSON *it;
        cJSON_ArrayForEach(it, a) {
            if (st.stations_n >= CQHAM_MAX_STATIONS) break;
            cqham_station_t *s = &st.stations[st.stations_n++];
            s->freq  = (int64_t)llround(jnum(it, "freq", 0) * 1e6);
            s->power = (int)jnum(it, "power", 100);
            s->tx    = jtrue(it, "tx");
            copy_call(s->call, jstr(it, "call"));
        }
    } else if (!strcmp(t, "txdenied")) {
        const char *r  = jstr(m, "reason");
        const char *by = jstr(m, "by");
        if (by) notice_l(true, "%s (%s)", denied_text(r), by);
        else notice_l(true, "%s", denied_text(r));
        stop_tx = true;
    } else if (!strcmp(t, "muted")) {
        notice_l(true, "Muteado por %s: %s", jstr(m, "by") ? jstr(m, "by") : "moderador",
                 jstr(m, "reason") ? jstr(m, "reason") : "");
        stop_tx = true;
    } else if (!strcmp(t, "unmuted")) {
        notice_l(false, "Ya puedes transmitir");
    } else if (!strcmp(t, "sinaudio")) {
        if (jtrue(m, "mute")) notice_l(true, "Muteado 60 s por transmitir sin voz");
        else notice_l(true, "No llega tu voz: habla o suelta el PTT");
    } else if (!strcmp(t, "audiobajo")) {
        notice_l(true, "Audio bajo: sube MIC");
    } else if (!strcmp(t, "wait")) {
        int secs = (int)jnum(m, "secs", 0);
        if (secs > 0 && !jtrue(m, "exempt")) notice_l(false, "Espera %d s para hablar", secs);
    } else if (!strcmp(t, "break_block")) {
        notice_l(true, "PARAR DE HABLAR: %s", jstr(m, "holder") ? jstr(m, "holder") : "");
    } else if (!strcmp(t, "sysaviso")) {
        const char *x = jstr(m, "texto");
        if (x) notice_l(true, "%s", x);
    } else if (!strcmp(t, "hf_tot")) {
        if (jtrue(m, "cut")) notice_l(true, "Tiempo maximo de TX: cortado");
        else notice_l(true, "TX: quedan %d s", (int)jnum(m, "left", 0));
    } else if (!strcmp(t, "hf_bands_list")) {
        const char *l = jstr(m, "link");
        snprintf(st.link, sizeof(st.link), "%s", l ? l : "");
        st.link_freq = (int64_t)llround(jnum(m, "freq", 0) * 1e6);
    } else if (!strcmp(t, "hf_aviso_remoto")) {
        const char *l = jstr(m, "link");
        if (l) snprintf(st.link, sizeof(st.link), "%s", l);
        const char *x = jstr(m, "texto");
        if (x) notice_l(false, "%s", x);
    } else if (!strcmp(t, "qsy_ctrl_holder") || !strcmp(t, "qsy11")) {
        copy_call(st.holder, jstr(m, "holder"));
    } else if (!strcmp(t, "scope_denied")) {
        st.scope_on = false;
        notice_l(true, "Scope remoto: necesitas licencia");
    } else if (!strcmp(t, "scope_data")) {
        const char *d = jstr(m, "d");
        if (d) {
            size_t n = cqham_b64_decode(d, st.scope, sizeof(st.scope));
            int    mx = 0;
            for (size_t i = 0; i < n; i++) if (st.scope[i] > mx) mx = st.scope[i];
            /* IC-7610: 0..160; SunSDR / Hermes: 0..255 */
            if (mx > 160) st.scope_max = 255;
            else if (st.scope_max == 0) st.scope_max = 160;
            st.scope_n   = (int)n;
            st.scope_fs  = (int64_t)llround(jnum(m, "fs", 0) * 1e6);
            st.scope_fe  = (int64_t)llround(jnum(m, "fe", 0) * 1e6);
            st.scope_txf = (int64_t)llround(jnum(m, "txf", 0) * 1e6);
            st.scope_tx  = jtrue(m, "tx");
            st.scope_on  = true;
            const char *l = jstr(m, "link");
            if (l) snprintf(st.link, sizeof(st.link), "%s", l);
            st.scope_seq++;
        }
    } else if (!strcmp(t, "scope_img")) {
        if (!st.scope_on) notice_l(false, "Este remoto envia el scope en video (no soportado aun)");
    } else if (!strcmp(t, "scope_audio")) {
        const char *d = jstr(m, "d");
        if (d) {
            static int16_t pcm[2048];
            size_t         n = cqham_b64_decode(d, (uint8_t *)pcm, sizeof(pcm)) / 2;
            pthread_mutex_unlock(&lock);
            cqham_audio_rx_pcm(pcm, n);
            pthread_mutex_lock(&lock);
        }
    } else if (!strcmp(t, "hf_qsy_ack") || !strcmp(t, "qsy_ack")) {
        if (jtrue(m, "ok") || jtrue(m, "seguir")) {
            double f = jnum(m, "freq", 0);
            if (f > 0) {
                st.freq = (int64_t)llround(f * 1e6);
                cfg.freq = st.freq;
            }
            resend_tune = true;
        } else {
            const char *r = jstr(m, "reason");
            if (r && !strcmp(r, "ctrlpending")) notice_l(false, "Pedido el control a %s", jstr(m, "holder") ? jstr(m, "holder") : "?");
            else if (r && !strcmp(r, "ctrlbusy")) notice_l(true, "Remoto en uso por %s", jstr(m, "holder") ? jstr(m, "holder") : "?");
            else if (r && !strcmp(r, "nolink")) notice_l(true, "No hay remoto en esta frecuencia");
            else if (r && !strcmp(r, "fuerarango")) notice_l(true, "Fuera del rango del remoto");
            else if (r && !strcmp(r, "remlimite")) notice_l(true, "Agotado el tiempo de remoto de hoy");
            else if (r && !strcmp(r, "nolic")) notice_l(true, "Sin licencia: remoto solo en 11m");
            else if (r && !strcmp(r, "cooldown")) notice_l(false, "Espera unos segundos");
            else if (r && !strcmp(r, "tuning")) notice_l(true, "El remoto esta acoplando la antena");
            else notice_l(true, "QSY rechazado (%s)", r ? r : "?");
        }
    } else if (!strcmp(t, "qsy_ctrl_req")) {
        copy_call(st.ctrl_req_by, jstr(m, "by"));
        notice_l(true, "%s pide el control: CEDER o NO", st.ctrl_req_by);
    } else if (!strcmp(t, "qsy_ctrl_granted")) {
        notice_l(false, "Tienes el control del remoto");
    } else if (!strcmp(t, "qsy_ctrl_denied")) {
        notice_l(true, "Control denegado: espera 5 min");
    } else if (!strcmp(t, "qsy_ctrl_blocked")) {
        notice_l(true, "Bloqueado: el titular te denego el control");
    } else if (!strcmp(t, "qsy_ctrl_lost")) {
        notice_l(true, "Has perdido el control del remoto");
    } else if (!strcmp(t, "qsy_ctrl_available")) {
        notice_l(false, "El remoto vuelve a estar libre");
    } else if (!strcmp(t, "hf_tuning")) {
        st.atu = jtrue(m, "on");
        const char *x = jstr(m, "texto");
        if (x) notice_l(false, "%s", x);
    } else if (!strcmp(t, "link_home")) {
        double f = jnum(m, "freq", 0);
        if (f > 0) {
            st.freq  = (int64_t)llround(f * 1e6);
            cfg.freq = st.freq;
            resend_tune = true;
        }
    }

    if (stop_tx) {
        st.ptt   = false;
        st.tx_on = false;
    }
    pthread_mutex_unlock(&lock);
    cJSON_Delete(m);

    if (stop_tx) {
        cqham_audio_tx(false);
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "t", "ptt");
        cJSON_AddBoolToObject(o, "on", false);
        send_json(o);
    }
    if (resend_tune) {
        send_tune();
    }
}

/***** Session thread *****/

static void sleep_ms_interruptible(int ms) {
    int64_t until = now_ms() + ms;
    while (running && !login_changed && now_ms() < until) {
        usleep(50000);
    }
}

static void session(void) {
    char server[64];
    int  port;
    bool tls, verify;
    char token[80];

    pthread_mutex_lock(&lock);
    snprintf(server, sizeof(server), "%s", cfg.server);
    snprintf(token, sizeof(token), "%s", cfg.token);
    port   = cfg.port;
    tls    = cfg.tls;
    verify = cfg.verify;
    status_l(CQHAM_ST_CONNECTING, "Conectando...");
    out_clear_l();
    pthread_mutex_unlock(&lock);

    char          err[128] = "";
    cqham_conn_t *c        = cqham_conn_open(server, port, tls, verify, err, sizeof(err));
    if (!c || cqham_ws_handshake(c, server, "/", err, sizeof(err)) < 0) {
        pthread_mutex_lock(&lock);
        status_l(CQHAM_ST_RETRY, err[0] ? err : "Sin conexion");
        pthread_mutex_unlock(&lock);
        cqham_conn_close(c);
        return;
    }

    /* Identify FIRST: the relay hangs up on anything sent before "accepted". */
    {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "t", "hello");
        cJSON_AddStringToObject(o, "token", token);
        cJSON_AddBoolToObject(o, "adpcm", true);
        cJSON_AddStringToObject(o, "app", "x6100");
        cJSON_AddStringToObject(o, "ver", "0.1");
        char *s = cJSON_PrintUnformatted(o);
        cJSON_Delete(o);
        int rc = s ? cqham_ws_send(c, CQHAM_WS_TEXT, s, strlen(s)) : -1;
        free(s);
        if (rc < 0) {
            pthread_mutex_lock(&lock);
            status_l(CQHAM_ST_RETRY, "Error enviando hello");
            pthread_mutex_unlock(&lock);
            cqham_conn_close(c);
            return;
        }
    }

    pthread_mutex_lock(&lock);
    status_l(CQHAM_ST_HELLO, "Identificando...");
    pthread_mutex_unlock(&lock);

    int64_t hello_at = now_ms();

    while (running && !login_changed) {
        uint8_t *payload = NULL;
        size_t   len     = 0;
        int      op      = cqham_ws_recv(c, &payload, &len, 15);

        if (op < 0 || op == CQHAM_WS_CLOSE) {
            free(payload);
            break;
        }
        if (op == CQHAM_WS_TEXT && payload) {
            handle_text((const char *)payload);
        } else if (op == CQHAM_WS_BIN && payload && len > 14) {
            cqham_audio_rx(payload, len);
        }
        free(payload);

        pthread_mutex_lock(&lock);
        cqham_status_t s = st.status;
        pthread_mutex_unlock(&lock);

        if (s == CQHAM_ST_STOPPED || (s == CQHAM_ST_RETRY && !cfg.token[0])) {
            break;
        }
        if (s == CQHAM_ST_HELLO) {
            if (now_ms() - hello_at > HELLO_TIMEOUT_MS) {
                pthread_mutex_lock(&lock);
                status_l(CQHAM_ST_RETRY, "CQHAM no acepto la sesion");
                pthread_mutex_unlock(&lock);
                break;
            }
            continue;   /* nothing but hello until accepted */
        }

        /* Drain the queue */
        for (;;) {
            pthread_mutex_lock(&lock);
            out_item_t *it = out_head;
            if (it) {
                out_head = it->next;
                if (!out_head) out_tail = NULL;
                out_bytes -= it->len;
                out_items--;
            }
            pthread_mutex_unlock(&lock);
            if (!it) break;
            int rc = cqham_ws_send(c, it->opcode, it->data, it->len);
            free(it);
            if (rc < 0) {
                op = -1;
                break;
            }
        }
        if (op < 0) break;
    }

    cqham_conn_close(c);

    pthread_mutex_lock(&lock);
    if (st.status != CQHAM_ST_STOPPED) {
        if (st.status != CQHAM_ST_RETRY || !st.status_text[0]) {
            status_l(CQHAM_ST_RETRY, "Conexion perdida: reconectando");
        }
    }
    st.tx_on = false;
    st.ptt   = false;
    st.talker[0]  = '\0';
    st.stations_n = 0;
    out_clear_l();
    pthread_mutex_unlock(&lock);
    cqham_audio_tx(false);
}

static void *client_thread(void *arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "cqham");

    while (running) {
        login_changed = false;

        pthread_mutex_lock(&lock);
        bool have_login = cfg.callsign[0] && cfg.password[0];
        bool have_token = cfg.token[0] != '\0';
        bool stopped    = st.status == CQHAM_ST_STOPPED;
        if (!have_login && !have_token) {
            status_l(CQHAM_ST_NEED_LOGIN, "Pulsa CUENTA para entrar con tu indicativo");
        }
        pthread_mutex_unlock(&lock);

        if (stopped || (!have_login && !have_token)) {
            sleep_ms_interruptible(500);
            continue;
        }
        if (!have_token && !do_login()) {
            sleep_ms_interruptible(RECONNECT_MS * 2);
            continue;
        }

        session();
        sleep_ms_interruptible(RECONNECT_MS);
    }
    return NULL;
}

/***** API *****/

void cqham_client_start(void) {
    if (running) return;

    pthread_mutex_lock(&lock);
    if (!cfg_loaded) {
        cqham_config_load(&cfg);
        cfg_loaded = true;
    }
    memset(&st, 0, sizeof(st));
    st.freq       = cfg.freq;
    st.mode       = (cqham_mode_t)cfg.mode;
    st.room       = cfg.room;
    st.power      = cfg.power;
    st.talker_sig = -1.0f;
    status_l(CQHAM_ST_IDLE, "");
    pthread_mutex_unlock(&lock);

    /* a peer closing the socket must not kill the GUI */
    signal(SIGPIPE, SIG_IGN);

    running = true;
    if (pthread_create(&thread, NULL, client_thread, NULL) != 0) {
        running = false;
    }
}

void cqham_client_stop(void) {
    if (!running) return;
    if (st.ptt) cqham_client_ptt(false);
    running = false;
    pthread_join(thread, NULL);

    pthread_mutex_lock(&lock);
    cfg.freq  = st.freq;
    cfg.mode  = st.mode;
    cfg.room  = st.room;
    cfg.power = st.power;
    cqham_config_save(&cfg);
    out_clear_l();
    status_l(CQHAM_ST_IDLE, "");
    pthread_mutex_unlock(&lock);
}

void cqham_client_get(cqham_state_t *out) {
    pthread_mutex_lock(&lock);
    *out = st;
    pthread_mutex_unlock(&lock);
}

void cqham_client_config(cqham_config_t *out) {
    pthread_mutex_lock(&lock);
    if (!cfg_loaded) {
        cqham_config_load(&cfg);
        cfg_loaded = true;
    }
    *out = cfg;
    pthread_mutex_unlock(&lock);
}

void cqham_client_set_login(const char *call, const char *password) {
    pthread_mutex_lock(&lock);
    snprintf(cfg.callsign, sizeof(cfg.callsign), "%s", call ? call : "");
    for (char *p = cfg.callsign; *p; p++) *p = (char)toupper((unsigned char)*p);
    snprintf(cfg.password, sizeof(cfg.password), "%s", password ? password : "");
    cfg.token[0] = '\0';
    cqham_config_save(&cfg);
    if (st.status == CQHAM_ST_STOPPED) status_l(CQHAM_ST_IDLE, "");
    pthread_mutex_unlock(&lock);
    login_changed = true;
}

void cqham_client_tune(int64_t freq, cqham_mode_t mode) {
    int                 bi = cqham_band_index_for(freq);
    const cqham_band_t *b  = cqham_band_at(bi);
    if (!b) return;
    if (b->fm_only) mode = CQHAM_FM;

    pthread_mutex_lock(&lock);
    bool changed = freq != st.freq || mode != st.mode;
    st.freq  = freq;
    st.mode  = mode;
    cfg.freq = freq;
    cfg.mode = mode;
    bool online = st.status == CQHAM_ST_ONLINE;
    pthread_mutex_unlock(&lock);

    if (changed && online) send_tune();
}

void cqham_client_set_room(char room) {
    pthread_mutex_lock(&lock);
    st.room  = room == 'b' ? 'b' : 'a';
    cfg.room = st.room;
    bool online = st.status == CQHAM_ST_ONLINE;
    pthread_mutex_unlock(&lock);
    if (online) send_tune();
}

void cqham_client_set_power(int watts) {
    if (watts < 1) watts = 1;
    if (watts > 100) watts = 100;
    pthread_mutex_lock(&lock);
    st.power  = watts;
    cfg.power = watts;
    pthread_mutex_unlock(&lock);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "t", "power");
    cJSON_AddNumberToObject(o, "w", watts);
    send_json(o);
}

void cqham_client_set_step(int hz) {
    pthread_mutex_lock(&lock);
    cfg.step = hz;
    pthread_mutex_unlock(&lock);
}

void cqham_client_set_mic_gain(int g) {
    pthread_mutex_lock(&lock);
    cfg.mic_gain = g < 0 ? 0 : (g > 100 ? 100 : g);
    pthread_mutex_unlock(&lock);
}

void cqham_client_set_volume(int v) {
    pthread_mutex_lock(&lock);
    cfg.volume = v < 0 ? 0 : (v > 100 ? 100 : v);
    pthread_mutex_unlock(&lock);
}

void cqham_client_ptt(bool on) {
    pthread_mutex_lock(&lock);
    bool online = st.status == CQHAM_ST_ONLINE;
    if (on && !online) {
        notice_l(true, "No conectado a CQHAM");
        pthread_mutex_unlock(&lock);
        return;
    }
    if (on == st.ptt) {
        pthread_mutex_unlock(&lock);
        return;
    }
    st.ptt = on;
    if (!on) st.tx_on = false;
    pthread_mutex_unlock(&lock);

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "t", "ptt");
    cJSON_AddBoolToObject(o, "on", on);
    send_json(o);
    cqham_audio_tx(on);
}

void cqham_client_remote(bool on) {
    pthread_mutex_lock(&lock);
    st.remote = on;
    if (!on) {
        st.scope_on = false;
        st.scope_n  = 0;
    }
    pthread_mutex_unlock(&lock);

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "t", "scope_sub");
    cJSON_AddBoolToObject(o, "on", on);
    send_json(o);
    if (on) send_simple("hf_bands_req");
}

void cqham_client_remote_probe(void) {
    send_simple("hf_bands_req");
}

void cqham_client_remote_qsy(int64_t freq) {
    int                 bi = cqham_band_index_for(freq);
    const cqham_band_t *b  = cqham_band_at(bi);
    cJSON              *o  = cJSON_CreateObject();
    if (b && b->rooms) {
        cJSON_AddStringToObject(o, "t", "qsy_req");
        cJSON_AddNumberToObject(o, "freq", (double)freq / 1e6);
        cJSON_AddBoolToObject(o, "paso", true);
    } else {
        cJSON_AddStringToObject(o, "t", "hf_qsy_req");
        cJSON_AddNumberToObject(o, "freq", (double)freq / 1e6);
        cJSON_AddBoolToObject(o, "scope", true);
        cJSON_AddBoolToObject(o, "ft8", false);
    }
    send_json(o);
}

void cqham_client_ctrl_answer(bool grant) {
    pthread_mutex_lock(&lock);
    bool pending = st.ctrl_req_by[0] != '\0';
    st.ctrl_req_by[0] = '\0';
    pthread_mutex_unlock(&lock);
    if (!pending) return;

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "t", "qsy_ctrl_resp");
    cJSON_AddBoolToObject(o, "grant", grant);
    send_json(o);
}

void cqham_client_tuning(int64_t *freq, cqham_mode_t *mode, int *power) {
    pthread_mutex_lock(&lock);
    if (freq) *freq = st.freq;
    if (mode) *mode = st.mode;
    if (power) *power = st.power;
    pthread_mutex_unlock(&lock);
}

void cqham_client_levels(int *volume, int *mic_gain) {
    pthread_mutex_lock(&lock);
    if (volume) *volume = cfg.volume;
    if (mic_gain) *mic_gain = cfg.mic_gain;
    pthread_mutex_unlock(&lock);
}

void cqham_client_send_audio(const uint8_t *frame, size_t len) {
    out_push(CQHAM_WS_BIN, frame, len);
}

uint32_t cqham_client_my_id(void) {
    return my_id;
}
