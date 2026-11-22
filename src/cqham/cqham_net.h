/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 *
 *  Network layer: a TCP connection, optionally TLS (GnuTLS, with Let's
 *  Encrypt's roots built in), an HTTP POST for /api/login and a minimal
 *  WebSocket client (RFC 6455: masked client frames, ping/pong, close).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

typedef struct cqham_conn cqham_conn_t;

/* WebSocket opcodes */
#define CQHAM_WS_CONT   0x0
#define CQHAM_WS_TEXT   0x1
#define CQHAM_WS_BIN    0x2
#define CQHAM_WS_CLOSE  0x8
#define CQHAM_WS_PING   0x9
#define CQHAM_WS_PONG   0xA

/* Connects. tls: wss/https. verify: check the certificate chain and the
 * host name (dates are not checked: the radio's clock may be wrong).
 * NULL on failure with the reason in err. */
cqham_conn_t *cqham_conn_open(const char *host, int port, bool tls, bool verify,
                              char *err, size_t errlen);
void cqham_conn_close(cqham_conn_t *c);
int  cqham_conn_fd(cqham_conn_t *c);

/* POST a JSON body to path; on success returns 0, the HTTP status and a
 * malloc'd, NUL-terminated response body (free it). */
int cqham_http_post_json(const char *host, int port, bool tls, bool verify, const char *path,
                         const char *json, int *status, char **body, char *err, size_t errlen);

/* WebSocket upgrade on an open connection. 0 on success. */
int cqham_ws_handshake(cqham_conn_t *c, const char *host, const char *path, char *err, size_t errlen);

/* Sends one complete (FIN) frame. Thread-safe. 0 on success. */
int cqham_ws_send(cqham_conn_t *c, int opcode, const void *data, size_t len);

/* Waits up to timeout_ms for a complete message. Returns its opcode
 * (TEXT, BIN, CLOSE, PING, PONG) with a malloc'd payload (free it, may be
 * NULL when len is 0), 0 on timeout, -1 when the connection is gone.
 * Pings are answered here and still returned. */
int cqham_ws_recv(cqham_conn_t *c, uint8_t **payload, size_t *len, int timeout_ms);

/* Base64 (standard alphabet, with padding). */
size_t cqham_b64_encode(const uint8_t *in, size_t n, char *out, size_t outlen);
/* Returns the decoded length, or 0 on bad input. out needs 3 * strlen(in) / 4. */
size_t cqham_b64_decode(const char *in, uint8_t *out, size_t outlen);
