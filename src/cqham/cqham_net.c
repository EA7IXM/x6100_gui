/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI — CQHAM client (EA7IXM)
 */

#include "cqham_net.h"
#include "cqham_ca.h"

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <gnutls/gnutls.h>

#define CONNECT_TIMEOUT_MS  8000
#define IO_TIMEOUT_MS       8000
#define MAX_MESSAGE         (1024 * 1024)

struct cqham_conn {
    int                               fd;
    bool                              tls;
    gnutls_session_t                  session;
    gnutls_certificate_credentials_t  cred;
    bool                              handshaked;
    pthread_mutex_t                   write_lock;

    /* bytes read past the HTTP header during the handshake */
    uint8_t                           pre[4096];
    size_t                            pre_len, pre_pos;

    /* a WebSocket message being assembled across fragments */
    uint8_t                          *frag;
    size_t                            frag_len;
    int                               frag_op;
};

static void set_err(char *err, size_t errlen, const char *fmt, const char *arg) {
    if (err && errlen) {
        snprintf(err, errlen, fmt, arg ? arg : "");
    }
}

static int64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/***** TCP *****/

static int tcp_connect(const char *host, int port, char *err, size_t errlen) {
    struct addrinfo  hints = {0}, *res = NULL, *ai;
    char             portstr[8];

    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    snprintf(portstr, sizeof(portstr), "%d", port);

    int rc = getaddrinfo(host, portstr, &hints, &res);
    if (rc != 0) {
        set_err(err, errlen, "DNS: %s", gai_strerror(rc));
        return -1;
    }

    int fd = -1;
    for (ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            continue;
        }
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        rc = connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (rc < 0 && errno == EINPROGRESS) {
            struct pollfd pfd = {.fd = fd, .events = POLLOUT};
            rc = poll(&pfd, 1, CONNECT_TIMEOUT_MS);
            if (rc == 1) {
                int       soerr = 0;
                socklen_t sl    = sizeof(soerr);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &sl);
                rc = soerr ? -1 : 0;
            } else {
                rc = -1;
            }
        }
        if (rc == 0) {
            fcntl(fd, F_SETFL, flags);   /* back to blocking */
            int one = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
            struct timeval tv = {.tv_sec = IO_TIMEOUT_MS / 1000};
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
            break;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0) {
        set_err(err, errlen, "No se puede conectar con %s", host);
    }
    return fd;
}

/***** Connection *****/

cqham_conn_t *cqham_conn_open(const char *host, int port, bool tls, bool verify,
                              char *err, size_t errlen) {
    static bool gnutls_ready = false;

    cqham_conn_t *c = calloc(1, sizeof(*c));
    if (!c) {
        set_err(err, errlen, "Sin memoria%s", NULL);
        return NULL;
    }
    pthread_mutex_init(&c->write_lock, NULL);
    c->tls = tls;

    c->fd = tcp_connect(host, port, err, errlen);
    if (c->fd < 0) {
        free(c);
        return NULL;
    }

    if (!tls) {
        return c;
    }

    if (!gnutls_ready) {
        gnutls_global_init();
        gnutls_ready = true;
    }

    gnutls_certificate_allocate_credentials(&c->cred);
    gnutls_datum_t ca = {.data = (unsigned char *)cqham_ca_pem, .size = sizeof(cqham_ca_pem) - 1};
    gnutls_certificate_set_x509_trust_mem(c->cred, &ca, GNUTLS_X509_FMT_PEM);
    /* The radio may boot with a wrong clock: check the chain, not the dates. */
    gnutls_certificate_set_verify_flags(c->cred, GNUTLS_VERIFY_DISABLE_TIME_CHECKS |
                                                 GNUTLS_VERIFY_DISABLE_TRUSTED_TIME_CHECKS);

    gnutls_init(&c->session, GNUTLS_CLIENT);
    gnutls_server_name_set(c->session, GNUTLS_NAME_DNS, host, strlen(host));
    gnutls_set_default_priority(c->session);
    gnutls_credentials_set(c->session, GNUTLS_CRD_CERTIFICATE, c->cred);
    if (verify) {
        gnutls_session_set_verify_cert(c->session, host, 0);
    }
    gnutls_transport_set_int(c->session, c->fd);
    gnutls_handshake_set_timeout(c->session, GNUTLS_DEFAULT_HANDSHAKE_TIMEOUT);

    int rc;
    do {
        rc = gnutls_handshake(c->session);
    } while (rc < 0 && gnutls_error_is_fatal(rc) == 0);

    if (rc < 0) {
        if (rc == GNUTLS_E_CERTIFICATE_VERIFICATION_ERROR) {
            set_err(err, errlen, "Certificado del servidor no valido%s", NULL);
        } else {
            set_err(err, errlen, "TLS: %s", gnutls_strerror(rc));
        }
        cqham_conn_close(c);
        return NULL;
    }
    c->handshaked = true;
    return c;
}

void cqham_conn_close(cqham_conn_t *c) {
    if (!c) {
        return;
    }
    if (c->tls && c->session) {
        if (c->handshaked) gnutls_bye(c->session, GNUTLS_SHUT_WR);
        gnutls_deinit(c->session);
    }
    if (c->cred) {
        gnutls_certificate_free_credentials(c->cred);
    }
    if (c->fd >= 0) {
        close(c->fd);
    }
    free(c->frag);
    pthread_mutex_destroy(&c->write_lock);
    free(c);
}

int cqham_conn_fd(cqham_conn_t *c) {
    return c ? c->fd : -1;
}

static int conn_write_all(cqham_conn_t *c, const void *buf, size_t n) {
    const uint8_t *p = buf;
    while (n > 0) {
        ssize_t w;
        if (c->tls) {
            w = gnutls_record_send(c->session, p, n);
            if (w == GNUTLS_E_AGAIN || w == GNUTLS_E_INTERRUPTED) {
                continue;
            }
        } else {
            w = send(c->fd, p, n, MSG_NOSIGNAL);
            if (w < 0 && errno == EINTR) {
                continue;
            }
        }
        if (w <= 0) {
            return -1;
        }
        p += w;
        n -= (size_t)w;
    }
    return 0;
}

/* Reads what is available: >0 bytes, 0 on timeout, -1 closed/error. */
static ssize_t conn_read(cqham_conn_t *c, void *buf, size_t n, int timeout_ms) {
    if (c->pre_pos < c->pre_len) {
        size_t k = c->pre_len - c->pre_pos;
        if (k > n) k = n;
        memcpy(buf, c->pre + c->pre_pos, k);
        c->pre_pos += k;
        return (ssize_t)k;
    }

    bool pending = c->tls && gnutls_record_check_pending(c->session) > 0;
    if (!pending) {
        struct pollfd pfd = {.fd = c->fd, .events = POLLIN};
        int           rc  = poll(&pfd, 1, timeout_ms);
        if (rc == 0) {
            return 0;
        }
        if (rc < 0) {
            return errno == EINTR ? 0 : -1;
        }
        if (pfd.revents & (POLLERR | POLLNVAL)) {
            return -1;
        }
    }

    ssize_t r;
    if (c->tls) {
        r = gnutls_record_recv(c->session, buf, n);
        if (r == GNUTLS_E_AGAIN || r == GNUTLS_E_INTERRUPTED) {
            return 0;
        }
    } else {
        r = recv(c->fd, buf, n, 0);
        if (r < 0 && (errno == EINTR || errno == EAGAIN)) {
            return 0;
        }
    }
    return r > 0 ? r : -1;
}

/* Exactly n bytes, giving up after timeout_ms in total. 0 ok, -1 error. */
static int conn_read_exact(cqham_conn_t *c, void *buf, size_t n, int timeout_ms) {
    uint8_t *p     = buf;
    int64_t  until = now_ms() + timeout_ms;
    while (n > 0) {
        int left = (int)(until - now_ms());
        if (left <= 0) {
            return -1;
        }
        ssize_t r = conn_read(c, p, n, left);
        if (r < 0) {
            return -1;
        }
        p += r;
        n -= (size_t)r;
    }
    return 0;
}

/***** HTTP *****/

int cqham_http_post_json(const char *host, int port, bool tls, bool verify, const char *path,
                         const char *json, int *status, char **body, char *err, size_t errlen) {
    *body   = NULL;
    *status = 0;

    cqham_conn_t *c = cqham_conn_open(host, port, tls, verify, err, errlen);
    if (!c) {
        return -1;
    }

    size_t jl  = strlen(json);
    char  *req = malloc(jl + 512);
    if (!req) {
        cqham_conn_close(c);
        return -1;
    }
    int hl = snprintf(req, 512,
                      "POST %s HTTP/1.1\r\n"
                      "Host: %s\r\n"
                      "User-Agent: X6100-CQHAM/0.1\r\n"
                      "Content-Type: application/json\r\n"
                      "Content-Length: %zu\r\n"
                      "Connection: close\r\n\r\n",
                      path, host, jl);
    memcpy(req + hl, json, jl);
    int rc = conn_write_all(c, req, hl + jl);
    free(req);
    if (rc < 0) {
        set_err(err, errlen, "Error enviando%s", NULL);
        cqham_conn_close(c);
        return -1;
    }

    size_t cap = 8192, len = 0;
    char  *buf = malloc(cap + 1);
    int64_t until = now_ms() + IO_TIMEOUT_MS;
    while (buf && now_ms() < until) {
        if (len == cap) {
            if (cap >= 256 * 1024) break;
            cap *= 2;
            char *nb = realloc(buf, cap + 1);
            if (!nb) break;
            buf = nb;
        }
        ssize_t r = conn_read(c, buf + len, cap - len, 500);
        if (r < 0) break;            /* closed: Connection: close */
        len += (size_t)r;
    }
    cqham_conn_close(c);
    if (!buf) {
        return -1;
    }
    buf[len] = '\0';

    if (sscanf(buf, "HTTP/%*s %d", status) != 1) {
        set_err(err, errlen, "Respuesta HTTP no valida%s", NULL);
        free(buf);
        return -1;
    }
    char *hdr_end = strstr(buf, "\r\n\r\n");
    char *start   = hdr_end ? hdr_end + 4 : buf + len;

    /* The relay answers with a plain body; a chunked one is joined here. */
    if (strcasestr(buf, "Transfer-Encoding: chunked") && hdr_end) {
        char *out = malloc(len + 1), *o = out, *p = start;
        while (out && *p) {
            unsigned long sz = strtoul(p, &p, 16);
            p = strstr(p, "\r\n");
            if (!p || sz == 0) break;
            p += 2;
            if ((size_t)(buf + len - p) < sz) sz = (unsigned long)(buf + len - p);
            memcpy(o, p, sz);
            o += sz;
            p += sz;
            if (strncmp(p, "\r\n", 2) == 0) p += 2;
        }
        if (out) {
            *o    = '\0';
            *body = out;
        }
    } else {
        *body = strdup(start);
    }
    free(buf);
    return *body ? 0 : -1;
}

/***** WebSocket *****/

int cqham_ws_handshake(cqham_conn_t *c, const char *host, const char *path, char *err, size_t errlen) {
    uint8_t key_raw[16];
    char    key[32];
    for (int i = 0; i < 16; i++) {
        key_raw[i] = (uint8_t)(rand() & 0xFF);
    }
    cqham_b64_encode(key_raw, 16, key, sizeof(key));

    char req[512];
    int  n = snprintf(req, sizeof(req),
                      "GET %s HTTP/1.1\r\n"
                      "Host: %s\r\n"
                      "User-Agent: X6100-CQHAM/0.1\r\n"
                      "Upgrade: websocket\r\n"
                      "Connection: Upgrade\r\n"
                      "Sec-WebSocket-Key: %s\r\n"
                      "Sec-WebSocket-Version: 13\r\n\r\n",
                      path, host, key);
    if (conn_write_all(c, req, (size_t)n) < 0) {
        set_err(err, errlen, "Error enviando%s", NULL);
        return -1;
    }

    char    resp[4096];
    size_t  len   = 0;
    int64_t until = now_ms() + IO_TIMEOUT_MS;
    char   *end   = NULL;
    while (!end && len < sizeof(resp) - 1 && now_ms() < until) {
        ssize_t r = conn_read(c, resp + len, sizeof(resp) - 1 - len, 500);
        if (r < 0) {
            set_err(err, errlen, "El servidor cerro la conexion%s", NULL);
            return -1;
        }
        len += (size_t)r;
        resp[len] = '\0';
        end = strstr(resp, "\r\n\r\n");
    }
    if (!end) {
        set_err(err, errlen, "Sin respuesta del servidor%s", NULL);
        return -1;
    }
    if (strncmp(resp, "HTTP/1.1 101", 12) != 0 && strncmp(resp, "HTTP/1.0 101", 12) != 0) {
        char line[64];
        snprintf(line, sizeof(line), "%.40s", resp);
        char *nl = strchr(line, '\r');
        if (nl) *nl = '\0';
        set_err(err, errlen, "WebSocket rechazado: %s", line);
        return -1;
    }

    /* Anything after the header is already WebSocket data. */
    size_t hdr = (size_t)(end + 4 - resp);
    if (len > hdr && len - hdr <= sizeof(c->pre)) {
        memcpy(c->pre, resp + hdr, len - hdr);
        c->pre_len = len - hdr;
        c->pre_pos = 0;
    }
    return 0;
}

int cqham_ws_send(cqham_conn_t *c, int opcode, const void *data, size_t len) {
    uint8_t  hdr[14];
    size_t   h = 0;
    uint8_t  mask[4];
    uint32_t r = (uint32_t)rand() ^ ((uint32_t)rand() << 16);

    memcpy(mask, &r, 4);
    hdr[h++] = (uint8_t)(0x80 | (opcode & 0x0F));
    if (len < 126) {
        hdr[h++] = (uint8_t)(0x80 | len);
    } else if (len < 65536) {
        hdr[h++] = 0x80 | 126;
        hdr[h++] = (uint8_t)(len >> 8);
        hdr[h++] = (uint8_t)len;
    } else {
        hdr[h++] = 0x80 | 127;
        for (int i = 7; i >= 0; i--) {
            hdr[h++] = (uint8_t)((uint64_t)len >> (8 * i));
        }
    }
    memcpy(hdr + h, mask, 4);
    h += 4;

    uint8_t  stackbuf[1024];
    uint8_t *frame = (h + len <= sizeof(stackbuf)) ? stackbuf : malloc(h + len);
    if (!frame) {
        return -1;
    }
    memcpy(frame, hdr, h);
    const uint8_t *src = data;
    for (size_t i = 0; i < len; i++) {
        frame[h + i] = src[i] ^ mask[i & 3];
    }

    pthread_mutex_lock(&c->write_lock);
    int rc = conn_write_all(c, frame, h + len);
    pthread_mutex_unlock(&c->write_lock);

    if (frame != stackbuf) {
        free(frame);
    }
    return rc;
}

int cqham_ws_recv(cqham_conn_t *c, uint8_t **payload, size_t *len, int timeout_ms) {
    *payload = NULL;
    *len     = 0;

    for (;;) {
        uint8_t h[2];
        ssize_t r = conn_read(c, h, 1, timeout_ms);
        if (r < 0) return -1;
        if (r == 0) return 0;

        /* a frame has started: the rest must follow soon */
        if (conn_read_exact(c, h + 1, 1, IO_TIMEOUT_MS) < 0) return -1;

        bool     fin    = (h[0] & 0x80) != 0;
        int      opcode = h[0] & 0x0F;
        bool     masked = (h[1] & 0x80) != 0;
        uint64_t plen   = h[1] & 0x7F;

        if (plen == 126) {
            uint8_t e[2];
            if (conn_read_exact(c, e, 2, IO_TIMEOUT_MS) < 0) return -1;
            plen = ((uint64_t)e[0] << 8) | e[1];
        } else if (plen == 127) {
            uint8_t e[8];
            if (conn_read_exact(c, e, 8, IO_TIMEOUT_MS) < 0) return -1;
            plen = 0;
            for (int i = 0; i < 8; i++) plen = (plen << 8) | e[i];
        }
        if (plen > MAX_MESSAGE) return -1;

        uint8_t mask[4] = {0};
        if (masked && conn_read_exact(c, mask, 4, IO_TIMEOUT_MS) < 0) return -1;

        uint8_t *data = NULL;
        if (plen) {
            data = malloc((size_t)plen + 1);
            if (!data) return -1;
            if (conn_read_exact(c, data, (size_t)plen, IO_TIMEOUT_MS) < 0) {
                free(data);
                return -1;
            }
            if (masked) {
                for (uint64_t i = 0; i < plen; i++) data[i] ^= mask[i & 3];
            }
            data[plen] = '\0';
        }

        if (opcode == CQHAM_WS_PING) {
            cqham_ws_send(c, CQHAM_WS_PONG, data, (size_t)plen);
            *payload = data;
            *len     = (size_t)plen;
            return CQHAM_WS_PING;
        }
        if (opcode == CQHAM_WS_PONG || opcode == CQHAM_WS_CLOSE) {
            *payload = data;
            *len     = (size_t)plen;
            return opcode;
        }

        /* Data frames, possibly fragmented */
        if (opcode != CQHAM_WS_CONT) {
            free(c->frag);
            c->frag     = NULL;
            c->frag_len = 0;
            c->frag_op  = opcode;
        }
        if (fin && opcode != CQHAM_WS_CONT) {
            *payload = data;
            *len     = (size_t)plen;
            return opcode;
        }
        uint8_t *nb = realloc(c->frag, c->frag_len + (size_t)plen + 1);
        if (!nb) {
            free(data);
            return -1;
        }
        c->frag = nb;
        if (plen) memcpy(c->frag + c->frag_len, data, (size_t)plen);
        c->frag_len += (size_t)plen;
        c->frag[c->frag_len] = '\0';
        free(data);

        if (fin) {
            *payload    = c->frag;
            *len        = c->frag_len;
            c->frag     = NULL;
            c->frag_len = 0;
            return c->frag_op;
        }
        /* not complete: wait for the next fragment */
        timeout_ms = IO_TIMEOUT_MS;
    }
}

/***** Base64 *****/

static const char b64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t cqham_b64_encode(const uint8_t *in, size_t n, char *out, size_t outlen) {
    size_t o = 0;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < n) v |= (uint32_t)in[i + 1] << 8;
        if (i + 2 < n) v |= in[i + 2];
        if (o + 4 >= outlen) break;
        out[o++] = b64chars[(v >> 18) & 63];
        out[o++] = b64chars[(v >> 12) & 63];
        out[o++] = (i + 1 < n) ? b64chars[(v >> 6) & 63] : '=';
        out[o++] = (i + 2 < n) ? b64chars[v & 63] : '=';
    }
    if (o < outlen) out[o] = '\0';
    return o;
}

static int b64val(char ch) {
    if (ch >= 'A' && ch <= 'Z') return ch - 'A';
    if (ch >= 'a' && ch <= 'z') return ch - 'a' + 26;
    if (ch >= '0' && ch <= '9') return ch - '0' + 52;
    if (ch == '+' || ch == '-') return 62;
    if (ch == '/' || ch == '_') return 63;
    return -1;
}

size_t cqham_b64_decode(const char *in, uint8_t *out, size_t outlen) {
    uint32_t acc  = 0;
    int      bits = 0;
    size_t   o    = 0;
    for (const char *p = in; *p; p++) {
        if (*p == '=') break;
        int v = b64val(*p);
        if (v < 0) continue;   /* skips whitespace */
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (o >= outlen) return o;
            out[o++] = (uint8_t)((acc >> bits) & 0xFF);
        }
    }
    return o;
}
