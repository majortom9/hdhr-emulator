/*
 * http_server.c — the HTTP surface that Plex/Emby/Jellyfin/Channels DVR
 * (and modern SiliconDust firmware itself) actually use day to day.
 * Listens on port 80, matching real hardware.
 *
 * /auto/vX.X doesn't name a specific tuner — real firmware auto-picks
 * one of its N physical tuners for such requests, so we do the same via
 * tuner_pool_claim_free() (see tuner.h), which also protects against
 * double-tuning a physical adapter that a control-plane target= push is
 * already using.
 */
#include "http_server.h"
#include "device_id.h"
#include "dvb_channel.h"
#include "dvb_stream.h"
#include "tuner.h"
#include "control.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdint.h>

#define HTTP_PORT 80

struct http_ctx {
    int fd;
    struct hdhr_config *cfg;
    struct hdhr_tuner *tuners;
};

static void send_headers(int fd, const char *status, const char *content_type, long content_length)
{
    char hdr[256];
    int n;
    if (content_length >= 0) {
        n = snprintf(hdr, sizeof(hdr),
                      "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %ld\r\n"
                      "Connection: close\r\nServer: hdhr-emulator\r\n\r\n",
                      status, content_type, content_length);
    } else {
        n = snprintf(hdr, sizeof(hdr),
                      "HTTP/1.1 %s\r\nContent-Type: %s\r\n"
                      "Connection: close\r\nServer: hdhr-emulator\r\n\r\n",
                      status, content_type);
    }
    if (write(fd, hdr, (size_t)n) < 0) { /* client already gone */ }
}

static void send_json(int fd, const char *json)
{
    send_headers(fd, "200 OK", "application/json", (long)strlen(json));
    if (write(fd, json, strlen(json)) < 0) { /* client already gone */ }
}

static void send_404(int fd)
{
    send_headers(fd, "404 Not Found", "text/plain", 0);
}

static void send_503(int fd, const char *msg)
{
    send_headers(fd, "503 Service Unavailable", "text/plain", (long)strlen(msg));
    if (write(fd, msg, strlen(msg)) < 0) { /* client already gone */ }
}

static int local_ip_for_peer(int connected_fd, char *out, size_t outlen)
{
    struct sockaddr_in local;
    socklen_t len = sizeof(local);
    if (getsockname(connected_fd, (struct sockaddr *)&local, &len) < 0) return -1;
    if (!inet_ntop(AF_INET, &local.sin_addr, out, outlen)) return -1;
    return 0;
}

static void handle_discover_json(int fd, const struct hdhr_config *cfg)
{
    char ip[16];
    if (local_ip_for_peer(fd, ip, sizeof(ip)) != 0) snprintf(ip, sizeof(ip), "0.0.0.0");

    char json[768];
    snprintf(json, sizeof(json),
        "{\n"
        "  \"FriendlyName\": \"%s\",\n"
        "  \"ModelNumber\": \"%s\",\n"
        "  \"FirmwareName\": \"%s\",\n"
        "  \"FirmwareVersion\": \"%s\",\n"
        "  \"DeviceID\": \"%08X\",\n"
        "  \"DeviceAuth\": \"\",\n"
        "  \"BaseURL\": \"http://%s:80\",\n"
        "  \"LineupURL\": \"http://%s:80/lineup.json\",\n"
        "  \"TunerCount\": %d\n"
        "}\n",
        cfg->friendly_name, cfg->model, cfg->firmware_name, cfg->firmware_version,
        cfg->device_id, ip, ip, cfg->tuner_count);
    send_json(fd, json);
}

static void handle_lineup_status_json(int fd)
{
    send_json(fd,
        "{\n"
        "  \"ScanInProgress\": 0,\n"
        "  \"ScanPossible\": 1,\n"
        "  \"Source\": \"Antenna\",\n"
        "  \"SourceList\": [\"Antenna\"]\n"
        "}\n");
}

static void handle_lineup_json(int fd, const struct hdhr_config *cfg)
{
    (void)cfg; /* not currently needed here — kept for symmetry/future use */
    char ip[16];
    if (local_ip_for_peer(fd, ip, sizeof(ip)) != 0) snprintf(ip, sizeof(ip), "0.0.0.0");

    size_t cap = 65536;
    size_t off = 0;
    char *body = malloc(cap);
    if (!body) { send_headers(fd, "500 Internal Server Error", "text/plain", 0); return; }
    off += (size_t)snprintf(body + off, cap - off, "[\n");

    int n = dvb_channel_count();
    for (int i = 0; i < n; i++) {
        const struct dvb_channel *ch = dvb_channel_at(i);
        if (!ch) continue;

        /* worst case a bit under 300 bytes for one entry (long name +
         * URL); keep plenty of headroom before formatting so snprintf
         * never has to truncate. */
        if (cap - off < 512) {
            cap *= 2;
            char *grown = realloc(body, cap);
            if (!grown) { free(body); send_headers(fd, "500 Internal Server Error", "text/plain", 0); return; }
            body = grown;
        }

        off += (size_t)snprintf(body + off, cap - off,
            "  {\"GuideNumber\": \"%d.%d\", \"GuideName\": \"%s\", "
            "\"URL\": \"http://%s:80/auto/v%d.%d\"}%s\n",
            ch->major, ch->minor, ch->short_name,
            ip, ch->major, ch->minor,
            (i == n - 1) ? "" : ",");
    }

    if (cap - off < 8) {
        cap += 8;
        char *grown = realloc(body, cap);
        if (!grown) { free(body); send_headers(fd, "500 Internal Server Error", "text/plain", 0); return; }
        body = grown;
    }
    off += (size_t)snprintf(body + off, cap - off, "]\n");

    send_json(fd, body);
    free(body);
}

/* requested_tuner_idx: -1 to auto-allocate any free tuner (the plain
 * /auto/vX.X path); >=0 to require that specific tuner slot (the
 * /tunerN/vX.X path, e.g. hdhomerun_config's "save /tunerN -"). */
static void stream_channel_to_client(int fd, const struct hdhr_config *cfg,
                                      struct hdhr_tuner *tuners, int requested_tuner_idx,
                                      int major, int minor)
{
    const struct dvb_channel *ch = dvb_find_channel(major, minor);
    if (!ch) {
        send_404(fd);
        return;
    }

    int reused_fd = -1;
    struct hdhr_tuner *t;
    if (requested_tuner_idx >= 0) {
        t = &tuners[requested_tuner_idx];
        if (!tuner_try_claim(t, ch->frequency_hz, ch->delivery, &reused_fd)) {
            send_503(fd, "tuner busy\n");
            return;
        }
    } else {
        t = tuner_pool_claim_free(tuners, cfg->tuner_count, ch->frequency_hz, ch->delivery, &reused_fd);
        if (!t) {
            send_503(fd, "all tuners busy\n");
            return;
        }
    }

    /* HTTP passthrough doesn't consult t->program or t->filter_override,
     * same established asymmetry as program (see tuner.h) — this path
     * always streams the plain named channel. */
    struct dvb_stream *ds = dvb_stream_open(t->adapter, cfg->dvb_frontend, cfg->dvb_demux,
                                             ch, DVB_PROGRAM_DEFAULT, NULL, reused_fd);
    if (!ds) {
        send_headers(fd, "502 Bad Gateway", "text/plain", 0);
        tuner_release(t);
        return;
    }
    tuner_bind_channel(t, ch, DVB_PROGRAM_DEFAULT);
    tuner_set_stream(t, ds);

    /* no Content-Length — this is a live, unbounded stream */
    char hdr[256];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: video/mpeg\r\n"
        "Connection: close\r\nServer: hdhr-emulator\r\n\r\n");
    if (write(fd, hdr, (size_t)hn) < 0) { tuner_release(t); return; }

    uint8_t buf[188 * 64];
    for (;;) {
        ssize_t n = dvb_stream_read(ds, buf, sizeof(buf));
        if (n <= 0) break;
        ssize_t off = 0;
        while (off < n) {
            ssize_t w = write(fd, buf + off, (size_t)(n - off));
            if (w <= 0) goto out;
            off += w;
        }
    }
out:
    tuner_release(t); /* closes ds internally */
}

static void handle_request(int fd, struct hdhr_config *cfg, struct hdhr_tuner *tuners)
{
    char req[2048] = {0};
    ssize_t n = read(fd, req, sizeof(req) - 1);
    if (n <= 0) return;

    char method[8] = {0}, path[512] = {0};
    if (sscanf(req, "%7s %511s", method, path) != 2) return;
    if (strcmp(method, "GET") != 0) {
        send_headers(fd, "405 Method Not Allowed", "text/plain", 0);
        return;
    }

    /* strip query string for routing */
    char *qs = strchr(path, '?');
    if (qs) *qs = '\0';

    if (strcmp(path, "/discover.json") == 0) {
        handle_discover_json(fd, cfg);
    } else if (strcmp(path, "/lineup_status.json") == 0) {
        handle_lineup_status_json(fd);
    } else if (strcmp(path, "/lineup.json") == 0) {
        handle_lineup_json(fd, cfg);
    } else if (strncmp(path, "/auto/v", 7) == 0) {
        int major = 0, minor = 0;
        sscanf(path + 7, "%d.%d", &major, &minor);
        stream_channel_to_client(fd, cfg, tuners, -1, major, minor);
    } else {
        int idx = -1, major = 0, minor = 0, consumed = 0;
        if (sscanf(path, "/tuner%d%n", &idx, &consumed) == 1 &&
            sscanf(path + consumed, "/v%d.%d", &major, &minor) == 2) {
            if (idx < 0 || idx >= cfg->tuner_count) {
                send_404(fd);
            } else {
                stream_channel_to_client(fd, cfg, tuners, idx, major, minor);
            }
        } else {
            send_404(fd);
        }
    }
}

static void *conn_thread_main(void *arg)
{
    struct http_ctx *hctx = (struct http_ctx *)arg;
    int one = 1;
    setsockopt(hctx->fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    handle_request(hctx->fd, hctx->cfg, hctx->tuners);

    close(hctx->fd);
    free(hctx);
    return NULL;
}

void *http_thread_main(void *arg)
{
    struct control_ctx *ctl = (struct control_ctx *)arg; /* reuse: {cfg, tuners} — see control.h */
    struct hdhr_config *cfg = ctl->cfg;
    struct hdhr_tuner *tuners = ctl->tuners;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("http: socket");
        return NULL;
    }

    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(HTTP_PORT);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("http: bind :80 (need root, or setcap CAP_NET_BIND_SERVICE)");
        close(fd);
        return NULL;
    }
    if (listen(fd, 32) < 0) {
        perror("http: listen");
        close(fd);
        return NULL;
    }

    fprintf(stderr, "http: listening on tcp/%d\n", HTTP_PORT);

    for (;;) {
        struct sockaddr_in peer;
        socklen_t peerlen = sizeof(peer);
        int cfd = accept(fd, (struct sockaddr *)&peer, &peerlen);
        if (cfd < 0) continue;

        struct http_ctx *hctx = malloc(sizeof(*hctx));
        hctx->fd = cfd;
        hctx->cfg = cfg;
        hctx->tuners = tuners;

        pthread_t th;
        if (pthread_create(&th, NULL, conn_thread_main, hctx) != 0) {
            close(cfd);
            free(hctx);
            continue;
        }
        pthread_detach(th);
    }

    close(fd);
    return NULL;
}
