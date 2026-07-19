/*
 * keepalive.c — listens on UDP port 5004 for client keepalive packets
 * and reclaims any target= UDP push whose client has gone silent too
 * long. See udp_stream.c's udp_push_start() for where
 * keepalive_match_addr/port/last_keepalive_time get set, and
 * keepalive.h for why this exists at all.
 */
#include "keepalive.h"
#include "udp_stream.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define KEEPALIVE_PORT 5004

/* How long without a matching keepalive before an active push is
 * reclaimed. Real client cadence is ~1/sec (see
 * hdhomerun_video_thread_execute() in libhdhomerun's hdhomerun_video.c),
 * so this gives generous (10x+) margin for transient packet loss or
 * scheduling jitter before treating a stream as genuinely abandoned.
 * Not verified against real firmware's own timeout (closed-source, no
 * reference available) — a reasonable, documented guess, same spirit
 * as several other timing constants in this project. */
#define KEEPALIVE_TIMEOUT_MS 15000

/* recv() timeout — bounds how promptly a stale push gets swept, and
 * doubles as the sweep's own polling interval (see listener_thread_main:
 * the sweep runs after every recvfrom(), whether it got a packet or
 * just timed out, so a lull with zero keepalive traffic at all still
 * gets checked on this cadence). */
#define SWEEP_INTERVAL_MS 2000

struct listener_ctx {
    struct hdhr_tuner *tuners;
    int tuner_count;
    int sock;
};

static long elapsed_ms(struct timespec a, struct timespec b)
{
    return (b.tv_sec - a.tv_sec) * 1000L + (b.tv_nsec - a.tv_nsec) / 1000000L;
}

static void sweep_stale_pushes(struct hdhr_tuner *tuners, int count)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    for (int i = 0; i < count; i++) {
        struct hdhr_tuner *t = &tuners[i];

        tuner_lock(t);
        bool active = t->udp_push_active;
        /* Skip a push we already told to stop: udp_push_stop() blocks
         * up to ~2s polling for the push thread to actually notice
         * stop_requested and exit, which can occasionally run longer
         * than one sweep tick if dvb_stream_read() is blocked waiting
         * on the capture thread — re-entering here before that settles
         * would just re-log and re-poll the same already-in-flight
         * shutdown. */
        bool already_stopping = active && t->udp_push_stop_requested;
        long since = active ? elapsed_ms(t->last_keepalive_time, now) : 0;
        tuner_unlock(t);

        if (active && !already_stopping && since > KEEPALIVE_TIMEOUT_MS) {
            fprintf(stderr, "keepalive: tuner%d: no client keepalive in %ldms, "
                             "reclaiming abandoned target= push\n", i, since);
            udp_push_stop(t); /* same shutdown path as an explicit target=none */
        }
    }
}

static void *listener_thread_main(void *arg)
{
    struct listener_ctx *ctx = arg;

    for (;;) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        uint8_t buf[16]; /* real keepalive payload is 4 bytes (the lockkey); a little slack */
        ssize_t n = recvfrom(ctx->sock, buf, sizeof(buf), 0, (struct sockaddr *)&src, &src_len);

        if (n >= 0) {
            for (int i = 0; i < ctx->tuner_count; i++) {
                struct hdhr_tuner *t = &ctx->tuners[i];
                tuner_lock(t);
                if (t->udp_push_active &&
                    t->keepalive_match_addr == src.sin_addr.s_addr &&
                    t->keepalive_match_port == ntohs(src.sin_port)) {
                    clock_gettime(CLOCK_MONOTONIC, &t->last_keepalive_time);
                    tuner_unlock(t);
                    break;
                }
                tuner_unlock(t);
            }
        }
        /* Whether a packet arrived or recv() just timed out, always
         * sweep — see SWEEP_INTERVAL_MS's comment. */
        sweep_stale_pushes(ctx->tuners, ctx->tuner_count);
    }
    return NULL;
}

bool keepalive_listener_start(struct hdhr_tuner *tuners, int tuner_count)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("keepalive: socket");
        return false;
    }

    struct timeval tv = { .tv_sec = SWEEP_INTERVAL_MS / 1000,
                           .tv_usec = (SWEEP_INTERVAL_MS % 1000) * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(KEEPALIVE_PORT);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("keepalive: bind");
        close(sock);
        return false;
    }

    struct listener_ctx *ctx = malloc(sizeof(*ctx));
    ctx->tuners = tuners;
    ctx->tuner_count = tuner_count;
    ctx->sock = sock;

    pthread_t th;
    if (pthread_create(&th, NULL, listener_thread_main, ctx) != 0) {
        perror("keepalive: pthread_create");
        close(sock);
        free(ctx);
        return false;
    }
    pthread_detach(th);
    fprintf(stderr, "keepalive: listening on udp/%d (timeout=%dms)\n",
            KEEPALIVE_PORT, KEEPALIVE_TIMEOUT_MS);
    return true;
}
