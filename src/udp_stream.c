/*
 * udp_stream.c — the control protocol's target= UDP push. Goes through
 * tuner_try_claim()/tuner_release() like any other stream consumer of a
 * physical tuner (see tuner.h) — a target= push and an HTTP /auto pull
 * are just two different sinks for the same underlying dvb_stream
 * capture, and can't both hold the same physical adapter at once.
 */
#include "udp_stream.h"
#include "dvb_stream.h"
#include "dvb_channel.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdint.h>

#define TS_PACKET_SIZE   188
#define TS_PER_DATAGRAM  7
#define DATAGRAM_SIZE    (TS_PACKET_SIZE * TS_PER_DATAGRAM) /* 1316 */

struct push_ctx {
    struct hdhr_config cfg; /* snapshot, so config reloads mid-stream don't race us */
    struct hdhr_tuner *tuner;
    struct dvb_channel channel; /* snapshot at push-start time */
    int  program_override;
    char dest_ip[64];
    int  dest_port;
};

static void *push_thread_main(void *arg)
{
    struct push_ctx *ctx = (struct push_ctx *)arg;
    struct hdhr_tuner *t = ctx->tuner;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("udp_stream: socket");
        goto done_no_claim;
    }

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)ctx->dest_port);
    if (inet_pton(AF_INET, ctx->dest_ip, &dst.sin_addr) != 1) {
        fprintf(stderr, "udp_stream: bad destination IP %s\n", ctx->dest_ip);
        close(sock);
        goto done_no_claim;
    }

    if (!tuner_try_claim(t)) {
        fprintf(stderr, "udp_stream: tuner%d: already in use by another stream, "
                         "refusing target= push\n", t->index);
        close(sock);
        goto done_no_claim;
    }

    struct dvb_stream *ds = dvb_stream_open(t->adapter, ctx->cfg.dvb_frontend, ctx->cfg.dvb_demux,
                                             &ctx->channel, ctx->program_override);
    if (!ds) {
        fprintf(stderr, "udp_stream: tuner%d: failed to open DVB stream for %d.%d\n",
                t->index, ctx->channel.major, ctx->channel.minor);
        close(sock);
        tuner_release(t);
        goto done_no_claim;
    }
    tuner_bind_channel(t, &ctx->channel, ctx->program_override);
    tuner_set_stream(t, ds);

    fprintf(stderr, "udp_stream: tuner%d: pushing %d.%d to %s:%d\n",
            t->index, ctx->channel.major, ctx->channel.minor, ctx->dest_ip, ctx->dest_port);

    uint8_t datagram[DATAGRAM_SIZE];
    size_t have = 0;
    uint64_t packets_sent = 0;
    uint64_t bytes_sent = 0;

    while (!t->udp_push_stop_requested) {
        ssize_t n = dvb_stream_read(ds, datagram + have, sizeof(datagram) - have);
        if (n <= 0) break; /* upstream EOF/error, or dvb_stream_close() was called under us */
        have += (size_t)n;

        if (have == DATAGRAM_SIZE) {
            ssize_t sent = sendto(sock, datagram, DATAGRAM_SIZE, 0,
                                   (struct sockaddr *)&dst, sizeof(dst));
            if (sent < 0) {
                perror("udp_stream: sendto");
                break;
            }
            packets_sent += TS_PER_DATAGRAM;
            bytes_sent += (uint64_t)sent;
            have = 0;
        }
    }

    close(sock);

    fprintf(stderr, "udp_stream: tuner%d: push stopped (%llu TS packets, %llu bytes sent)\n",
            t->index, (unsigned long long)packets_sent, (unsigned long long)bytes_sent);

    tuner_release(t); /* closes ds internally */
done_no_claim:
    free(ctx);
    return NULL;
}

int udp_push_start(const struct hdhr_config *cfg, struct hdhr_tuner *t,
                    const char *dest_ip, int dest_port)
{
    udp_push_stop(t); /* only one push per tuner at a time */

    tuner_lock(t);
    if (!t->vch_resolved) {
        tuner_unlock(t);
        fprintf(stderr, "udp_stream: tuner%d: no channel resolved, refusing target= push\n", t->index);
        return -1;
    }
    const struct dvb_channel *ch = dvb_find_channel(t->vch_major, t->vch_minor);
    if (!ch) {
        tuner_unlock(t);
        fprintf(stderr, "udp_stream: tuner%d: resolved channel %d.%d no longer in the "
                         "channel database (rescanned since?)\n", t->index, t->vch_major, t->vch_minor);
        return -1;
    }

    struct push_ctx *ctx = calloc(1, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->tuner = t;
    ctx->channel = *ch;
    ctx->program_override = t->program;
    snprintf(ctx->dest_ip, sizeof(ctx->dest_ip), "%s", dest_ip);
    ctx->dest_port = dest_port;

    t->udp_push_stop_requested = 0;
    t->udp_push_active = 1;
    int rc = pthread_create(&t->udp_push_thread, NULL, push_thread_main, ctx);
    if (rc != 0) {
        t->udp_push_active = 0;
        free(ctx);
        tuner_unlock(t);
        return -1;
    }
    pthread_detach(t->udp_push_thread);
    tuner_unlock(t);
    return 0;
}

void udp_push_stop(struct hdhr_tuner *t)
{
    tuner_lock(t);
    int active = t->udp_push_active;
    if (active) {
        t->udp_push_stop_requested = 1;
    }
    tuner_unlock(t);

    /* Thread is detached (so start/stop can't deadlock against the control
     * thread) — poll briefly for it to notice the stop request. */
    if (active) {
        for (int i = 0; i < 400 && t->udp_push_active; i++) {
            usleep(5000);
        }
    }
}
