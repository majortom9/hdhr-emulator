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
#include <time.h>

#define TS_PACKET_SIZE   188
#define TS_PER_DATAGRAM  7
#define DATAGRAM_SIZE    (TS_PACKET_SIZE * TS_PER_DATAGRAM) /* 1316 */

struct push_ctx {
    struct hdhr_config cfg; /* snapshot, so config reloads mid-stream don't race us */
    struct hdhr_tuner *tuner;
    struct dvb_channel channel; /* snapshot at push-start time */
    int  program_override;
    char pid_filter[256]; /* snapshot of t->filter_override at push-start time; see tuner.h */
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

    int reused_fd = -1;
    if (!tuner_try_claim(t, ctx->channel.frequency_hz, ctx->channel.delivery, &reused_fd)) {
        fprintf(stderr, "udp_stream: tuner%d: already in use by another stream, "
                         "refusing target= push\n", t->index);
        close(sock);
        goto done_no_claim;
    }

    struct dvb_stream *ds = dvb_stream_open(t->adapter, ctx->cfg.dvb_frontend, ctx->cfg.dvb_demux,
                                             &ctx->channel, ctx->program_override, ctx->pid_filter,
                                             reused_fd);
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

    /* Reset target back to "none" here rather than only on an explicit
     * target=none SET — this loop can also exit on its own (upstream
     * EOF/error, sendto failure, or keepalive.c reclaiming an
     * abandoned push), and t->target is what /tunerN/target GET
     * reports; leaving it at a stale "udp://..."/"rtp://..." value
     * after the push actually stopped would be a lie. */
    tuner_lock(t);
    snprintf(t->target, sizeof(t->target), "none");
    tuner_unlock(t);

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
    const struct dvb_channel *ch = NULL;
    if (t->vch_resolved) {
        ch = dvb_find_channel(t->vch_major, t->vch_minor);
        if (!ch) {
            tuner_unlock(t);
            fprintf(stderr, "udp_stream: tuner%d: resolved channel %d.%d no longer in the "
                             "channel database (rescanned since?)\n", t->index, t->vch_major, t->vch_minor);
            return -1;
        }
    } else if (t->tuned_frequency_hz != 0) {
        /* No named virtual channel selected (e.g. a raw /tunerN/channel
         * RF tune rather than /tunerN/vchannel) — real usage streams
         * directly off channel+program in this case (a manual RF
         * tune + explicit program number, or program=0 for full-mux
         * passthrough, neither of which needs PSIP name resolution).
         * Any channel already known on this mux works as the base
         * dvb_stream_open() struct: it only supplies the PAT/PMT/PID
         * lookup starting point, and dvb_stream_open() itself resolves
         * program_override against the mux's siblings (or ignores the
         * base entirely for program=0's full-mux passthrough). */
        const struct dvb_channel *siblings[DVB_CHANNEL_MAX];
        int n = dvb_channels_on_freq(t->tuned_frequency_hz, siblings, DVB_CHANNEL_MAX);
        if (n > 0) ch = siblings[0];
        if (!ch) {
            tuner_unlock(t);
            fprintf(stderr, "udp_stream: tuner%d: no channels known on %u Hz "
                             "(scan it first via /tunerN/channel)\n", t->index, t->tuned_frequency_hz);
            return -1;
        }
    } else {
        tuner_unlock(t);
        fprintf(stderr, "udp_stream: tuner%d: no channel/frequency set, refusing target= push\n", t->index);
        return -1;
    }

    struct push_ctx *ctx = calloc(1, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->tuner = t;
    ctx->channel = *ch;
    ctx->program_override = t->program;
    snprintf(ctx->pid_filter, sizeof(ctx->pid_filter), "%s", t->filter_override);
    snprintf(ctx->dest_ip, sizeof(ctx->dest_ip), "%s", dest_ip);
    ctx->dest_port = dest_port;

    /* Binary form of the destination, for keepalive.c to match incoming
     * client keepalive packets against — see tuner.h's
     * keepalive_match_addr/port comment. Failure to parse just leaves
     * it as 0.0.0.0:0, which won't match any real client's source
     * address, so a bad IP here degrades to "never reclaimed" rather
     * than a crash; dvb_stream_open() inside the push thread will
     * separately fail/log on a genuinely bad destination once it
     * actually tries to use dest_ip. */
    struct in_addr addr;
    t->keepalive_match_addr = (inet_pton(AF_INET, dest_ip, &addr) == 1) ? addr.s_addr : 0;
    t->keepalive_match_port = (uint16_t)dest_port;
    clock_gettime(CLOCK_MONOTONIC, &t->last_keepalive_time); /* grace period starts now */

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
