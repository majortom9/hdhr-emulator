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

/* How long to wait for the tuner to free up (target_channel_wait) or for
 * dvb_channel_db to pick up the current mux (see udp_push_start()'s own
 * comment for why the latter is needed at all) before giving up.
 * Comfortably under a real client's own single-attempt reply patience
 * (libhdhomerun's HDHOMERUN_CONTROL_RECV_TIMEOUT is 2500ms, confirmed by
 * reading hdhomerun_control.c directly — the /tunerN/target SET goes
 * through the same generic GETSET mechanism as /tunerN/channel, so the
 * same ceiling applies here). */
#define TARGET_CHANNEL_WAIT_MS 2000
#define TARGET_CHANNEL_POLL_MS 50

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

    /* _wait, not tuner_try_claim(): the tuner is very commonly still
     * held for a few hundred ms by our own channel_scan_thread_main
     * finishing its post-lock PSIP read (see control.c) — a client
     * that fires target= immediately after its /tunerN/channel SET's
     * reply (which only waits for the *lock*, not the PSIP read; see
     * CHANNEL_SET_WAIT_MS's own comment) routinely races that same
     * short-lived internal claim, not another live stream. Confirmed
     * live (2026-07-22): a real tvheadend instance did exactly this on
     * essentially every channel and got "already in use" here even
     * though nothing else was actually streaming. This still runs on
     * its own background thread (spawned by udp_push_start, which has
     * already replied to the client by the time we get here), so
     * waiting doesn't cost the client anything. */
    int reused_fd = -1;
    if (!tuner_try_claim_wait(t, TARGET_CHANNEL_WAIT_MS, ctx->channel.frequency_hz,
                               ctx->channel.delivery, &reused_fd)) {
        fprintf(stderr, "udp_stream: tuner%d: still in use after waiting, "
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
    bool vch_resolved = t->vch_resolved;
    int vch_major = t->vch_major, vch_minor = t->vch_minor;
    uint32_t tuned_freq = t->tuned_frequency_hz;
    enum hdhr_delivery_system tuned_delivery = t->tuned_delivery;
    int program = t->program;
    tuner_unlock(t);

    const struct dvb_channel *ch = NULL;
    struct dvb_channel synth;
    int program_override = program;
    if (vch_resolved) {
        ch = dvb_find_channel(vch_major, vch_minor);
        if (!ch) {
            fprintf(stderr, "udp_stream: tuner%d: resolved channel %d.%d no longer in the "
                             "channel database (rescanned since?)\n", t->index, vch_major, vch_minor);
            return -1;
        }
    } else if (tuned_freq != 0 && program <= 0) {
        /* No named virtual channel selected (e.g. a raw /tunerN/channel
         * RF tune rather than /tunerN/vchannel), and no specific
         * program requested either (program is still the untouched
         * DVB_PROGRAM_DEFAULT (-1) sentinel, or an explicit 0) -- there
         * was never a resolved vchannel to have a "default program" to
         * speak of, so there's nothing to look up in dvb_channel_db at
         * all. This is exactly tvheadend's own tvhdhomerun input's
         * pattern while mux-scanning (confirmed live, 2026-07-22): it
         * tunes a raw frequency and immediately asks for target=,
         * intending to parse PAT/PMT/SI out of the raw transport
         * stream *itself* to discover services on that mux -- it never
         * sets /tunerN/program. A single immediate dvb_channel_db
         * lookup here (as this used to do) raced tvheadend's own
         * near-instant target= call against our asynchronous PSIP
         * read (which only starts after the lock result, not the full
         * channel metadata, is returned -- see CHANNEL_SET_WAIT_MS's
         * own comment in control.c) and lost on nearly every channel,
         * including ones that locked cleanly and had real PSIP data
         * moments later -- tvheadend logged "failed to set target: 0"
         * and gave up rather than ever actually streaming. Building a
         * synthetic channel from tuned_freq/tuned_delivery (both known
         * synchronously the instant the tune itself locks, no PSIP
         * wait needed) and forcing full-mux passthrough sidesteps the
         * race entirely, and is the behavior actually being asked for.
         *
         * But this shortcut only holds if the tune actually *locked* --
         * skipping dvb_channel_db here also skips the one place that
         * used to implicitly require a successful scan first. Confirmed
         * live (2026-07-22): without this check, an unlocked frequency
         * (e.g. one of tvheadend's own scan sweep's many empty muxes)
         * still opened a "stream" with no real data, which then held
         * the tuner claimed indefinitely (dvb_stream_read() never
         * produces data or EOF for an unlocked mux) -- every subsequent
         * /tunerN/channel SET on this tuner, including the client's next
         * real playback attempt, got refused as "tuner busy streaming"
         * until the keepalive-abandonment timeout eventually reclaimed
         * it. last_attempt_freq/last_attempt_locked are exactly what the
         * /tunerN/channel SET's own reply already waited on (see
         * CHANNEL_SET_WAIT_MS's own comment in control.c) -- if a client
         * got a real reply for this frequency at all, these are already
         * set correctly for it by the time any target= could follow. */
        tuner_lock(t);
        bool locked = (t->last_attempt_freq == tuned_freq && t->last_attempt_locked);
        tuner_unlock(t);
        if (!locked) {
            fprintf(stderr, "udp_stream: tuner%d: %u Hz did not lock, refusing target= push\n",
                    t->index, tuned_freq);
            return -1;
        }
        memset(&synth, 0, sizeof(synth));
        synth.frequency_hz = tuned_freq;
        synth.delivery = tuned_delivery;
        ch = &synth;
        program_override = 0;
    } else if (tuned_freq != 0) {
        /* An explicit specific program (>0) was requested via
         * /tunerN/program without ever selecting a resolved vchannel —
         * genuinely needs a dvb_channel_db entry to resolve that
         * program's own PSIP-derived PIDs, which the case above can't
         * shortcut. Retries briefly (outside t->lock -- this can take
         * up to TARGET_CHANNEL_WAIT_MS, and other tuner operations
         * shouldn't block on it) instead of failing on the very first
         * lookup, for the same asynchronous-PSIP-read reason described
         * above. */
        const struct dvb_channel *siblings[DVB_CHANNEL_MAX];
        struct timespec deadline;
        clock_gettime(CLOCK_MONOTONIC, &deadline);
        deadline.tv_sec += TARGET_CHANNEL_WAIT_MS / 1000;
        deadline.tv_nsec += (TARGET_CHANNEL_WAIT_MS % 1000) * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec += 1;
            deadline.tv_nsec -= 1000000000L;
        }
        for (;;) {
            int n = dvb_channels_on_freq(tuned_freq, siblings, DVB_CHANNEL_MAX);
            if (n > 0) { ch = siblings[0]; break; }
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            if (now.tv_sec > deadline.tv_sec ||
                (now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec)) {
                break;
            }
            usleep(TARGET_CHANNEL_POLL_MS * 1000);
        }
        if (!ch) {
            fprintf(stderr, "udp_stream: tuner%d: no channels known on %u Hz "
                             "(scan it first via /tunerN/channel)\n", t->index, tuned_freq);
            return -1;
        }
        /* Re-verify under lock -- the tuner's own selection could have
         * changed to something else entirely while we were waiting
         * above (a channel=none, or a completely different channel
         * SET). Proceeding to stream the *old* target's channel data
         * to a client that asked for something new would be wrong. */
        tuner_lock(t);
        bool still_current = (t->tuned_frequency_hz == tuned_freq && !t->vch_resolved);
        tuner_unlock(t);
        if (!still_current) {
            fprintf(stderr, "udp_stream: tuner%d: selection changed while waiting for %u Hz's "
                             "channel data -- refusing stale target= push\n", t->index, tuned_freq);
            return -1;
        }
    } else {
        fprintf(stderr, "udp_stream: tuner%d: no channel/frequency set, refusing target= push\n", t->index);
        return -1;
    }

    tuner_lock(t);
    struct push_ctx *ctx = calloc(1, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->tuner = t;
    ctx->channel = *ch;
    ctx->program_override = program_override;
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
