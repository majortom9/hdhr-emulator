/*
 * control.c — TCP control protocol on port 65001.
 *
 * Real HDHomeRun clients (hdhomerun_config, MythTV's HDHR input, older
 * NextPVR, the SiliconDust apps) use this to get/set values under a
 * Unix-path-like namespace. We implement the subset that matters for a
 * directly-DVB-driven ATSC tuner (no TVheadend — see dvb_scan.c/
 * dvb_stream.c for the RF tuning and PSIP channel mapping this used to
 * delegate):
 *
 *   /sys/model, /sys/features, /sys/version        (read-only)
 *   /tunerN/channel    raw RF tune: "auto:<freq_hz>" | "8vsb:<freq_hz>" |
 *                      "us-bcast:<N>" | "none". Claims the tuner and
 *                      tunes + performs live PSIP detection
 *                      (dvb_scan_tune_and_lock + dvb_scan_read_psip,
 *                      same mechanism dvb_scan.c's startup scan uses) in
 *                      a detached background thread, merging anything
 *                      found into the shared channel database — this is
 *                      what backs hdhomerun_config's own `scan`
 *                      subcommand. The SET reply waits up to
 *                      CHANNEL_SET_WAIT_MS for just the lock result (not
 *                      the full PSIP read) so a follow-up
 *                      /tunerN/status poll sees it immediately in the
 *                      common case, but won't block indefinitely — some
 *                      DVB drivers can take far longer than that inside
 *                      a single blocking ioctl on a dead frequency; see
 *                      the comment at the SET handler's channel branch.
 *                      Does not select a specific virtual
 *                      channel/program by itself.
 *   /tunerN/channelmap GET-only, always "us-bcast" (8VSB/US-OTA only,
 *                      per project scope — SET is rejected)
 *   /tunerN/vchannel   set "<major>.<minor>" to select a channel found by
 *                      the ATSC scan (dvb_scan.c) — resolves major.minor
 *                      to a frequency + program number in our own
 *                      channel database, real RF tuning happens lazily
 *                      when a stream is actually opened (target= or an
 *                      HTTP pull), not at vchannel-set time, since only
 *                      one physical tuner lock can be held at once (see
 *                      tuner.h's claim/release model).
 *   /tunerN/program    MPEG program number filter — 0 for full-mux
 *                      passthrough, or a specific program_number to pick
 *                      a different subchannel sharing the selected
 *                      channel's mux (see dvb_stream.h)
 *   /tunerN/target     "none" | "udp://ip:port" — drives udp_stream.c
 *   /tunerN/lockkey    stored/reported; NOT enforced against concurrent
 *                      writers yet (fine for a single-household LAN tool,
 *                      called out here so it isn't assumed otherwise)
 *   /tunerN/status     live signal lock/strength/quality while streaming
 *                      (from the DVB frontend's own S2API stats), or a
 *                      "channel selected, not yet streaming" summary
 *                      otherwise
 *   /tunerN/streaminfo live per-mux virtual-channel listing, built on
 *                      every GET from the shared channel database and
 *                      whichever frequency this tuner is currently
 *                      RF-tuned to (channel or vchannel) — independent
 *                      of whether anything is actively streaming, same
 *                      as real firmware. Format matches what
 *                      hdhomerun_channelscan.c actually parses:
 *                      "tsid=0x%04X\n" then one "<program>: <major>.<minor>
 *                      <name>\n" line per virtual channel on that mux.
 */
#include "control.h"
#include "hdhr_pkt.h"
#include "dvb_channel.h"
#include "dvb_stream.h"
#include "dvb_frontend.h"
#include "dvb_scan.h"
#include "atsc_freq.h"
#include "udp_stream.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

struct conn_ctx {
    int fd;
    struct control_ctx *ctl;
};

static ssize_t read_full(int fd, void *buf, size_t len)
{
    size_t got = 0;
    uint8_t *p = buf;
    while (got < len) {
        ssize_t n = read(fd, p + got, len - got);
        if (n < 0) return -1;
        if (n == 0) return (ssize_t)got; /* peer closed */
        got += (size_t)n;
    }
    return (ssize_t)got;
}

static void send_error_reply(int fd, const char *name, const char *msg)
{
    struct hdhr_pkt out;
    hdhr_pkt_start_frame(&out);
    hdhr_pkt_write_tlv_str(&out, HDHR_TAG_GETSET_NAME, name);
    hdhr_pkt_write_tlv_str(&out, HDHR_TAG_ERROR_MESSAGE, msg);
    size_t len = hdhr_pkt_seal_frame(&out, HDHR_TYPE_GETSET_RPY);
    if (write(fd, out.buffer, len) < 0) { /* client already gone */ }
}

static void send_value_reply(int fd, const char *name, const char *value)
{
    struct hdhr_pkt out;
    hdhr_pkt_start_frame(&out);
    hdhr_pkt_write_tlv_str(&out, HDHR_TAG_GETSET_NAME, name);
    hdhr_pkt_write_tlv_str(&out, HDHR_TAG_GETSET_VALUE, value);
    size_t len = hdhr_pkt_seal_frame(&out, HDHR_TYPE_GETSET_RPY);
    if (write(fd, out.buffer, len) < 0) { /* client already gone */ }
}

/* Parses "/tunerN/leaf" -> tuner index + leaf pointer. Returns -1 if the
 * name doesn't match that shape. */
static int parse_tuner_path(const char *name, int max_tuners, const char **leaf)
{
    int idx = -1, consumed = 0;
    if (sscanf(name, "/tuner%d%n", &idx, &consumed) != 1) return -1;
    if (idx < 0 || idx >= max_tuners) return -1;
    if (name[consumed] != '/') return -1;
    *leaf = name + consumed + 1;
    return idx;
}

static int parse_udp_target(const char *value, char *ip_out, size_t ip_out_len, int *port_out)
{
    /* accepts "udp://ip:port" (and tolerates "rtp://" the same way, since
     * we only implement the plain-UDP TS packing described in the
     * project's README — no RTP header). */
    const char *p = strstr(value, "://");
    if (!p) return -1;
    p += 3;
    const char *colon = strrchr(p, ':');
    if (!colon) return -1;

    size_t iplen = (size_t)(colon - p);
    if (iplen == 0 || iplen >= ip_out_len) return -1;
    memcpy(ip_out, p, iplen);
    ip_out[iplen] = '\0';

    *port_out = atoi(colon + 1);
    if (*port_out <= 0 || *port_out > 65535) return -1;
    return 0;
}

static void handle_sys_get(int fd, const struct hdhr_config *cfg, const char *name, const char *leaf)
{
    if (strcmp(leaf, "model") == 0) {
        send_value_reply(fd, name, cfg->model);
    } else if (strcmp(leaf, "version") == 0) {
        send_value_reply(fd, name, cfg->firmware_version);
    } else if (strcmp(leaf, "features") == 0) {
        /* Confirmed against a genuine HDHomeRun3's actual /sys/features
         * response (2026-07-19): three newline-separated "name: values"
         * lines (channelmap, modulation, auto-modulation), each a
         * space-separated list — not the flat "channel-signal-strength
         * ..." capability-token string this used to send (an earlier,
         * unconfirmed guess). hdhomerun_config_gui reads the
         * "channelmap:" line to populate its Channel selector's
         * available maps and valid channel-number range; without it in
         * the expected format, the selector showed blank and its
         * channel spinner fell back to a meaningless default (observed:
         * stuck at 255). Scoped to just what we actually support
         * (8VSB/US-OTA only, per project scope) rather than echoing a
         * real device's full multi-standard list (us-cable, qam256,
         * etc.) which we can't actually tune. */
        send_value_reply(fd, name,
            "channelmap: us-bcast\n"
            "modulation: 8vsb\n"
            "auto-modulation: auto\n");
    } else {
        send_error_reply(fd, name, "ERROR: parameter is read-only or unknown");
    }
}

static void handle_tuner_get(int fd, struct hdhr_tuner *t, const char *name, const char *leaf)
{
    char val[512];

    tuner_lock(t);
    if (strcmp(leaf, "channel") == 0) {
        snprintf(val, sizeof(val), "%s", t->channel);
    } else if (strcmp(leaf, "channelmap") == 0) {
        /* Only one channelmap supported (8VSB/US-OTA only, per project
         * scope) — see atsc_freq.h. */
        snprintf(val, sizeof(val), "us-bcast");
    } else if (strcmp(leaf, "vchannel") == 0) {
        if (t->vch_resolved) {
            snprintf(val, sizeof(val), "%d.%d", t->vch_major, t->vch_minor);
        } else {
            snprintf(val, sizeof(val), "none");
        }
    } else if (strcmp(leaf, "program") == 0) {
        snprintf(val, sizeof(val), "%d", t->program);
    } else if (strcmp(leaf, "target") == 0) {
        snprintf(val, sizeof(val), "%s", t->target);
    } else if (strcmp(leaf, "lockkey") == 0) {
        if (t->lockkey) snprintf(val, sizeof(val), "0x%08X", t->lockkey);
        else snprintf(val, sizeof(val), "none");
    } else if (strcmp(leaf, "status") == 0) {
        int ffd = t->active_stream ? dvb_stream_frontend_fd(t->active_stream) : -1;
        if (ffd >= 0) {
            struct dvb_signal_stats stats;
            dvb_frontend_read_stats(ffd, &stats);
            if (stats.symbol_quality_pct < 0) {
                /* Modern DVBv5 block-count stats unavailable on this
                 * driver (confirmed: lgdt3306a doesn't populate them at
                 * all) — fall back to the legacy FE_READ_UNCORRECTED_BLOCKS
                 * ioctl, windowed against the fixed ATSC segment rate. */
                int legacy_pct = dvb_stream_get_legacy_seq_pct(t->active_stream);
                if (legacy_pct >= 0) stats.symbol_quality_pct = legacy_pct;
            }
            double bps = 0.0, pps = 0.0;
            dvb_stream_get_rate(t->active_stream, &bps, &pps);
            snprintf(val, sizeof(val),
                     "ch=%s lock=%s ss=%d snq=%d seq=%d bps=%.0f pps=%.0f",
                     t->channel,
                     stats.has_lock ? "8vsb" : "none",
                     stats.signal_strength_pct < 0 ? 0 : stats.signal_strength_pct,
                     stats.snr_quality_pct < 0 ? 0 : stats.snr_quality_pct,
                     stats.symbol_quality_pct < 0 ? 0 : stats.symbol_quality_pct,
                     bps, pps);
        } else if (t->scan_stats_valid && t->scan_stats_freq == t->tuned_frequency_hz) {
            /* No active stream, but a /tunerN/channel scan has published
             * at least one live reading for the frequency we're
             * currently on — report it instead of the bare placeholder
             * below, same reasoning as the streaming branch above (see
             * tuner.h's scan_stats comment).
             *
             * The lock=/none word itself comes from t->status, NOT
             * stats->has_lock: stats is only refreshed every
             * PROGRESS_CB_INTERVAL_MS (250ms, see dvb_frontend.c), so
             * right at the moment a channel actually achieves lock,
             * there's a window where the authoritative result (set by
             * finalize_lock_result once dvb_scan_tune_and_lock's own
             * loop returns) already says locked but the last-published
             * stats snapshot hasn't caught up yet — confirmed live: a
             * client saw "lock=none ss=97 snq=70" for a channel that
             * had, in fact, already locked. t->status is updated
             * exactly once the real result is known (see
             * handle_tuner_set/finalize_lock_result), so it's the
             * correct source for this word; scan_stats remains the
             * right source for ss=/snq=/seq= specifically, which are
             * meant to be best-effort/live rather than a single final
             * answer. */
            struct dvb_signal_stats *stats = &t->scan_stats;
            char lock_word[32] = "none";
            const char *lock_field = strstr(t->status, "lock=");
            if (lock_field) {
                sscanf(lock_field + 5, "%31s", lock_word);
            }
            snprintf(val, sizeof(val),
                     "ch=%s lock=%s ss=%d snq=%d seq=%d",
                     t->channel,
                     lock_word,
                     stats->signal_strength_pct < 0 ? 0 : stats->signal_strength_pct,
                     stats->snr_quality_pct < 0 ? 0 : stats->snr_quality_pct,
                     stats->symbol_quality_pct < 0 ? 0 : stats->symbol_quality_pct);
        } else {
            snprintf(val, sizeof(val), "%s", t->status);
        }
    } else if (strcmp(leaf, "streaminfo") == 0) {
        if (t->tuned_frequency_hz == 0) {
            snprintf(val, sizeof(val), "none");
        } else {
            const struct dvb_channel *chs[32];
            int n = dvb_channels_on_freq(t->tuned_frequency_hz, chs, 32);
            if (n == 0) {
                snprintf(val, sizeof(val), "none");
            } else {
                size_t off = 0;
                off += (size_t)snprintf(val + off, sizeof(val) - off,
                                         "tsid=0x%04X\n", chs[0]->channel_tsid);
                for (int i = 0; i < n && off < sizeof(val); i++) {
                    off += (size_t)snprintf(val + off, sizeof(val) - off,
                                             "%u: %u.%u %s\n",
                                             chs[i]->program_number, chs[i]->major,
                                             chs[i]->minor, chs[i]->short_name);
                }
            }
        }
    } else {
        tuner_unlock(t);
        send_error_reply(fd, name, "ERROR: parameter is write-only or unknown");
        return;
    }
    tuner_unlock(t);
    send_value_reply(fd, name, val);
}

static bool parse_channel_value(const char *value, uint32_t *out_freq, int *out_rf_channel)
{
    if (strncmp(value, "auto:", 5) == 0) {
        uint32_t n = (uint32_t)strtoul(value + 5, NULL, 10);
        if (n == 0) return false;
        /* "auto:" is ambiguous by design in the real protocol: it
         * accepts either a small channel NUMBER or a raw frequency in
         * Hz under the same prefix, and real firmware distinguishes by
         * magnitude. Confirmed live via packet capture — the scan path
         * (hdhomerun_channelscan.c's channelscan_find_lock()) sends
         * "auto:<freq_hz>" (e.g. "auto:497000000"), but
         * hdhomerun_config_gui's manual channel-number up/down spinner
         * sends "auto:<N>" with N being the small channel number itself
         * (e.g. "auto:32", "auto:33") — confirmed via a live tcpdump
         * capture of the GUI's spinner clicks, which is why turning
         * that spinner never visibly worked against this daemon: we
         * were tuning to a literal 32 Hz "frequency", which obviously
         * never locks. ATSC channel numbers here only ever run 2-36;
         * any real RF frequency is tens of millions of Hz, so a low
         * threshold unambiguously separates the two forms. */
        if (n < 1000) {
            uint32_t freq = atsc_channel_to_freq((int)n);
            if (freq == 0) return false; /* unknown/out-of-range channel number */
            *out_freq = freq;
            *out_rf_channel = (int)n;
            return true;
        }
        *out_freq = n;
        *out_rf_channel = atsc_freq_to_channel(n); /* 0 if not an exact table match — still fine to tune */
        return true;
    }
    if (strncmp(value, "8vsb:", 5) == 0) {
        /* Unlike "auto:", "8vsb:" always means an explicit frequency
         * (modulation:frequency) — no ambiguity to resolve here. */
        uint32_t freq = (uint32_t)strtoul(value + 5, NULL, 10);
        if (freq == 0) return false;
        *out_freq = freq;
        *out_rf_channel = atsc_freq_to_channel(freq);
        return true;
    }
    if (strncmp(value, "us-bcast:", 9) == 0) {
        int ch = atoi(value + 9);
        uint32_t freq = atsc_channel_to_freq(ch);
        if (freq == 0) return false; /* unknown/out-of-range channel number */
        *out_freq = freq;
        *out_rf_channel = ch;
        return true;
    }
    return false;
}

/* Runs the RF tune + lock (dvb_scan_tune_and_lock) and, if locked, the
 * PAT/PMT/TVCT read (dvb_scan_read_psip) for a /tunerN/channel SET in a
 * background thread, so the SET reply isn't hostage to the DVB driver's
 * worst-case blocking behavior on a dead frequency (see the comment at
 * the SET handler's call site). The calling connection thread waits on
 * `cond` for up to CHANNEL_SET_WAIT_MS for the *lock* result specifically
 * (not the full PSIP read, which can legitimately take longer on a busy
 * mux) so it can reply with an accurate lock status in the common case;
 * `caller_gave_up` tells this thread whether it or the caller is
 * responsible for finalizing tuner state and freeing this struct.
 *
 * A /tunerN/channel SET that arrives while this thread is still running
 * doesn't wait for the tuner to free up at all (see handle_tuner_set's
 * channel branch) — it enqueues its frequency onto t->pending_queue (a
 * bounded FIFO, see tuner.h) and replies immediately. Once this thread
 * finishes its current attempt, it dequeues the next entry before
 * releasing the tuner and, if there is one, loops around to tune it next
 * instead of exiting — so back-to-back requests (e.g. hdhomerun_config's
 * `scan`, which fires its next SET as soon as it gets *any* reply for
 * the current one) hand off to this same in-flight worker rather than a
 * fresh request ever blocking on tuner_try_claim, no matter how long the
 * DVB driver's worst case takes. Every queued frequency eventually gets
 * a real attempt, in order — see tuner.h's pending_queue comment for why
 * this is a real FIFO rather than a single overwritable slot. */
#define CHANNEL_SET_WAIT_MS 1800

struct channel_scan_ctx {
    struct hdhr_tuner *t;
    const struct hdhr_config *cfg;
    int rf_channel;
    uint32_t freq;

    pthread_mutex_t mutex;
    pthread_cond_t cond;
    bool lock_known;      /* set once dvb_scan_tune_and_lock returns */
    bool locked;          /* valid once lock_known */
    bool caller_gave_up;  /* set by the caller if it stopped waiting */
};

/* Finalizes tuner state after the lock result is known. Only touches
 * status if the tuner is still on the frequency we scanned — if the
 * client already moved on (another /channel SET, or a "none") while we
 * were working, our result is stale and shouldn't clobber the current
 * one. */
static void finalize_lock_result(struct hdhr_tuner *t, uint32_t freq, bool locked)
{
    tuner_lock(t);
    if (t->tuned_frequency_hz == freq) {
        snprintf(t->status, sizeof(t->status), "ch=%s lock=%s", t->channel, locked ? "8vsb" : "none");
    }
    tuner_unlock(t);
}

/* Passed as the progress_ctx to dvb_scan_tune_and_lock()'s progress
 * callback — carries the frequency the *current* loop iteration is
 * attempting, since channel_scan_thread_main reuses one thread across
 * several frequencies (see the pending-queue drain below) and a stale
 * freq here would mislabel published stats. */
struct scan_progress_ctx {
    struct hdhr_tuner *t;
    uint32_t freq;
    /* Fresh per attempt (declared inside channel_scan_thread_main's
     * loop, so a new frequency never inherits a previous one's window)
     * — see dvb_frontend_legacy_seq_pct()'s own comment for why this is
     * needed at all: this driver doesn't populate the modern DVBv5
     * error-block stats, so symbol_quality_pct is otherwise always -1
     * during a scan. */
    struct dvb_legacy_seq_state legacy_seq;
};

/* Called from dvb_frontend_wait_lock()'s poll loop, on the scan
 * thread's own stack — never a different thread touching fd
 * concurrently (see dvb_frontend_progress_cb's comment for why that
 * matters). Publishes a live signal-stat snapshot so /tunerN/status can
 * report real ss=/snq=/seq= while a scan is still working, matching
 * real HDHomeRun firmware (see tuner.h's scan_stats comment for why
 * this matters beyond cosmetics: libhdhomerun's own client gives up
 * polling immediately if ss looks like "no signal"). */
static void publish_scan_stats(void *ctx, int fd)
{
    struct scan_progress_ctx *pc = ctx;
    struct dvb_signal_stats stats;
    dvb_frontend_read_stats(fd, &stats);

    if (stats.symbol_quality_pct < 0) {
        /* Modern DVBv5 block-count stats unavailable on this driver
         * (confirmed lgdt3306a) — fall back to the legacy
         * FE_READ_UNCORRECTED_BLOCKS-based estimate, same as the
         * streaming path already does (see handle_tuner_get's /status
         * handler). This matters beyond cosmetics: libhdhomerun's own
         * channelscan_find_lock() waits up to 5s for
         * symbol_error_quality to reach 100 after achieving basic lock
         * before it'll even start checking for programs — without a
         * real seq= here, that 5s wait was unconditional on *every*
         * locked channel, since seq= stayed 0 the entire time. */
        int legacy_pct = dvb_frontend_legacy_seq_pct(fd, &pc->legacy_seq);
        if (legacy_pct >= 0) stats.symbol_quality_pct = legacy_pct;
    }

    tuner_lock(pc->t);
    pc->t->scan_stats = stats;
    pc->t->scan_stats_valid = true;
    pc->t->scan_stats_freq = pc->freq;
    tuner_unlock(pc->t);
}

static void *channel_scan_thread_main(void *arg)
{
    struct channel_scan_ctx *ctx = arg;
    struct hdhr_tuner *t = ctx->t;
    uint32_t freq = ctx->freq;
    int rf_channel = ctx->rf_channel;

    for (;;) {
        struct scan_progress_ctx pc = { .t = t, .freq = freq };
        int ffd;
        bool locked = dvb_scan_tune_and_lock(t->adapter, ctx->cfg->dvb_frontend, freq, &ffd,
                                              publish_scan_stats, &pc);

        pthread_mutex_lock(&ctx->mutex);
        ctx->locked = locked;
        ctx->lock_known = true;
        bool must_finalize = ctx->caller_gave_up;
        pthread_cond_signal(&ctx->cond);
        pthread_mutex_unlock(&ctx->mutex);

        if (must_finalize) {
            /* The caller already replied without us — this thread now owns
             * reporting the lock result once it's known. */
            finalize_lock_result(t, freq, locked);
        }

        if (locked) {
            /* Give the legacy uncorrected-blocks counter a real window
             * to measure before PSIP reading closes this fd.
             * publish_scan_stats() during the wait-for-lock loop above
             * only ever gets one sample in the common (fast-locking)
             * case, since polling stops the instant FE_HAS_LOCK is
             * detected — not enough for
             * dvb_frontend_legacy_seq_pct()'s windowed calculation
             * (needs >=0.5s between two samples on the same state).
             * Confirmed live: without this, seq= stayed 0 indefinitely
             * even on a strongly-locked channel, since polling simply
             * never continued long enough to form a window. A short
             * deliberate pause here gives libhdhomerun's own
             * channelscan_find_lock() a real seq= to check instead of
             * always burning its full 5-second "settle" timeout. */
            publish_scan_stats(&pc, ffd);
            usleep(600 * 1000);
            publish_scan_stats(&pc, ffd);

            dvb_scan_read_psip(t->adapter, ctx->cfg->dvb_demux, rf_channel, freq, ffd);
        }

        /* Dequeue the next queued request (see the FIFO's own comment
         * in tuner.h) before releasing the tuner, instead of always
         * releasing here and making the next SET wait to reclaim it. */
        tuner_lock(t);
        if (t->pending_count > 0) {
            freq = t->pending_queue[t->pending_head].freq;
            rf_channel = t->pending_queue[t->pending_head].rf_channel;
            t->pending_head = (t->pending_head + 1) % TUNER_PENDING_QUEUE_CAP;
            t->pending_count--;
            tuner_unlock(t);

            pthread_mutex_lock(&ctx->mutex);
            ctx->freq = freq;
            ctx->rf_channel = rf_channel;
            ctx->lock_known = false;
            ctx->locked = false;
            /* Whoever queued this one already got its own immediate
             * reply and isn't waiting on ctx->cond — this thread is the
             * only one left who can finalize its result. */
            ctx->caller_gave_up = true;
            pthread_mutex_unlock(&ctx->mutex);
            continue;
        }
        tuner_unlock(t);
        break;
    }

    tuner_release(t);
    pthread_mutex_destroy(&ctx->mutex);
    pthread_cond_destroy(&ctx->cond);
    free(ctx);
    return NULL;
}

static void handle_tuner_set(int fd, const struct hdhr_config *cfg, struct hdhr_tuner *t,
                              const char *name, const char *leaf, const char *value)
{
    if (strcmp(leaf, "channelmap") == 0) {
        send_error_reply(fd, name, "ERROR: only \"us-bcast\" is supported (8VSB/US-OTA only)");
        return;
    }

    if (strcmp(leaf, "vchannel") == 0) {
        int major = 0, minor = 0;
        if (sscanf(value, "%d.%d", &major, &minor) < 1) {
            send_error_reply(fd, name, "ERROR: invalid channel format, expected major.minor");
            return;
        }
        const struct dvb_channel *ch = dvb_find_channel(major, minor);
        if (!ch) {
            send_error_reply(fd, name, "ERROR: channel not found in scanned lineup "
                                        "(has a scan completed? see startup log)");
            return;
        }

        udp_push_stop(t); /* switching channel invalidates any active push */
        tuner_bind_channel(t, ch, DVB_PROGRAM_DEFAULT);
        tuner_lock(t);
        snprintf(t->status, sizeof(t->status), "ch=%s lock=none", t->channel);
        tuner_unlock(t);

        char val[32];
        snprintf(val, sizeof(val), "%d.%d", major, minor);
        send_value_reply(fd, name, val);
        return;
    }

    if (strcmp(leaf, "channel") == 0) {
        if (strcmp(value, "none") == 0) {
            udp_push_stop(t);
            tuner_lock(t);
            snprintf(t->channel, sizeof(t->channel), "none");
            t->tuned_frequency_hz = 0;
            t->vch_resolved = false;
            snprintf(t->status, sizeof(t->status), "state=idle");
            tuner_unlock(t);
            send_value_reply(fd, name, "none");
            return;
        }

        uint32_t freq;
        int rf_channel;
        if (!parse_channel_value(value, &freq, &rf_channel)) {
            send_error_reply(fd, name, "ERROR: expected \"none\", \"auto:<freq_hz>\", "
                                        "or \"us-bcast:<N>\"");
            return;
        }

        /* Real semantics: /tunerN/channel is a raw RF tune, independent
         * of any specific virtual channel/program selection — that
         * happens separately via vchannel or program. It performs live
         * PSIP detection at tune time (dvb_scan_tune_and_lock +
         * dvb_scan_read_psip), same as genuine firmware, which is what
         * lets hdhomerun_config's own `scan` subcommand work.
         *
         * Claiming is non-blocking (tuner_try_claim, not the old
         * wait-up-to-12s tuner_try_claim_wait): a manual scan sends its
         * next channel's SET as soon as it gets *any* reply for the
         * current one, so blocking this reply on the tuner freeing up
         * meant a single slow/dead frequency (some DVB drivers,
         * confirmed lgdt3306a, can legitimately block *inside a single
         * ioctl* for several seconds retrying a lock — an
         * uninterruptible kernel sleep, confirmed via /proc's own D
         * process state, that no userspace mechanism including signals
         * can abort) could make the *next* request's reply arrive well
         * past hdhomerun_config's own patience, aborting the whole scan
         * with a hard "communication error" even though the daemon
         * itself was fine. If the tuner's busy because of another
         * /tunerN/channel scan already in flight (not a live stream),
         * this frequency is queued instead (see channel_scan_thread_main)
         * and this replies immediately — the in-flight worker tunes it
         * next on its own once it's done with its current attempt. A
         * live stream has no worker to hand off to, so that case still
         * fails fast below.
         *
         * The tune+scan itself runs in a detached background thread
         * (channel_scan_thread_main) rather than blocking this reply on
         * the whole thing, for the same dead-frequency reason above. But
         * replying before the lock is known isn't right either —
         * hdhomerun_config checks /tunerN/status right after a SET
         * returns, so we separately wait up to CHANNEL_SET_WAIT_MS for
         * just the *lock* result (comfortably above the ~2s a real
         * lock+PSIP read takes, comfortably below the driver's
         * dead-frequency worst case) so the common case still reports an
         * accurate lock status immediately; past that budget we reply
         * with what we've got and let the background thread finish and
         * update status whenever it's actually done.
         * channel_scan_thread_main() owns releasing the claim taken here
         * once nothing's left queued. */
        if (!tuner_try_claim(t)) {
            tuner_lock(t);
            bool has_active_stream = (t->active_stream != NULL);
            if (!has_active_stream) {
                /* Busy because of a previous /tunerN/channel scan still
                 * running on this same tuner — enqueue this frequency
                 * for that worker to drain (see channel_scan_thread_main
                 * and tuner.h's pending_queue) instead of either
                 * blocking this reply or refusing outright. The queue is
                 * sized generously above the ATSC frequency table count,
                 * but if it's somehow still full, fall back to a plain
                 * busy refusal rather than silently dropping a request. */
                if (t->pending_count < TUNER_PENDING_QUEUE_CAP) {
                    int tail = (t->pending_head + t->pending_count) % TUNER_PENDING_QUEUE_CAP;
                    t->pending_queue[tail].freq = freq;
                    t->pending_queue[tail].rf_channel = rf_channel;
                    t->pending_count++;
                    t->tuned_frequency_hz = freq;
                    t->vch_resolved = false;
                    snprintf(t->channel, sizeof(t->channel), "8vsb:%u", freq);
                    snprintf(t->status, sizeof(t->status), "ch=%s lock=none", t->channel);
                    tuner_unlock(t);
                    send_value_reply(fd, name, t->channel);
                    return;
                }
                tuner_unlock(t);
                fprintf(stderr, "control: tuner%d/channel SET to %s refused — pending queue full "
                                 "(%d already queued)\n", t->index, value, TUNER_PENDING_QUEUE_CAP);
                send_error_reply(fd, name, "ERROR: tuner busy (scan queue full)");
                return;
            }
            tuner_unlock(t);
            fprintf(stderr, "control: tuner%d/channel SET to %s refused — tuner busy streaming\n",
                             t->index, value);
            send_error_reply(fd, name, "ERROR: tuner busy (currently streaming)");
            return;
        }

        tuner_lock(t);
        t->tuned_frequency_hz = freq;
        t->vch_resolved = false; /* raw tune doesn't select a specific virtual channel yet */
        /* "8vsb:<freq_hz>" — confirmed against a genuine HDHomeRun3's
         * actual "Physical Channel" display in hdhomerun_config_gui
         * (2026-07-18): it reports modulation:frequency, not
         * "us-bcast:<N>" (that channel-map-name form is only the
         * *input* syntax for the Channel selector, not what a real
         * device echoes back). rf_channel is still recorded on the
         * dvb_channel entries themselves (see dvb_scan.c) — it's just
         * not part of this reported string. */
        snprintf(t->channel, sizeof(t->channel), "8vsb:%u", freq);
        snprintf(t->status, sizeof(t->status), "ch=%s lock=none", t->channel);
        tuner_unlock(t);

        struct channel_scan_ctx *ctx = malloc(sizeof(*ctx));
        ctx->t = t;
        ctx->cfg = cfg;
        ctx->rf_channel = rf_channel;
        ctx->freq = freq;
        pthread_mutex_init(&ctx->mutex, NULL);
        pthread_cond_init(&ctx->cond, NULL);
        ctx->lock_known = false;
        ctx->locked = false;
        ctx->caller_gave_up = false;

        pthread_t th;
        if (pthread_create(&th, NULL, channel_scan_thread_main, ctx) != 0) {
            fprintf(stderr, "control: tuner%d/channel: failed to start scan thread\n", t->index);
            pthread_mutex_destroy(&ctx->mutex);
            pthread_cond_destroy(&ctx->cond);
            free(ctx);
            tuner_release(t); /* don't leave the tuner claimed forever if we can't scan */
            send_value_reply(fd, name, t->channel);
            return;
        }
        pthread_detach(th);

        struct timespec deadline;
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_sec += CHANNEL_SET_WAIT_MS / 1000;
        deadline.tv_nsec += (long)(CHANNEL_SET_WAIT_MS % 1000) * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_nsec -= 1000000000L;
            deadline.tv_sec += 1;
        }

        pthread_mutex_lock(&ctx->mutex);
        while (!ctx->lock_known) {
            if (pthread_cond_timedwait(&ctx->cond, &ctx->mutex, &deadline) == ETIMEDOUT) break;
        }
        bool have_result = ctx->lock_known;
        bool locked_result = ctx->locked;
        if (!have_result) ctx->caller_gave_up = true;
        pthread_mutex_unlock(&ctx->mutex);

        if (have_result) finalize_lock_result(t, freq, locked_result);

        send_value_reply(fd, name, t->channel);
        return;
    }

    if (strcmp(leaf, "program") == 0) {
        int prog = atoi(value);
        tuner_lock(t);
        t->program = prog;
        tuner_unlock(t);
        send_value_reply(fd, name, value);
        return;
    }

    if (strcmp(leaf, "target") == 0) {
        if (strcmp(value, "none") == 0) {
            udp_push_stop(t);
            tuner_lock(t);
            snprintf(t->target, sizeof(t->target), "none");
            tuner_unlock(t);
            send_value_reply(fd, name, "none");
            return;
        }

        char ip[64];
        int port;
        if (parse_udp_target(value, ip, sizeof(ip), &port) != 0) {
            send_error_reply(fd, name, "ERROR: expected \"none\" or \"udp://ip:port\"");
            return;
        }
        if (udp_push_start(cfg, t, ip, port) != 0) {
            send_error_reply(fd, name, "ERROR: unable to start stream (no channel set?)");
            return;
        }
        tuner_lock(t);
        snprintf(t->target, sizeof(t->target), "%s", value);
        tuner_unlock(t);
        send_value_reply(fd, name, value);
        return;
    }

    if (strcmp(leaf, "lockkey") == 0) {
        tuner_lock(t);
        if (strcmp(value, "none") == 0) {
            t->lockkey = 0;
        } else {
            t->lockkey = (uint32_t)strtoul(value, NULL, 0);
        }
        tuner_unlock(t);
        send_value_reply(fd, name, value);
        return;
    }

    send_error_reply(fd, name, "ERROR: parameter is read-only or unknown");
}

static void dispatch_getset(struct conn_ctx *cctx, const char *name, int have_value, const char *value)
{
    struct hdhr_config *cfg = cctx->ctl->cfg;
    struct hdhr_tuner *tuners = cctx->ctl->tuners;

    if (strncmp(name, "/sys/", 5) == 0) {
        if (have_value) {
            send_error_reply(cctx->fd, name, "ERROR: parameter is read-only");
        } else {
            handle_sys_get(cctx->fd, cfg, name, name + 5);
        }
        return;
    }

    const char *leaf = NULL;
    int idx = parse_tuner_path(name, cfg->tuner_count, &leaf);
    if (idx < 0) {
        send_error_reply(cctx->fd, name, "ERROR: unknown parameter");
        return;
    }

    if (have_value) {
        handle_tuner_set(cctx->fd, cfg, &tuners[idx], name, leaf, value);
    } else {
        handle_tuner_get(cctx->fd, &tuners[idx], name, leaf);
    }
}

static void *conn_thread_main(void *arg)
{
    struct conn_ctx *cctx = (struct conn_ctx *)arg;
    int fd = cctx->fd;

    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    for (;;) {
        uint8_t header[4];
        ssize_t n = read_full(fd, header, 4);
        if (n != 4) break;

        uint16_t type = ((uint16_t)header[0] << 8) | header[1];
        uint16_t paylen = ((uint16_t)header[2] << 8) | header[3];

        uint8_t rest[HDHR_MAX_PACKET_SIZE];
        if (4 + (size_t)paylen + 4 > sizeof(rest)) break; /* absurd length, bail */
        n = read_full(fd, rest, (size_t)paylen + 4);
        if (n != (ssize_t)paylen + 4) break;

        uint8_t frame[HDHR_MAX_PACKET_SIZE];
        memcpy(frame, header, 4);
        memcpy(frame + 4, rest, (size_t)paylen + 4);
        size_t framelen = 4 + (size_t)paylen + 4;

        struct hdhr_pkt pkt;
        uint16_t frame_type;
        if (hdhr_pkt_open_frame(&pkt, frame, framelen, &frame_type) != 0) {
            break; /* bad CRC — real firmware just drops the connection */
        }
        if (frame_type != HDHR_TYPE_GETSET_REQ) {
            continue; /* ignore anything we don't implement (e.g. upgrade) */
        }
        (void)type;

        char name[128] = {0};
        char value[192] = {0};
        int have_name = 0, have_value = 0;

        uint8_t tag;
        const uint8_t *tval;
        size_t tlen;
        int r;
        while ((r = hdhr_pkt_read_tlv(&pkt, &tag, &tval, &tlen)) == 1) {
            if (tag == HDHR_TAG_GETSET_NAME && tlen > 0 && tlen < sizeof(name)) {
                memcpy(name, tval, tlen);
                name[tlen] = '\0'; /* tolerate a missing trailing NUL, per spec's own advice */
                have_name = 1;
            } else if (tag == HDHR_TAG_GETSET_VALUE && tlen < sizeof(value)) {
                memcpy(value, tval, tlen);
                value[tlen] = '\0';
                have_value = 1;
            }
        }
        if (r < 0 || !have_name) continue;

        dispatch_getset(cctx, name, have_value, value);
    }

    close(fd);
    free(cctx);
    return NULL;
}

void *control_thread_main(void *arg)
{
    struct control_ctx *ctl = (struct control_ctx *)arg;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("control: socket");
        return NULL;
    }

    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(HDHR_CONTROL_TCP_PORT);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("control: bind :65001/tcp (need root, or setcap CAP_NET_BIND_SERVICE)");
        close(fd);
        return NULL;
    }
    if (listen(fd, 16) < 0) {
        perror("control: listen");
        close(fd);
        return NULL;
    }

    fprintf(stderr, "control: listening on tcp/%d\n", HDHR_CONTROL_TCP_PORT);

    for (;;) {
        struct sockaddr_in peer;
        socklen_t peerlen = sizeof(peer);
        int cfd = accept(fd, (struct sockaddr *)&peer, &peerlen);
        if (cfd < 0) continue;

        struct conn_ctx *cctx = malloc(sizeof(*cctx));
        cctx->fd = cfd;
        cctx->ctl = ctl;

        pthread_t th;
        if (pthread_create(&th, NULL, conn_thread_main, cctx) != 0) {
            close(cfd);
            free(cctx);
            continue;
        }
        pthread_detach(th);
    }

    close(fd);
    return NULL;
}
