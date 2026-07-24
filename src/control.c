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
 *   help               GET-only, bare item (no /sys/ or /tunerN/ prefix) —
 *                      lists every path below, same style as a genuine
 *                      HDHomeRun3's own `get help` reply.
 *   /sys/model         firmware codename (e.g. "hdhomerun3_atsc") — NOT
 *                      the marketing model number, that's /sys/hwmodel
 *                      (confirmed against a genuine HDHomeRun3; an
 *                      earlier version of this had the two swapped).
 *   /sys/hwmodel, /sys/features, /sys/version       (read-only)
 *   /sys/copyright     read-only; our own wording, not real firmware's
 *                      actual value (a literal Silicondust notice) — see
 *                      its own comment in handle_sys_get for why.
 *   /sys/8vsb_override accepted/echoed but inert — this project always
 *                      requests VSB_8 regardless (US-OTA 8VSB only, per
 *                      scope), so there's nothing to override.
 *   /sys/debug, /tunerN/debug   diagnostic dumps. Real firmware's own
 *                      values are hardware-specific counters (memory
 *                      pools, PLL calibration, DMA/queue stats) this
 *                      daemon doesn't have; these report genuine
 *                      process/tuner state instead of fabricating
 *                      plausible-looking fake ones.
 *   /sys/restart       SET-only, "<resource>" argument (value accepted
 *                      but not distinguished — this daemon is one
 *                      process either way). Rejected unless
 *                      allow_remote_restart=1 in the config (default
 *                      off): the wire protocol has no real
 *                      authentication, so this is an explicit opt-in,
 *                      not something any LAN client gets by default. No
 *                      in-place re-exec — exits the process; a
 *                      supervisor (systemd Restart=always) is what's
 *                      expected to bring it back.
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
 *   /tunerN/channelmap us-bcast (default), or one of the UNTESTED cable/
 *                      Korean maps (us-cable/us-hrc/us-irc/kr-bcast/
 *                      kr-cable — see channel_map.h). Changing it clears
 *                      any tuned channel/vchannel state, since a channel
 *                      *number* means something different in every map.
 *   /tunerN/vchannel   set "<major>.<minor>" to select a channel found by
 *                      the ATSC scan (dvb_scan.c) — resolves major.minor
 *                      to a frequency + program number in our own
 *                      channel database, and immediately opens a
 *                      background "hold" tune (tuner_open_hold(), see
 *                      ARCHITECTURE.md §16) so /tunerN/status reports
 *                      genuinely live signal stats even before anything
 *                      actually streams it — matching real firmware,
 *                      which engages a selected tuner's frontend
 *                      continuously regardless of whether a client is
 *                      pulling video. The demux/PID-filtered capture
 *                      itself still only opens once a stream is actually
 *                      requested (target= or an HTTP pull), since only
 *                      one physical tuner lock can be held at once (see
 *                      tuner.h's claim/release model).
 *   /tunerN/program    MPEG program number filter — 0 for full-mux
 *                      passthrough, or a specific program_number to pick
 *                      a different subchannel sharing the selected
 *                      channel's mux (see dvb_stream.h). Recomputes
 *                      /tunerN/filter to that program's own PIDs
 *                      (confirmed against a genuine HDHomeRun3), which
 *                      means it also clears any explicit filter
 *                      override that was in place.
 *   /tunerN/filter     explicit PID whitelist, "0x<nnnn>[-0x<nnnn>] ..."
 *                      (see pid_filter.h) — a lower-level override that
 *                      takes over PID selection from program entirely
 *                      when set. GET reflects either that explicit
 *                      override, or (the common case) the PID set
 *                      program/channel would actually stream, computed
 *                      fresh each read. Cleared whenever channel,
 *                      vchannel, or program changes. Only the target=
 *                      push path (udp_stream.c) consults it — same
 *                      established scope as program itself; HTTP
 *                      passthrough always streams the plain named
 *                      channel regardless of either.
 *   /tunerN/target     "none" | "udp://ip:port" — drives udp_stream.c.
 *                      A push abandoned by an uncleanly-terminated
 *                      client (crashed, killed, network dropped — never
 *                      sent target=none) is automatically reclaimed by
 *                      keepalive.c once its client stops sending the
 *                      keepalive packets real libhdhomerun clients send
 *                      to UDP port 5004 roughly once a second while
 *                      streaming.
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
#include "pid_filter.h"
#include "channel_map.h"
#include "mpeg_section.h"
#include "psip.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

struct conn_ctx {
    int fd;
    struct control_ctx *ctl;
};

/* Set once at the top of control_thread_main() (the first thread main()
 * starts) — good enough approximation of daemon start for /sys/debug's
 * cosmetic uptime figure, not meant to be to-the-millisecond precise. */
static time_t g_start_time;

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

/* /sys/8vsb_override's stored value — cosmetic only, since dvb_frontend.c
 * always requests VSB_8 regardless (this project's scope is US-OTA
 * 8VSB only; see atsc_freq.h). Real firmware uses this to force 8VSB
 * on marginal signals where auto-modulation detection picks the wrong
 * standard, which isn't a distinction that applies to us. Accepted and
 * echoed back so clients that unconditionally read/restore it (e.g. a
 * config-backup script) don't get an error, but it has no effect on
 * tuning. Device-wide, not per-tuner, matching the real leaf's own
 * /sys/ (not /tunerN/) scope. */
static char g_8vsb_override[32] = "none";

/* Deliberately our own wording, not a copy of real firmware's actual
 * value (a literal Silicondust copyright notice) — echoing that back
 * would misattribute this project's own independently-written,
 * clean-room code (see README.md), a different kind of claim than the
 * wire-protocol mimicry the rest of this daemon does. */
#define HDHR_EMULATOR_COPYRIGHT \
    "hdhr-emu — independent, clean-room HDHomeRun-protocol-compatible " \
    "emulator (github.com/majortom9/hdhr-emu). Not Silicondust software; " \
    "not affiliated with or endorsed by SiliconDust USA Inc."

static void handle_sys_get(int fd, const struct hdhr_config *cfg, const char *name, const char *leaf)
{
    if (strcmp(leaf, "copyright") == 0) {
        send_value_reply(fd, name, HDHR_EMULATOR_COPYRIGHT);
    } else if (strcmp(leaf, "model") == 0) {
        /* Confirmed against a genuine HDHomeRun3 (2026-07-19): /sys/model
         * over the control protocol returns the firmware codename
         * ("hdhomerun3_atsc"), not the marketing model number
         * ("HDHR3-US") — that's /sys/hwmodel instead. http_server.c's
         * discovery.json already keeps these straight (ModelNumber vs
         * FirmwareName); this GETSET leaf previously returned cfg->model
         * here, which was backwards relative to real firmware. */
        send_value_reply(fd, name, cfg->firmware_name);
    } else if (strcmp(leaf, "hwmodel") == 0) {
        send_value_reply(fd, name, cfg->model);
    } else if (strcmp(leaf, "8vsb_override") == 0) {
        send_value_reply(fd, name, g_8vsb_override);
    } else if (strcmp(leaf, "debug") == 0) {
        /* Real firmware's /sys/debug dumps internal hardware counters
         * (memory pool stats, per-tuner PLL calibration, Ethernet link
         * state) that don't correspond to anything this daemon
         * actually tracks — fabricating plausible-looking fake values
         * for them would be worse than not having this leaf at all.
         * Report genuine process-level info instead. */
        char val[128];
        snprintf(val, sizeof(val), "pid=%d uptime=%lds", (int)getpid(),
                 (long)(time(NULL) - g_start_time));
        send_value_reply(fd, name, val);
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
         * stuck at 255). Byte-for-byte the same as the real device's own
         * value (matching it was cheap; unlike copying its /sys/copyright
         * text, listing capabilities the hardware genuinely has isn't a
         * false authorship claim). us-cable/us-hrc/us-irc/kr-bcast/
         * kr-cable and qam64/qam256 are UNTESTED against real signal —
         * see channel_map.h and dvb_frontend_tune_qam()'s own comments. */
        send_value_reply(fd, name,
            "channelmap: us-bcast us-cable us-hrc us-irc kr-bcast kr-cable\n"
            "modulation: 8vsb qam256 qam64\n"
            "auto-modulation: auto auto6t auto6c qam\n");
    } else {
        send_error_reply(fd, name, "ERROR: parameter is read-only or unknown");
    }
}

static void handle_sys_set(int fd, const struct hdhr_config *cfg, const char *name,
                            const char *leaf, const char *value)
{
    if (strcmp(leaf, "8vsb_override") == 0) {
        snprintf(g_8vsb_override, sizeof(g_8vsb_override), "%s", value);
        send_value_reply(fd, name, g_8vsb_override);
        return;
    }
    if (strcmp(leaf, "restart") == 0) {
        /* Off by default — this is a network peer asking the daemon to
         * exit, and the wire protocol has no real authentication
         * (lockkey is an advisory courtesy value real clients set, not
         * a security boundary). allow_remote_restart in the config
         * file is an explicit, deliberate opt-in for admins who want
         * it (e.g. to recover a wedged tuner from a script) rather than
         * something any client on the LAN gets by default. */
        if (!cfg->allow_remote_restart) {
            send_error_reply(fd, name, "ERROR: remote restart disabled "
                                        "(see allow_remote_restart in config)");
            return;
        }
        /* Real firmware's <resource> argument presumably lets it
         * restart a narrower subsystem than the whole box, but nothing
         * in the client library gives concrete values to target, and
         * this daemon is one process either way — any accepted value
         * triggers a full exit. No in-place re-exec: a process
         * supervisor (systemd's Restart=always, etc — see
         * systemd/hdhr-emu.service) is what's expected to bring it
         * back up. Reply before exiting so the client sees a clean
         * acknowledgment rather than a communication error; the reply
         * is already handed to the kernel's socket buffer by the time
         * send_value_reply() returns, so it survives the exit(). */
        fprintf(stderr, "control: /sys/restart requested (resource=\"%s\") -- exiting\n", value);
        send_value_reply(fd, name, value);
        exit(0);
    }
    send_error_reply(fd, name, "ERROR: parameter is read-only or unknown");
}

/* Computes what /tunerN/filter should report when no explicit
 * filter_override is set — mirrors what dvb_stream_open() would
 * actually select for the tuner's current channel+program state.
 * Confirmed against a genuine HDHomeRun3 (2026-07-19): PAT is excluded
 * from the displayed string even though it's always streamed
 * regardless (see dvb_stream.c's own PAT handling), and the full
 * "0x0000-0x1fff" range is shown whenever program==0 (full mux) or
 * nothing is usefully resolved yet — matches what a freshly-tuned real
 * device reported before any program had been explicitly selected. */
static void compute_auto_filter(struct hdhr_tuner *t, char *out, size_t out_len)
{
    if (t->program == 0 || t->tuned_frequency_hz == 0) {
        snprintf(out, out_len, "0x0000-0x1fff");
        return;
    }

    const struct dvb_channel *siblings[DVB_CHANNEL_MAX];
    int n = dvb_channels_on_freq(t->tuned_frequency_hz, siblings, DVB_CHANNEL_MAX);
    const struct dvb_channel *target = NULL;
    for (int i = 0; i < n; i++) {
        if (siblings[i]->program_number == (uint16_t)t->program) {
            target = siblings[i];
            break;
        }
    }
    if (!target && t->vch_resolved) target = dvb_find_channel(t->vch_major, t->vch_minor);
    if (!target) {
        snprintf(out, out_len, "0x0000-0x1fff");
        return;
    }

    uint16_t pids[4];
    int count = 0;
    if (target->pmt_pid) pids[count++] = target->pmt_pid;
    if (target->pcr_pid) pids[count++] = target->pcr_pid;
    if (target->video_pid) pids[count++] = target->video_pid;
    if (target->audio_pid) pids[count++] = target->audio_pid;

    if (count == 0) {
        snprintf(out, out_len, "none");
        return;
    }
    pid_filter_format(pids, count, out, out_len);
}

/* Temporary diagnostic (added 2026-07-20, remove once resolved):
 * investigating an intermittent "communication error" reported by real
 * hdhomerun_config `scan` runs. Leading theory: dvb_frontend_read_stats()
 * calls several DTV_STAT_* ioctls on this exact driver (lgdt3306a),
 * which is *already* documented elsewhere in this codebase (see
 * dvb_frontend.c's PROGRESS_CB_INTERVAL_MS comment) to occasionally
 * block for multiple seconds under marginal-signal conditions — that
 * finding was specific to the scan-progress path, which throttles how
 * often it calls in specifically to dodge this; a plain /tunerN/status
 * GET has no such throttle and calls it fresh on every single poll, on
 * the same connection thread that's supposed to reply fast. 800ms is
 * comfortably under libhdhomerun's own 2500ms GETSET recv timeout, so a
 * log here would mean a real client's own request very plausibly timed
 * out on this exact call. */
#define SLOW_STAT_READ_WARN_MS 800

static void log_if_slow(const char *label, int tuner_idx, const struct timespec *start)
{
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &end);
    long ms = (end.tv_sec - start->tv_sec) * 1000L + (end.tv_nsec - start->tv_nsec) / 1000000L;
    if (ms >= SLOW_STAT_READ_WARN_MS) {
        fprintf(stderr, "control: tuner%d: %s took %ldms\n", tuner_idx, label, ms);
    }
}

static void handle_tuner_get(int fd, struct hdhr_tuner *t, const char *name, const char *leaf)
{
    char val[512];

    tuner_lock(t);
    if (strcmp(leaf, "channel") == 0) {
        snprintf(val, sizeof(val), "%s", t->channel);
    } else if (strcmp(leaf, "channelmap") == 0) {
        snprintf(val, sizeof(val), "%s", t->channelmap);
    } else if (strcmp(leaf, "vchannel") == 0) {
        if (t->vch_resolved) {
            snprintf(val, sizeof(val), "%d.%d", t->vch_major, t->vch_minor);
        } else {
            snprintf(val, sizeof(val), "none");
        }
    } else if (strcmp(leaf, "program") == 0) {
        snprintf(val, sizeof(val), "%d", t->program);
    } else if (strcmp(leaf, "filter") == 0) {
        if (t->filter_override[0]) {
            snprintf(val, sizeof(val), "%s", t->filter_override);
        } else {
            compute_auto_filter(t, val, sizeof(val));
        }
    } else if (strcmp(leaf, "target") == 0) {
        snprintf(val, sizeof(val), "%s", t->target);
    } else if (strcmp(leaf, "lockkey") == 0) {
        if (t->lockkey) snprintf(val, sizeof(val), "0x%08X", t->lockkey);
        else snprintf(val, sizeof(val), "none");
    } else if (strcmp(leaf, "status") == 0) {
        int ffd = t->active_stream ? dvb_stream_frontend_fd(t->active_stream) : -1;
        if (ffd >= 0) {
            struct dvb_signal_stats stats;
            struct timespec t0;
            clock_gettime(CLOCK_MONOTONIC, &t0);
            dvb_frontend_read_stats(ffd, &stats);
            log_if_slow("dvb_frontend_read_stats (active_stream)", t->index, &t0);
            if (stats.symbol_quality_pct < 0) {
                /* Modern DVBv5 block-count stats unavailable on this
                 * driver (confirmed: lgdt3306a doesn't populate them at
                 * all) — fall back to the legacy FE_READ_UNCORRECTED_BLOCKS
                 * ioctl, windowed against the fixed ATSC segment rate. */
                int legacy_pct = dvb_stream_get_legacy_seq_pct(t->active_stream, stats.has_lock);
                if (legacy_pct >= 0) stats.symbol_quality_pct = legacy_pct;
            }
            double bps = 0.0, pps = 0.0;
            dvb_stream_get_rate(t->active_stream, &bps, &pps);
            /* -1 ("no reading available at all") collapses to 0, same as
             * a genuinely-measured 0 — this daemon used to floor every
             * available reading at 1 to keep those two cases visually
             * distinct, but real hardware genuinely reports a literal 0%
             * for snq=/seq= once there's no usable signal (confirmed
             * live, 2026-07-20, on a real HDHomeRun3 tuned to a
             * no-chance-of-locking channel) — flooring our own reading
             * was diverging from real behavior, not protecting anything;
             * see dvb_frontend.c's clamp_pct() comment for why the one
             * concrete client concern that motivated the floor
             * (hdhomerun_device_wait_for_lock()'s ss<45 check) was never
             * actually threatened by a literal 0 vs. 1 either. */
            snprintf(val, sizeof(val),
                     "ch=%s lock=%s ss=%d snq=%d seq=%d bps=%.0f pps=%.0f",
                     t->channel,
                     stats.has_lock ? "8vsb" : "none",
                     stats.signal_strength_pct < 0 ? 0 : stats.signal_strength_pct,
                     stats.snr_quality_pct < 0 ? 0 : stats.snr_quality_pct,
                     stats.symbol_quality_pct < 0 ? 0 : stats.symbol_quality_pct,
                     bps, pps);
        } else if (t->held_fd >= 0) {
            /* No active stream, but this channel is still selected and
             * held (see held_fd's own comment / tuner_open_hold()),
             * matching real hardware's continuously-engaged-once-
             * selected behavior. Reads the cache held_stats_thread_main()
             * (tuner.c) keeps refreshing on its own background thread,
             * rather than calling dvb_frontend_read_stats() directly
             * here -- confirmed live (2026-07-20) that call can itself
             * block for multiple seconds on this hardware (up to 6.8s
             * observed) under marginal conditions, which previously
             * risked exceeding a real client's own patience and
             * producing a genuine "communication error" on what should
             * be a cheap, fast status poll. */
            if (t->held_stats_cache_valid) {
                struct dvb_signal_stats stats = t->held_stats_cache;
                snprintf(val, sizeof(val),
                         "ch=%s lock=%s ss=%d snq=%d seq=%d",
                         t->channel,
                         stats.has_lock ? (t->tuned_delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb") : "none",
                         stats.signal_strength_pct < 0 ? 0 : stats.signal_strength_pct,
                         stats.snr_quality_pct < 0 ? 0 : stats.snr_quality_pct,
                         stats.symbol_quality_pct < 0 ? 0 : stats.symbol_quality_pct);
            } else {
                /* Hold was only just opened -- the background refresher
                 * hasn't published its first reading yet. Same "still
                 * checking, not confirmed no signal" placeholder a
                 * queued channel-scan request uses below, and for the
                 * same reason (see that comment): ss=0 here would read
                 * as signal_present=false to a real client and could
                 * make it give up immediately instead of waiting a
                 * moment for a real reading. */
                snprintf(val, sizeof(val), "ch=%s lock=none ss=45 snq=0 seq=0", t->channel);
            }
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
    } else if (strcmp(leaf, "debug") == 0) {
        /* Real firmware's /tunerN/debug dumps hardware-level counters
         * (DMA/queue stats, PLL calibration, transport-stream error
         * tallies) this daemon doesn't track — see /sys/debug's own
         * comment for why fabricating those would be worse than
         * omitting them. Report the state we actually have instead:
         * the same summary /tunerN/status already tracks, plus fields
         * status doesn't surface (busy/program/target). */
        snprintf(val, sizeof(val), "%s program=%d target=%s busy=%d",
                 t->status, t->program, t->target, t->busy ? 1 : 0);
    } else {
        tuner_unlock(t);
        send_error_reply(fd, name, "ERROR: parameter is write-only or unknown");
        return;
    }
    tuner_unlock(t);
    send_value_reply(fd, name, val);
}

/* channel_map.c's table covers everything except "us-bcast" (see its
 * own header comment — that map stays on the original, real-hardware-
 * validated atsc_freq.h/.c instead of being folded in). These three
 * helpers paper over that split so callers below don't need to special-
 * case "us-bcast" themselves every time. */
static uint32_t resolve_channel_in_map(const char *map_name, int channel)
{
    if (strcmp(map_name, "us-bcast") == 0) return atsc_channel_to_freq(channel);
    const struct channel_map_def *map = channel_map_find(map_name);
    return map ? channel_map_channel_to_freq(map, channel) : 0;
}

static int reverse_resolve_freq_in_map(const char *map_name, uint32_t freq)
{
    if (strcmp(map_name, "us-bcast") == 0) return atsc_freq_to_channel(freq);
    const struct channel_map_def *map = channel_map_find(map_name);
    return map ? channel_map_freq_to_channel(map, freq) : 0;
}

static enum hdhr_delivery_system delivery_for_map(const char *map_name)
{
    if (strcmp(map_name, "us-bcast") == 0) return HDHR_DELIVERY_ATSC_VSB;
    const struct channel_map_def *map = channel_map_find(map_name);
    return map ? map->delivery : HDHR_DELIVERY_ATSC_VSB; /* shouldn't happen; safe default */
}

/* `channelmap` is the tuner's *currently active* /tunerN/channelmap
 * (see tuner.h) — resolves "auto:"/"8vsb:"/"qam:" (and a bare small
 * "auto:N" channel number) against whichever map that is, so the same
 * value works regardless of which map happens to be selected. An
 * explicit "<mapname>:N" prefix (e.g. "us-cable:23") overrides that and
 * resolves against the named map instead, independent of what's
 * currently active. */
static bool parse_channel_value(const char *channelmap, const char *value,
                                 uint32_t *out_freq, int *out_rf_channel)
{
    /* "auto<N>t:<freq_hz>" -- e.g. "auto6t:605028615" -- is tvheadend's
     * own tvhdhomerun input's tune string for ATSC-T/VSB_8 (confirmed by
     * reading tvhdhomerun_frontend.c's tvhdhomerun_frontend_tune()
     * directly: DVB_TYPE_ATSC_T + DVB_MOD_VSB_8 always builds
     * "auto6t:%u", the "6" being ATSC's fixed 6MHz channel bandwidth,
     * mirroring the same "auto<bandwidth>t:" pattern it also uses for
     * DVB-T's own variable bandwidth). Confirmed live (2026-07-22): a
     * real tvheadend instance configured against this daemon as an
     * HDHomeRun ATSC source sent exactly this, and this project's
     * parser didn't recognize the digit+"t" between "auto" and the
     * colon at all, falling through to the generic "<mapname>:N" path
     * and failing instantly (resolve_channel_in_map("auto6t", ...)
     * never matches a real map name) -- tvheadend logged a "failed to
     * tune" within 18ms, far too fast to be a genuine attempt. Always
     * an explicit Hz frequency here, same as "8vsb:"/"qam:" below --
     * no channel-number ambiguity to resolve, since a real client
     * constructing this form always has an exact frequency in hand
     * already. The bandwidth digit itself is ignored -- this project
     * has no variable-bandwidth delivery system to apply it to. */
    if (strncmp(value, "auto", 4) == 0 && isdigit((unsigned char)value[4])) {
        const char *p = value + 4;
        while (isdigit((unsigned char)*p)) p++;
        if (p[0] == 't' && p[1] == ':') {
            uint32_t freq = (uint32_t)strtoul(p + 2, NULL, 10);
            if (freq == 0) return false;
            *out_freq = freq;
            *out_rf_channel = reverse_resolve_freq_in_map(channelmap, freq);
            return true;
        }
    }
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
            uint32_t freq = resolve_channel_in_map(channelmap, (int)n);
            if (freq == 0) return false; /* unknown/out-of-range channel number */
            *out_freq = freq;
            *out_rf_channel = (int)n;
            return true;
        }
        *out_freq = n;
        *out_rf_channel = reverse_resolve_freq_in_map(channelmap, n); /* 0 if not an exact table match — still fine to tune */
        return true;
    }
    if (strncmp(value, "8vsb:", 5) == 0 || strncmp(value, "qam:", 4) == 0) {
        /* Unlike "auto:", these always mean an explicit frequency
         * (modulation:frequency) — no ambiguity to resolve here. Both
         * accepted regardless of active channelmap/delivery, same as a
         * client re-submitting whatever this daemon itself echoes back
         * as t->channel. */
        const char *colon = strchr(value, ':');
        uint32_t freq = (uint32_t)strtoul(colon + 1, NULL, 10);
        if (freq == 0) return false;
        *out_freq = freq;
        *out_rf_channel = reverse_resolve_freq_in_map(channelmap, freq);
        return true;
    }

    /* Explicit "<mapname>:N" — e.g. "us-cable:23" — independent of
     * whatever channelmap is currently active. "us-bcast:N" is this
     * project's original, real-hardware-validated form; the other five
     * map names route through channel_map.c instead. */
    const char *colon = strchr(value, ':');
    if (colon) {
        size_t prefix_len = (size_t)(colon - value);
        char prefix[16];
        if (prefix_len > 0 && prefix_len < sizeof(prefix)) {
            memcpy(prefix, value, prefix_len);
            prefix[prefix_len] = '\0';
            int ch = atoi(colon + 1);
            uint32_t freq = resolve_channel_in_map(prefix, ch);
            if (freq != 0) {
                *out_freq = freq;
                *out_rf_channel = ch;
                return true;
            }
        }
    }
    return false;
}

/* Runs the RF tune + lock (dvb_scan_tune_and_lock) and, if locked, the
 * PAT/PMT/TVCT read (dvb_scan_read_psip) for a /tunerN/channel SET in a
 * background thread, so the SET reply isn't hostage to the DVB driver's
 * worst-case blocking behavior on a dead frequency (see the comment at
 * the SET handler's call site). The calling connection thread waits on
 * `t->result_cond` (see its own comment in tuner.h) for up to
 * CHANNEL_SET_WAIT_MS for the *lock* result specifically (not the full
 * PSIP read, which can legitimately take longer on a busy mux) so it can
 * reply with an accurate lock status in the common case.
 *
 * A /tunerN/channel SET that arrives while this thread is still running
 * doesn't wait for the tuner to free up at all (see handle_tuner_set's
 * channel branch) — it enqueues its frequency onto t->pending_queue (a
 * bounded FIFO, see tuner.h) and, exactly like the very first request in
 * a batch, then waits on the same result_cond for its own turn. Once
 * this thread finishes its current attempt, it dequeues the next entry
 * before releasing the tuner and, if there is one, loops around to tune
 * it next instead of exiting — so back-to-back requests (e.g.
 * hdhomerun_config's `scan`, which fires its next SET as soon as it gets
 * *any* reply for the current one) hand off to this same in-flight
 * worker rather than a fresh request ever blocking on tuner_try_claim,
 * no matter how long the DVB driver's worst case takes. Every queued
 * frequency eventually gets a real attempt, in order — see tuner.h's
 * pending_queue comment for why this is a real FIFO rather than a single
 * overwritable slot. */
/* Must exceed this hardware's typical real per-frequency processing
 * time (confirmed live, 2026-07-22: failed-lock attempts routinely take
 * ~1950-2260ms, not the ~1500ms LOCK_TIMEOUT_MS alone would suggest --
 * real ioctl/driver overhead on top), not just approximate it. At the
 * previous value (1800ms), the connection thread's own wait consistently
 * timed out *before* the worker even finished its current attempt.
 * 2300ms comfortably exceeds the typical real duration while staying
 * under a real client's own single-attempt reply patience
 * (libhdhomerun's HDHOMERUN_CONTROL_RECV_TIMEOUT is 2500ms, confirmed by
 * reading hdhomerun_control.c directly) -- occasionally missing that
 * margin on a driver's own worst-case dead-frequency block (confirmed up
 * to ~8s) is still fine, since the client retries once more on its own
 * before treating it as a real communication error.
 *
 * Raising this alone does NOT guarantee a synchronous client (e.g.
 * hdhomerun_config_gui's scan up/down) always gets its own answer in
 * time -- see the "KNOWN LIMITATION" note at the queued-request wait
 * below for why: real per-frequency cost on this hardware is close
 * enough to this budget itself that there's very little margin left
 * over once a client's own request pacing is also gated on our reply,
 * confirmed via a live packet capture and extensive testing -- not a
 * queueing bug (queue depth was confirmed 0, i.e. no backlog, when
 * this still happened). hdhomerun_config's own `scan` and direct
 * /tunerN/status polling are both unaffected. */
#define CHANNEL_SET_WAIT_MS 2300

/* How long channel_scan_thread_main waits on an empty pending_queue for
 * one more request before actually giving up and releasing the tuner --
 * see pending_cond's own comment in tuner.h for the confirmed-live
 * problem this solves (the worker exiting and a fresh invocation
 * reopening the frontend, defeating open-once-per-batch and causing a
 * real chip re-init on nearly every such restart). Measured live
 * (2026-07-21) via a wait-duration diagnostic: an earlier 3000ms bound
 * missed most gaps outright (the wait routinely ran out the full 3000ms
 * with nothing arriving at all, not just cutting it close), consistent
 * with the client's own per-channel cycle -- up to ~2.5s local
 * wait-for-lock polling plus up to several more seconds of streaminfo
 * settling on a channel that actually locked -- often exceeding 3s
 * end to end. 6000ms is still comfortably under a full scan's own
 * total duration and doesn't meaningfully delay other consumers once a
 * scan has genuinely finished (nothing more queued for a full 6s is a
 * good sign the client is actually done, not just between channels). */
#define PENDING_WAIT_MS 6000

struct channel_scan_ctx {
    struct hdhr_tuner *t;
    const struct hdhr_config *cfg;
    int rf_channel;
    uint32_t freq;
    enum hdhr_delivery_system delivery; /* from t->channelmap at SET time — see channel_map.h */
    int stale_held_fd; /* from tuner_try_claim_fast() -- see channel_scan_thread_main's
                          * own first action, and tuner_try_claim_fast()'s comment for
                          * why resolving it can't happen on the connection thread. */
    uint32_t generation; /* captured from t->scan_generation at spawn time -- see its
                             own comment in tuner.h for why this, not tuned_frequency_hz,
                             is the right staleness check for this worker's own results. */
};

/* Finalizes tuner state after the lock result for `freq` is known.
 * Publishes it as long as the tuner's current selection *session* is
 * still the one this result came from (see scan_generation's own
 * comment in tuner.h) -- NOT gated on whether `freq` is still the
 * latest-requested frequency, since a worker draining its own
 * pending_queue backlog will routinely finish an EARLIER-queued
 * frequency after a client has already queued LATER ones in the same
 * session; that's still a live, genuine, worth-publishing result, not
 * a stale one. Formats "ch=" from `freq`/`delivery` directly rather
 * than echoing t->channel, since t->channel reflects the latest
 * REQUESTED target, which by the time this runs may well be a later
 * frequency than the one this specific result is actually about. */
static void finalize_lock_result(struct hdhr_tuner *t, uint32_t freq, bool locked,
                                  enum hdhr_delivery_system delivery, uint32_t generation)
{
    tuner_lock(t);
    if (t->scan_generation == generation) {
        /* "qam" rather than the specific "qam64"/"qam256" constellation:
         * we tune with QAM_AUTO (see dvb_frontend_tune_qam()) and don't
         * read back which one the driver actually locked, unlike 8VSB
         * where there's only one possibility. UNTESTED against real
         * cable signal either way. */
        const char *lock_word = locked ? (delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb") : "none";
        char ch_str[32];
        snprintf(ch_str, sizeof(ch_str), "%s:%u",
                 delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb", freq);
        snprintf(t->status, sizeof(t->status), "ch=%s lock=%s", ch_str, lock_word);
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
        int legacy_pct = dvb_frontend_legacy_seq_pct(fd, &pc->legacy_seq, stats.has_lock);
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

    /* Deferred from tuner_try_claim_fast() -- this can block briefly
     * (joining a stale hold's background stats refresher, see
     * tuner_resolve_stale_held_fd()'s comment), which is exactly why
     * it's done here, on this background thread, rather than on the
     * connection thread that already replied to the SET that got us
     * here. */
    tuner_resolve_stale_held_fd(t, ctx->stale_held_fd);

    /* One frontend open for this whole queued-frequency drain, not one
     * per frequency -- see dvb_scan_tune_and_lock()'s own comment for
     * why (confirmed live via dmesg: the old per-frequency open/close
     * triggered a full chip re-init on every single retune, something
     * the user's own atscdx tool never does across an entire scan).
     * Safe to hold across every frequency in this loop specifically
     * because this thread already holds the tuner *claim* across all
     * of them too (busy stays true until tuner_release() below,
     * regardless of how many queued frequencies get drained) -- unlike
     * main.c's startup scan, which deliberately releases the claim
     * between frequencies and so can't reuse an fd this way either. */
    int ffd = dvb_frontend_open(t->adapter, ctx->cfg->dvb_frontend);
    bool have_ffd = (ffd >= 0);
    if (!have_ffd) {
        fprintf(stderr, "control: tuner%d/channel: failed to open frontend for scan\n", t->index);
    }

    /* Same one-open-for-the-whole-batch idea as the frontend fd above,
     * applied to PAT/VCT's demux filters too -- see
     * dvb_scan_read_psip()'s own comment for why this is safe (neither
     * filter's parameters change across this batch) and where the
     * atscdx comparison that prompted it came from. PMT still opens a
     * fresh filter per program per frequency (see
     * read_pmts_concurrent()), since which PIDs it needs varies per
     * mux. */
    int pat_fd = have_ffd ? mpeg_section_filter_open(t->adapter, ctx->cfg->dvb_demux,
                                                       0x0000, 0x00, 2000) : -1;
    int vct_fd = have_ffd ? mpeg_section_filter_open(t->adapter, ctx->cfg->dvb_demux,
                                                       PSIP_BASE_PID,
                                                       ctx->delivery == HDHR_DELIVERY_QAM ? 0xC9 : 0xC8,
                                                       2000) : -1;
    bool have_psip_fds = (pat_fd >= 0 && vct_fd >= 0);
    if (have_ffd && !have_psip_fds) {
        fprintf(stderr, "control: tuner%d/channel: failed to open PAT/VCT demux filters for scan\n",
                t->index);
    }

    for (;;) {
        struct timespec attempt_t0;
        clock_gettime(CLOCK_MONOTONIC, &attempt_t0);

        struct scan_progress_ctx pc = { .t = t, .freq = freq };
        bool locked = have_ffd && dvb_scan_tune_and_lock(ffd, freq, ctx->delivery,
                                                           publish_scan_stats, &pc);
        {
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            long ms = (t1.tv_sec - attempt_t0.tv_sec) * 1000L + (t1.tv_nsec - attempt_t0.tv_nsec) / 1000000L;
            fprintf(stderr, "control: tuner%d/channel: tune_and_lock(%u Hz) locked=%d took %ldms\n",
                    t->index, freq, locked, ms);
        }

        /* Publish this attempt's result immediately -- before PSIP
         * reading, which can take a while longer -- so any connection
         * thread waiting on result_cond for exactly this (generation,
         * freq) pair (see its own comment in tuner.h) gets its answer
         * as soon as it's genuinely known, whether this is the very
         * first frequency in this session or one dequeued from
         * pending_queue. finalize_lock_result() is safe to call
         * unconditionally now -- it's generation-gated, not tied to
         * whether any particular caller is still around waiting. */
        tuner_lock(t);
        t->last_attempt_freq = freq;
        t->last_attempt_locked = locked;
        t->last_attempt_generation = ctx->generation;
        pthread_cond_broadcast(&t->result_cond);
        tuner_unlock(t);
        finalize_lock_result(t, freq, locked, ctx->delivery, ctx->generation);

        if (locked) {
            /* Give the legacy uncorrected-blocks counter a real window
             * to measure before PSIP reading. publish_scan_stats()
             * during the wait-for-lock loop above only ever gets one
             * sample in the common (fast-locking) case, since polling
             * stops the instant FE_HAS_LOCK is detected — not enough
             * for dvb_frontend_legacy_seq_pct()'s windowed calculation
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

            struct timespec psip_t0;
            clock_gettime(CLOCK_MONOTONIC, &psip_t0);
            int n_found = have_psip_fds
                ? dvb_scan_read_psip(t->adapter, ctx->cfg->dvb_demux, pat_fd, vct_fd,
                                     rf_channel, freq, ctx->delivery)
                : 0;
            struct timespec psip_t1;
            clock_gettime(CLOCK_MONOTONIC, &psip_t1);
            long psip_ms = (psip_t1.tv_sec - psip_t0.tv_sec) * 1000L + (psip_t1.tv_nsec - psip_t0.tv_nsec) / 1000000L;
            fprintf(stderr, "control: tuner%d/channel: read_psip(%u Hz) found=%d took %ldms\n",
                    t->index, freq, n_found, psip_ms);
        }

        /* Dequeue the next queued request (see the FIFO's own comment
         * in tuner.h) before releasing the tuner, instead of always
         * releasing here and making the next SET wait to reclaim it.
         *
         * If the queue is empty, wait up to PENDING_WAIT_MS for one more
         * request before actually giving up -- see pending_cond's own
         * comment in tuner.h. Without this, a real client's own pacing
         * between SETs (it locally polls status/streaminfo for a while
         * before sending the next one) routinely outlasted how long we
         * take to process the current frequency, so the queue was often
         * still genuinely empty right here -- exiting immediately meant
         * this thread released the tuner and closed its held-open
         * frontend fd, only for the very next SET to spawn a fresh
         * invocation of this function with a fresh dvb_frontend_open()
         * (confirmed live via an invocation counter: 9 separate restarts
         * across one 35-frequency scan), each one its own chance at a
         * chip re-init -- silently defeating the whole point of holding
         * one fd open across a batch. */
        tuner_lock(t);
        if (t->pending_count == 0) {
            struct timespec deadline;
            clock_gettime(CLOCK_REALTIME, &deadline);
            deadline.tv_sec += PENDING_WAIT_MS / 1000;
            deadline.tv_nsec += (PENDING_WAIT_MS % 1000) * 1000000L;
            if (deadline.tv_nsec >= 1000000000L) {
                deadline.tv_sec += 1;
                deadline.tv_nsec -= 1000000000L;
            }
            while (t->pending_count == 0 && t->claim_waiters == 0) {
                if (pthread_cond_timedwait(&t->pending_cond, &t->lock, &deadline) == ETIMEDOUT) {
                    break; /* recheck pending_count below rather than trusting the return
                            * code alone -- same reasoning as tuner_try_claim_wait's own
                            * timedwait loop */
                }
            }
            /* claim_waiters > 0 means someone (e.g. udp_stream.c's
             * target= push) is actually blocked right now wanting this
             * tuner -- see claim_waiters' own comment in tuner.h. Don't
             * make them sit through the rest of this speculative wait
             * for a next scan frequency that may never come; fall
             * through to the pending_count check below (still 0) and
             * release immediately instead. */
        }
        if (t->pending_count > 0) {
            freq = t->pending_queue[t->pending_head].freq;
            rf_channel = t->pending_queue[t->pending_head].rf_channel;
            t->pending_head = (t->pending_head + 1) % TUNER_PENDING_QUEUE_CAP;
            t->pending_count--;
            tuner_unlock(t);
            continue;
        }
        tuner_unlock(t);
        break;
    }

    if (pat_fd >= 0) mpeg_section_filter_close(pat_fd);
    if (vct_fd >= 0) mpeg_section_filter_close(vct_fd);

    /* Once per drained batch (not per frequency) -- a CLI-driven
     * `hdhomerun_config scan` can walk dozens of queued frequencies in
     * a couple seconds, and this is the same channel db main.c's
     * startup scan also saves to, so a manual rescan's new/updated
     * channels persist across the next restart too. */
    dvb_channel_db_save(ctx->cfg->channel_cache_file);

    /* Engage the physical tuner continuously for whichever frequency
     * this thread last processed, regardless of whether it actually
     * locked -- matching real hardware, which keeps a selected tuner's
     * frontend engaged (and reporting whatever live AGC/lock state it
     * has, even "no signal") rather than only while a client happens
     * to be actively streaming it. Deliberately done once here, after
     * the whole pending-queue is drained, rather than per-frequency
     * inside the loop above -- a fast multi-channel scan
     * (hdhomerun_config's `scan`) can process dozens of queued
     * frequencies in a couple seconds, and opening+closing a hold for
     * each one that's about to be immediately superseded would just be
     * wasted work.
     *
     * Only if this is still the tuner's current selection *session* --
     * see scan_generation's own comment in tuner.h. Otherwise some
     * other request (a channel=none, or a channelmap change) ended it
     * while this thread was in the pending_cond wait above, and
     * blindly re-holding our own now-superseded `freq` here would stomp
     * that newer, already-applied state right back to what we were
     * last working on. Confirmed live (2026-07-21): with the wait above
     * now able to run up to PENDING_WAIT_MS (6s), a user manually
     * stepping channels (hdhomerun_config_gui's scan up/down, which is
     * just a raw /tunerN/channel SET per press) followed shortly after
     * by a channel=none left /tunerN/status reporting a real, live lock
     * on the *previous* channel with "ch=none" -- this thread's own
     * leftover hold from before the wait was even added, since the
     * race window (near-instant exit, pre-fix) simply never got wide
     * enough to hit in practice until now. (This used to compare
     * against t->tuned_frequency_hz instead -- wrong for the same
     * reason finalize_lock_result() switched away from it: that field
     * also changes on every new request WITHIN this same session, not
     * just on a genuine session boundary.)
     *
     * Hands off ffd directly (tuner_open_hold_from_fd(), not
     * tuner_open_hold()) rather than closing it and having that
     * function open+retune a fresh one from scratch -- ffd is already
     * open and tuned to exactly `freq` (and, in the common case,
     * already locked, since dvb_scan_tune_and_lock() just ran on it).
     * Confirmed live (2026-07-21): the old close-then-reopen sequence
     * meant the driver had to briefly reacquire lock all over again,
     * which /tunerN/status genuinely (not a placeholder) reported as a
     * real multi-second "lock=none ss=0" dip -- harmless when it
     * happened within the first second after a SET (clients already
     * tolerate that while confirming initial lock), but a confusing
     * false "signal lost" once the pending_cond wait above started
     * delaying it to several seconds *after* status had already shown
     * a solid, confirmed lock. See tuner_open_hold_from_fd()'s own
     * comment in tuner.h. */
    tuner_lock(t);
    bool still_current = (t->scan_generation == ctx->generation);
    tuner_unlock(t);
    if (still_current && have_ffd) {
        tuner_open_hold_from_fd(t, ffd);
    } else if (have_ffd) {
        dvb_frontend_close(ffd);
    }

    /* Clear before releasing the claim -- see scan_worker_active's own
     * comment in tuner.h. Nothing can queue behind this thread once
     * this is false and the claim is about to be free anyway. */
    tuner_lock(t);
    t->scan_worker_active = false;
    tuner_unlock(t);
    tuner_release(t);
    free(ctx);
    return NULL;
}

static void handle_tuner_set(int fd, const struct hdhr_config *cfg, struct hdhr_tuner *t,
                              const char *name, const char *leaf, const char *value)
{
    if (strcmp(leaf, "channelmap") == 0) {
        if (strcmp(value, "us-bcast") != 0 && !channel_map_find(value)) {
            send_error_reply(fd, name, "ERROR: unknown channel map");
            return;
        }
        /* Channel *numbers* mean something different in every map (e.g.
         * "7" is 177MHz in us-bcast but a different frequency in
         * us-irc), so any previously-tuned channel/vchannel state from
         * the old map is now meaningless — clear it the same way an
         * explicit channel=none does, rather than leaving a stale tune
         * that'll misbehave against the new map's frequency plan. */
        udp_push_stop(t);
        tuner_close_hold(t);
        tuner_lock(t);
        snprintf(t->channelmap, sizeof(t->channelmap), "%s", value);
        snprintf(t->channel, sizeof(t->channel), "none");
        t->tuned_frequency_hz = 0;
        t->vch_resolved = false;
        t->filter_override[0] = '\0';
        /* Explicit end of the current selection session -- see
         * scan_generation's own comment in tuner.h and channel=none's
         * own identical bump just above (this is the same reset, just
         * via a channelmap change instead). */
        t->scan_generation++;
        /* "ch=none lock=none ss=0 snq=0 seq=0 bps=0 pps=0" — matches a
         * genuine HDHomeRun3's own idle status verbatim (see
         * tuner_pool_init()'s comment for why the "ch=" token
         * specifically matters, not just cosmetics). */
        snprintf(t->status, sizeof(t->status), "ch=none lock=none ss=0 snq=0 seq=0 bps=0 pps=0");
        tuner_unlock(t);
        send_value_reply(fd, name, value);
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
        t->filter_override[0] = '\0'; /* new mux — any prior filter's PIDs no longer apply */
        tuner_unlock(t);
        /* Immediately engage the physical tuner and keep it engaged for
         * as long as this channel stays selected, matching real
         * hardware -- without this, /tunerN/status would only ever
         * show genuinely live signal stats while something was
         * actively streaming (see tuner_open_hold()'s own comment); a
         * client that just selects a channel and polls status (e.g. a
         * signal-monitoring tool) would otherwise see a frozen
         * one-time snapshot instead. Non-blocking -- doesn't wait for
         * lock, so this doesn't delay the reply below. */
        tuner_open_hold(t, ch->frequency_hz, ch->delivery);

        char val[32];
        snprintf(val, sizeof(val), "%d.%d", major, minor);
        send_value_reply(fd, name, val);
        return;
    }

    if (strcmp(leaf, "channel") == 0) {
        if (strcmp(value, "none") == 0) {
            udp_push_stop(t);
            tuner_close_hold(t);
            tuner_lock(t);
            snprintf(t->channel, sizeof(t->channel), "none");
            t->tuned_frequency_hz = 0;
            t->vch_resolved = false;
            t->filter_override[0] = '\0';
            /* Explicit end of the current selection session -- see
             * scan_generation's own comment in tuner.h. Any in-flight
             * scan worker's *future* results (it may still be mid-
             * backlog, see PENDING_WAIT_MS) are now for a superseded
             * session and won't publish over this. */
            t->scan_generation++;
            /* Matches a genuine HDHomeRun3's own idle status verbatim —
             * see tuner_pool_init()'s comment for why the "ch=" token
             * specifically matters, not just cosmetics. */
            snprintf(t->status, sizeof(t->status), "ch=none lock=none ss=0 snq=0 seq=0 bps=0 pps=0");
            tuner_unlock(t);
            send_value_reply(fd, name, "none");
            return;
        }

        char channelmap[16];
        tuner_lock(t);
        snprintf(channelmap, sizeof(channelmap), "%s", t->channelmap);
        tuner_unlock(t);
        enum hdhr_delivery_system delivery = delivery_for_map(channelmap);

        uint32_t freq;
        int rf_channel;
        if (!parse_channel_value(channelmap, value, &freq, &rf_channel)) {
            send_error_reply(fd, name, "ERROR: expected \"none\", \"auto:<freq_hz>\", "
                                        "or \"<channelmap>:<N>\"");
            return;
        }

        /* Any hold from a *previous* channel selection gets detached
         * from the tuner (t->held_fd cleared) as part of
         * tuner_try_claim_fast() below, so /tunerN/status immediately
         * stops showing it -- but the actual close (which can block
         * briefly, see tuner_try_claim_fast()'s comment) is deliberately
         * deferred to channel_scan_thread_main's own first action,
         * rather than done synchronously right here on the connection
         * thread. An explicit tuner_close_hold() used to run here
         * instead; confirmed live (2026-07-20) that produced a real
         * "communication error" on the very first channel of a scan
         * whenever a stale hold's background stats refresher happened
         * to be mid-slow-read at that exact moment. */

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
        /* _fast, not tuner_try_claim() -- this path always does its own
         * fresh tune+PSIP scan via dvb_scan_tune_and_lock(), never
         * dvb_stream_open(), so there's no held-fd reuse to want here
         * either way; but resolving (stopping + closing) any existing
         * hold still needs to happen off this connection thread -- see
         * tuner_try_claim_fast()'s own comment. stale_held_fd gets
         * carried into ctx below and resolved as
         * channel_scan_thread_main's first action. */
        int stale_held_fd = -1;
        if (!tuner_try_claim_fast(t, &stale_held_fd)) {
            tuner_lock(t);
            bool has_active_stream = (t->active_stream != NULL);
            /* Only queue if channel_scan_thread_main is the one actually
             * holding the claim -- it's the sole drainer of pending_queue.
             * `busy` can also be true because of main.c's one-shot startup
             * scan (dvb_scan_run-style, talks to the adapter/frontend/demux
             * directly, no notion of pending_queue at all): queueing there
             * would silently strand the request forever (see
             * scan_worker_active's own comment in tuner.h -- confirmed
             * live: /tunerN/status looked like it worked via
             * tuner_release()'s unrelated re-hold logic, but
             * /tunerN/vchannel stayed "none" forever since no real PSIP
             * scan ever ran). */
            bool queueable = (!has_active_stream && t->scan_worker_active);
            if (queueable) {
                /* Busy because of a previous /tunerN/channel scan still
                 * running on this same tuner — enqueue this frequency
                 * for that worker to drain (see channel_scan_thread_main
                 * and tuner.h's pending_queue) instead of either
                 * blocking this reply or refusing outright. The queue is
                 * sized generously above the ATSC frequency table count
                 * (40 vs. 35) — a single real scan should never fill it,
                 * so if it's somehow still full, that's not a busy scan
                 * making healthy progress, it's backlog left over from
                 * repeated/overlapping scan attempts (confirmed live:
                 * running `scan` again before a previous run's worker had
                 * drained piled a second full band's worth of requests
                 * behind the first, and it kept happening on every
                 * further attempt since the queue never got a chance to
                 * empty out). Refusing outright used to mean *every*
                 * request for the rest of that backlog's lifetime failed
                 * too — potentially minutes of a scan looking completely
                 * broken. Whoever queued all that has almost certainly
                 * already moved on, so drop it and start fresh with just
                 * this newest request instead of compounding the
                 * problem further.
                 *
                 * Deliberately NOT a small proactive threshold anymore --
                 * an earlier version (PENDING_BACKLOG_LIMIT=2) dropped
                 * the backlog the instant more than 2 requests piled up,
                 * on the theory that it'd keep live status from trailing
                 * a fast client (hdhomerun_config_gui's scan up/down) too
                 * far behind. Confirmed live (2026-07-21) via the real
                 * GUI (not a synthetic reproduction) that this actively
                 * broke correctness: it repeatedly dropped 2-3 already-
                 * queued channels at a time, including known-good ones
                 * (177MHz among them) that would have locked and
                 * correctly stopped the scan -- discarding a channel
                 * outright is strictly worse than merely reporting it
                 * late, since finalize_lock_result() now publishes any
                 * genuine result for a frequency the worker actually
                 * gets to reach, no matter how deep in the backlog (see
                 * scan_generation's own comment in tuner.h) -- so once
                 * genuine results reliably surface, there's no need to
                 * pre-emptively drop anything; only truly-unbounded
                 * backlog (the ~40-deep case below) still needs a reset. */
                if (t->pending_count >= TUNER_PENDING_QUEUE_CAP) {
                    fprintf(stderr, "control: tuner%d/channel: pending queue was full (%d queued) --"
                                     " dropping stale backlog, starting fresh with %s\n",
                                     t->index, t->pending_count, value);
                    t->pending_head = 0;
                    t->pending_count = 0;
                }
                int tail = (t->pending_head + t->pending_count) % TUNER_PENDING_QUEUE_CAP;
                t->pending_queue[tail].freq = freq;
                t->pending_queue[tail].rf_channel = rf_channel;
                t->pending_count++;
                /* Wakes channel_scan_thread_main if it's currently waiting
                 * on an empty queue (see pending_cond's own comment). */
                pthread_cond_signal(&t->pending_cond);
                t->tuned_frequency_hz = freq;
                t->vch_resolved = false;
                t->filter_override[0] = '\0';
                snprintf(t->channel, sizeof(t->channel), "%s:%u",
                         delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb", freq);
                /* ss=45 lock=none, not ss=0 -- this is a placeholder for
                 * a frequency that's merely queued, not yet actually
                 * attempted (channel_scan_thread_main hasn't gotten to
                 * it). A bare "lock=none" with no ss=/snq=/seq= fields
                 * at all (what this used to say) parses as ss=0 on the
                 * real client (hdhomerun_device_get_status_parse()
                 * defaults an absent tag to 0), which reads as
                 * signal_present=false (ss<45) and makes
                 * hdhomerun_device_wait_for_lock() give up on its very
                 * first poll -- confirmed live: a busy scan would then
                 * race through the entire rest of the band reporting
                 * fabricated "no lock" results for channels that were
                 * never actually tuned to at all, since every one of
                 * them looked instantly "confirmed no signal" the
                 * moment it was merely queued. 45 is deliberately right
                 * at signal_present's own threshold -- enough to keep
                 * wait_for_lock() polling (up to its own ~2.5s budget)
                 * hoping for a real result instead of bailing
                 * immediately, without claiming an actual lock. Once
                 * this thread's own worker genuinely starts on this
                 * frequency, publish_scan_stats() supersedes this with
                 * real readings before this placeholder's ever read
                 * again in practice. */
                snprintf(t->status, sizeof(t->status),
                         "ch=%s lock=none ss=45 snq=0 seq=0 bps=0 pps=0", t->channel);
                /* Session this request is joining -- queuing never bumps
                 * scan_generation itself (see its own comment in
                 * tuner.h), it just joins whatever session is already
                 * running. */
                uint32_t generation = t->scan_generation;
                tuner_unlock(t);

                /* Wait for THIS specific frequency's real result, same
                 * bounded budget (CHANNEL_SET_WAIT_MS) and same
                 * reasoning as the fresh-claim path just below -- see
                 * result_cond's own comment in tuner.h for why queued
                 * requests need this exactly as much as the first one
                 * in a batch does. finalize_lock_result() (called by
                 * channel_scan_thread_main itself) will have already
                 * updated t->status by the time this returns with a
                 * match, so the reply below is sent only once a real
                 * answer -- or the same timeout budget as every other
                 * request gets -- has passed.
                 *
                 * KNOWN LIMITATION (2026-07-22): this budget, however
                 * generous, is not always enough for a client whose own
                 * request pacing is itself gated on our reply (e.g.
                 * hdhomerun_config_gui's scan up/down, confirmed via a
                 * live packet capture and extensive live testing) --
                 * real per-frequency processing on this hardware
                 * routinely runs close to or above CHANNEL_SET_WAIT_MS
                 * itself (dominated by the LOCK_TIMEOUT_MS this project
                 * deliberately keeps for weak-signal sensitivity), which
                 * leaves near-zero margin once even a single queued
                 * request is involved -- confirmed queue depth was
                 * consistently 0 (no backlog build-up) when this still
                 * happened, so it isn't a queueing bug; it's a genuine,
                 * thin-margin timing mismatch between this hardware's
                 * real lock-attempt cost and a synchronous GUI client's
                 * own request cadence. The underlying generation/freq
                 * matching mechanism itself is correct (verified via
                 * direct value comparison and a controlled, faithful
                 * client simulation, both 100% reliable) -- when this
                 * budget is missed, the client just gets an unconfirmed
                 * placeholder reply and has to learn the real result
                 * from its own next /tunerN/status poll instead, same
                 * as it always could. hdhomerun_config's own `scan` and
                 * direct /tunerN/status checks are unaffected -- both
                 * fully reliable. */
                struct timespec deadline;
                clock_gettime(CLOCK_REALTIME, &deadline);
                deadline.tv_sec += CHANNEL_SET_WAIT_MS / 1000;
                deadline.tv_nsec += (long)(CHANNEL_SET_WAIT_MS % 1000) * 1000000L;
                if (deadline.tv_nsec >= 1000000000L) {
                    deadline.tv_nsec -= 1000000000L;
                    deadline.tv_sec += 1;
                }
                tuner_lock(t);
                while (!(t->last_attempt_generation == generation && t->last_attempt_freq == freq)) {
                    if (pthread_cond_timedwait(&t->result_cond, &t->lock, &deadline) == ETIMEDOUT) break;
                }
                tuner_unlock(t);

                send_value_reply(fd, name, t->channel);
                return;
            }
            tuner_unlock(t);
            if (has_active_stream) {
                fprintf(stderr, "control: tuner%d/channel SET to %s refused — tuner busy streaming\n",
                                 t->index, value);
                send_error_reply(fd, name, "ERROR: tuner busy (currently streaming)");
            } else {
                /* Busy, but not from a channel_scan_thread_main this
                 * request could queue behind (see scan_worker_active's
                 * comment) — almost certainly main.c's one-shot startup
                 * scan, which holds tuner0's claim for its entire ~1-2
                 * minute run. Fail fast and clean rather than silently
                 * stranding the request in a queue nobody drains. */
                fprintf(stderr, "control: tuner%d/channel SET to %s refused — tuner busy "
                                 "(one-shot scan in progress, not queueable)\n",
                                 t->index, value);
                send_error_reply(fd, name, "ERROR: tuner busy (scan in progress)");
            }
            return;
        }

        tuner_lock(t);
        t->tuned_frequency_hz = freq;
        t->vch_resolved = false; /* raw tune doesn't select a specific virtual channel yet */
        t->filter_override[0] = '\0'; /* new mux — any prior filter's PIDs no longer apply */
        /* "8vsb:<freq_hz>" — confirmed against a genuine HDHomeRun3's
         * actual "Physical Channel" display in hdhomerun_config_gui
         * (2026-07-18): it reports modulation:frequency, not
         * "us-bcast:<N>" (that channel-map-name form is only the
         * *input* syntax for the Channel selector, not what a real
         * device echoes back). rf_channel is still recorded on the
         * dvb_channel entries themselves (see dvb_scan.c) — it's just
         * not part of this reported string. "qam:<freq_hz>" for the
         * cable maps is this project's own best guess at the
         * equivalent — UNTESTED, no real cable-tuned device to confirm
         * the exact modulation string against (see channel_map.h). */
        snprintf(t->channel, sizeof(t->channel), "%s:%u",
                 delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb", freq);
        /* ss=45, not a bare "lock=none" with no ss= at all -- same
         * placeholder, same reasoning, as the queued-request branch
         * above: this can now be visible for a few real seconds (while
         * channel_scan_thread_main resolves a stale hold before it even
         * starts tuning, see tuner_try_claim_fast()'s comment), and a
         * bare "lock=none" parses as ss=0 on a real client, which reads
         * as "confirmed no signal" and can make it give up immediately
         * instead of waiting for a genuine result. */
        snprintf(t->status, sizeof(t->status), "ch=%s lock=none ss=45 snq=0 seq=0", t->channel);
        /* Marks this claim as one channel_scan_thread_main itself owns
         * and will drain pending_queue for -- see scan_worker_active's
         * own comment in tuner.h. Cleared by channel_scan_thread_main
         * right before it releases the tuner. */
        t->scan_worker_active = true;
        /* A genuinely NEW selection session starts here -- see
         * scan_generation's own comment in tuner.h. Bumped (not just
         * read) so any earlier session's worker, if still finishing up
         * a stale-hold teardown or similar, is unambiguously superseded
         * from this point on. */
        uint32_t generation = ++t->scan_generation;
        tuner_unlock(t);

        struct channel_scan_ctx *ctx = malloc(sizeof(*ctx));
        ctx->t = t;
        ctx->cfg = cfg;
        ctx->rf_channel = rf_channel;
        ctx->freq = freq;
        ctx->delivery = delivery;
        ctx->generation = generation;
        ctx->stale_held_fd = stale_held_fd;

        pthread_t th;
        if (pthread_create(&th, NULL, channel_scan_thread_main, ctx) != 0) {
            fprintf(stderr, "control: tuner%d/channel: failed to start scan thread\n", t->index);
            /* channel_scan_thread_main never runs to do this itself --
             * resolve it here instead so a stale hold's fd/thread don't
             * leak. Blocking here is acceptable: this is the rare
             * pthread_create()-failed path, not the normal one. */
            tuner_resolve_stale_held_fd(t, stale_held_fd);
            free(ctx);
            /* channel_scan_thread_main never runs, so it can't clear this
             * itself -- see scan_worker_active's own comment. */
            tuner_lock(t);
            t->scan_worker_active = false;
            tuner_unlock(t);
            tuner_release(t); /* don't leave the tuner claimed forever if we can't scan */
            send_value_reply(fd, name, t->channel);
            return;
        }
        pthread_detach(th);

        /* Wait for THIS specific frequency's real result via
         * result_cond (see its own comment in tuner.h) -- same
         * mechanism and same bounded budget the queued-request path
         * above uses, since (with channel_scan_thread_main now able to
         * stay alive across a whole multi-request session, see
         * PENDING_WAIT_MS) this connection's own request is no longer
         * reliably the only one ever waiting synchronously for a real
         * answer. finalize_lock_result() (called by
         * channel_scan_thread_main itself, unconditionally) will have
         * already updated t->status by the time this returns with a
         * match. */
        struct timespec deadline;
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_sec += CHANNEL_SET_WAIT_MS / 1000;
        deadline.tv_nsec += (long)(CHANNEL_SET_WAIT_MS % 1000) * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_nsec -= 1000000000L;
            deadline.tv_sec += 1;
        }
        tuner_lock(t);
        while (!(t->last_attempt_generation == generation && t->last_attempt_freq == freq)) {
            if (pthread_cond_timedwait(&t->result_cond, &t->lock, &deadline) == ETIMEDOUT) break;
        }
        tuner_unlock(t);

        send_value_reply(fd, name, t->channel);
        return;
    }

    if (strcmp(leaf, "program") == 0) {
        int prog = atoi(value);
        tuner_lock(t);
        t->program = prog;
        /* Real firmware recomputes /tunerN/filter to the new program's
         * own PIDs when program is set (confirmed live, 2026-07-19) --
         * clearing filter_override lets compute_auto_filter() do that
         * on the next GET, and lets the next stream open pick up the
         * new program's PIDs instead of a stale explicit filter tied
         * to whatever program was previously selected. */
        t->filter_override[0] = '\0';
        tuner_unlock(t);
        send_value_reply(fd, name, value);
        return;
    }

    if (strcmp(leaf, "filter") == 0) {
        if (strcmp(value, "none") == 0 || value[0] == '\0') {
            tuner_lock(t);
            t->filter_override[0] = '\0';
            tuner_unlock(t);
            send_value_reply(fd, name, "none");
            return;
        }

        struct pid_filter pf;
        if (!pid_filter_parse(value, &pf)) {
            send_error_reply(fd, name, "ERROR: expected \"none\" or \"0x<nnnn>[-0x<nnnn>] ...\"");
            return;
        }

        tuner_lock(t);
        snprintf(t->filter_override, sizeof(t->filter_override), "%s", value);
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

    /* "help" (bare, no /sys/ or /tunerN/ prefix) — confirmed against a
     * genuine HDHomeRun3's actual `get help` response (2026-07-19): a
     * "Supported configuration options:" header followed by one path
     * per line. Only meaningful as a GET; a hypothetical `set help ...`
     * falls through to the unknown-parameter path below like any other
     * unrecognized item. */
    if (!have_value && strcmp(name, "help") == 0) {
        send_value_reply(cctx->fd, name,
            "Supported configuration options:\n"
            "/sys/8vsb_override\n"
            "/sys/copyright\n"
            "/sys/debug\n"
            "/sys/features\n"
            "/sys/hwmodel\n"
            "/sys/model\n"
            "/sys/restart <resource>\n"
            "/sys/version\n"
            "/tuner<n>/channel <modulation>:<freq|ch>\n"
            "/tuner<n>/channelmap <channelmap>\n"
            "/tuner<n>/debug\n"
            "/tuner<n>/filter \"0x<nnnn>-0x<nnnn> [...]\"\n"
            "/tuner<n>/lockkey\n"
            "/tuner<n>/program <program number>\n"
            "/tuner<n>/status\n"
            "/tuner<n>/streaminfo\n"
            "/tuner<n>/target <ip>:<port>\n"
            "/tuner<n>/vchannel <major.minor>\n");
        return;
    }

    if (strncmp(name, "/sys/", 5) == 0) {
        if (have_value) {
            handle_sys_set(cctx->fd, cfg, name, name + 5, value);
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
    g_start_time = time(NULL);

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
