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
    "hdhr-emulator — independent, clean-room HDHomeRun-protocol-compatible " \
    "emulator (github.com/majortom9/hdhr-emulator). Not Silicondust software; " \
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
         * systemd/hdhr-emulator.service) is what's expected to bring it
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
            /* -1 ("no reading available") collapses to 1, not 0 — the
             * wire protocol's bare-integer ss=/snq=/seq= fields have no
             * separate "N/A" slot, and dvb_frontend.c's
             * clamp_pct_floor1() already floors every genuinely
             * available reading at 1 too, so a literal 0 is never
             * emitted by this daemon at all. Some clients (e.g.
             * libhdhomerun's hdhomerun_device_wait_for_lock()) key off
             * a low reading to mean "confirmed no signal, stop
             * polling"; 0 is the value most likely to be special-cased
             * that way by such logic, so avoid it entirely rather than
             * pick which case gets to keep it. */
            snprintf(val, sizeof(val),
                     "ch=%s lock=%s ss=%d snq=%d seq=%d bps=%.0f pps=%.0f",
                     t->channel,
                     stats.has_lock ? "8vsb" : "none",
                     stats.signal_strength_pct < 0 ? 1 : stats.signal_strength_pct,
                     stats.snr_quality_pct < 0 ? 1 : stats.snr_quality_pct,
                     stats.symbol_quality_pct < 0 ? 1 : stats.symbol_quality_pct,
                     bps, pps);
        } else if (t->held_fd >= 0) {
            /* No active stream, but this channel is still selected and
             * held (see held_fd's own comment / tuner_open_hold()) --
             * read genuinely live stats straight from that fd, the
             * same way the active_stream branch above does, matching
             * real hardware's continuously-engaged-once-selected
             * behavior. Unlike the scan_stats branch below, stats.has_lock
             * is authoritative here (not just a periodic snapshot that
             * can lag behind t->status), since this is a fresh read on
             * every single poll. */
            struct dvb_signal_stats stats;
            dvb_frontend_read_stats(t->held_fd, &stats);
            if (stats.symbol_quality_pct < 0) {
                int legacy_pct = dvb_frontend_legacy_seq_pct(t->held_fd, &t->held_legacy_seq);
                if (legacy_pct >= 0) stats.symbol_quality_pct = legacy_pct;
            }
            snprintf(val, sizeof(val),
                     "ch=%s lock=%s ss=%d snq=%d seq=%d",
                     t->channel,
                     stats.has_lock ? (t->tuned_delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb") : "none",
                     stats.signal_strength_pct < 0 ? 1 : stats.signal_strength_pct,
                     stats.snr_quality_pct < 0 ? 1 : stats.snr_quality_pct,
                     stats.symbol_quality_pct < 0 ? 1 : stats.symbol_quality_pct);
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
                     stats->signal_strength_pct < 0 ? 1 : stats->signal_strength_pct,
                     stats->snr_quality_pct < 0 ? 1 : stats->snr_quality_pct,
                     stats->symbol_quality_pct < 0 ? 1 : stats->symbol_quality_pct);
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
    enum hdhr_delivery_system delivery; /* from t->channelmap at SET time — see channel_map.h */

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
static void finalize_lock_result(struct hdhr_tuner *t, uint32_t freq, bool locked,
                                  enum hdhr_delivery_system delivery)
{
    tuner_lock(t);
    if (t->tuned_frequency_hz == freq) {
        /* "qam" rather than the specific "qam64"/"qam256" constellation:
         * we tune with QAM_AUTO (see dvb_frontend_tune_qam()) and don't
         * read back which one the driver actually locked, unlike 8VSB
         * where there's only one possibility. UNTESTED against real
         * cable signal either way. */
        const char *lock_word = locked ? (delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb") : "none";
        snprintf(t->status, sizeof(t->status), "ch=%s lock=%s", t->channel, lock_word);
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
        bool locked = dvb_scan_tune_and_lock(t->adapter, ctx->cfg->dvb_frontend, freq,
                                              ctx->delivery, &ffd,
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
            finalize_lock_result(t, freq, locked, ctx->delivery);
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

            dvb_scan_read_psip(t->adapter, ctx->cfg->dvb_demux, rf_channel, freq,
                                ctx->delivery, ffd);
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
     * wasted work. */
    tuner_open_hold(t, freq, ctx->delivery);

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
        snprintf(t->status, sizeof(t->status), "state=idle");
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
            snprintf(t->status, sizeof(t->status), "state=idle");
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

        /* Close any hold from a *previous* channel selection before
         * starting this new tune attempt, so a stale old-channel fd
         * doesn't linger during it -- /tunerN/status correctly falls
         * back to scan_stats' live in-progress reporting below until
         * this attempt's own hold gets opened (see
         * channel_scan_thread_main). */
        tuner_close_hold(t);

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
                    t->filter_override[0] = '\0';
                    snprintf(t->channel, sizeof(t->channel), "%s:%u",
                             delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb", freq);
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
        snprintf(t->status, sizeof(t->status), "ch=%s lock=none", t->channel);
        tuner_unlock(t);

        struct channel_scan_ctx *ctx = malloc(sizeof(*ctx));
        ctx->t = t;
        ctx->cfg = cfg;
        ctx->rf_channel = rf_channel;
        ctx->freq = freq;
        ctx->delivery = delivery;
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

        if (have_result) finalize_lock_result(t, freq, locked_result, delivery);

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
