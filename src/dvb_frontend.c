/*
 * dvb_frontend.c — direct Linux DVB S2API frontend control. Replaces
 * TVheadend's RF-tuning role entirely: we own frequency lock and signal
 * quality reporting ourselves now.
 */
#include "dvb_frontend.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/dvb/frontend.h>

int dvb_frontend_open(int adapter, int frontend)
{
    char path[64];
    snprintf(path, sizeof(path), "/dev/dvb/adapter%d/frontend%d", adapter, frontend);
    /* O_RDWR: tuning requires write access; nonblock so wait_lock can poll
     * status itself rather than blocking inside the kernel driver. */
    int fd = open(path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "dvb_frontend: open %s failed: %s\n", path, strerror(errno));
    }
    return fd;
}

void dvb_frontend_close(int fd)
{
    if (fd >= 0) close(fd);
}

int dvb_frontend_tune_8vsb(int fd, uint32_t frequency_hz)
{
    struct dtv_property props[4];
    memset(props, 0, sizeof(props));

    props[0].cmd = DTV_CLEAR;
    props[1].cmd = DTV_DELIVERY_SYSTEM;
    props[1].u.data = SYS_ATSC;
    props[2].cmd = DTV_MODULATION;
    props[2].u.data = VSB_8;
    props[3].cmd = DTV_FREQUENCY;
    props[3].u.data = frequency_hz;

    struct dtv_properties clear_and_set = { .num = 4, .props = props };
    if (ioctl(fd, FE_SET_PROPERTY, &clear_and_set) < 0) {
        fprintf(stderr, "dvb_frontend: FE_SET_PROPERTY (freq=%u) failed: %s\n",
                frequency_hz, strerror(errno));
        return -1;
    }

    struct dtv_property tune_prop = { .cmd = DTV_TUNE };
    struct dtv_properties tune = { .num = 1, .props = &tune_prop };
    if (ioctl(fd, FE_SET_PROPERTY, &tune) < 0) {
        fprintf(stderr, "dvb_frontend: DTV_TUNE failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/* How often the (potentially expensive) progress callback actually
 * fires, vs. how often we poll FE_READ_STATUS (step_ms). Calling it on
 * every single poll tick turned out to be a real problem: the callback
 * reads full signal stats (several more ioctls beyond FE_READ_STATUS),
 * and on this driver those can *also* block for multiple seconds under
 * marginal-signal conditions, not just DTV_TUNE/FE_READ_STATUS as
 * originally assumed — confirmed live via gdb, a scan attempt got stuck
 * for minutes on one frequency because nearly every 50ms tick triggered
 * another multi-second blocking stat-read. 250ms is still comfortably
 * inside libhdhomerun's own wait_for_lock() polling cadence (also
 * 250ms), so this doesn't reduce how responsive /tunerN/status looks to
 * a real client. */
#define PROGRESS_CB_INTERVAL_MS 250

bool dvb_frontend_wait_lock(int fd, int timeout_ms, dvb_frontend_progress_cb cb, void *cb_ctx)
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    const int step_ms = 50;
    int since_cb_ms = PROGRESS_CB_INTERVAL_MS; /* fire on the first iteration */

    for (;;) {
        fe_status_t status = 0;
        if (ioctl(fd, FE_READ_STATUS, &status) == 0) {
            if (cb && since_cb_ms >= PROGRESS_CB_INTERVAL_MS) {
                cb(cb_ctx, fd);
                since_cb_ms = 0;
            }
            if (status & FE_HAS_LOCK) return true;
        }

        usleep(step_ms * 1000);
        since_cb_ms += step_ms;

        /* Real wall-clock elapsed time, not a per-iteration counter: an
         * earlier version just did `elapsed += step_ms` each pass,
         * which silently assumed every iteration takes ~step_ms. That
         * held while FE_READ_STATUS itself was the only ioctl in the
         * loop (normally fast) — once an iteration's *own* work could
         * legitimately take several real seconds (see above), that
         * assumption meant the loop could run for many minutes of wall
         * time while believing only a fraction of timeout_ms had
         * passed. This still can't bound a single already-in-flight
         * blocking ioctl (nothing can — see the driver's own
         * uninterruptible-sleep behavior noted elsewhere), but it does
         * stop repeatedly re-entering that risk far past the caller's
         * intended budget. */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000L +
                           (now.tv_nsec - start.tv_nsec) / 1000000L;
        if (elapsed_ms >= timeout_ms) break;
    }
    return false;
}

/* Converts one DTV_STAT_* readout to a 0-100 percentage. Decibel-scale
 * values (CNR) get a rough linear map over a plausible ATSC SNR range;
 * relative-scale values (signal strength) are already 0-65535-ish per
 * driver convention and get rescaled; counters (error blocks) are
 * inverted against a recent-block window. This mirrors what real
 * HDHomeRun firmware does — a driver-dependent approximation, not a
 * calibrated absolute measurement. */
static int scale_decibel_to_pct(int64_t millidb, double min_db, double max_db)
{
    double db = millidb / 1000.0;
    if (db < min_db) db = min_db;
    if (db > max_db) db = max_db;
    return (int)((db - min_db) / (max_db - min_db) * 100.0 + 0.5);
}

static int scale_relative_to_pct(uint64_t uvalue)
{
    /* FE_SCALE_RELATIVE is defined as 0-65535 across the full range. */
    if (uvalue > 65535) uvalue = 65535;
    return (int)(uvalue * 100 / 65535);
}

/* Set from config (debug_signal_stats=1) via dvb_frontend_set_debug().
 * When on, every stat read logs the driver's raw scale type and value
 * to stderr — the only reliable way to calibrate signal_strength_pct's
 * dB range, since S2API doesn't mandate what physical reference
 * (dBmV, dBm, or something else) a driver's DECIBEL scale actually
 * uses for DTV_STAT_SIGNAL_STRENGTH. */
static bool g_debug_signal_stats = false;

void dvb_frontend_set_debug(bool enabled)
{
    g_debug_signal_stats = enabled;
}

static const char *scale_name(enum fecap_scale_params scale)
{
    switch (scale) {
        case FE_SCALE_NOT_AVAILABLE: return "not-available";
        case FE_SCALE_DECIBEL:       return "decibel";
        case FE_SCALE_RELATIVE:      return "relative";
        case FE_SCALE_COUNTER:       return "counter";
        default:                     return "unknown";
    }
}

/* For FE_SCALE_COUNTER stats where we need the raw cumulative value
 * (not a single stat converted to a percentage in isolation) — used for
 * DTV_STAT_ERROR_BLOCK_COUNT / DTV_STAT_TOTAL_BLOCK_COUNT, which only
 * mean something as a ratio of each other. Returns -1 if the stat isn't
 * available. */
static int64_t read_counter_raw(int fd, uint32_t cmd, const char *label)
{
    struct dtv_property prop;
    memset(&prop, 0, sizeof(prop));
    prop.cmd = cmd;
    struct dtv_properties props = { .num = 1, .props = &prop };

    if (ioctl(fd, FE_GET_PROPERTY, &props) < 0) return -1;
    if (prop.u.st.len == 0) return -1;

    struct dtv_stats *s = &prop.u.st.stat[0];
    if (s->scale != FE_SCALE_COUNTER) return -1; /* not the type we expect for this cmd */

    if (g_debug_signal_stats) {
        fprintf(stderr, "dvb_frontend: [debug] %s: scale=counter raw=%llu\n",
                         label, (unsigned long long)s->uvalue);
    }
    return (int64_t)s->uvalue;
}

static int read_stat_property(int fd, uint32_t cmd, const char *label, double db_min, double db_max)
{
    struct dtv_property prop;
    memset(&prop, 0, sizeof(prop));
    prop.cmd = cmd;
    struct dtv_properties props = { .num = 1, .props = &prop };

    if (ioctl(fd, FE_GET_PROPERTY, &props) < 0) return -1;
    if (prop.u.st.len == 0) return -1; /* driver doesn't support this stat */

    struct dtv_stats *s = &prop.u.st.stat[0];

    if (g_debug_signal_stats) {
        if (s->scale == FE_SCALE_DECIBEL) {
            fprintf(stderr, "dvb_frontend: [debug] %s: scale=%s raw=%.3f (raw units, "
                             "reference unknown — driver-defined)\n",
                             label, scale_name(s->scale), s->svalue / 1000.0);
        } else {
            fprintf(stderr, "dvb_frontend: [debug] %s: scale=%s raw=%llu\n",
                             label, scale_name(s->scale), (unsigned long long)s->uvalue);
        }
    }

    switch (s->scale) {
        case FE_SCALE_DECIBEL:
            return scale_decibel_to_pct(s->svalue, db_min, db_max);
        case FE_SCALE_RELATIVE:
            return scale_relative_to_pct(s->uvalue);
        case FE_SCALE_COUNTER:
            return (int)(s->uvalue > INT32_MAX ? INT32_MAX : s->uvalue);
        case FE_SCALE_NOT_AVAILABLE:
        default:
            return -1;
    }
}

static bool read_raw_decibel(int fd, uint32_t cmd, double *out_db)
{
    struct dtv_property prop;
    memset(&prop, 0, sizeof(prop));
    prop.cmd = cmd;
    struct dtv_properties props = { .num = 1, .props = &prop };

    if (ioctl(fd, FE_GET_PROPERTY, &props) < 0) return false;
    if (prop.u.st.len == 0) return false;

    struct dtv_stats *s = &prop.u.st.stat[0];
    if (s->scale != FE_SCALE_DECIBEL) return false;

    *out_db = s->svalue / 1000.0;
    return true;
}

bool dvb_frontend_read_raw_signal_dbm(int fd, double *out_dbm)
{
    return read_raw_decibel(fd, DTV_STAT_SIGNAL_STRENGTH, out_dbm);
}

bool dvb_frontend_read_raw_snr_db(int fd, double *out_db)
{
    return read_raw_decibel(fd, DTV_STAT_CNR, out_db);
}

void dvb_frontend_read_stats(int fd, struct dvb_signal_stats *out)
{
    memset(out, 0, sizeof(*out));
    out->signal_strength_pct = -1;
    out->snr_quality_pct = -1;
    out->symbol_quality_pct = -1;

    fe_status_t status = 0;
    if (ioctl(fd, FE_READ_STATUS, &status) == 0) {
        out->has_lock = (status & FE_HAS_LOCK) != 0;
    }

    /* DTV_STAT_SIGNAL_STRENGTH on the FE_SCALE_DECIBEL scale is defined
     * by the kernel DVBv5 API itself as 0.001 dBm units (power, not
     * dBmV) — see Documentation/userspace-api/media/dvb/frontend-stat-properties.rst.
     * lgdt3306a (and other properly DVBv5-compliant frontends) follow
     * this, so unlike signal-strength-in-dB in general, this specific
     * case has a defined reference rather than being purely
     * driver-dependent.
     *
     * Ceiling recalibrated 2026-07-19 against a real HDHomeRun3 (same
     * antenna feed, tools/calibrate_stats.c's sweep vs. hdhomerun_config
     * get /tunerN/status on 7 real locked channels): the original
     * -48.75 dBm ceiling (derived from real firmware's documented
     * ss=100%-at-0dBmV point, converted via the 75-ohm dBmV<->dBm
     * relationship) consistently overshot the real device's ss% by
     * +9 to +17 points on strong signals, while matching it exactly
     * (63% both) on the one weaker channel sampled — i.e. the curve was
     * anchored correctly at the bottom but too easy to reach 100% at
     * the top. -43.0 dBm is the average of each sample's implied
     * ceiling (100*(dbm-floor)/pct + floor, floor held at -85), and
     * roughly halves the average error across all 7 points (10.9 -> 3.6
     * pct-point average |delta|). Only 7 data points, all from one
     * antenna/splitter setup and none near actual lock-loss, so this
     * remains an approximation, not an exact match — re-run the sweep
     * against more/weaker real channels if it drifts again. The floor
     * (-85 dBm = 0%) is unchanged: still a judgment call based on
     * typical ATSC 8VSB tuner sensitivity specs (usable lock generally
     * extends to roughly -83 to -90 dBm before the AGC maxes out), not
     * verified by this sweep (no marginal-signal samples were
     * available). Enable debug_signal_stats=1 and compare against a
     * real HDHomeRun on the same antenna feed to refine either further. */
    out->signal_strength_pct = read_stat_property(fd, DTV_STAT_SIGNAL_STRENGTH,
                                                   "signal_strength", -85.0, -43.0);
    /* Calibrated against a real HDHomeRun with two reference points:
     * snq=100% at 33dB SNR (the ceiling), and snq clustering ~35-40% at
     * 15.2dB SNR. Solving for the floor that satisfies both (using the
     * 37.5% midpoint) gives ~4.5dB — notably lower than the ATSC 8VSB
     * Viterbi lock threshold (~15dB) that this floor was originally
     * (incorrectly) guessed to be. Real firmware apparently gives a lot
     * more headroom than "0% at the edge of losing lock" — most
     * solidly-locked signals read comfortably above 50%, and the meter
     * only really drops hard as you approach actual failure. */
    out->snr_quality_pct = read_stat_property(fd, DTV_STAT_CNR, "cnr", 4.5, 33.0);

    /* DTV_STAT_ERROR_BLOCK_COUNT and DTV_STAT_TOTAL_BLOCK_COUNT are both
     * cumulative counters since the frontend was tuned (per the kernel
     * docs — "Block counts... measure the number of blocks and block
     * errors after FEC"), not deltas since the last read. The earlier
     * version of this code treated ERROR_BLOCK_COUNT as if any nonzero
     * reading meant "5% off, permanently" — which meant a single error
     * early in a long-running tuned session would sink symbol_quality_pct
     * forever and it could never recover, even with a perfect signal
     * afterward. The actual post-FEC error rate is the *ratio* of the
     * two counters: 0 BER (no errors across everything received) is
     * 100%, and the percentage properly reflects the true cumulative
     * error rate rather than a one-way ratchet. */
    int64_t error_blocks = read_counter_raw(fd, DTV_STAT_ERROR_BLOCK_COUNT, "error_block_count");
    int64_t total_blocks = read_counter_raw(fd, DTV_STAT_TOTAL_BLOCK_COUNT, "total_block_count");

    if (error_blocks < 0 || total_blocks <= 0) {
        /* stat(s) unavailable (common on several USB ATSC chipsets), or
         * no blocks counted yet (right after tuning) — report
         * unavailable rather than a misleading 0 or 100. */
        out->symbol_quality_pct = -1;
    } else {
        double error_ratio = (double)error_blocks / (double)total_blocks;
        int pct = (int)((1.0 - error_ratio) * 100.0 + 0.5);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        out->symbol_quality_pct = pct;
    }
}

/* ATSC 8VSB segment rate — see dvb_stream.c's own copy of this same
 * constant (used identically there for the streaming-time version of
 * this calculation) for the full derivation; duplicated here rather
 * than shared to avoid touching that already-verified code for this
 * unrelated scan-time addition. */
#define ATSC_SEGMENTS_PER_SEC 12935.38

int dvb_frontend_legacy_seq_pct(int fd, struct dvb_legacy_seq_state *state)
{
    uint32_t ucblocks;
    if (!dvb_frontend_read_legacy_ucblocks(fd, &ucblocks)) {
        return -1;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (!state->have_baseline) {
        state->have_baseline = true;
        state->last_ucblocks = ucblocks;
        state->last_time = now;
        state->last_pct = -1;
        return -1; /* no window yet — next call will have one */
    }

    double elapsed = (now.tv_sec - state->last_time.tv_sec) +
                      (now.tv_nsec - state->last_time.tv_nsec) / 1e9;
    if (elapsed < 0.5) {
        return state->last_pct;
    }

    uint32_t delta_blocks = (ucblocks >= state->last_ucblocks) ?
                             (ucblocks - state->last_ucblocks) : 0;
    double expected_blocks = ATSC_SEGMENTS_PER_SEC * elapsed;
    double error_ratio = expected_blocks > 0 ? (delta_blocks / expected_blocks) : 0.0;
    int pct = (int)((1.0 - error_ratio) * 100.0 + 0.5);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    state->last_ucblocks = ucblocks;
    state->last_time = now;
    state->last_pct = pct;
    return pct;
}

bool dvb_frontend_read_legacy_ucblocks(int fd, uint32_t *out_ucblocks)
{
    uint32_t val = 0;
    if (ioctl(fd, FE_READ_UNCORRECTED_BLOCKS, &val) < 0) {
        if (g_debug_signal_stats) {
            fprintf(stderr, "dvb_frontend: [debug] legacy FE_READ_UNCORRECTED_BLOCKS: "
                             "not supported (%s)\n", strerror(errno));
        }
        return false;
    }
    if (g_debug_signal_stats) {
        fprintf(stderr, "dvb_frontend: [debug] legacy FE_READ_UNCORRECTED_BLOCKS: raw=%u\n", val);
    }
    *out_ucblocks = val;
    return true;
}
