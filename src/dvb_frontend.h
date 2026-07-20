#ifndef HDHR_DVB_FRONTEND_H
#define HDHR_DVB_FRONTEND_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

struct dvb_signal_stats {
    bool has_lock;
    /* All normalized to 0-100 where the driver provides a usable scale;
     * -1 means "not available from this driver/frontend". Real
     * HDHomeRun ss/snq/seq are also driver-dependent approximations —
     * we're matching that spirit, not a hardware-independent absolute. */
    int signal_strength_pct; /* ss  */
    int snr_quality_pct;     /* snq — derived from CNR (dB) */
    int symbol_quality_pct;  /* seq — derived from post-Viterbi error block rate */
};

/* Opens /dev/dvb/adapter<adapter>/frontend<frontend>. Returns fd, or -1
 * with errno set. */
int dvb_frontend_open(int adapter, int frontend);
void dvb_frontend_close(int fd);

/* Clears prior tuning state and tunes to the given frequency as 8VSB
 * (US terrestrial ATSC). Non-blocking — does not wait for lock. Returns
 * 0 on success, -1 on ioctl failure (errno set). */
int dvb_frontend_tune_8vsb(int fd, uint32_t frequency_hz);

/* Same shape as dvb_frontend_tune_8vsb(), but for in-band clear QAM
 * cable (US/KR — SYS_DVBC_ANNEX_B, QAM_AUTO). UNTESTED against real
 * cable signal — see the comment above its implementation in
 * dvb_frontend.c. */
int dvb_frontend_tune_qam(int fd, uint32_t frequency_hz);

/* Called once per poll iteration inside dvb_frontend_wait_lock(), from
 * the SAME thread that owns fd (never a different thread) — see that
 * function's own comment for why this matters. */
typedef void (*dvb_frontend_progress_cb)(void *ctx, int fd);

/* Polls FE_READ_STATUS until FE_HAS_LOCK or timeout_ms elapses. Returns
 * true if locked. If cb is non-NULL, it's called on every iteration
 * that gets a status reading (before the lock check), so a caller can
 * publish live signal stats (dvb_frontend_read_stats) as they become
 * available rather than only once this returns — useful for a caller
 * that wants /tunerN/status to reflect real progress during a slow
 * lock attempt, not just the final result. Note this only helps for
 * iterations that actually complete: some DVB drivers (confirmed
 * lgdt3306a) can block for several seconds *inside a single
 * FE_READ_STATUS call itself* on a marginal/dead frequency, during
 * which no callback fires at all — there's no way to get partial
 * progress out of a syscall that hasn't returned yet. */
bool dvb_frontend_wait_lock(int fd, int timeout_ms, dvb_frontend_progress_cb cb, void *cb_ctx);

/* Reads current signal stats via the S2API DTV_STAT_* properties. Safe
 * to call whether or not locked (has_lock reflects current state; other
 * fields are best-effort and may be -1 if unlocked or unsupported). */
void dvb_frontend_read_stats(int fd, struct dvb_signal_stats *out);

/* Enables raw scale/value logging on every stat read (see dvb_frontend.c
 * for why this matters — DTV_STAT_SIGNAL_STRENGTH's dB reference is
 * driver-defined, not standardized, so calibrating signal_strength_pct
 * against a real HDHomeRun's known ss/dBmV behavior requires seeing
 * what your specific driver actually returns first). */
void dvb_frontend_set_debug(bool enabled);

/* Raw (unscaled) DTV_STAT_SIGNAL_STRENGTH / DTV_STAT_CNR readouts in
 * their native physical units (dBm / dB) — the same values
 * dvb_frontend_read_stats()'s signal_strength_pct/snr_quality_pct are
 * derived from via scale_decibel_to_pct(), but before that floor/ceiling
 * mapping is applied. Returns false if the driver doesn't report the
 * stat on the FE_SCALE_DECIBEL scale (not supported, or a different
 * scale entirely) — same condition that makes the corresponding _pct
 * field -1. For calibrating those floor/ceiling constants against a
 * real device (see tools/calibrate_stats.c), not needed at daemon
 * runtime. */
bool dvb_frontend_read_raw_signal_dbm(int fd, double *out_dbm);
bool dvb_frontend_read_raw_snr_db(int fd, double *out_db);

/* Fallback for drivers (confirmed: lgdt3306a) that don't populate the
 * modern DVBv5 DTV_STAT_ERROR_BLOCK_COUNT/DTV_STAT_TOTAL_BLOCK_COUNT
 * stats at all — reads the older pre-S2API FE_READ_UNCORRECTED_BLOCKS
 * ioctl instead (a cumulative counter, same shape as the modern stat,
 * just without a paired "total blocks" denominator — see dvb_stream.c's
 * windowed-rate caller for how the denominator is derived instead).
 * Returns false if this ioctl isn't supported either. */
bool dvb_frontend_read_legacy_ucblocks(int fd, uint32_t *out_ucblocks);

/* Windowed baseline state for dvb_frontend_legacy_seq_pct() — owned by
 * the caller, persisted across repeated calls for the same tuning
 * attempt (zero-initialize before first use, e.g. via `= {0}` on a
 * fresh struct per attempt so a previous frequency's counts never leak
 * into a new one's). */
struct dvb_legacy_seq_state {
    bool             have_baseline;
    uint32_t         last_ucblocks;
    struct timespec  last_time;
    int              last_pct;
};

/* Same idea as dvb_stream.c's dvb_stream_get_legacy_seq_pct() (windowed
 * post-FEC quality estimate from the cumulative
 * FE_READ_UNCORRECTED_BLOCKS counter, for drivers — confirmed
 * lgdt3306a — that don't populate the modern DVBv5 block-count stats),
 * but taking plain state instead of a struct dvb_stream so it can also
 * be used outside an active stream (e.g. control.c's live scan-progress
 * reporting). Needs at least two calls on the same *state spaced >=0.5s
 * apart before returning a real value (matches
 * dvb_stream_get_legacy_seq_pct's own settle behavior) — returns -1
 * until then, or if the ioctl isn't supported at all. */
int dvb_frontend_legacy_seq_pct(int fd, struct dvb_legacy_seq_state *state);

#endif /* HDHR_DVB_FRONTEND_H */
