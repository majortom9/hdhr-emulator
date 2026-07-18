#ifndef HDHR_DVB_FRONTEND_H
#define HDHR_DVB_FRONTEND_H

#include <stdint.h>
#include <stdbool.h>

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

/* Polls FE_READ_STATUS until FE_HAS_LOCK or timeout_ms elapses. Returns
 * true if locked. */
bool dvb_frontend_wait_lock(int fd, int timeout_ms);

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

/* Fallback for drivers (confirmed: lgdt3306a) that don't populate the
 * modern DVBv5 DTV_STAT_ERROR_BLOCK_COUNT/DTV_STAT_TOTAL_BLOCK_COUNT
 * stats at all — reads the older pre-S2API FE_READ_UNCORRECTED_BLOCKS
 * ioctl instead (a cumulative counter, same shape as the modern stat,
 * just without a paired "total blocks" denominator — see dvb_stream.c's
 * windowed-rate caller for how the denominator is derived instead).
 * Returns false if this ioctl isn't supported either. */
bool dvb_frontend_read_legacy_ucblocks(int fd, uint32_t *out_ucblocks);

#endif /* HDHR_DVB_FRONTEND_H */
