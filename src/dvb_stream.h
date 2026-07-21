#ifndef HDHR_DVB_STREAM_H
#define HDHR_DVB_STREAM_H

#include <stdint.h>
#include <sys/types.h>
#include "dvb_channel.h"

/* program_override semantics, matching the real /tunerN/program GETSET:
 *   DVB_PROGRAM_DEFAULT  — stream the channel's own program (normal case)
 *   0                    — full mux passthrough, no PID filtering at all
 *   >0                   — a specific program_number; resolved against
 *                          other channels sharing the same mux frequency
 *                          (see dvb_channel.h's dvb_channels_on_freq) */
#define DVB_PROGRAM_DEFAULT (-1)

struct dvb_stream;

/* Tunes the given adapter's frontend to `channel`'s mux, waits for lock,
 * sets up PID filters (or full-mux passthrough), and starts a background
 * thread capturing the resulting TS into an internal ring buffer.
 * Returns NULL on failure (no lock, ioctl error, etc — check stderr for
 * which). `frontend`/`demux_num` are almost always 0 on consumer USB
 * tuners; see config.h.
 *
 * pid_filter: NULL or "" to use program_override's normal resolution
 * (the common case); otherwise an explicit /tunerN/filter wire-format
 * PID list (see pid_filter.h) that takes over PID selection entirely,
 * ignoring program_override — matching real firmware, where an
 * explicit filter is a lower-level override that supersedes
 * program-based selection. PAT (0x0000) is still always added on top
 * so a player can actually identify the stream, same as the
 * program_override path already does. A filter whose total PID count
 * is too wide to enumerate as individual demux PES filters (e.g. the
 * real device's own "0x0000-0x1fff" default, or anything else spanning
 * more than MAX_DEMUX_FDS-1 PIDs) falls back to full-mux passthrough
 * instead of failing outright.
 *
 * adopt_fd: -1 to open+tune+wait-for-lock a fresh frontend fd as usual;
 * >=0 to adopt an already-open frontend fd that's already tuned and
 * locked to `channel`'s mux (ownership transfers in either case — on
 * success dvb_stream owns it, on failure this function closes it).
 * This is tuner_try_claim()'s held-fd handoff (see tuner.h): closing an
 * already-locked hold just to reopen+relock the same frequency from
 * scratch a moment later isn't just wasteful, it can genuinely fail —
 * confirmed live, this driver doesn't always relock within
 * LOCK_TIMEOUT_MS on an immediate close/reopen of the same frontend,
 * even though the hold had it solidly locked seconds earlier. */
struct dvb_stream *dvb_stream_open(int adapter, int frontend, int demux_num,
                                    const struct dvb_channel *channel,
                                    int program_override, const char *pid_filter,
                                    int adopt_fd);

/* Blocking read, same contract tvh_stream_read() had: 0 = EOF/closing,
 * -1 = error, otherwise bytes read. */
ssize_t dvb_stream_read(struct dvb_stream *s, void *buf, size_t len);

void dvb_stream_close(struct dvb_stream *s);

/* Read-only access to the underlying frontend fd, so callers (control.c's
 * /tunerN/status) can pull live S2API signal stats while a stream is
 * open. Do not close or tune this fd directly — dvb_stream owns it. */
int dvb_stream_frontend_fd(const struct dvb_stream *s);

/* Live throughput since dvb_stream_open(), for the status/streaminfo
 * "bps="/"pps=" fields real hdhomerun_config(_gui) actually parses (see
 * hdhomerun_device_get_tuner_status() in libhdhomerun — it reads these
 * out of /tunerN/status itself, not just streaminfo, and that's what
 * feeds the GUI's "Network Rate" field). */
void dvb_stream_get_rate(const struct dvb_stream *s, double *bits_per_second,
                          double *packets_per_second);

/* Fallback "symbol quality" percentage for frontends that don't populate
 * the modern DVBv5 DTV_STAT_ERROR_BLOCK_COUNT/DTV_STAT_TOTAL_BLOCK_COUNT
 * stats (confirmed: lgdt3306a) — derived from the legacy
 * FE_READ_UNCORRECTED_BLOCKS ioctl's cumulative counter, windowed the
 * same way dvb_stream_get_rate() windows throughput, against the fixed
 * ATSC 8VSB segment rate (~12935.4 segments/sec, one RS-coded block per
 * segment — a physical constant of the standard, not a driver-dependent
 * guess) as the "total blocks" denominator the legacy ioctl doesn't
 * provide directly. Returns -1 if the legacy ioctl isn't supported
 * either, or not enough time has passed since stream open to measure a
 * window yet. Caller (control.c) should only use this when the modern
 * stat is unavailable — see dvb_frontend_read_stats(). */
int dvb_stream_get_legacy_seq_pct(struct dvb_stream *s);

#endif /* HDHR_DVB_STREAM_H */
