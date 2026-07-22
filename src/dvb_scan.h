#ifndef HDHR_DVB_SCAN_H
#define HDHR_DVB_SCAN_H

#include <stdint.h>
#include <stdbool.h>

#include "dvb_frontend.h"
#include "channel_map.h"

/* Tunes through every table entry in atsc_freq_table on the given
 * adapter/frontend/demux, and for each frequency that locks, parses
 * PAT+PMT+TVCT to populate the global dvb_channel database (see
 * dvb_channel.h). Clears the existing database first — this is a full
 * rescan, not an incremental update.
 *
 * Blocks for a while (up to ~50 frequencies x ~2s each worst case, ~100s)
 * — call from a background thread, not the startup path directly if you
 * want the daemon responsive to discovery/control during the scan.
 *
 * Returns the total number of virtual channels found across all locked
 * muxes. */
int dvb_scan_run(int adapter, int frontend, int demux_num);

/* Tunes an already-open frontend (ffd) to freq_hz and waits for lock.
 * Returns whether lock was achieved. Never opens or closes ffd itself
 * -- the caller owns that fd's whole lifecycle now, across as many
 * calls to this function (different frequencies) as it likes, and is
 * expected to follow a successful call with dvb_scan_read_psip() before
 * moving to the next frequency.
 *
 * This used to open a fresh fd per call and close it on failure (the
 * caller got it back via an out-param on success). Changed after
 * comparing against the user's own atscdx tool (libdvbv5-based), which
 * opens its frontend once for an entire multi-channel scan and only
 * ever retunes it -- confirmed live via dmesg that this daemon's old
 * per-frequency open/close was triggering a full lgdt3306a_init/
 * si2157_init chip re-init on *every* frequency (35+ times per scan),
 * where atscdx's own scan showed zero init cycles after its initial
 * open, only lgdt3306a_search. A full chip re-init is a heavier,
 * likely more failure-prone operation than a plain retune of an
 * already-open frontend, and paying it 35x more than necessary was a
 * plausible contributor to this hardware's occasional multi-second
 * stalls. See control.c's channel_scan_thread_main() and main.c's
 * scan_thread_main() for the two different real callers -- the former
 * now opens once for its whole queued-frequency drain (safe: it
 * already held the tuner claim across all of them anyway); the latter
 * deliberately still opens/closes per frequency, because it also
 * releases the tuner *claim* between frequencies (a separate, already-
 * validated design choice, see its own comment) -- and an open fd here
 * would defeat that entirely, since another consumer's own open() on
 * the same physical frontend can't succeed while this one is still
 * open, claim or no claim.
 *
 * NOTE ON WORST-CASE LATENCY: some DVB demod drivers (confirmed on
 * lgdt3306a) retry the lock internally several times *inside a single
 * blocking ioctl* before reporting failure on a dead frequency — our own
 * internal lock-wait timeout can't interrupt a syscall that's already
 * blocked inside the kernel/driver, so a call that ultimately fails to
 * lock can take substantially longer (~7s observed) than that timeout
 * would suggest. A successful lock, by contrast, is fast (well under
 * 2s observed) since the driver only needs its first attempt. Callers on
 * a latency-sensitive path (e.g. a control-protocol reply) should run
 * this in a background thread and bound their own wait with a condition
 * variable rather than assuming this call itself is bounded — see
 * control.c's /tunerN/channel SET handler for the pattern.
 *
 * `delivery`: HDHR_DELIVERY_ATSC_VSB tunes 8VSB (this project's
 * original, real-hardware-validated path); HDHR_DELIVERY_QAM tunes
 * in-band clear QAM for the cable channel maps -- UNTESTED against
 * real cable signal, see channel_map.h. */
bool dvb_scan_tune_and_lock(int ffd, uint32_t freq_hz,
                             enum hdhr_delivery_system delivery,
                             dvb_frontend_progress_cb progress_cb, void *progress_ctx);

/* Reads PAT+PMT+TVCT off an already-locked frontend (whichever fd the
 * caller passed to the dvb_scan_tune_and_lock() call that just
 * succeeded -- this function doesn't take that fd itself, since it
 * never touches it directly, only the adapter's demux; the caller must
 * simply keep the frontend open and tuned for the duration of this
 * call) and merges any channels found into the shared database
 * (dvb_channel_db_add() updates existing entries in place rather than
 * duplicating them — safe to call repeatedly on the same frequency).
 * `rf_channel` is the RF channel number to record on any channels
 * found, for the "<channelmap>:N" /tunerN/channel format -- pass 0 if
 * unknown.
 *
 * pat_fd/vct_fd: already-open, already-armed demux filter fds (PAT is
 * PID 0x0000/table_id 0x00; VCT is PSIP_BASE_PID/whichever table_id
 * matches `delivery`, see below) -- the caller opens these once and
 * reuses them across every frequency in a whole scan batch, exactly
 * like it does for the frontend fd passed to dvb_scan_tune_and_lock().
 * Neither filter's parameters ever change over the life of a batch
 * (a batch has one fixed delivery system throughout), so nothing
 * needs re-arming between calls -- matching atscdx's own structure,
 * which opens its one demux fd once at the very start of an entire
 * scan, never per-channel (source read directly, and its own comments
 * say as much: "Open the demux device once at the beginning" /
 * "Modified to accept demux_fd parameter instead of opening/closing
 * demux each time"). This project still needs PAT+PMT in addition to
 * VCT (atscdx only ever reads VCT, since it just needs a channel name
 * for a report, not real PIDs for later playback) -- PMT still opens
 * a fresh filter per program per call (see read_pmts_concurrent()),
 * since which PIDs it needs varies per mux.
 *
 * `delivery` selects which PSIP table vct_fd must have been armed for:
 * HDHR_DELIVERY_ATSC_VSB reads TVCT (table_id 0xC8, terrestrial);
 * HDHR_DELIVERY_QAM reads CVCT (table_id 0xC9, cable) -- structurally
 * identical wire format per ATSC A/65, see psip.c. Real cable
 * operators inconsistently carry usable CVCT at all; if
 * HDHR_DELIVERY_QAM and no CVCT is found but PAT/PMT did resolve real
 * programs, falls back to exposing each program directly as
 * "<rf_channel>.<program_number>" (major.minor) rather than dropping
 * the mux entirely -- matches how real hardware/software (e.g.
 * TVheadend) commonly handles PSIP-less clear QAM. This fallback path
 * is UNTESTED against real cable signal.
 *
 * Returns the number of channels added. Blocks for up to a few
 * seconds. */
int dvb_scan_read_psip(int adapter, int demux_num, int pat_fd, int vct_fd,
                        int rf_channel, uint32_t freq_hz,
                        enum hdhr_delivery_system delivery);

#endif /* HDHR_DVB_SCAN_H */
