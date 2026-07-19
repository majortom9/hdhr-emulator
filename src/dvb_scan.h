#ifndef HDHR_DVB_SCAN_H
#define HDHR_DVB_SCAN_H

#include <stdint.h>
#include <stdbool.h>

#include "dvb_frontend.h"

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

/* Tunes to freq_hz and waits for lock. On success, leaves the frontend
 * OPEN and returns its fd via *out_ffd for a follow-up
 * dvb_scan_read_psip() call; on failure, closes the frontend itself and
 * sets *out_ffd to -1. Returns whether lock was achieved.
 *
 * NOTE ON WORST-CASE LATENCY: some DVB demod drivers (confirmed on
 * lgdt3306a) retry the lock internally several times *inside a single
 * blocking ioctl* before reporting failure on a dead frequency — our own
 * internal lock-wait timeout can't interrupt a syscall that's already
 * blocked inside the kernel/driver, so a call that ultimately fails to
 * lock can take substantially longer (~10s observed) than that timeout
 * would suggest. A successful lock, by contrast, is fast (well under
 * 2s observed) since the driver only needs its first attempt. Callers on
 * a latency-sensitive path (e.g. a control-protocol reply) should run
 * this in a background thread and bound their own wait with a condition
 * variable rather than assuming this call itself is bounded — see
 * control.c's /tunerN/channel SET handler for the pattern. */
bool dvb_scan_tune_and_lock(int adapter, int frontend, uint32_t freq_hz, int *out_ffd,
                             dvb_frontend_progress_cb progress_cb, void *progress_ctx);

/* Reads PAT+PMT+TVCT off an already-locked frontend (ffd, as returned by
 * dvb_scan_tune_and_lock on success) and merges any channels found into
 * the shared database (dvb_channel_db_add() updates existing entries in
 * place rather than duplicating them — safe to call repeatedly on the
 * same frequency). Always closes ffd before returning, whether or not
 * anything usable was found. `rf_channel` is the RF channel number
 * (2-36) to record on any channels found, for the "us-bcast:N"
 * /tunerN/channel format — pass 0 if unknown. Returns the number of
 * channels added. Blocks for up to a few seconds. */
int dvb_scan_read_psip(int adapter, int demux_num, int rf_channel, uint32_t freq_hz, int ffd);

#endif /* HDHR_DVB_SCAN_H */
