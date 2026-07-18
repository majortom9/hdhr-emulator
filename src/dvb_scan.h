#ifndef HDHR_DVB_SCAN_H
#define HDHR_DVB_SCAN_H

#include <stdint.h>
#include <stdbool.h>

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

/* Tunes and reads PAT+PMT+TVCT for exactly one frequency, merging any
 * channels found into the shared database (dvb_channel_db_add() updates
 * existing entries in place rather than duplicating them — safe to call
 * repeatedly on the same frequency). This is what backs a real
 * /tunerN/channel SET: genuine HDHomeRun firmware performs live PSIP
 * detection at tune time rather than relying solely on a pre-built
 * lineup, which is exactly what tools like `hdhomerun_config ... scan`
 * expect to trigger. `rf_channel` is the RF channel number (2-36) to
 * record on any channels found, for the "us-bcast:N" /tunerN/channel
 * format — pass 0 if unknown (e.g. tuning to an arbitrary frequency not
 * in atsc_freq_table). Returns true if the frontend achieved lock,
 * regardless of whether any usable ATSC PSIP was found there (a locked
 * non-ATSC or PSIP-less mux is still a "successful tune" from the
 * caller's perspective). Blocks for up to a few seconds. */
bool dvb_scan_frequency(int adapter, int frontend, int demux_num, int rf_channel, uint32_t freq_hz);

#endif /* HDHR_DVB_SCAN_H */
