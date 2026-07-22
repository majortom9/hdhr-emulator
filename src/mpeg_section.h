#ifndef HDHR_MPEG_SECTION_H
#define HDHR_MPEG_SECTION_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

/* Opens /dev/dvb/adapter<adapter>/demux<demux_num>, filters for sections
 * on `pid` whose first byte (table_id) equals `table_id`, with the
 * kernel doing CRC32 validation (DMX_CHECK_CRC) so callers never see a
 * corrupt section. Returns an fd ready to read() from, or -1 on error.
 * `timeout_ms` is passed to the kernel filter as its own giveup timer;
 * callers should also apply their own read timeout via poll(). */
int mpeg_section_filter_open(int adapter, int demux_num, uint16_t pid,
                              uint8_t table_id, uint32_t timeout_ms);

/* Reads one complete, CRC-verified section into buf (caller-sized,
 * 4096 bytes is enough for any PSI/PSIP section — max section length
 * is 4093 bytes per MPEG-2 spec). Returns the section length on
 * success, 0 on timeout, -1 on error. poll()s internally up to
 * timeout_ms, separate from the kernel-side filter timeout above. */
ssize_t mpeg_section_read(int fd, uint8_t *buf, size_t bufsize, int timeout_ms);

/* Re-arms an already-open filter fd (from mpeg_section_filter_open) for
 * a fresh section on the same pid/table_id. Needed whenever the fd is
 * being reused across a frontend retune instead of closed and reopened
 * fresh: DMX_SET_FILTER + DMX_IMMEDIATE_START is what actually resets
 * the kernel's demux ring buffer for that filter, and merely leaving
 * the fd open across a retune does NOT discard sections that arrived
 * under the previous tune -- confirmed live: reusing PAT/VCT fds across
 * a scan batch without this produced a stale prior mux's TSID/programs
 * bleeding into the next frequency's report. Call this right after
 * dvb_scan_tune_and_lock() succeeds and before the next read, every
 * time -- matches atscdx's own structure, which keeps its one demux fd
 * open for a whole scan (saving repeated open() calls) but still issues
 * a fresh DMX_SET_FILTER before every single channel's read. */
int mpeg_section_filter_rearm(int fd, uint16_t pid, uint8_t table_id, uint32_t timeout_ms);

void mpeg_section_filter_close(int fd);

#endif /* HDHR_MPEG_SECTION_H */
