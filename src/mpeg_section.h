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

void mpeg_section_filter_close(int fd);

#endif /* HDHR_MPEG_SECTION_H */
