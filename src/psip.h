#ifndef HDHR_PSIP_H
#define HDHR_PSIP_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define PSIP_BASE_PID 0x1FFB /* where ATSC MGT/TVCT/CVCT/EIT/ETT normally live */

/* --- PAT: table_id 0x00, PID 0x0000 --- */
struct pat_entry {
    uint16_t program_number;
    uint16_t pmt_pid;
};
#define PAT_MAX_ENTRIES 64

/* Parses a full PAT section (including the 3-byte table_id/section_length
 * header and trailing 4-byte CRC — CRC itself is not re-validated here,
 * since the kernel's DMX_CHECK_CRC already guaranteed it on the way in).
 * Skips program_number 0 (network PID) entries. Returns entry count, or
 * -1 on a malformed section. */
int psip_parse_pat(const uint8_t *sec, size_t len,
                    struct pat_entry *out, int max_out);

/* --- PMT: table_id 0x02, PID from PAT --- */
#define PMT_MAX_STREAMS 32
struct pmt_stream {
    uint8_t  stream_type;
    uint16_t elementary_pid;
};
struct pmt_info {
    uint16_t program_number;
    uint16_t pcr_pid;
    int      stream_count;
    struct pmt_stream streams[PMT_MAX_STREAMS];
};

int psip_parse_pmt(const uint8_t *sec, size_t len, struct pmt_info *out);

/* Picks the primary video/audio PID out of a parsed PMT's stream list.
 * ATSC video is stream_type 0x02 (MPEG-2) or 0x1B (H.264); ATSC audio is
 * almost always 0x81 (AC-3, ATSC A/52) though 0x0F (AAC/ADTS) shows up
 * on some streams. Returns false if no recognizable video stream was
 * found (audio_pid may legitimately be 0 for a data-only PMT entry). */
bool psip_pmt_pick_av(const struct pmt_info *pmt, uint16_t *video_pid,
                       uint8_t *video_stream_type, uint16_t *audio_pid,
                       uint8_t *audio_stream_type);

/* --- TVCT: table_id 0xC8, PID 0x1FFB (terrestrial virtual channel table) --- */
#define TVCT_MAX_CHANNELS 64
struct tvct_entry {
    char     short_name[16]; /* UTF-16 source, ASCII-lossy-converted, NUL-terminated */
    int      major_channel_number;
    int      minor_channel_number;
    uint16_t channel_tsid;
    uint16_t program_number;
    bool     hidden;
};

/* Parses one TVCT section. A mux with more channels than fit in one
 * section will span multiple sections (section_number/last_section_number
 * in the header) — callers scanning a mux should keep reading sections
 * until they've seen last_section_number+1 distinct section_numbers, or
 * a short timeout elapses (see dvb_scan.c). Returns entry count parsed
 * from this section, or -1 on a malformed section. *out_section_number
 * and *out_last_section_number are filled so the caller can track
 * completeness across a multi-section table. */
int psip_parse_tvct(const uint8_t *sec, size_t len,
                     struct tvct_entry *out, int max_out,
                     int *out_section_number, int *out_last_section_number);

#endif /* HDHR_PSIP_H */
