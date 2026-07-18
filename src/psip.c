/*
 * psip.c — pure parsing functions, no I/O. PAT/PMT layouts are the
 * ubiquitous, extremely stable MPEG-2 Systems (H.222.0) structures.
 *
 * The TVCT (ATSC A/65 terrestrial virtual channel table) byte layout
 * below is implemented from the published field structure and verified
 * with a self-consistent round-trip test (psip_test.c encodes a
 * synthetic TVCT section with a known-good encoder and confirms this
 * parser reads back the same values) — it has NOT been validated against
 * a real off-air capture. If your real tuner's channel numbers/names
 * come back wrong or empty, that's the first place to look; run with
 * PSIP debug logging (see dvb_scan.c) and compare the raw bytes against
 * the offsets commented below.
 */
#include "psip.h"
#include <string.h>
#include <stdio.h>

static uint16_t rd16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static uint32_t rd32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

int psip_parse_pat(const uint8_t *sec, size_t len, struct pat_entry *out, int max_out)
{
    if (len < 12) return -1; /* header(8) + at least one entry(4) + CRC(4) minimum-ish */
    uint16_t section_length = ((sec[1] & 0x0F) << 8) | sec[2];
    size_t total = 3 + section_length;
    if (total > len || total < 12) return -1;

    size_t entries_start = 8;
    size_t entries_end = total - 4; /* stop before CRC32 */
    int count = 0;

    for (size_t off = entries_start; off + 4 <= entries_end && count < max_out; off += 4) {
        uint16_t program_number = rd16(sec + off);
        uint16_t pid = rd16(sec + off + 2) & 0x1FFF;
        if (program_number == 0) continue; /* network PID entry, not a program */
        out[count].program_number = program_number;
        out[count].pmt_pid = pid;
        count++;
    }
    return count;
}

int psip_parse_pmt(const uint8_t *sec, size_t len, struct pmt_info *out)
{
    if (len < 12) return -1;
    uint16_t section_length = ((sec[1] & 0x0F) << 8) | sec[2];
    size_t total = 3 + section_length;
    if (total > len || total < 12) return -1;

    memset(out, 0, sizeof(*out));
    out->program_number = rd16(sec + 3);
    out->pcr_pid = rd16(sec + 8) & 0x1FFF;

    uint16_t program_info_length = rd16(sec + 10) & 0x0FFF;
    size_t off = 12 + program_info_length;
    size_t end = total - 4;

    while (off + 5 <= end && out->stream_count < PMT_MAX_STREAMS) {
        uint8_t stream_type = sec[off];
        uint16_t elem_pid = rd16(sec + off + 1) & 0x1FFF;
        uint16_t es_info_length = rd16(sec + off + 3) & 0x0FFF;

        out->streams[out->stream_count].stream_type = stream_type;
        out->streams[out->stream_count].elementary_pid = elem_pid;
        out->stream_count++;

        off += 5 + es_info_length;
    }
    return out->stream_count;
}

bool psip_pmt_pick_av(const struct pmt_info *pmt, uint16_t *video_pid,
                       uint8_t *video_stream_type, uint16_t *audio_pid,
                       uint8_t *audio_stream_type)
{
    *video_pid = 0; *video_stream_type = 0;
    *audio_pid = 0; *audio_stream_type = 0;
    bool have_video = false;

    for (int i = 0; i < pmt->stream_count; i++) {
        uint8_t t = pmt->streams[i].stream_type;
        if (!have_video && (t == 0x02 || t == 0x1B)) { /* MPEG-2 or H.264 video */
            *video_pid = pmt->streams[i].elementary_pid;
            *video_stream_type = t;
            have_video = true;
        } else if (*audio_pid == 0 && (t == 0x81 || t == 0x0F)) { /* AC-3 or AAC */
            *audio_pid = pmt->streams[i].elementary_pid;
            *audio_stream_type = t;
        }
    }
    return have_video;
}

/* Lossy UTF-16BE -> ASCII: real ATSC short_names are almost always
 * plain ASCII call signs ("KABC-HD" etc); non-ASCII code units are
 * replaced with '?' rather than attempting real UTF-8 transcoding,
 * since HDHomeRun lineup.json's GuideName is consumed as plain text by
 * every client we care about here anyway. */
static void decode_short_name(const uint8_t *utf16be_7units, char *out, size_t outsize)
{
    size_t o = 0;
    for (int i = 0; i < 7 && o + 1 < outsize; i++) {
        uint16_t unit = rd16(utf16be_7units + i * 2);
        if (unit == 0) break;
        out[o++] = (unit >= 0x20 && unit < 0x7F) ? (char)unit : '?';
    }
    out[o] = '\0';
    /* trim trailing spaces some broadcasters pad with */
    while (o > 0 && out[o - 1] == ' ') out[--o] = '\0';
}

int psip_parse_tvct(const uint8_t *sec, size_t len,
                     struct tvct_entry *out, int max_out,
                     int *out_section_number, int *out_last_section_number)
{
    if (len < 12) return -1;
    uint16_t section_length = ((sec[1] & 0x0F) << 8) | sec[2];
    size_t total = 3 + section_length;
    if (total > len || total < 12) return -1;

    if (out_section_number) *out_section_number = sec[6];
    if (out_last_section_number) *out_last_section_number = sec[7];

    /* byte 8 = protocol_version, byte 9 = num_channels_in_section */
    uint8_t num_channels = sec[9];
    size_t off = 10;
    size_t end = total - 4; /* before CRC32 */
    int count = 0;

    for (uint8_t i = 0; i < num_channels && count < max_out; i++) {
        if (off + 32 > end) break; /* truncated/malformed section */

        struct tvct_entry *e = &out[count];
        memset(e, 0, sizeof(*e));

        decode_short_name(sec + off, e->short_name, sizeof(e->short_name));

        /* bytes [14..16] relative (off+14..off+16): reserved(4) major(10) minor(10) */
        uint32_t triplet = ((uint32_t)sec[off + 14] << 16) | ((uint32_t)sec[off + 15] << 8) | sec[off + 16];
        e->major_channel_number = (triplet >> 10) & 0x3FF;
        e->minor_channel_number = triplet & 0x3FF;

        /* [17] modulation_mode, [18..21] carrier_frequency — not needed,
         * we already know the frequency from what we tuned to scan this
         * mux; skip. */
        e->channel_tsid = rd16(sec + off + 22);
        e->program_number = rd16(sec + off + 24);

        /* Verified against the actual ATSC A/65:2013 spec text, Table 6.4
         * (Bit Stream Syntax for the Terrestrial Virtual Channel Table):
         *   ETM_location(2) | access_controlled(1) | hidden(1) |
         *   reserved(2) | hide_guide(1) | reserved(3) | service_type(6)
         * — 16 bits total, spanning this byte and the next. hidden is
         * bit position 4 from the MSB of this byte (mask 0x10), NOT
         * 0x08 as an earlier version of this code had it — that mask
         * was actually reading one of the two *reserved* bits, which
         * the spec (section 4.2) mandates default to '1'. That bug
         * caused every channel on every mux to read as "hidden" in
         * real-world testing, which is what caught it. */
        uint8_t flags_byte = sec[off + 26];
        e->hidden = (flags_byte & 0x10) != 0;

        uint16_t descriptors_length = rd16(sec + off + 30) & 0x03FF;

        off += 32 + descriptors_length;
        count++;
    }

    (void)rd32; /* carrier_frequency intentionally unused; keep helper for symmetry/future use */
    return count;
}
