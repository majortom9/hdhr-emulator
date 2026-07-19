/*
 * dvb_scan.c — full ATSC channel scan: tune every candidate RF
 * frequency, and for any that lock, read PAT + each program's PMT + the
 * mux's TVCT to populate the virtual-channel database.
 */
#include "dvb_scan.h"
#include "atsc_freq.h"
#include "dvb_frontend.h"
#include "dvb_channel.h"
#include "mpeg_section.h"
#include "psip.h"

#include <stdio.h>
#include <string.h>

#define LOCK_TIMEOUT_MS       1500
#define SECTION_READ_TIMEOUT_MS 2000
#define PMT_MAX_PROGRAMS_TEMP  PAT_MAX_ENTRIES

struct pmt_cache_entry {
    uint16_t program_number;
    struct pmt_info pmt;
};

static int read_pat(int adapter, int demux_num, struct pat_entry *out, int max_out)
{
    int fd = mpeg_section_filter_open(adapter, demux_num, 0x0000, 0x00, SECTION_READ_TIMEOUT_MS);
    if (fd < 0) return -1;

    uint8_t buf[4096];
    ssize_t n = mpeg_section_read(fd, buf, sizeof(buf), SECTION_READ_TIMEOUT_MS);
    mpeg_section_filter_close(fd);

    if (n <= 0) return -1;
    return psip_parse_pat(buf, (size_t)n, out, max_out);
}

static bool read_pmt(int adapter, int demux_num, uint16_t pmt_pid, struct pmt_info *out)
{
    int fd = mpeg_section_filter_open(adapter, demux_num, pmt_pid, 0x02, SECTION_READ_TIMEOUT_MS);
    if (fd < 0) return false;

    uint8_t buf[4096];
    ssize_t n = mpeg_section_read(fd, buf, sizeof(buf), SECTION_READ_TIMEOUT_MS);
    mpeg_section_filter_close(fd);

    if (n <= 0) return false;
    return psip_parse_pmt(buf, (size_t)n, out) >= 0;
}

/* Reads TVCT sections until every section_number 0..last_section_number
 * has been seen (typically just one section — a TVCT rarely needs to
 * span multiple sections in practice, but the spec allows it), or a
 * handful of read attempts pass without progress. */
static int read_tvct(int adapter, int demux_num, struct tvct_entry *out, int max_out)
{
    int fd = mpeg_section_filter_open(adapter, demux_num, PSIP_BASE_PID, 0xC8, SECTION_READ_TIMEOUT_MS);
    if (fd < 0) return -1;

    bool seen[256] = {false};
    int last_section = -1;
    int total = 0;
    int attempts = 0;

    while (attempts < 8 && total < max_out) {
        uint8_t buf[4096];
        ssize_t n = mpeg_section_read(fd, buf, sizeof(buf), SECTION_READ_TIMEOUT_MS);
        attempts++;
        if (n <= 0) break;

        int sec_num = -1, last_sec_num = -1;
        int got = psip_parse_tvct(buf, (size_t)n, out + total, max_out - total,
                                   &sec_num, &last_sec_num);
        if (got < 0) continue;
        if (sec_num >= 0 && sec_num < 256) {
            if (seen[sec_num]) continue; /* duplicate delivery, skip re-adding */
            seen[sec_num] = true;
        }
        last_section = last_sec_num;
        total += got;

        if (last_section >= 0) {
            bool complete = true;
            for (int s = 0; s <= last_section; s++) {
                if (!seen[s]) { complete = false; break; }
            }
            if (complete) break;
        }
    }

    mpeg_section_filter_close(fd);
    return total;
}

bool dvb_scan_tune_and_lock(int adapter, int frontend, uint32_t freq_hz, int *out_ffd)
{
    *out_ffd = -1;

    int ffd = dvb_frontend_open(adapter, frontend);
    if (ffd < 0) return false;

    if (dvb_frontend_tune_8vsb(ffd, freq_hz) != 0) {
        dvb_frontend_close(ffd);
        return false;
    }
    if (!dvb_frontend_wait_lock(ffd, LOCK_TIMEOUT_MS)) {
        dvb_frontend_close(ffd); /* no signal on this frequency — normal, most will miss */
        return false;
    }

    *out_ffd = ffd;
    return true;
}

int dvb_scan_read_psip(int adapter, int demux_num, int rf_channel, uint32_t freq_hz, int ffd)
{
    fprintf(stderr, "dvb_scan: locked %u Hz, reading PAT/PMT/TVCT...\n", freq_hz);

    struct pat_entry pat[PAT_MAX_ENTRIES];
    int pat_count = read_pat(adapter, demux_num, pat, PAT_MAX_ENTRIES);
    if (pat_count <= 0) {
        fprintf(stderr, "dvb_scan: %u Hz locked but no PAT — skipping\n", freq_hz);
        dvb_frontend_close(ffd);
        return 0;
    }

    struct pmt_cache_entry pmt_cache[PMT_MAX_PROGRAMS_TEMP];
    int pmt_cache_count = 0;
    for (int i = 0; i < pat_count && pmt_cache_count < PMT_MAX_PROGRAMS_TEMP; i++) {
        struct pmt_info pmt;
        if (read_pmt(adapter, demux_num, pat[i].pmt_pid, &pmt)) {
            pmt_cache[pmt_cache_count].program_number = pat[i].program_number;
            pmt_cache[pmt_cache_count].pmt = pmt;
            pmt_cache_count++;
        }
    }

    struct tvct_entry tvct[TVCT_MAX_CHANNELS];
    int tvct_count = read_tvct(adapter, demux_num, tvct, TVCT_MAX_CHANNELS);

    dvb_frontend_close(ffd);

    if (tvct_count <= 0) {
        fprintf(stderr, "dvb_scan: %u Hz: PAT ok (%d programs) but no TVCT — "
                         "non-ATSC mux or PSIP not on the expected PID, skipping\n",
                         freq_hz, pat_count);
        return 0;
    }

    int added = 0;
    for (int i = 0; i < tvct_count; i++) {
        if (tvct[i].hidden) {
            fprintf(stderr, "dvb_scan: %u Hz: %d.%d (%s) is marked hidden in PSIP — "
                             "skipping (control/data feed, not a user-selectable channel). "
                             "If this looks wrong (e.g. most/all channels on a mux showing "
                             "hidden), see the bit-layout note in psip.c's psip_parse_tvct().\n",
                             freq_hz, tvct[i].major_channel_number, tvct[i].minor_channel_number,
                             tvct[i].short_name);
            continue;
        }

        struct pmt_info *pmt = NULL;
        for (int j = 0; j < pmt_cache_count; j++) {
            if (pmt_cache[j].program_number == tvct[i].program_number) {
                pmt = &pmt_cache[j].pmt;
                break;
            }
        }
        if (!pmt) {
            fprintf(stderr, "dvb_scan: %u Hz: TVCT channel %d.%d (%s) has no matching "
                             "PMT (program_number=%u) — skipping\n",
                             freq_hz, tvct[i].major_channel_number, tvct[i].minor_channel_number,
                             tvct[i].short_name, tvct[i].program_number);
            continue;
        }

        struct dvb_channel ch;
        memset(&ch, 0, sizeof(ch));
        ch.major = tvct[i].major_channel_number;
        ch.minor = tvct[i].minor_channel_number;
        ch.rf_channel = rf_channel;
        snprintf(ch.short_name, sizeof(ch.short_name), "%.15s", tvct[i].short_name);
        ch.frequency_hz = freq_hz;
        ch.channel_tsid = tvct[i].channel_tsid;
        ch.program_number = tvct[i].program_number;
        ch.pcr_pid = pmt->pcr_pid;

        uint16_t vpid, apid;
        uint8_t vtype, atype;
        bool have_video = psip_pmt_pick_av(pmt, &vpid, &vtype, &apid, &atype);
        ch.video_pid = vpid;
        ch.video_stream_type = vtype;
        ch.audio_pid = apid;
        ch.audio_stream_type = atype;

        /* find the PMT's own PID back out of the PAT entry we already have */
        for (int k = 0; k < pat_count; k++) {
            if (pat[k].program_number == tvct[i].program_number) {
                ch.pmt_pid = pat[k].pmt_pid;
                break;
            }
        }

        if (!have_video) {
            fprintf(stderr, "dvb_scan: %u Hz: %d.%d (%s) has no recognizable video "
                             "stream — adding anyway (radio/data subchannel?)\n",
                             freq_hz, ch.major, ch.minor, ch.short_name);
        }

        if (dvb_channel_db_add(&ch)) {
            added++;
            fprintf(stderr, "dvb_scan: found %d.%d \"%s\" (program %u, video pid 0x%04X, "
                             "audio pid 0x%04X)\n",
                             ch.major, ch.minor, ch.short_name, ch.program_number,
                             ch.video_pid, ch.audio_pid);
        } else {
            fprintf(stderr, "dvb_scan: channel table full (DVB_CHANNEL_MAX), dropping %d.%d\n",
                             ch.major, ch.minor);
        }
    }
    return added;
}

int dvb_scan_run(int adapter, int frontend, int demux_num)
{
    dvb_channel_db_clear();
    fprintf(stderr, "dvb_scan: starting full scan on adapter%d (this takes a couple minutes)\n", adapter);

    int total = 0;
    for (int i = 0; i < ATSC_FREQ_TABLE_COUNT; i++) {
        int ffd;
        if (dvb_scan_tune_and_lock(adapter, frontend, atsc_freq_table[i].frequency_hz, &ffd)) {
            total += dvb_scan_read_psip(adapter, demux_num, atsc_freq_table[i].channel,
                                         atsc_freq_table[i].frequency_hz, ffd);
        }
    }

    fprintf(stderr, "dvb_scan: complete — %d virtual channel(s) found\n", total);
    return total;
}
