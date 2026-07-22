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
#include <unistd.h>
#include <poll.h>
#include <time.h>

/* Was 1500ms. Tried 300ms (matching atscdx's own ~150-200ms patience)
 * to dodge this hardware's confirmed occasional multi-second stall
 * inside a single tune-and-wait-for-lock call -- real regression, only
 * 5 virtual channels found vs. the usual 40-70+. Tried 600ms next --
 * recovered only the strongest signals (6 RF channels, all -47 to
 * -51dBm); every medium-strength channel that reliably locks at 1500ms
 * (-54 to -58dBm range) still failed. The relationship looks
 * continuous, not a cliff with a sweet spot -- weaker signals
 * genuinely need more time to lock, which is real RF physics, not a
 * bug. 1000ms next: expected around 10 RF channels if the curve holds. */
#define LOCK_TIMEOUT_MS       1000
#define SECTION_READ_TIMEOUT_MS 2000
#define PMT_MAX_PROGRAMS_TEMP  PAT_MAX_ENTRIES

struct pmt_cache_entry {
    uint16_t program_number;
    struct pmt_info pmt;
};

/* Takes an already-open, already-armed demux filter fd (PID 0x0000,
 * table_id 0x00 -- PAT's filter never changes, regardless of
 * frequency or delivery system) rather than opening/closing one on
 * every call. Matching atscdx's own structure (its VCT read opens its
 * one demux fd once, at the very start of the whole scan, not per
 * channel) -- this project needs PAT+PMT too (atscdx only ever reads
 * VCT, since it just needs a channel name for a report, not real PIDs
 * for later playback), but PAT's own filter is just as fixed as VCT's,
 * so the same reuse applies. */
static int read_pat(int fd, struct pat_entry *out, int max_out)
{
    uint8_t buf[4096];
    ssize_t n = mpeg_section_read(fd, buf, sizeof(buf), SECTION_READ_TIMEOUT_MS);
    if (n <= 0) return -1;
    return psip_parse_pat(buf, (size_t)n, out, max_out);
}

/* Reads every program's PMT concurrently instead of one at a time --
 * each lives on its own PID, so the demux hardware can filter all of
 * them at once, but reading them sequentially (each with its own
 * SECTION_READ_TIMEOUT_MS budget via read_pmt()'s old single-fd form)
 * meant a mux with many subchannels paid that cost once per program.
 * Confirmed live (2026-07-21): a 9-program mux's PMT reads alone took
 * ~2.8s total sequentially -- a real contributor to the client racing
 * ahead of our own worker while it's still busy (see
 * channel_scan_thread_main's own comment in control.c). Opens one
 * filter per program up front, then poll()s all of them together
 * until every one has produced a section or a shared timeout budget
 * (not per-program) runs out -- total wall-clock cost becomes close to
 * the single slowest PMT, not the sum of all of them. */
static int read_pmts_concurrent(int adapter, int demux_num,
                                  const struct pat_entry *pat, int pat_count,
                                  struct pmt_cache_entry *pmt_cache, int max_cache)
{
    int n = pat_count;
    if (n > PMT_MAX_PROGRAMS_TEMP) n = PMT_MAX_PROGRAMS_TEMP;

    int fds[PMT_MAX_PROGRAMS_TEMP];
    int pat_idx[PMT_MAX_PROGRAMS_TEMP]; /* which pat[] entry each fds[] slot belongs to */
    bool done[PMT_MAX_PROGRAMS_TEMP];
    int nfds = 0;

    for (int i = 0; i < n; i++) {
        int fd = mpeg_section_filter_open(adapter, demux_num, pat[i].pmt_pid, 0x02, SECTION_READ_TIMEOUT_MS);
        if (fd < 0) continue;
        fds[nfds] = fd;
        pat_idx[nfds] = i;
        done[nfds] = false;
        nfds++;
    }

    int cache_count = 0;
    int remaining = nfds;
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (remaining > 0) {
        struct pollfd pfds[PMT_MAX_PROGRAMS_TEMP];
        int slot_of[PMT_MAX_PROGRAMS_TEMP];
        int np = 0;
        for (int k = 0; k < nfds; k++) {
            if (done[k]) continue;
            pfds[np].fd = fds[k];
            pfds[np].events = POLLIN;
            pfds[np].revents = 0;
            slot_of[np] = k;
            np++;
        }
        if (np == 0) break;

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000L + (now.tv_nsec - start.tv_nsec) / 1000000L;
        long remaining_ms = SECTION_READ_TIMEOUT_MS - elapsed_ms;
        if (remaining_ms <= 0) break;

        int pr = poll(pfds, (nfds_t)np, (int)remaining_ms);
        if (pr <= 0) break; /* shared budget exhausted, or a real poll error --
                              * either way, whatever hasn't answered yet never will in time */

        for (int j = 0; j < np; j++) {
            if (!(pfds[j].revents & POLLIN)) continue;
            int slot = slot_of[j];
            done[slot] = true;
            remaining--;

            uint8_t buf[4096];
            ssize_t rn = read(fds[slot], buf, sizeof(buf));
            if (rn < 0) {
                continue; /* EOVERFLOW or a real error -- treat as a miss, same as mpeg_section_read() does */
            }
            struct pmt_info pmt;
            if (psip_parse_pmt(buf, (size_t)rn, &pmt) >= 0 && cache_count < max_cache) {
                pmt_cache[cache_count].program_number = pat[pat_idx[slot]].program_number;
                pmt_cache[cache_count].pmt = pmt;
                cache_count++;
            }
        }
    }

    for (int k = 0; k < nfds; k++) {
        mpeg_section_filter_close(fds[k]);
    }
    return cache_count;
}

/* Reads TVCT/CVCT sections until every section_number 0..last_section_number
 * has been seen (typically just one section — a TVCT/CVCT rarely needs
 * to span multiple sections in practice, but the spec allows it), or a
 * handful of read attempts pass without progress. table_id is 0xC8 for
 * TVCT (terrestrial) or 0xC9 for CVCT (cable) — psip_parse_tvct() parses
 * both identically, they share the same wire format per ATSC A/65. */
/* Takes an already-open, already-armed demux filter fd (PSIP_BASE_PID,
 * whichever table_id -- 0xC8/TVCT or 0xC9/CVCT -- matches this whole
 * scan batch's delivery system, see the caller) instead of opening/
 * closing one on every call -- see read_pat()'s own comment for why.
 * table_id stays constant across every frequency a single scan batch
 * processes (delivery is fixed per channel_scan_ctx/dvb_scan_run
 * call), so one fd covers the whole batch safely. */
static int read_vct(int fd, struct tvct_entry *out, int max_out)
{
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

    return total;
}

bool dvb_scan_tune_and_lock(int ffd, uint32_t freq_hz,
                             enum hdhr_delivery_system delivery,
                             dvb_frontend_progress_cb progress_cb, void *progress_ctx)
{
    int tune_rc = (delivery == HDHR_DELIVERY_QAM)
                  ? dvb_frontend_tune_qam(ffd, freq_hz)
                  : dvb_frontend_tune_8vsb(ffd, freq_hz);
    if (tune_rc != 0) return false;

    return dvb_frontend_wait_lock(ffd, LOCK_TIMEOUT_MS, progress_cb, progress_ctx);
    /* no signal on this frequency — normal, most will miss. Caller owns
     * ffd either way now (see this function's own header comment for
     * why) -- doesn't close it here regardless of outcome. */
}

/* Fallback for HDHR_DELIVERY_QAM muxes with no usable CVCT: exposes
 * every PAT program that resolved a PMT directly, as major=rf_channel,
 * minor=program_number (e.g. "24.3") — real cable operators
 * inconsistently carry CVCT at all, and dropping the whole mux just
 * because PSIP isn't there would throw away real, playable programs.
 * UNTESTED against real cable signal. */
static int add_channels_without_vct(int rf_channel, uint32_t freq_hz,
                                     const struct pat_entry *pat, int pat_count,
                                     const struct pmt_cache_entry *pmt_cache, int pmt_cache_count)
{
    int added = 0;
    for (int i = 0; i < pmt_cache_count; i++) {
        struct dvb_channel ch;
        memset(&ch, 0, sizeof(ch));
        ch.major = rf_channel;
        ch.minor = (int)pmt_cache[i].program_number;
        ch.rf_channel = rf_channel;
        ch.delivery = HDHR_DELIVERY_QAM; /* this fallback only ever runs for QAM muxes */
        snprintf(ch.short_name, sizeof(ch.short_name), "%u-%u", rf_channel, pmt_cache[i].program_number);
        ch.frequency_hz = freq_hz;
        ch.program_number = pmt_cache[i].program_number;
        ch.pcr_pid = pmt_cache[i].pmt.pcr_pid;

        uint16_t vpid, apid;
        uint8_t vtype, atype;
        psip_pmt_pick_av(&pmt_cache[i].pmt, &vpid, &vtype, &apid, &atype);
        ch.video_pid = vpid;
        ch.video_stream_type = vtype;
        ch.audio_pid = apid;
        ch.audio_stream_type = atype;

        for (int k = 0; k < pat_count; k++) {
            if (pat[k].program_number == pmt_cache[i].program_number) {
                ch.pmt_pid = pat[k].pmt_pid;
                break;
            }
        }

        if (dvb_channel_db_add(&ch)) {
            added++;
            fprintf(stderr, "dvb_scan: found %d.%d (no CVCT, raw program number) video pid "
                             "0x%04X, audio pid 0x%04X\n",
                             ch.major, ch.minor, ch.video_pid, ch.audio_pid);
        } else {
            fprintf(stderr, "dvb_scan: channel table full (DVB_CHANNEL_MAX), dropping %d.%d\n",
                             ch.major, ch.minor);
        }
    }
    return added;
}

int dvb_scan_read_psip(int adapter, int demux_num, int pat_fd, int vct_fd,
                        int rf_channel, uint32_t freq_hz,
                        enum hdhr_delivery_system delivery)
{
    fprintf(stderr, "dvb_scan: locked %u Hz, reading PAT/PMT/%s...\n", freq_hz,
            delivery == HDHR_DELIVERY_QAM ? "CVCT" : "TVCT");

    /* Re-arm both reused filters for this frequency before reading --
     * without this, stale sections buffered under the *previous* tune
     * can still be sitting in the kernel's demux ring and get delivered
     * here instead of this frequency's real data (see
     * mpeg_section_filter_rearm()'s own comment; confirmed live via a
     * scan reporting one frequency's TSID/programs bleeding into the
     * next). */
    mpeg_section_filter_rearm(pat_fd, 0x0000, 0x00, SECTION_READ_TIMEOUT_MS);
    mpeg_section_filter_rearm(vct_fd, PSIP_BASE_PID,
                               delivery == HDHR_DELIVERY_QAM ? 0xC9 : 0xC8,
                               SECTION_READ_TIMEOUT_MS);

    struct pat_entry pat[PAT_MAX_ENTRIES];
    int pat_count = read_pat(pat_fd, pat, PAT_MAX_ENTRIES);
    if (pat_count <= 0) {
        fprintf(stderr, "dvb_scan: %u Hz locked but no PAT — skipping\n", freq_hz);
        return 0;
    }

    struct pmt_cache_entry pmt_cache[PMT_MAX_PROGRAMS_TEMP];
    int pmt_cache_count = read_pmts_concurrent(adapter, demux_num, pat, pat_count,
                                                 pmt_cache, PMT_MAX_PROGRAMS_TEMP);

    struct tvct_entry tvct[TVCT_MAX_CHANNELS];
    int tvct_count = read_vct(vct_fd, tvct, TVCT_MAX_CHANNELS);

    if (tvct_count <= 0) {
        if (delivery == HDHR_DELIVERY_QAM && pmt_cache_count > 0) {
            fprintf(stderr, "dvb_scan: %u Hz: PAT ok (%d programs) but no CVCT — "
                             "falling back to raw program numbers\n", freq_hz, pat_count);
            return add_channels_without_vct(rf_channel, freq_hz, pat, pat_count,
                                             pmt_cache, pmt_cache_count);
        }
        fprintf(stderr, "dvb_scan: %u Hz: PAT ok (%d programs) but no %s — "
                         "non-ATSC mux or PSIP not on the expected PID, skipping\n",
                         freq_hz, pat_count, delivery == HDHR_DELIVERY_QAM ? "CVCT" : "TVCT");
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
        ch.delivery = delivery;
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

    /* One frontend open for the whole scan, not one per frequency --
     * see dvb_scan_tune_and_lock()'s own header comment for why. Same
     * idea for PAT/VCT's demux filters -- see dvb_scan_read_psip()'s
     * own comment. */
    int ffd = dvb_frontend_open(adapter, frontend);
    if (ffd < 0) return 0;

    int pat_fd = mpeg_section_filter_open(adapter, demux_num, 0x0000, 0x00, SECTION_READ_TIMEOUT_MS);
    int vct_fd = mpeg_section_filter_open(adapter, demux_num, PSIP_BASE_PID, 0xC8, SECTION_READ_TIMEOUT_MS);

    int total = 0;
    if (pat_fd >= 0 && vct_fd >= 0) {
        for (int i = 0; i < ATSC_FREQ_TABLE_COUNT; i++) {
            if (dvb_scan_tune_and_lock(ffd, atsc_freq_table[i].frequency_hz,
                                        HDHR_DELIVERY_ATSC_VSB, NULL, NULL)) {
                total += dvb_scan_read_psip(adapter, demux_num, pat_fd, vct_fd,
                                             atsc_freq_table[i].channel,
                                             atsc_freq_table[i].frequency_hz,
                                             HDHR_DELIVERY_ATSC_VSB);
            }
        }
    }

    if (pat_fd >= 0) mpeg_section_filter_close(pat_fd);
    if (vct_fd >= 0) mpeg_section_filter_close(vct_fd);
    dvb_frontend_close(ffd);
    fprintf(stderr, "dvb_scan: complete — %d virtual channel(s) found\n", total);
    return total;
}
