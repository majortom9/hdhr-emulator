/*
 * dvb_stream.c — tunes a frontend, sets demux PID filters (or full-mux
 * passthrough), and captures the resulting multiplexed TS from
 * /dev/dvb/adapter?/dvr? into a ring buffer, same consumer contract
 * tvh_stream.c used to provide (dvb_stream_read/close mirror
 * tvh_stream_read/close exactly) so http_server.c/udp_stream.c barely
 * had to change.
 */
#include "dvb_stream.h"
#include "dvb_frontend.h"
#include "dvb_channel.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <pthread.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/dvb/dmx.h>

#define RING_CAP (1 << 20) /* 1 MiB, same size tvh_stream used */
#define MAX_DEMUX_FDS 6
#define LOCK_TIMEOUT_MS 2000
#define FULL_MUX_PID 0x2000 /* DVB API wildcard: "all PIDs" full-TS passthrough */

struct dvb_stream {
    int frontend_fd;
    int demux_fds[MAX_DEMUX_FDS];
    uint16_t demux_pids[MAX_DEMUX_FDS]; /* which PID each demux_fds[i] is filtering, for real dedup */
    int demux_fd_count;
    int dvr_fd;
    pthread_t thread;

    uint8_t ring[RING_CAP];
    size_t  head, tail, fill;
    pthread_mutex_t mtx;
    pthread_cond_t  not_empty, not_full;

    volatile int closing;
    volatile int eof;

    /* live throughput bookkeeping — see dvb_stream_get_rate(). total_bytes
     * is the lifetime counter (used to detect any activity at all);
     * window_* track a short recent interval so the reported rate is
     * "what's happening now", not a lifetime average that stays skewed
     * by an initial buffer-fill burst for a long time after startup. */
    uint64_t total_bytes;
    struct timespec start_time;
    uint64_t window_start_bytes;
    struct timespec window_start_time;
    double last_bps;
    double last_pps;

    /* legacy symbol-quality fallback (FE_READ_UNCORRECTED_BLOCKS) — see
     * dvb_stream_get_legacy_seq_pct(). Only used for drivers (confirmed:
     * lgdt3306a) that don't populate the modern DVBv5 block-count stats. */
    bool     legacy_seq_have_baseline;
    uint32_t legacy_seq_last_ucblocks;
    struct timespec legacy_seq_last_time;
    int      legacy_seq_last_pct;
};

static int open_pes_pid_filter(int adapter, int demux_num, uint16_t pid)
{
    char path[64];
    snprintf(path, sizeof(path), "/dev/dvb/adapter%d/demux%d", adapter, demux_num);

    int fd = open(path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "dvb_stream: open %s failed: %s\n", path, strerror(errno));
        return -1;
    }

    struct dmx_pes_filter_params params;
    memset(&params, 0, sizeof(params));
    params.pid = pid;
    params.input = DMX_IN_FRONTEND;
    params.output = DMX_OUT_TS_TAP;
    params.pes_type = DMX_PES_OTHER;

    if (ioctl(fd, DMX_SET_PES_FILTER, &params) < 0) {
        fprintf(stderr, "dvb_stream: DMX_SET_PES_FILTER pid=0x%04X failed: %s\n",
                pid, strerror(errno));
        close(fd);
        return -1;
    }
    if (ioctl(fd, DMX_START) < 0) {
        fprintf(stderr, "dvb_stream: DMX_START pid=0x%04X failed: %s\n", pid, strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

/* Adds a PID filter unless it's 0 (meaning "not applicable", e.g. no
 * audio found) or a duplicate of one already opened — PCR very commonly
 * shares the video PID on ATSC broadcasts, and opening two separate PES
 * filters on the same PID genuinely duplicates every packet on that PID
 * into the shared DVR device (each filter independently matches and
 * emits it), not just a cosmetic double-count — this corrupts the
 * resulting elementary stream (a decoder sees interleaved duplicate PES
 * packets) in addition to inflating the measured bitrate. */
static bool add_pid_if_new(struct dvb_stream *s, int adapter, int demux_num, uint16_t pid)
{
    if (pid == 0) return true;
    for (int i = 0; i < s->demux_fd_count; i++) {
        if (s->demux_pids[i] == pid) return true; /* already filtering this PID — nothing to do */
    }
    if (s->demux_fd_count >= MAX_DEMUX_FDS) {
        fprintf(stderr, "dvb_stream: too many PID filters requested (max %d)\n", MAX_DEMUX_FDS);
        return false;
    }
    int fd = open_pes_pid_filter(adapter, demux_num, pid);
    if (fd < 0) return false;
    s->demux_pids[s->demux_fd_count] = pid;
    s->demux_fds[s->demux_fd_count] = fd;
    s->demux_fd_count++;
    return true;
}

static void *reader_thread_main(void *arg)
{
    struct dvb_stream *s = (struct dvb_stream *)arg;
    uint8_t chunk[188 * 64];

    while (!s->closing) {
        struct pollfd pfd = { .fd = s->dvr_fd, .events = POLLIN };
        int pr = poll(&pfd, 1, 500);
        if (pr <= 0) continue; /* timeout or interrupted — loop back and re-check closing */

        ssize_t n = read(s->dvr_fd, chunk, sizeof(chunk));
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EINTR)) continue;
            break; /* real error or EOF */
        }

        pthread_mutex_lock(&s->mtx);
        s->total_bytes += (uint64_t)n;
        size_t off = 0;
        while (off < (size_t)n && !s->closing) {
            while (s->fill == RING_CAP && !s->closing) {
                pthread_cond_wait(&s->not_full, &s->mtx);
            }
            if (s->closing) break;

            size_t space = RING_CAP - s->fill;
            size_t remaining = (size_t)n - off;
            size_t chunk_len = remaining < space ? remaining : space;
            size_t first = RING_CAP - s->head;
            if (first > chunk_len) first = chunk_len;
            memcpy(s->ring + s->head, chunk + off, first);
            if (chunk_len > first) memcpy(s->ring, chunk + off + first, chunk_len - first);
            s->head = (s->head + chunk_len) % RING_CAP;
            s->fill += chunk_len;
            off += chunk_len;
            pthread_cond_signal(&s->not_empty);
        }
        pthread_mutex_unlock(&s->mtx);
    }

    pthread_mutex_lock(&s->mtx);
    s->eof = 1;
    pthread_cond_signal(&s->not_empty);
    pthread_mutex_unlock(&s->mtx);
    return NULL;
}

struct dvb_stream *dvb_stream_open(int adapter, int frontend, int demux_num,
                                    const struct dvb_channel *channel,
                                    int program_override)
{
    struct dvb_stream *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    pthread_mutex_init(&s->mtx, NULL);
    pthread_cond_init(&s->not_empty, NULL);
    pthread_cond_init(&s->not_full, NULL);

    s->frontend_fd = dvb_frontend_open(adapter, frontend);
    if (s->frontend_fd < 0) { free(s); return NULL; }

    if (dvb_frontend_tune_8vsb(s->frontend_fd, channel->frequency_hz) != 0) {
        dvb_frontend_close(s->frontend_fd);
        free(s);
        return NULL;
    }
    if (!dvb_frontend_wait_lock(s->frontend_fd, LOCK_TIMEOUT_MS)) {
        fprintf(stderr, "dvb_stream: no lock on %u Hz for %d.%d\n",
                channel->frequency_hz, channel->major, channel->minor);
        dvb_frontend_close(s->frontend_fd);
        free(s);
        return NULL;
    }

    bool ok = true;
    if (program_override == 0) {
        fprintf(stderr, "dvb_stream: %d.%d: full-mux passthrough (program=0)\n",
                channel->major, channel->minor);
        ok = add_pid_if_new(s, adapter, demux_num, FULL_MUX_PID);
    } else {
        const struct dvb_channel *target = channel;
        struct dvb_channel resolved;
        if (program_override > 0 && program_override != channel->program_number) {
            const struct dvb_channel *siblings[DVB_CHANNEL_MAX];
            int n = dvb_channels_on_freq(channel->frequency_hz, siblings, DVB_CHANNEL_MAX);
            bool found = false;
            for (int i = 0; i < n; i++) {
                if (siblings[i]->program_number == (uint16_t)program_override) {
                    resolved = *siblings[i];
                    target = &resolved;
                    found = true;
                    break;
                }
            }
            if (!found) {
                fprintf(stderr, "dvb_stream: requested program %d not found on this mux, "
                                 "falling back to %d.%d's own program %u\n",
                                 program_override, channel->major, channel->minor,
                                 channel->program_number);
            }
        }

        /* Always include PAT + this program's PMT so a player can
         * actually identify and decode the program, plus PCR/video/audio.
         * add_pid_if_new now genuinely dedups by tracking each demux
         * fd's PID (see its own comment) — this matters because PCR and
         * video very commonly share a PID on ATSC broadcasts, and an
         * earlier version of this code opened a second real filter on
         * the same PID in that case, duplicating packets rather than
         * harmlessly merging them. */
        ok = ok && add_pid_if_new(s, adapter, demux_num, 0x0000); /* PAT */
        ok = ok && add_pid_if_new(s, adapter, demux_num, target->pmt_pid);
        ok = ok && add_pid_if_new(s, adapter, demux_num, target->pcr_pid);
        ok = ok && add_pid_if_new(s, adapter, demux_num, target->video_pid);
        ok = ok && add_pid_if_new(s, adapter, demux_num, target->audio_pid);
    }

    if (!ok || s->demux_fd_count == 0) {
        for (int i = 0; i < s->demux_fd_count; i++) close(s->demux_fds[i]);
        dvb_frontend_close(s->frontend_fd);
        free(s);
        return NULL;
    }

    char dvr_path[64];
    snprintf(dvr_path, sizeof(dvr_path), "/dev/dvb/adapter%d/dvr%d", adapter, demux_num);
    s->dvr_fd = open(dvr_path, O_RDONLY);
    if (s->dvr_fd < 0) {
        fprintf(stderr, "dvb_stream: open %s failed: %s\n", dvr_path, strerror(errno));
        for (int i = 0; i < s->demux_fd_count; i++) close(s->demux_fds[i]);
        dvb_frontend_close(s->frontend_fd);
        free(s);
        return NULL;
    }

    if (pthread_create(&s->thread, NULL, reader_thread_main, s) != 0) {
        close(s->dvr_fd);
        for (int i = 0; i < s->demux_fd_count; i++) close(s->demux_fds[i]);
        dvb_frontend_close(s->frontend_fd);
        free(s);
        return NULL;
    }

    clock_gettime(CLOCK_MONOTONIC, &s->start_time);
    s->window_start_time = s->start_time;
    s->window_start_bytes = 0;
    return s;
}

ssize_t dvb_stream_read(struct dvb_stream *s, void *buf, size_t len)
{
    pthread_mutex_lock(&s->mtx);
    while (s->fill == 0 && !s->eof && !s->closing) {
        pthread_cond_wait(&s->not_empty, &s->mtx);
    }
    if (s->fill == 0) {
        pthread_mutex_unlock(&s->mtx);
        return 0;
    }

    size_t chunk_len = len < s->fill ? len : s->fill;
    size_t first = RING_CAP - s->tail;
    if (first > chunk_len) first = chunk_len;
    memcpy(buf, s->ring + s->tail, first);
    if (chunk_len > first) memcpy((uint8_t *)buf + first, s->ring, chunk_len - first);
    s->tail = (s->tail + chunk_len) % RING_CAP;
    s->fill -= chunk_len;

    pthread_cond_signal(&s->not_full);
    pthread_mutex_unlock(&s->mtx);
    return (ssize_t)chunk_len;
}

void dvb_stream_close(struct dvb_stream *s)
{
    if (!s) return;

    pthread_mutex_lock(&s->mtx);
    s->closing = 1;
    pthread_cond_broadcast(&s->not_full);
    pthread_cond_broadcast(&s->not_empty);
    pthread_mutex_unlock(&s->mtx);

    pthread_join(s->thread, NULL);

    close(s->dvr_fd);
    for (int i = 0; i < s->demux_fd_count; i++) close(s->demux_fds[i]);
    dvb_frontend_close(s->frontend_fd);

    pthread_mutex_destroy(&s->mtx);
    pthread_cond_destroy(&s->not_empty);
    pthread_cond_destroy(&s->not_full);
    free(s);
}

int dvb_stream_frontend_fd(const struct dvb_stream *s)
{
    return s ? s->frontend_fd : -1;
}

/* ATSC 8VSB segment rate, per ATSC A/53: symbol_rate (10,762,237 Hz,
 * exact) / segment_length (832 symbols, including 4-symbol segment
 * sync) = ~12935.38 segments/sec. One RS(207,187)-coded block per data
 * segment, decoding to one 188-byte TS packet — this ignores the
 * ~0.32% overhead from the once-per-313-segments Field Sync Segment
 * (which carries no data block), a deliberate simplification for a
 * coarse quality indicator rather than an exact accounting. This is a
 * fixed physical constant of the ATSC standard, not a driver-dependent
 * guess like the dBm/dBmV signal-strength scale was. */
#define ATSC_SEGMENTS_PER_SEC 12935.38

int dvb_stream_get_legacy_seq_pct(struct dvb_stream *s)
{
    if (!s) return -1;

    uint32_t ucblocks;
    if (!dvb_frontend_read_legacy_ucblocks(s->frontend_fd, &ucblocks)) {
        return -1; /* driver doesn't support this ioctl either */
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (!s->legacy_seq_have_baseline) {
        s->legacy_seq_have_baseline = true;
        s->legacy_seq_last_ucblocks = ucblocks;
        s->legacy_seq_last_time = now;
        s->legacy_seq_last_pct = -1;
        return -1; /* no window yet — next call will have one */
    }

    double elapsed = (now.tv_sec - s->legacy_seq_last_time.tv_sec) +
                      (now.tv_nsec - s->legacy_seq_last_time.tv_nsec) / 1e9;

    /* Same "don't measure a near-zero window" guard as dvb_stream_get_rate(),
     * and for the same reason: return the last computed value rather than
     * a noisy figure from an interval that's too short to be meaningful. */
    if (elapsed < 0.5) {
        return s->legacy_seq_last_pct;
    }

    /* ucblocks is a cumulative counter (confirmed via the kernel DVB API
     * docs: "number of uncorrected blocks detected... during its
     * lifetime"), so guard against a counter reset (channel change,
     * driver quirk) producing a negative delta. */
    uint32_t delta_blocks = (ucblocks >= s->legacy_seq_last_ucblocks)
                             ? (ucblocks - s->legacy_seq_last_ucblocks) : 0;

    double expected_blocks = ATSC_SEGMENTS_PER_SEC * elapsed;
    double error_ratio = expected_blocks > 0 ? (delta_blocks / expected_blocks) : 0.0;
    int pct = (int)((1.0 - error_ratio) * 100.0 + 0.5);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    s->legacy_seq_last_ucblocks = ucblocks;
    s->legacy_seq_last_time = now;
    s->legacy_seq_last_pct = pct;
    return pct;
}

void dvb_stream_get_rate(const struct dvb_stream *s, double *bits_per_second,
                          double *packets_per_second)
{
    *bits_per_second = 0.0;
    *packets_per_second = 0.0;
    if (!s) return;

    /* total_bytes/window_* are written under s->mtx by the reader
     * thread; cast away const to lock the same mutex for a consistent
     * read+update (the struct's mutex itself isn't logically
     * const-violating here). */
    struct dvb_stream *ns = (struct dvb_stream *)s;
    pthread_mutex_lock(&ns->mtx);
    uint64_t bytes_now = s->total_bytes;
    uint64_t window_bytes = s->window_start_bytes;
    struct timespec window_start = s->window_start_time;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = (now.tv_sec - window_start.tv_sec) +
                      (now.tv_nsec - window_start.tv_nsec) / 1e9;

    /* Windowed rate: bytes captured since the *last* status query,
     * divided by the time since that query — reflects current
     * throughput, not a lifetime average that stays distorted by
     * whatever burst happened right after the stream opened (e.g. the
     * ring buffer's initial fill). Require at least 200ms between
     * windows so back-to-back status polls don't divide by
     * near-zero and produce a noisy/inflated instantaneous spike. */
    if (elapsed >= 0.2) {
        uint64_t delta_bytes = bytes_now - window_bytes;
        ns->last_bps = (delta_bytes * 8.0) / elapsed;
        ns->last_pps = (delta_bytes / 188.0) / elapsed;

        ns->window_start_bytes = bytes_now;
        ns->window_start_time = now;
    }
    /* else: too soon since the last window closed (e.g. a GUI polling
     * status faster than 5x/sec) — return the last computed rate rather
     * than flickering to 0 between windows. */
    *bits_per_second = ns->last_bps;
    *packets_per_second = ns->last_pps;

    pthread_mutex_unlock(&ns->mtx);
}
