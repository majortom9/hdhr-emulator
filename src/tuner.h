#ifndef HDHR_TUNER_H
#define HDHR_TUNER_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "config.h"
#include "dvb_stream.h"
#include "dvb_frontend.h"

/* Sized above ATSC_FREQ_TABLE_COUNT (35, see atsc_freq.h) — see
 * hdhr_tuner.pending_queue below. */
#define TUNER_PENDING_QUEUE_CAP 40

/* Mirrors the real device's tunerN GETSET namespace (paths of the form
 * /tunerN/<leaf>). All string fields follow the same conventions as
 * genuine firmware:
 *   channel  = "none" | "<modulation>:<freq_hz>"   e.g. "8vsb:497000000"
 *   program  = mpeg program number | "0" (full mux) | "-1" (none set)
 *   target   = "none" | "udp://<ip>:<port>"
 *   lockkey  = 0 (unlocked) or an opaque token owned by whoever locked it
 *
 * Each tuner slot maps 1:1 to one physical /dev/dvb/adapterN — a real
 * USB tuner can only be locked to one RF frequency at a time, so
 * `busy`/`active_stream` are the single point of truth for "is this
 * physical hardware currently in use", shared between the control
 * protocol's target= UDP push (control.c) and HTTP passthrough pulls
 * (http_server.c). Both must go through tuner_try_claim()/tuner_release()
 * rather than opening a dvb_stream directly, or two viewers could
 * silently fight over the same tuner's frontend. */
struct hdhr_tuner {
    int      index;
    int      adapter; /* physical /dev/dvb/adapterN, from config */
    pthread_mutex_t lock;
    pthread_cond_t  free_cond; /* signaled by tuner_release(); see tuner_try_claim_wait() */

    char     channel[64];
    int      program;
    char     target[64];
    uint32_t lockkey;

    int      vch_major;
    int      vch_minor;
    bool     vch_resolved;
    uint32_t tuned_frequency_hz; /* 0 = not RF-tuned yet; set by channel or vchannel SET.
                                   * Drives /tunerN/streaminfo's live per-mux program
                                   * listing independent of whether anything is
                                   * actively streaming — see control.c. */

    char     status[160];

    /* Live signal stats published by an in-flight /tunerN/channel scan
     * (control.c's channel_scan_thread_main, via
     * dvb_scan_tune_and_lock's progress callback) — lets /tunerN/status
     * report a real `ss=` while a scan is still working, the same way
     * real HDHomeRun firmware does, instead of always showing ss=0 until
     * the scan fully finishes. Real ss= matters for more than cosmetics:
     * libhdhomerun's own hdhomerun_device_wait_for_lock() treats ss<45
     * as "confirmed no signal" and stops polling immediately — with our
     * old always-0 placeholder, it couldn't tell "no signal" from
     * "haven't finished checking yet" and always bailed out early.
     * Only ever written by the scan thread that owns the frontend fd
     * (never a different thread touching that fd concurrently — see
     * dvb_frontend_progress_cb's own comment), but guarded by `lock`
     * like every other field here since it's read from a different
     * thread (whichever connection is handling a /tunerN/status GET).
     * scan_stats_freq lets a reader detect staleness the same way
     * finalize_lock_result() does for `status`: only trust these values
     * if they were published for the frequency the tuner is on *now*. */
    struct dvb_signal_stats scan_stats;
    bool     scan_stats_valid;
    uint32_t scan_stats_freq;

    bool     busy;
    struct dvb_stream *active_stream;

    /* FIFO of /tunerN/channel SETs that arrived while a previous one's
     * scan thread (control.c's channel_scan_thread_main) is still
     * running on this tuner. Rather than block each request's reply
     * waiting for the tuner to free up — which could take as long as
     * the DVB driver's worst-case dead-frequency block (confirmed
     * multi-second, uninterruptible even by signal on lgdt3306a) and
     * blow past a client's own reply-wait patience — each is enqueued
     * here and the in-flight worker drains it itself once done with its
     * current attempt, instead of releasing the tuner. Sized well above
     * ATSC_FREQ_TABLE_COUNT (35): a client scan (hdhomerun_config's
     * `scan`, one SET per RF channel) can queue at most that many
     * requests behind one in-flight attempt. An earlier version of this
     * used a single overwritable slot ("only the newest matters") —
     * that made sense when replies were still slow enough to pace the
     * client naturally, but once replies became fast (this queue's own
     * fix), the client fired through the whole band in ~1s per request
     * and every request but the last got silently clobbered before the
     * worker ever got to it (confirmed live: 33 of 35 channels in a
     * manual scan never got attempted at all, including ones with real
     * signal). A real FIFO instead guarantees every requested frequency
     * eventually gets a genuine tune+PSIP attempt and the shared channel
     * database ends up fully populated — the tradeoff is that the
     * client's own displayed per-line LOCK status can lag well behind
     * whichever channel the client itself has already moved on to
     * asking about, since finalize_lock_result() only updates
     * `status`/`channel` for whichever frequency is still current by
     * the time an attempt actually finishes. Guarded by `lock`. */
    struct {
        uint32_t freq;
        int      rf_channel;
    } pending_queue[TUNER_PENDING_QUEUE_CAP];
    int      pending_head;  /* index of the next entry to dequeue */
    int      pending_count; /* number of entries currently queued */

    /* set by udp_stream.c while a target= push is active, so control.c
     * knows whether to stop a previous push before starting a new one */
    volatile int udp_push_active;
    volatile int udp_push_stop_requested;
    pthread_t    udp_push_thread;

    /* Destination of the current target= UDP push, in binary form, set
     * by udp_push_start() before the push thread starts (valid only
     * while udp_push_active is true). Lets keepalive.c's listener
     * match an incoming client keepalive packet's source address
     * against the right tuner in O(1) without re-parsing `target`'s
     * string form on every packet. keepalive_match_addr is network
     * byte order (matches sockaddr_in.sin_addr.s_addr directly);
     * keepalive_match_port is host byte order. */
    uint32_t keepalive_match_addr;
    uint16_t keepalive_match_port;

    /* Monotonic timestamp of the most recent client keepalive packet
     * matched to this tuner's active push, or of push start if none
     * has arrived yet (see udp_push_start()) — real libhdhomerun
     * clients send one every ~1s while streaming
     * (hdhomerun_video_thread_send_keepalive()) so control.c/udp_stream.c
     * used to have no way to notice a client that died uncleanly
     * (crash, kill -9, network drop) and never sent target=none;
     * without this the tuner stayed locked to a dead destination
     * forever. keepalive.c reclaims it once too much time passes
     * without a match. Guarded by `lock` like every other field here. */
    struct timespec last_keepalive_time;
};

/* cfg supplies each slot's physical adapter number (cfg->dvb_adapter[i]). */
void tuner_pool_init(struct hdhr_tuner *tuners, int count, const struct hdhr_config *cfg);

/* All accessors lock internally; safe to call from any thread. */
void tuner_lock(struct hdhr_tuner *t);
void tuner_unlock(struct hdhr_tuner *t);

/* Atomically claims this tuner's physical hardware if it's currently
 * idle. Returns false (without side effects) if already busy — caller
 * must then either try a different tuner (HTTP auto-allocation) or
 * report "tuner busy" (an explicit /tunerN/... request). On success the
 * caller owns the tuner until tuner_release(); it's expected to open a
 * dvb_stream and store it via tuner_set_stream(). */
bool tuner_try_claim(struct hdhr_tuner *t);

/* Like tuner_try_claim(), but if the tuner is already busy, waits up to
 * timeout_ms for it to free up (woken by tuner_release()) instead of
 * failing immediately. Use this for requests that can reasonably queue
 * behind whatever's currently using the tuner (e.g. a manual channel
 * scan's next frequency, right behind the previous one's) rather than
 * ones that should fail fast if the tuner isn't immediately available
 * (e.g. a live stream request, which still wants tuner_try_claim()).
 * Returns false if still busy after the timeout. */
bool tuner_try_claim_wait(struct hdhr_tuner *t, int timeout_ms);

/* Records the dvb_stream now backing this (already-claimed) tuner, so
 * other code (e.g. status/streaminfo reporting) can see it. */
void tuner_set_stream(struct hdhr_tuner *t, struct dvb_stream *s);

/* Closes any active stream (safe to call with none active) and marks
 * the tuner idle again. */
void tuner_release(struct hdhr_tuner *t);

/* Records which virtual channel + effective program an active stream is
 * actually serving, so /tunerN/status, /tunerN/vchannel, and
 * /tunerN/channel report live truth regardless of whether the stream
 * was started via the control protocol's target= or an HTTP pull —
 * without this, e.g. an HTTP-only viewer would leave the tuner's
 * control-plane state at its stale/default values even while actively
 * streaming. Call after a successful tuner_try_claim()+dvb_stream_open(),
 * before tuner_set_stream(). */
void tuner_bind_channel(struct hdhr_tuner *t, const struct dvb_channel *ch, int program_override);

/* Scans for the first idle tuner and claims it. Returns NULL if all
 * tuners are currently busy. Used by HTTP's channel-only /auto/vX.X
 * path, which doesn't name a specific tuner slot — same auto-allocation
 * behavior real HDHomeRun firmware provides across its N tuners. */
struct hdhr_tuner *tuner_pool_claim_free(struct hdhr_tuner *tuners, int count);

#endif /* HDHR_TUNER_H */
