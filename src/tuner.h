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
    int      frontend; /* frontend index within that adapter, from config
                          * (cfg->dvb_frontend, same value for every tuner
                          * slot) -- copied here so tuner_open_hold()/
                          * tuner_release() can self-sufficiently re-tune
                          * without needing cfg threaded through every
                          * call site. */
    pthread_mutex_t lock;
    pthread_cond_t  free_cond; /* signaled by tuner_release(); see tuner_try_claim_wait() */

    char     channel[64];
    int      program;
    char     target[64];
    uint32_t lockkey;

    /* /tunerN/channelmap — which channel map (see channel_map.h)
     * "us-bcast:<N>"/plain channel-number tuning gets resolved against.
     * "us-bcast" (this project's original, thoroughly real-hardware-
     * validated ATSC-only scope) is the default and isn't itself in
     * channel_map.c's table (that module only covers the other five —
     * see its own header comment); the other five (us-cable/us-hrc/
     * us-irc/kr-bcast/kr-cable) are UNTESTED against real signal.
     * Changing this clears channel/vchannel state, same as a frequency
     * change, since a channel *number* means something different in
     * every map (e.g. "7" is 177MHz in us-bcast but a different
     * frequency in us-irc). */
    char     channelmap[16];

    /* /tunerN/filter's explicit PID override, in wire format (see
     * pid_filter.h) — "" (empty) means "not set", i.e. PID selection
     * falls back to program's normal resolution. Confirmed against a
     * genuine HDHomeRun3 (2026-07-19): setting /tunerN/program
     * recomputes filter to that program's own PID set, so it's cleared
     * (not merely left stale) whenever program or channel/vchannel
     * changes — see handle_tuner_set. Snapshotted into udp_stream.c's
     * push_ctx at push-start time, same as program; only ever consulted
     * by the target= push path, not HTTP passthrough (same established
     * asymmetry as program itself). */
    char     filter_override[256];

    int      vch_major;
    int      vch_minor;
    bool     vch_resolved;
    uint32_t tuned_frequency_hz; /* 0 = not RF-tuned yet; set by channel or vchannel SET.
                                   * Drives /tunerN/streaminfo's live per-mux program
                                   * listing independent of whether anything is
                                   * actively streaming — see control.c. */
    enum hdhr_delivery_system tuned_delivery; /* which modulation tuned_frequency_hz
                                   * needs (8VSB vs QAM) -- set alongside
                                   * tuned_frequency_hz (tuner_bind_channel(), and
                                   * control.c's raw /tunerN/channel SET), read back
                                   * by tuner_release() to know how to re-tune when
                                   * re-establishing a hold (see held_fd below). */

    char     status[160];

    /* Background "hold": a frontend fd kept open+tuned to
     * tuned_frequency_hz/tuned_delivery for as long as a channel is
     * selected, independent of whether anything is actively streaming
     * it -- -1 when not held. Real HDHomeRun firmware engages a
     * selected tuner's physical frontend continuously and keeps
     * reporting genuinely live signal stats regardless of whether a
     * client is pulling video; without this, this project's own lazy
     * "only tune when actually streaming" design (see tuner_bind_channel()'s
     * comment) meant /tunerN/status went stale the instant a channel
     * scan/verify finished, showing a frozen one-time snapshot
     * (`scan_stats` below) for as long as a channel was merely
     * selected but not being watched -- confirmed live, and a real
     * problem for third-party signal-monitoring tools that poll
     * channel+status without ever actually streaming.
     *
     * Opened by tuner_open_hold() (control.c's vchannel SET, and
     * channel_scan_thread_main once its own scan/verify tune
     * completes); closed by tuner_close_hold() (an explicit
     * channel=none or channelmap change) or automatically by
     * tuner_try_claim()/tuner_try_claim_wait() the instant something
     * wants to actually stream this tuner (a held fd and an
     * active_stream's own fd must never be open on the same physical
     * frontend device node at once). tuner_release() re-opens a fresh
     * hold once streaming ends, if a channel is still logically
     * selected -- so live status resumes automatically once playback
     * stops, matching real hardware. Guarded by `lock`. */
    int      held_fd;
    /* Windowed state for the legacy FE_READ_UNCORRECTED_BLOCKS-based
     * seq= fallback (see dvb_frontend_legacy_seq_pct()) while reading
     * held_fd specifically -- a held tune has no struct dvb_stream to
     * own this the way active streaming does (see dvb_stream.c's own
     * copy of this same state), so it lives here instead. Reset
     * (zeroed) every time a new hold is opened, so a previous
     * channel's counts never leak into a new one's window. Exclusively
     * read/written by held_stats_thread_main() (see held_stats_thread
     * below) -- NOT guarded by `lock`, since only that one background
     * thread ever touches it. */
    struct dvb_legacy_seq_state held_legacy_seq;

    /* Background refresher for held_fd's own live signal stats --
     * confirmed live (2026-07-20) that dvb_frontend_read_stats() can
     * itself block for multiple seconds on this hardware under marginal
     * conditions (same driver-level risk already documented for tuning,
     * see dvb_frontend.c's PROGRESS_CB_INTERVAL_MS comment, just not
     * previously known to affect plain stat reads on an *already*-tuned
     * fd too -- one observed instance took 6.8 real seconds). Calling
     * that directly from a connection thread answering a /tunerN/status
     * GET risked exceeding a real client's own patience and produced a
     * genuine "communication error". held_stats_thread_main() now owns
     * all reads of held_fd's stats, refreshing held_stats_cache
     * periodically on its own thread; /tunerN/status only ever reads
     * the cache, never calling the ioctl itself. Started by
     * tuner_open_hold() once held_fd is set; stopped (and joined -- see
     * stop_held_stats_thread() in tuner.c) before held_fd is ever
     * closed or handed off, by every call site that does so
     * (tuner_close_hold(), tuner_try_claim()/tuner_try_claim_wait()'s
     * resolve_claimed_hold()). The thread never closes held_fd itself --
     * ownership of the fd's lifecycle stays exactly where it already
     * was; this only adds a cache in front of reading it. Guarded by
     * `lock`, like every other field here, except held_stats_thread
     * itself (only ever touched by whichever code currently owns
     * starting/joining it, never concurrently). */
    pthread_t held_stats_thread;
    bool      held_stats_thread_started;
    volatile bool held_stats_stop_requested;
    struct dvb_signal_stats held_stats_cache;
    bool      held_stats_cache_valid;

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

    /* True only while control.c's channel_scan_thread_main actually owns
     * this tuner's claim -- distinct from `busy`, which also goes true
     * for claims nothing drains a queue for (main.c's one-shot startup
     * scan, which talks to the adapter/frontend/demux directly and has
     * no notion of pending_queue at all). handle_tuner_set's /tunerN/
     * channel SET path checks this before deciding to enqueue a request
     * onto pending_queue below: queueing only makes sense if some
     * worker will actually drain it. Confirmed live: once main.c's
     * startup scan started holding its claim for the whole ~1-2 minute
     * scan instead of releasing between frequencies, a SET landing
     * during that window got silently queued here with nothing to ever
     * service it -- /tunerN/status looked like it worked (tuner_release()'s
     * own "re-establish a hold on the last tuned frequency" coincidentally
     * picked up the frequency the queuing code had stashed and opened a
     * real tuned hold on it) but /tunerN/vchannel stayed "none" forever,
     * since no actual PSIP scan ever ran for it. Set true by
     * handle_tuner_set right after a successful tuner_try_claim_fast(),
     * cleared by channel_scan_thread_main itself right before it releases
     * the tuner. */
    bool     scan_worker_active;

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

    /* Signaled whenever something new is pushed onto pending_queue (see
     * handle_tuner_set's queuing branch). channel_scan_thread_main waits
     * on this briefly (PENDING_WAIT_MS, control.c) instead of exiting the
     * instant it finds the queue empty -- confirmed live (2026-07-21) via
     * a scan-thread-invocation counter that a real client's own pacing
     * between /tunerN/channel SETs routinely outlasts how long our own
     * worker takes to process the current frequency, so the queue is
     * often still genuinely empty the moment we finish. Exiting right
     * then meant the worker released the tuner and fully closed its
     * held-open frontend fd, only for the client's very next SET to spawn
     * a *fresh* channel_scan_thread_main with a *fresh*
     * dvb_frontend_open() -- silently defeating open-once-per-batch
     * (confirmed via dmesg: 9 separate invocations logged across a single
     * 35-frequency scan, each its own chance at a chip re-init, largely
     * explaining why this path kept showing ~23 re-inits per scan even
     * after both the frontend and PAT/VCT fds were switched to open-once-
     * reuse-many). A short bounded wait bridges most of those gaps
     * instead, without adding any new queueing semantics -- the FIFO
     * itself stays exactly as-is, since it's solving a different, real
     * problem (see pending_queue's own comment): replying to a SET
     * immediately, rather than blocking until the real tune result is
     * known, is what keeps a slow/dead frequency (confirmed up to ~8s
     * worst case, driver-internal and uninterruptible) from exceeding a
     * real client's own total reply patience (libhdhomerun's
     * HDHOMERUN_CONTROL_RECV_TIMEOUT is 2500ms, retried once -- so ~5s
     * total, confirmed by reading hdhomerun_control.c directly -- shorter
     * than our own worst case) and surfacing as a hard communication
     * error. */
    pthread_cond_t pending_cond;

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
 * dvb_stream and store it via tuner_set_stream().
 *
 * Also resolves this tuner's background hold (see held_fd/
 * tuner_open_hold()), if one is open, since a hold and an
 * active_stream's own frontend fd must never both be open on the same
 * physical frontend device node at once. If out_reused_fd is non-NULL
 * and the hold is already tuned to want_freq_hz/want_delivery, ownership
 * of that already-open, already-*locked* fd transfers to the caller via
 * *out_reused_fd instead of being closed — pass it straight through to
 * dvb_stream_open()'s adopt_fd. Closing an already-locked hold only to
 * have the caller reopen+relock the same frequency a moment later isn't
 * just wasteful, it can genuinely fail to relock in time (confirmed
 * live) — reusing it sidesteps that entirely. If out_reused_fd is NULL,
 * or the hold doesn't match, any open hold is simply closed as before
 * and *out_reused_fd (if given) is set to -1. */
bool tuner_try_claim(struct hdhr_tuner *t, uint32_t want_freq_hz,
                      enum hdhr_delivery_system want_delivery, int *out_reused_fd);

/* Like tuner_try_claim(), but if the tuner is already busy, waits up to
 * timeout_ms for it to free up (woken by tuner_release()) instead of
 * failing immediately. Use this for requests that can reasonably queue
 * behind whatever's currently using the tuner (e.g. a manual channel
 * scan's next frequency, right behind the previous one's) rather than
 * ones that should fail fast if the tuner isn't immediately available
 * (e.g. a live stream request, which still wants tuner_try_claim()).
 * Returns false if still busy after the timeout. Resolves an existing
 * hold on success, same as tuner_try_claim() (see its comment for the
 * want_freq_hz/want_delivery/out_reused_fd reuse behavior). */
bool tuner_try_claim_wait(struct hdhr_tuner *t, int timeout_ms, uint32_t want_freq_hz,
                           enum hdhr_delivery_system want_delivery, int *out_reused_fd);

/* Like tuner_try_claim(), but does NOT resolve an existing hold itself --
 * hands back whatever held_fd was open (or -1) via *out_stale_held_fd,
 * already atomically detached from the tuner (t->held_fd is cleared
 * before this returns, so any concurrent /tunerN/status GET immediately
 * stops seeing it), but not yet stopped/closed. The caller must pass it
 * to tuner_resolve_stale_held_fd() *on its own background thread*, not
 * inline here.
 *
 * Why this exists at all: resolving a stale hold (tuner_try_claim()'s
 * normal behavior) means stop_held_stats_thread() joining held_fd's
 * background refresher, which can itself block for as long as that
 * thread's current stat-read takes -- confirmed live to occasionally
 * reach several seconds on this hardware. tuner_try_claim() accepts that
 * cost because its callers (udp_stream.c's push thread, http_server.c's
 * per-connection thread) already run on their own background thread, so
 * blocking there doesn't threaten any reply's timing. control.c's raw
 * /tunerN/channel SET is different: it must claim synchronously, on the
 * connection thread, specifically to decide fast/queued/refused for its
 * own reply (see its own comment) -- so THAT resolution can't happen
 * inline either, or the exact problem this whole file is designed
 * around (a client's own reply patience) reappears one level up.
 * Confirmed live, 2026-07-20: leaving the old synchronous
 * tuner_close_hold() call in front of tuner_try_claim() produced a real
 * "communication error" on the very first channel of a scan, when a
 * stale hold's refresher happened to be mid-slow-read at that moment. */
bool tuner_try_claim_fast(struct hdhr_tuner *t, int *out_stale_held_fd);

/* Finishes what tuner_try_claim_fast() deferred: stops (joins) the
 * stale held fd's background stats refresher if one was running, then
 * closes the fd. Safe to call with stale_held_fd < 0 (no-op). Meant to
 * be called from the caller's OWN background thread (e.g.
 * channel_scan_thread_main, as its very first action) -- calling this
 * synchronously on a connection thread defeats the entire point of
 * deferring it via tuner_try_claim_fast() in the first place. */
void tuner_resolve_stale_held_fd(struct hdhr_tuner *t, int stale_held_fd);

/* Records the dvb_stream now backing this (already-claimed) tuner, so
 * other code (e.g. status/streaminfo reporting) can see it. */
void tuner_set_stream(struct hdhr_tuner *t, struct dvb_stream *s);

/* Closes any active stream (safe to call with none active) and marks
 * the tuner idle again. If a channel is still logically selected
 * (tuned_frequency_hz != 0 -- i.e. wasn't cleared to "none" during the
 * stream) and nothing already re-established one in the meantime,
 * re-opens a background hold (see held_fd/tuner_open_hold()) so
 * /tunerN/status resumes reporting live stats immediately once
 * streaming ends, rather than going stale until the next explicit
 * channel/vchannel SET -- matching real hardware, which never stops
 * engaging a selected tuner's frontend just because a viewer left. */
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

/* Opens+tunes (non-blocking -- does not wait for lock) a background
 * "hold" frontend for freq_hz/delivery, closing any previous hold on
 * this tuner first, so /tunerN/status can report genuinely live
 * signal stats for a selected channel even when nothing is actively
 * streaming it -- matching real hardware's continuously-engaged-once-
 * selected behavior. See held_fd's own comment in tuner.h.
 *
 * Does nothing (silently) if this tuner currently has an active_stream
 * -- that stream's own frontend fd already serves live status (see
 * control.c's /tunerN/status handler), and opening a second fd on the
 * same physical frontend device node here would conflict with it. */
void tuner_open_hold(struct hdhr_tuner *t, uint32_t freq_hz, enum hdhr_delivery_system delivery);

/* Same end state as tuner_open_hold(), but takes ownership of an
 * already-open, already-tuned (and, in the common case, already-locked)
 * frontend fd directly instead of opening a fresh one and retuning from
 * scratch -- for a caller (control.c's channel_scan_thread_main) that
 * just finished its own dvb_scan_tune_and_lock() on this exact fd/
 * frequency and would otherwise be throwing away a perfectly good, live
 * lock only to immediately redo the same work. Confirmed live
 * (2026-07-21): the old close-then-tuner_open_hold() sequence meant the
 * driver had to briefly reacquire lock all over again on the fresh
 * retune, which /tunerN/status genuinely (not a placeholder) reported
 * as a real "lock=none ss=0" dip for several seconds before recovering
 * -- cosmetically harmless when it happened within the first second or
 * so after a SET (which real clients already expect and tolerate while
 * confirming initial lock), but a visible, confusing false "signal
 * lost" once it started happening several seconds *after* status had
 * already shown a solid, confirmed lock (a side effect of
 * channel_scan_thread_main's own pending_cond wait delaying when this
 * handoff happens -- see PENDING_WAIT_MS in control.c).
 *
 * Same active_stream/race-recheck semantics as tuner_open_hold() (see
 * its own comment) -- if either applies, this closes fd itself rather
 * than leaking it, same as the fresh-open path already did on its own
 * failure returns. */
void tuner_open_hold_from_fd(struct hdhr_tuner *t, int fd);

/* Closes any open hold without opening a new one -- e.g. an explicit
 * /tunerN/channel=none SET or a channelmap change. Safe to call with
 * no hold open. */
void tuner_close_hold(struct hdhr_tuner *t);

/* Scans for the first idle tuner and claims it. Returns NULL if all
 * tuners are currently busy. Used by HTTP's channel-only /auto/vX.X
 * path, which doesn't name a specific tuner slot — same auto-allocation
 * behavior real HDHomeRun firmware provides across its N tuners.
 * want_freq_hz/want_delivery/out_reused_fd are passed straight through
 * to each candidate's tuner_try_claim() — see its comment. */
struct hdhr_tuner *tuner_pool_claim_free(struct hdhr_tuner *tuners, int count,
                                           uint32_t want_freq_hz,
                                           enum hdhr_delivery_system want_delivery,
                                           int *out_reused_fd);

#endif /* HDHR_TUNER_H */
