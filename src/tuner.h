#ifndef HDHR_TUNER_H
#define HDHR_TUNER_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "config.h"
#include "dvb_stream.h"

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

    bool     busy;
    struct dvb_stream *active_stream;

    /* set by udp_stream.c while a target= push is active, so control.c
     * knows whether to stop a previous push before starting a new one */
    volatile int udp_push_active;
    volatile int udp_push_stop_requested;
    pthread_t    udp_push_thread;
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
