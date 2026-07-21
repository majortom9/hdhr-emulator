#include "tuner.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>

void tuner_pool_init(struct hdhr_tuner *tuners, int count, const struct hdhr_config *cfg)
{
    for (int i = 0; i < count; i++) {
        memset(&tuners[i], 0, sizeof(tuners[i]));
        tuners[i].index = i;
        tuners[i].adapter = cfg->dvb_adapter[i];
        tuners[i].frontend = cfg->dvb_frontend;
        tuners[i].held_fd = -1;
        pthread_mutex_init(&tuners[i].lock, NULL);
        pthread_cond_init(&tuners[i].free_cond, NULL);
        snprintf(tuners[i].channel, sizeof(tuners[i].channel), "none");
        snprintf(tuners[i].channelmap, sizeof(tuners[i].channelmap), "us-bcast");
        tuners[i].program = -1; /* DVB_PROGRAM_DEFAULT sentinel — see dvb_stream.h */
        snprintf(tuners[i].target, sizeof(tuners[i].target), "none");
        /* Matches a genuine HDHomeRun3's own idle /tunerN/status verbatim
         * (confirmed live, 2026-07-20: "ch=none lock=none ss=0 snq=0
         * seq=0 bps=0 pps=0" on an unselected tuner) — not just
         * cosmetic: libhdhomerun's hdhomerun_device_get_tuner_status()
         * parses status->channel out of this string's "ch=" token and
         * leaves it as an empty string if "ch=" is absent (status is
         * memset to 0 first, and there's no separate fallback), so a
         * placeholder without "ch=" at all (this used to be the bare
         * "state=idle") made hdhomerun_config_gui's "Physical Channel"
         * field show blank instead of "none" once a channel was
         * deselected. */
        snprintf(tuners[i].status, sizeof(tuners[i].status),
                 "ch=none lock=none ss=0 snq=0 seq=0 bps=0 pps=0");
    }
}

void tuner_lock(struct hdhr_tuner *t)   { pthread_mutex_lock(&t->lock); }
void tuner_unlock(struct hdhr_tuner *t) { pthread_mutex_unlock(&t->lock); }

/* Shared by tuner_try_claim()/tuner_try_claim_wait() once busy has been
 * claimed and held_fd/tuned_frequency_hz/tuned_delivery captured under
 * the lock — resolves the captured held fd against the caller's wanted
 * freq/delivery, either handing it back via *out_reused_fd (ownership
 * transferred, do NOT close) or closing it, per tuner_try_claim()'s own
 * doc comment. Takes the tuner's tuned_freq_hz/tuned_delivery as params
 * (rather than reading t-> directly) since it runs outside the lock
 * (dvb_frontend_close() can block) and those fields aren't otherwise
 * guaranteed stable once busy is claimed. */
static void resolve_claimed_hold(int held, uint32_t tuned_freq_hz,
                                   enum hdhr_delivery_system tuned_delivery,
                                   uint32_t want_freq_hz, enum hdhr_delivery_system want_delivery,
                                   int *out_reused_fd)
{
    bool reuse = (held >= 0 && out_reused_fd &&
                  tuned_freq_hz == want_freq_hz && tuned_delivery == want_delivery);
    if (reuse) {
        *out_reused_fd = held;
        return;
    }
    if (out_reused_fd) *out_reused_fd = -1;
    if (held >= 0) dvb_frontend_close(held); /* see held_fd's own comment: never
                                                * concurrent with an active_stream's fd */
}

bool tuner_try_claim(struct hdhr_tuner *t, uint32_t want_freq_hz,
                      enum hdhr_delivery_system want_delivery, int *out_reused_fd)
{
    tuner_lock(t);
    if (t->busy) {
        tuner_unlock(t);
        return false;
    }
    t->busy = true;
    int held = t->held_fd;
    t->held_fd = -1;
    uint32_t tuned_freq_hz = t->tuned_frequency_hz;
    enum hdhr_delivery_system tuned_delivery = t->tuned_delivery;
    tuner_unlock(t);

    resolve_claimed_hold(held, tuned_freq_hz, tuned_delivery, want_freq_hz, want_delivery, out_reused_fd);
    return true;
}

bool tuner_try_claim_wait(struct hdhr_tuner *t, int timeout_ms, uint32_t want_freq_hz,
                           enum hdhr_delivery_system want_delivery, int *out_reused_fd)
{
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_nsec -= 1000000000L;
        deadline.tv_sec += 1;
    }

    tuner_lock(t);
    while (t->busy) {
        if (pthread_cond_timedwait(&t->free_cond, &t->lock, &deadline) == ETIMEDOUT) {
            break; /* recheck t->busy below rather than trusting the return code alone —
                     * cond_timedwait always re-locks before returning, and the tuner
                     * could've freed on the very same tick as the timeout */
        }
    }
    if (t->busy) {
        tuner_unlock(t);
        return false;
    }
    t->busy = true;
    int held = t->held_fd;
    t->held_fd = -1;
    uint32_t tuned_freq_hz = t->tuned_frequency_hz;
    enum hdhr_delivery_system tuned_delivery = t->tuned_delivery;
    tuner_unlock(t);

    resolve_claimed_hold(held, tuned_freq_hz, tuned_delivery, want_freq_hz, want_delivery, out_reused_fd);
    return true;
}

void tuner_set_stream(struct hdhr_tuner *t, struct dvb_stream *s)
{
    tuner_lock(t);
    t->active_stream = s;
    tuner_unlock(t);
}

void tuner_release(struct hdhr_tuner *t)
{
    tuner_lock(t);
    struct dvb_stream *s = t->active_stream;
    t->active_stream = NULL;
    t->busy = false;
    uint32_t freq = t->tuned_frequency_hz;
    enum hdhr_delivery_system delivery = t->tuned_delivery;
    bool already_held = (t->held_fd >= 0);
    pthread_cond_signal(&t->free_cond); /* wake anyone in tuner_try_claim_wait() */
    tuner_unlock(t);

    if (s) dvb_stream_close(s); /* close outside the lock — it blocks on thread join */

    /* Re-establish a background hold so /tunerN/status keeps reporting
     * genuinely live stats even after streaming stops, matching real
     * hardware's continuously-engaged-once-selected behavior (see
     * held_fd's own comment) -- but only if a channel is still
     * logically selected (freq != 0, i.e. wasn't cleared to "none"
     * during the stream) and nothing already re-established one in
     * the meantime (e.g. a fresh channel/vchannel SET that arrived
     * while this stream was still winding down). */
    if (freq != 0 && !already_held) {
        tuner_open_hold(t, freq, delivery);
    }
}

void tuner_open_hold(struct hdhr_tuner *t, uint32_t freq_hz, enum hdhr_delivery_system delivery)
{
    tuner_lock(t);
    if (t->active_stream) {
        /* Someone's actively streaming this tuner already -- its own
         * frontend fd already serves live status, and opening a second
         * fd on the same physical frontend device node here would
         * conflict with it. */
        tuner_unlock(t);
        return;
    }
    int old_held = t->held_fd;
    t->held_fd = -1;
    tuner_unlock(t);
    if (old_held >= 0) dvb_frontend_close(old_held);

    int fd = dvb_frontend_open(t->adapter, t->frontend);
    if (fd < 0) return;

    int rc = (delivery == HDHR_DELIVERY_QAM) ? dvb_frontend_tune_qam(fd, freq_hz)
                                              : dvb_frontend_tune_8vsb(fd, freq_hz);
    if (rc != 0) {
        dvb_frontend_close(fd);
        return;
    }

    tuner_lock(t);
    /* Recheck -- another thread could have raced in (started actively
     * streaming, or opened its own hold) while we were opening this fd
     * outside the lock. */
    if (t->active_stream || t->held_fd >= 0) {
        tuner_unlock(t);
        dvb_frontend_close(fd);
        return;
    }
    t->held_fd = fd;
    t->held_legacy_seq = (struct dvb_legacy_seq_state){0};
    tuner_unlock(t);
}

void tuner_close_hold(struct hdhr_tuner *t)
{
    tuner_lock(t);
    int fd = t->held_fd;
    t->held_fd = -1;
    tuner_unlock(t);
    if (fd >= 0) dvb_frontend_close(fd);
}

struct hdhr_tuner *tuner_pool_claim_free(struct hdhr_tuner *tuners, int count,
                                           uint32_t want_freq_hz,
                                           enum hdhr_delivery_system want_delivery,
                                           int *out_reused_fd)
{
    for (int i = 0; i < count; i++) {
        if (tuner_try_claim(&tuners[i], want_freq_hz, want_delivery, out_reused_fd)) return &tuners[i];
    }
    return NULL;
}

void tuner_bind_channel(struct hdhr_tuner *t, const struct dvb_channel *ch, int program_override)
{
    tuner_lock(t);
    t->vch_major = ch->major;
    t->vch_minor = ch->minor;
    t->vch_resolved = true;
    t->tuned_frequency_hz = ch->frequency_hz;
    t->tuned_delivery = ch->delivery;
    /* real firmware's "ch=" is the RF tuning descriptor, not the virtual
     * channel — hdhomerun_config(_gui) displays this verbatim as
     * "Physical Channel". "8vsb:<freq_hz>" — confirmed against a genuine
     * HDHomeRun3's actual display (2026-07-18): it reports
     * modulation:frequency, not "us-bcast:<N>" (that channel-map-name
     * form is only the *input* syntax for the Channel selector control,
     * not what a real device echoes back as its tuned state). "qam:" for
     * QAM channels is this project's own best guess at the equivalent,
     * matching the same convention control.c's raw /tunerN/channel SET
     * already uses -- UNTESTED, no real cable-tuned device available to
     * confirm the exact modulation string against (see channel_map.h).
     * This path (vchannel select / streaming bind) previously always
     * said "8vsb" regardless of ch->delivery -- a gap from when QAM
     * support was added, since this function predates that work and
     * wasn't updated at the time. */
    snprintf(t->channel, sizeof(t->channel), "%s:%u",
             ch->delivery == HDHR_DELIVERY_QAM ? "qam" : "8vsb", ch->frequency_hz);
    /* DVB_PROGRAM_DEFAULT (-1) means "no override" — report the program
     * actually being streamed (the channel's own), not the -1 sentinel,
     * since -1 isn't a value a client setting /tunerN/program would
     * ever send or expect to read back. */
    t->program = (program_override == DVB_PROGRAM_DEFAULT) ? ch->program_number : program_override;
    tuner_unlock(t);
}
