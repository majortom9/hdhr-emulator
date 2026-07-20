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
        pthread_mutex_init(&tuners[i].lock, NULL);
        pthread_cond_init(&tuners[i].free_cond, NULL);
        snprintf(tuners[i].channel, sizeof(tuners[i].channel), "none");
        snprintf(tuners[i].channelmap, sizeof(tuners[i].channelmap), "us-bcast");
        tuners[i].program = -1; /* DVB_PROGRAM_DEFAULT sentinel — see dvb_stream.h */
        snprintf(tuners[i].target, sizeof(tuners[i].target), "none");
        snprintf(tuners[i].status, sizeof(tuners[i].status), "state=idle");
    }
}

void tuner_lock(struct hdhr_tuner *t)   { pthread_mutex_lock(&t->lock); }
void tuner_unlock(struct hdhr_tuner *t) { pthread_mutex_unlock(&t->lock); }

bool tuner_try_claim(struct hdhr_tuner *t)
{
    tuner_lock(t);
    if (t->busy) {
        tuner_unlock(t);
        return false;
    }
    t->busy = true;
    tuner_unlock(t);
    return true;
}

bool tuner_try_claim_wait(struct hdhr_tuner *t, int timeout_ms)
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
    tuner_unlock(t);
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
    pthread_cond_signal(&t->free_cond); /* wake anyone in tuner_try_claim_wait() */
    tuner_unlock(t);

    if (s) dvb_stream_close(s); /* close outside the lock — it blocks on thread join */
}

struct hdhr_tuner *tuner_pool_claim_free(struct hdhr_tuner *tuners, int count)
{
    for (int i = 0; i < count; i++) {
        if (tuner_try_claim(&tuners[i])) return &tuners[i];
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
    /* real firmware's "ch=" is the RF tuning descriptor, not the virtual
     * channel — hdhomerun_config(_gui) displays this verbatim as
     * "Physical Channel". "8vsb:<freq_hz>" — confirmed against a genuine
     * HDHomeRun3's actual display (2026-07-18): it reports
     * modulation:frequency, not "us-bcast:<N>" (that channel-map-name
     * form is only the *input* syntax for the Channel selector control,
     * not what a real device echoes back as its tuned state). */
    snprintf(t->channel, sizeof(t->channel), "8vsb:%u", ch->frequency_hz);
    /* DVB_PROGRAM_DEFAULT (-1) means "no override" — report the program
     * actually being streamed (the channel's own), not the -1 sentinel,
     * since -1 isn't a value a client setting /tunerN/program would
     * ever send or expect to read back. */
    t->program = (program_override == DVB_PROGRAM_DEFAULT) ? ch->program_number : program_override;
    tuner_unlock(t);
}
