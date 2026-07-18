#include "dvb_channel.h"
#include <string.h>
#include <pthread.h>

static struct dvb_channel g_channels[DVB_CHANNEL_MAX];
static int g_count = 0;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

void dvb_channel_db_clear(void)
{
    pthread_mutex_lock(&g_lock);
    g_count = 0;
    pthread_mutex_unlock(&g_lock);
}

bool dvb_channel_db_add(const struct dvb_channel *ch)
{
    pthread_mutex_lock(&g_lock);

    for (int i = 0; i < g_count; i++) {
        if (g_channels[i].major == ch->major && g_channels[i].minor == ch->minor) {
            g_channels[i] = *ch; /* update in place — e.g. a re-scan found new PIDs/name */
            pthread_mutex_unlock(&g_lock);
            return true;
        }
    }

    bool ok = g_count < DVB_CHANNEL_MAX;
    if (ok) {
        g_channels[g_count++] = *ch;
    }
    pthread_mutex_unlock(&g_lock);
    return ok;
}

int dvb_channel_count(void)
{
    pthread_mutex_lock(&g_lock);
    int n = g_count;
    pthread_mutex_unlock(&g_lock);
    return n;
}

const struct dvb_channel *dvb_channel_at(int i)
{
    /* Not lock-protected on the read of the returned pointer's contents —
     * matches the same caller contract tvh_channel_at() had (read-only
     * access from HTTP/control threads, refreshes are infrequent full
     * rescans via dvb_channel_db_clear + re-adds). */
    if (i < 0 || i >= g_count) return NULL;
    return &g_channels[i];
}

const struct dvb_channel *dvb_find_channel(int major, int minor)
{
    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < g_count; i++) {
        if (g_channels[i].major == major && g_channels[i].minor == minor) {
            pthread_mutex_unlock(&g_lock);
            return &g_channels[i];
        }
    }
    pthread_mutex_unlock(&g_lock);
    return NULL;
}

int dvb_channels_on_freq(uint32_t frequency_hz, const struct dvb_channel **out, int max_out)
{
    int n = 0;
    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < g_count && n < max_out; i++) {
        if (g_channels[i].frequency_hz == frequency_hz) {
            out[n++] = &g_channels[i];
        }
    }
    pthread_mutex_unlock(&g_lock);
    return n;
}
