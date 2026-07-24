#include "dvb_channel.h"
#include <string.h>
#include <stdio.h>
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

bool dvb_channel_db_save(const char *path)
{
    if (!path || path[0] == '\0') return false;

    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "dvb_channel: failed to write cache file %s\n", path);
        return false;
    }

    fprintf(f, "# hdhr-emulator channel cache -- generated, do not edit by hand.\n"
               "# major minor rf_channel delivery frequency_hz channel_tsid "
               "program_number pmt_pid pcr_pid video_pid video_stream_type "
               "audio_pid audio_stream_type short_name\n");

    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < g_count; i++) {
        const struct dvb_channel *ch = &g_channels[i];
        fprintf(f, "%d %d %d %d %u %u %u %u %u %u %u %u %u %s\n",
                ch->major, ch->minor, ch->rf_channel, (int)ch->delivery,
                ch->frequency_hz, (unsigned)ch->channel_tsid,
                (unsigned)ch->program_number, (unsigned)ch->pmt_pid,
                (unsigned)ch->pcr_pid, (unsigned)ch->video_pid,
                (unsigned)ch->video_stream_type, (unsigned)ch->audio_pid,
                (unsigned)ch->audio_stream_type, ch->short_name);
    }
    int n = g_count;
    pthread_mutex_unlock(&g_lock);

    fclose(f);
    fprintf(stderr, "dvb_channel: saved %d channel(s) to cache %s\n", n, path);
    return true;
}

int dvb_channel_db_load(const char *path)
{
    if (!path || path[0] == '\0') return 0;

    FILE *f = fopen(path, "r");
    if (!f) return 0; /* nothing cached yet (e.g. first-ever run) -- not an error */

    char line[256];
    int n = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\0') continue;

        struct dvb_channel ch;
        memset(&ch, 0, sizeof(ch));
        int delivery_i = 0;
        int consumed = 0;

        /* Fixed numeric fields first, short_name last (rest of the
         * line) since it's the only field that can contain a space
         * (real ATSC short_names sometimes do) -- see the matching
         * write order in dvb_channel_db_save(). %n gives us the byte
         * offset right after the last numeric field so we can grab
         * everything after it verbatim. */
        int got = sscanf(line, "%d %d %d %d %u %hu %hu %hu %hu %hu %hhu %hu %hhu %n",
                          &ch.major, &ch.minor, &ch.rf_channel, &delivery_i,
                          &ch.frequency_hz, &ch.channel_tsid, &ch.program_number,
                          &ch.pmt_pid, &ch.pcr_pid, &ch.video_pid,
                          &ch.video_stream_type, &ch.audio_pid, &ch.audio_stream_type,
                          &consumed);
        if (got != 13) {
            fprintf(stderr, "dvb_channel: malformed cache line, aborting load: %s", line);
            fclose(f);
            return -1;
        }
        ch.delivery = (enum hdhr_delivery_system)delivery_i;

        char *name = line + consumed;
        while (*name == ' ') name++;
        size_t len = strlen(name);
        while (len > 0 && (name[len-1] == '\n' || name[len-1] == '\r')) name[--len] = '\0';
        snprintf(ch.short_name, sizeof(ch.short_name), "%s", name);

        if (dvb_channel_db_add(&ch)) n++;
    }

    fclose(f);
    fprintf(stderr, "dvb_channel: loaded %d channel(s) from cache %s\n", n, path);
    return n;
}
