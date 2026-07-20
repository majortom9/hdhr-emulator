#ifndef HDHR_CHANNEL_MAP_H
#define HDHR_CHANNEL_MAP_H

#include <stdint.h>
#include <stdbool.h>

/* Channel-map definitions for everything *other* than "us-bcast" (which
 * stays on its own existing atsc_freq.c table/API — thoroughly
 * real-hardware validated, deliberately left untouched rather than
 * folded into this more general range-based system for the sake of
 * uniformity). Range/frequency data verified against a genuine copy of
 * Silicondust's libhdhomerun (hdhomerun_channels.c's
 * hdhomerun_channelmap_range_* tables) — protocol facts (channel
 * ranges, base frequencies, per-channel spacing), not copied code, same
 * clean-room approach as the rest of this project (see README.md).
 *
 * UNTESTED against real cable/Korean signal — see README.md's "What's
 * compile-verified vs. what needs real hardware" section. us-bcast
 * remains the only channel map this project has actually locked real
 * channels on.
 */

enum hdhr_delivery_system {
    HDHR_DELIVERY_ATSC_VSB, /* 8VSB terrestrial (US/KR broadcast) */
    HDHR_DELIVERY_QAM,      /* DVB-C Annex B (US/KR cable) */
};

struct channel_map_range {
    int      channel_start;
    int      channel_end;   /* inclusive */
    uint32_t base_freq_hz;  /* frequency of channel_start */
    uint32_t spacing_hz;    /* added per channel above channel_start */
};

struct channel_map_def {
    const char *name; /* e.g. "us-cable" — matches the /tunerN/channelmap wire value */
    const struct channel_map_range *ranges; /* {0,0,0,0}-terminated */
    enum hdhr_delivery_system delivery;
};

/* Looks up a channel map by its wire-format name (e.g. "us-cable").
 * Returns NULL for "us-bcast" (use atsc_freq.h for that one) or any
 * unrecognized name. */
const struct channel_map_def *channel_map_find(const char *name);

/* Resolves a channel number to a frequency within this map. Returns 0
 * if the channel number isn't covered by any of the map's ranges. */
uint32_t channel_map_channel_to_freq(const struct channel_map_def *map, int channel);

/* Reverse lookup: exact frequency match -> channel number. Returns 0
 * if freq_hz doesn't land exactly on a channel in this map (e.g. an
 * arbitrary frequency a client tuned directly, outside the map). */
int channel_map_freq_to_channel(const struct channel_map_def *map, uint32_t freq_hz);

/* Total channel count across all of a map's ranges — for bounding a
 * scan loop the same way ATSC_FREQ_TABLE_COUNT does for atsc_freq.c. */
int channel_map_count(const struct channel_map_def *map);

/* Nth channel (0-indexed, in range-table order) and its frequency, for
 * scan iteration without the caller needing to understand the
 * range/spacing structure itself. Returns false if index is out of
 * bounds. */
bool channel_map_nth(const struct channel_map_def *map, int index,
                      int *channel_out, uint32_t *freq_out);

#endif /* HDHR_CHANNEL_MAP_H */
