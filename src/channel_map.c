#include "channel_map.h"
#include <string.h>

/* US cable channels (Standard/EIA-542). */
static const struct channel_map_range range_us_cable[] = {
    {  2,   4,  57000000, 6000000},
    {  5,   6,  79000000, 6000000},
    {  7,  13, 177000000, 6000000},
    { 14,  22, 123000000, 6000000},
    { 23,  94, 219000000, 6000000},
    { 95,  99,  93000000, 6000000},
    {100, 158, 651000000, 6000000},
    {  0,   0,         0,       0},
};

/* US cable channels (HRC — harmonically related carrier). */
static const struct channel_map_range range_us_hrc[] = {
    {  2,   4,  55752700, 6000300},
    {  5,   6,  79753900, 6000300},
    {  7,  13, 175758700, 6000300},
    { 14,  22, 121756000, 6000300},
    { 23,  94, 217760800, 6000300},
    { 95,  99,  91754500, 6000300},
    {100, 158, 649782400, 6000300},
    {  0,   0,         0,       0},
};

/* US cable channels (IRC — incrementally related carrier). */
static const struct channel_map_range range_us_irc[] = {
    {  2,   4,  57012500, 6000000},
    {  5,   6,  81012500, 6000000},
    {  7,  13, 177012500, 6000000},
    { 14,  22, 123012500, 6000000},
    { 23,  41, 219012500, 6000000},
    { 42,  42, 333025000, 6000000},
    { 43,  94, 339012500, 6000000},
    { 95,  97,  93012500, 6000000},
    { 98,  99, 111025000, 6000000},
    {100, 158, 651012500, 6000000},
    {  0,   0,         0,       0},
};

/* KR cable channels. */
static const struct channel_map_range range_kr_cable[] = {
    {  2,   4,  57000000, 6000000},
    {  5,   6,  79000000, 6000000},
    {  7,  13, 177000000, 6000000},
    { 14,  22, 123000000, 6000000},
    { 23, 153, 219000000, 6000000},
    {  0,   0,         0,       0},
};

/* KR broadcast channels — same range as US broadcast (kr-bcast points at
 * the identical table in libhdhomerun too, not a distinct Korean plan). */
static const struct channel_map_range range_kr_bcast[] = {
    {  2,   4,  57000000, 6000000},
    {  5,   6,  79000000, 6000000},
    {  7,  13, 177000000, 6000000},
    { 14,  36, 473000000, 6000000},
    {  0,   0,         0,       0},
};

static const struct channel_map_def maps[] = {
    {"us-cable", range_us_cable, HDHR_DELIVERY_QAM},
    {"us-hrc",   range_us_hrc,   HDHR_DELIVERY_QAM},
    {"us-irc",   range_us_irc,   HDHR_DELIVERY_QAM},
    {"kr-cable", range_kr_cable, HDHR_DELIVERY_QAM},
    {"kr-bcast", range_kr_bcast, HDHR_DELIVERY_ATSC_VSB},
};
#define MAP_COUNT (sizeof(maps) / sizeof(maps[0]))

const struct channel_map_def *channel_map_find(const char *name)
{
    for (size_t i = 0; i < MAP_COUNT; i++) {
        if (strcmp(maps[i].name, name) == 0) return &maps[i];
    }
    return NULL;
}

uint32_t channel_map_channel_to_freq(const struct channel_map_def *map, int channel)
{
    for (const struct channel_map_range *r = map->ranges; r->channel_end != 0; r++) {
        if (channel >= r->channel_start && channel <= r->channel_end) {
            return r->base_freq_hz + (uint32_t)(channel - r->channel_start) * r->spacing_hz;
        }
    }
    return 0;
}

int channel_map_freq_to_channel(const struct channel_map_def *map, uint32_t freq_hz)
{
    for (const struct channel_map_range *r = map->ranges; r->channel_end != 0; r++) {
        for (int ch = r->channel_start; ch <= r->channel_end; ch++) {
            if (r->base_freq_hz + (uint32_t)(ch - r->channel_start) * r->spacing_hz == freq_hz) {
                return ch;
            }
        }
    }
    return 0;
}

int channel_map_count(const struct channel_map_def *map)
{
    int total = 0;
    for (const struct channel_map_range *r = map->ranges; r->channel_end != 0; r++) {
        total += r->channel_end - r->channel_start + 1;
    }
    return total;
}

bool channel_map_nth(const struct channel_map_def *map, int index, int *channel_out, uint32_t *freq_out)
{
    if (index < 0) return false;
    for (const struct channel_map_range *r = map->ranges; r->channel_end != 0; r++) {
        int span = r->channel_end - r->channel_start + 1;
        if (index < span) {
            int channel = r->channel_start + index;
            *channel_out = channel;
            *freq_out = r->base_freq_hz + (uint32_t)(channel - r->channel_start) * r->spacing_hz;
            return true;
        }
        index -= span;
    }
    return false;
}
