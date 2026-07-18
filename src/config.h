#ifndef HDHR_CONFIG_H
#define HDHR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define HDHR_MAX_TUNERS 8

struct hdhr_config {
    uint32_t device_id;             /* 32-bit device id advertised in discovery      */
    char     friendly_name[64];     /* cosmetic, shown in some clients                */
    char     model[32];             /* e.g. "HDHR3-US" — early Gen3 ATSC model        */
    char     firmware_name[32];     /* e.g. "hdhomerun3_atsc"                         */
    char     firmware_version[32];  /* advertised firmware version string             */
    int      tuner_count;           /* number of tuners to advertise (<= HDHR_MAX_TUNERS) */
    char     bind_ip[16];           /* dotted-quad IP this box advertises to clients  */

    /* DVB backend — each hdhr_tuner slot i maps to physical adapter
     * dvb_adapter[i] (default: i itself, i.e. tuner0 -> adapter0,
     * tuner1 -> adapter1, ...). frontend/demux/dvr numbers within an
     * adapter default to 0, which covers essentially every consumer USB
     * ATSC tuner (single frontend, single demux). */
    int      dvb_adapter[HDHR_MAX_TUNERS];
    int      dvb_frontend;          /* frontend number within each adapter, default 0 */
    int      dvb_demux;             /* demux number within each adapter, default 0    */
    bool     scan_on_startup;       /* default true */
    bool     debug_signal_stats;    /* default false — see dvb_frontend.h */
};

void config_defaults(struct hdhr_config *cfg);

/* Minimal "key=value" file loader, one setting per line, '#' comments.
 * Returns 0 on success (including "file not found", in which case
 * defaults are left in place), -1 on a malformed file. */
int config_load(const char *path, struct hdhr_config *cfg);

#endif /* HDHR_CONFIG_H */
