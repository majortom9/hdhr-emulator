#ifndef HDHR_DVB_CHANNEL_H
#define HDHR_DVB_CHANNEL_H

#include <stdint.h>
#include <stdbool.h>
#include "channel_map.h"

#define DVB_CHANNEL_MAX 256

struct dvb_channel {
    int      major;
    int      minor;
    char     short_name[16];
    int      rf_channel;    /* physical RF channel number, for the
                              * "<channelmap>:N" format real firmware uses
                              * in /tunerN/channel — see tuner_bind_channel() */
    enum hdhr_delivery_system delivery; /* HDHR_DELIVERY_ATSC_VSB or
                                          * HDHR_DELIVERY_QAM — which tune
                                          * function dvb_stream_open() must
                                          * use to actually stream this
                                          * channel later, since a stream
                                          * open only gets a dvb_channel
                                          * pointer, not the channelmap it
                                          * was scanned under. Set once at
                                          * scan time (dvb_scan.c) and
                                          * never changes for a given
                                          * major.minor. */
    uint32_t frequency_hz;
    uint16_t channel_tsid;  /* transport_stream_id, for streaminfo's "tsid=" line */
    uint16_t program_number;
    uint16_t pmt_pid;
    uint16_t pcr_pid;
    uint16_t video_pid;
    uint8_t  video_stream_type;
    uint16_t audio_pid;      /* 0 if none found */
    uint8_t  audio_stream_type;
};

void dvb_channel_db_clear(void);
/* Adds a channel, or updates the existing entry in place if one with the
 * same major.minor already exists (e.g. from a prior full scan or an
 * earlier on-demand dvb_scan_frequency() call) — without this, repeated
 * scans would accumulate duplicate rows over the life of the daemon.
 * Returns false only if the table is full AND this is a genuinely new
 * major.minor (an update to an existing entry always succeeds). */
bool dvb_channel_db_add(const struct dvb_channel *ch);

int dvb_channel_count(void);
const struct dvb_channel *dvb_channel_at(int i);
const struct dvb_channel *dvb_find_channel(int major, int minor);

/* Fills out[] with every channel sharing the given mux frequency (used to
 * resolve a /tunerN/program override to a sibling subchannel's PIDs
 * without a fresh scan). Returns the count written (capped at max_out). */
int dvb_channels_on_freq(uint32_t frequency_hz, const struct dvb_channel **out, int max_out);

/* Persists the current channel db to a plain-text cache file (one line
 * per channel, all fields the daemon actually needs to stream that
 * channel again -- unlike lineup.json, which only has GuideNumber/
 * GuideName/URL and can't reconstruct a tunable dvb_channel). Used so a
 * restart can serve a known-good lineup immediately instead of the
 * usual empty-until-first-scan-completes window; see dvb_channel_db_load().
 * Overwrites path unconditionally. Returns false on any I/O error (path
 * empty/NULL is treated as "caching disabled", not an error). */
bool dvb_channel_db_save(const char *path);

/* Loads a cache file written by dvb_channel_db_save(), upserting each
 * entry via dvb_channel_db_add() (so calling this into a non-empty db
 * updates in place rather than duplicating). Meant to be called once at
 * startup, before the background full scan begins -- the scan will
 * overwrite stale entries and drop ones that no longer exist once it
 * completes. Missing file or empty/NULL path is not an error (nothing to
 * load yet, e.g. first-ever run). Returns the number of channels loaded,
 * or -1 on a malformed file. */
int dvb_channel_db_load(const char *path);

#endif /* HDHR_DVB_CHANNEL_H */
