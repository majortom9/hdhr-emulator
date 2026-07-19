#ifndef HDHR_KEEPALIVE_H
#define HDHR_KEEPALIVE_H

#include <stdbool.h>
#include "tuner.h"

/* Starts a detached background thread that listens on UDP port 5004
 * for client keepalive packets — real libhdhomerun clients send one
 * every ~1s while a target= stream is active
 * (hdhomerun_video_thread_send_keepalive() in hdhomerun_video.c) — and
 * reclaims (stops) any tuner's target= push that hasn't heard a
 * matching one in too long. UDP has no inherent "connection dropped"
 * signal the way TCP does, so without this, a client that dies
 * uncleanly (crash, kill -9, network drop, or anything else that skips
 * an explicit target=none) leaves that tuner locked to a dead
 * destination forever. Returns false on socket setup failure (port
 * already in use, etc) — the daemon still runs without it, just
 * without abandoned-stream protection. */
bool keepalive_listener_start(struct hdhr_tuner *tuners, int tuner_count);

#endif /* HDHR_KEEPALIVE_H */
