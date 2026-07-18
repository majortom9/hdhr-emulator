#ifndef HDHR_UDP_STREAM_H
#define HDHR_UDP_STREAM_H

#include "config.h"
#include "tuner.h"

/* Starts pushing the tuner's currently-resolved virtual channel to
 * ip:port as raw UDP datagrams, 7 x 188-byte MPEG-TS packets per
 * datagram (1316 bytes) — the classic packing used by early HDHomeRun
 * "target=" pushes, chosen to stay under Ethernet MTU with room for
 * IP/UDP headers. Stops any push already running on this tuner first.
 * Returns 0 on success (thread launched), -1 if the tuner has no
 * resolved channel, the tuner's physical hardware is already claimed by
 * another stream (see tuner.h), or the destination can't be reached. */
int udp_push_start(const struct hdhr_config *cfg, struct hdhr_tuner *t,
                    const char *dest_ip, int dest_port);

/* Idempotent; safe to call even if nothing is running. Blocks until the
 * push thread has actually stopped. */
void udp_push_stop(struct hdhr_tuner *t);

#endif /* HDHR_UDP_STREAM_H */
