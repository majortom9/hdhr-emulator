#ifndef HDHR_HTTP_SERVER_H
#define HDHR_HTTP_SERVER_H

#include "config.h"

/* Serves the HTTP surface real HDHomeRun firmware (and every modern DVR
 * client — Plex, Emby, Jellyfin, Channels DVR) actually talks to day to
 * day: discover.json, lineup.json, lineup_status.json, and the
 * /auto/v<channel> stream endpoint. A plain /auto/vX.X request doesn't
 * name a tuner slot, so it auto-allocates any currently-idle one via
 * tuner_pool_claim_free() (see tuner.h) — same arbitration real firmware
 * does internally across its N physical tuners, needed here because we
 * now own the DVB hardware directly instead of delegating to TVheadend.
 * Runs forever; launch in its own thread. `arg` must point to a
 * heap-allocated struct control_ctx (see control.h) that outlives it —
 * reused here rather than a bare struct hdhr_config* because HTTP pulls
 * need access to the same tuner pool control.c's target= pushes do. */
void *http_thread_main(void *arg);

#endif /* HDHR_HTTP_SERVER_H */
