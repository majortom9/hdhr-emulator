#ifndef HDHR_DISCOVERY_H
#define HDHR_DISCOVERY_H

#include "config.h"

/* Runs forever, servicing UDP broadcast discovery requests on port 65001.
 * Intended to be launched in its own thread. `arg` must point to a
 * struct hdhr_config that outlives the thread. */
void *discovery_thread_main(void *arg);

#endif /* HDHR_DISCOVERY_H */
