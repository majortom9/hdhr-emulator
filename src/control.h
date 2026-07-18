#ifndef HDHR_CONTROL_H
#define HDHR_CONTROL_H

#include "config.h"
#include "tuner.h"

struct control_ctx {
    struct hdhr_config *cfg;
    struct hdhr_tuner  *tuners; /* array of cfg->tuner_count */
};

/* Runs forever, accepting TCP control connections on port 65001 and
 * servicing GETSET_REQ frames. Intended to be launched in its own
 * thread; `arg` must point to a heap-allocated struct control_ctx that
 * outlives the thread (main.c owns it). */
void *control_thread_main(void *arg);

#endif /* HDHR_CONTROL_H */
