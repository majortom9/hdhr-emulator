/*
 * main.c — hdhr-emulator: makes a USB ATSC tuner look and behave like an
 * early-Gen3 HDHomeRun on the wire (UDP discovery, TCP control, and
 * HTTP + UDP/RTP-unicast data planes), driving the tuner directly via
 * the Linux DVB API (no TVheadend).
 *
 * Usage: hdhr-emulator [config-file]   (default: /etc/hdhr-emulator.conf)
 *        hdhr-emulator --gen-device-id
 *            Prints one freshly-generated, checksum-valid device ID and
 *            exits — no config file, no sockets. Paste the result into
 *            device_id= in your config so it stays fixed across restarts
 *            (otherwise a new random-but-valid one is picked every start,
 *            which churns DVR pairing in Plex/Emby/etc).
 *
 * Needs to bind ports 65001 (udp+tcp) and 80 (tcp), so run as root or
 * grant CAP_NET_BIND_SERVICE:
 *   setcap 'cap_net_bind_service=+ep' ./hdhr-emulator
 *
 * Also needs read/write access to each /dev/dvb/adapterN's frontend,
 * demux, and dvr device nodes — either run as root, or add the user to
 * the "video" group (or whichever group owns those device nodes on your
 * distro) and grant CAP_NET_BIND_SERVICE separately as above.
 */
#include "config.h"
#include "device_id.h"
#include "tuner.h"
#include "discovery.h"
#include "control.h"
#include "http_server.h"
#include "keepalive.h"
#include "dvb_scan.h"
#include "dvb_frontend.h"
#include "dvb_channel.h"
#include "atsc_freq.h"
#include "mpeg_section.h"
#include "psip.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>

static struct hdhr_config g_cfg;
static struct hdhr_tuner  g_tuners[HDHR_MAX_TUNERS];

/* How long a scan step waits to claim tuner0 before skipping that
 * frequency for this pass — see the per-frequency claiming note below. */
#define SCAN_STEP_CLAIM_WAIT_MS 5000

/* Runs the initial (and, if scan_on_startup fires again later via a
 * future admin action, any subsequent) full ATSC scan in the
 * background so discovery/control/HTTP come up immediately rather than
 * blocking on a ~1-2 minute scan. Uses tuner 0's adapter/frontend/demux
 * — one physical tuner briefly does the scanning work for the whole
 * box, since the channel database is shared across all tuner slots.
 *
 * Claims tuner0 ONCE for the entire scan and holds the frontend/PAT/VCT
 * fds open the whole time, retuning in place -- matching every
 * reference scanning tool checked against (dvbv5-scan and atscdx, both
 * libdvbv5-based, and w_scan2/scan-s2, which uses raw ioctls like this
 * project does): all three open their frontend once before the scan
 * loop and only close it after the last frequency. An earlier version
 * of this function released the claim between every frequency instead,
 * on the theory that it gave another consumer (an HTTP pull, a manual
 * /tunerN/channel SET) a periodic window to grab tuner0 during the
 * scan rather than being locked out for its full ~1-2 minutes. That
 * turned out not to deliver what it promised: a DVB frontend device
 * can only have one open fd at all, at the kernel level, regardless of
 * our own claim bookkeeping -- so a waiting consumer could only ever
 * actually succeed in the brief instant our fd was genuinely closed
 * between frequencies, not just whenever the claim flag said "free".
 * Paying a full close+reopen (and the chip re-init that goes with it,
 * confirmed live via dmesg: lgdt3306a_init/si2157_init on nearly every
 * retune) per frequency for a narrow, mostly-illusory window wasn't
 * worth it. Now tuner0 is genuinely unavailable to other consumers for
 * the whole scan, same as it would be on any of the reference tools
 * above -- they simply accept that as normal for a dedicated scan. */
static void *scan_thread_main(void *arg)
{
    (void)arg;
    /* Deliberately does NOT clear the db first -- dvb_channel_db_add()
     * already upserts by major.minor (same as the CLI-driven rescan
     * path in control.c's channel_scan_thread_main, which never clears
     * either), so a channel found again here just updates in place.
     * Clearing up front used to be harmless (nothing persisted across
     * a restart anyway), but now that dvb_channel_db_load() may have
     * just populated the db from a cache file a few milliseconds ago,
     * clearing here raced it every time and won essentially always --
     * confirmed live: lineup.json read immediately after a restart came
     * back with 0 channels despite the log showing "loaded 75
     * channel(s) from cache", since this line wiped them out again
     * before the scan had found a single one of its own. Not clearing
     * means a channel that's gone off-air since the cache was last
     * saved lingers until a fresh manual rescan notices it's gone --
     * the same already-accepted limitation as the CLI-scan path. */
    fprintf(stderr, "dvb_scan: starting full scan on adapter%d (this takes a couple minutes)\n",
             g_cfg.dvb_adapter[0]);

    int total = 0;
    if (!tuner_try_claim_wait(&g_tuners[0], SCAN_STEP_CLAIM_WAIT_MS, 0, HDHR_DELIVERY_ATSC_VSB, NULL)) {
        fprintf(stderr, "dvb_scan: tuner0 busy, skipping startup scan entirely\n");
        return NULL;
    }

    int ffd = dvb_frontend_open(g_cfg.dvb_adapter[0], g_cfg.dvb_frontend);
    int pat_fd = (ffd >= 0) ? mpeg_section_filter_open(g_cfg.dvb_adapter[0], g_cfg.dvb_demux,
                                                         0x0000, 0x00, 2000) : -1;
    int vct_fd = (ffd >= 0) ? mpeg_section_filter_open(g_cfg.dvb_adapter[0], g_cfg.dvb_demux,
                                                         PSIP_BASE_PID, 0xC8, 2000) : -1;

    if (ffd >= 0 && pat_fd >= 0 && vct_fd >= 0) {
        for (int i = 0; i < ATSC_FREQ_TABLE_COUNT; i++) {
            if (dvb_scan_tune_and_lock(ffd, atsc_freq_table[i].frequency_hz,
                                        HDHR_DELIVERY_ATSC_VSB, NULL, NULL)) {
                total += dvb_scan_read_psip(g_cfg.dvb_adapter[0], g_cfg.dvb_demux,
                                             pat_fd, vct_fd,
                                             atsc_freq_table[i].channel,
                                             atsc_freq_table[i].frequency_hz,
                                             HDHR_DELIVERY_ATSC_VSB);
            }
        }
    } else {
        fprintf(stderr, "dvb_scan: failed to open frontend/demux for startup scan\n");
    }

    if (pat_fd >= 0) mpeg_section_filter_close(pat_fd);
    if (vct_fd >= 0) mpeg_section_filter_close(vct_fd);
    if (ffd >= 0) dvb_frontend_close(ffd);
    tuner_release(&g_tuners[0]);

    fprintf(stderr, "dvb_scan: complete — %d virtual channel(s) found\n", total);
    dvb_channel_db_save(g_cfg.channel_cache_file);
    return NULL;
}

int main(int argc, char **argv)
{
    if (argc > 1 && (strcmp(argv[1], "--gen-device-id") == 0 || strcmp(argv[1], "-g") == 0)) {
        /* Seed from more than just time(NULL) so back-to-back invocations
         * in the same second (e.g. scripted) still differ. */
        uint32_t seed = ((uint32_t)time(NULL) ^ ((uint32_t)getpid() << 16)) & 0x00FFFFFFu;
        uint32_t id = hdhr_device_id_generate(seed);
        printf("0x%08X\n", id);
        return 0;
    }

    signal(SIGPIPE, SIG_IGN); /* client disconnects mid-stream shouldn't kill us */

    const char *config_path = (argc > 1) ? argv[1] : "/etc/hdhr-emulator.conf";
    config_defaults(&g_cfg);
    if (config_load(config_path, &g_cfg) != 0) {
        fprintf(stderr, "main: failed to parse %s\n", config_path);
        return 1;
    }
    dvb_frontend_set_debug(g_cfg.debug_signal_stats);

    if (!hdhr_device_id_is_valid(g_cfg.device_id)) {
        fprintf(stderr,
            "main: warning: configured device_id 0x%08X fails the HDHomeRun "
            "checksum — official SiliconDust apps may reject it (third-party "
            "DVR software won't care). Remove device_id from the config to "
            "get a valid auto-generated one, or run --gen-device-id.\n", g_cfg.device_id);
    }

    fprintf(stderr,
        "hdhr-emulator starting: friendly_name=\"%s\" model=%s device_id=0x%08X "
        "tuners=%d (adapters:", g_cfg.friendly_name, g_cfg.model, g_cfg.device_id, g_cfg.tuner_count);
    for (int i = 0; i < g_cfg.tuner_count; i++) {
        fprintf(stderr, " tuner%d->/dev/dvb/adapter%d", i, g_cfg.dvb_adapter[i]);
    }
    fprintf(stderr, ")\n");

    tuner_pool_init(g_tuners, g_cfg.tuner_count, &g_cfg);

    /* Load any previously-saved channel cache before anything starts
     * serving requests, so lineup.json/streaming work immediately from
     * the last-known lineup instead of sitting empty for the ~1-2
     * minutes the background scan below takes. The scan (if enabled)
     * still runs regardless and overwrites/refreshes these entries
     * (and re-saves the cache) once it completes. */
    dvb_channel_db_load(g_cfg.channel_cache_file);

    struct control_ctx ctl = { .cfg = &g_cfg, .tuners = g_tuners };

    pthread_t th_discovery, th_control, th_http, th_scan;
    pthread_create(&th_discovery, NULL, discovery_thread_main, &g_cfg);
    pthread_create(&th_control, NULL, control_thread_main, &ctl);
    pthread_create(&th_http, NULL, http_thread_main, &ctl);
    keepalive_listener_start(g_tuners, g_cfg.tuner_count);

    if (g_cfg.scan_on_startup) {
        pthread_create(&th_scan, NULL, scan_thread_main, NULL);
        pthread_detach(th_scan); /* one-shot; main doesn't need to join it */
    } else {
        fprintf(stderr, "main: scan_on_startup=0 — channel database is empty until "
                         "a scan is triggered (not yet implemented as a runtime "
                         "action; restart with scan_on_startup=1 for now)\n");
    }

    pthread_join(th_discovery, NULL);
    pthread_join(th_control, NULL);
    pthread_join(th_http, NULL);

    return 0;
}
