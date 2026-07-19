/*
 * tools/calibrate_stats.c — standalone RF signal-stat calibration
 * sweep. Tunes every US ATSC RF channel (2-36) in turn on a given DVB
 * adapter and, for whichever lock, prints the driver's raw
 * DTV_STAT_SIGNAL_STRENGTH (dBm) and DTV_STAT_CNR (dB) readings
 * alongside what the daemon's current dvb_frontend_read_stats()
 * calibration maps them to. Not part of the daemon build (see
 * Makefile's `calibrate` target, which compiles this against
 * dvb_frontend.c/atsc_freq.c directly rather than via make's src/
 * wildcard build) — this is a one-off dev tool for picking new
 * floor/ceiling constants in dvb_frontend.c, not something a deployed
 * emulator needs at runtime.
 *
 * Prints a CSV sweep to stdout and a min/max summary to stderr.
 */
#include "dvb_frontend.h"
#include "atsc_freq.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define LOCK_TIMEOUT_MS 1500
#define SETTLE_MS        500 /* let CNR/signal readings settle just after lock */

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <dvb-adapter-num> [frontend-num]\n", argv[0]);
        return 1;
    }
    int adapter = atoi(argv[1]);
    int frontend = (argc >= 3) ? atoi(argv[2]) : 0;

    double worst_ss = 1e9, best_ss = -1e9;
    double worst_snr = 1e9, best_snr = -1e9;
    int worst_ss_ch = 0, best_ss_ch = 0, worst_snr_ch = 0, best_snr_ch = 0;
    int locked_count = 0;

    printf("chan,freq_hz,locked,ss_dbm,ss_pct,snr_db,snq_pct\n");

    for (int i = 0; i < ATSC_FREQ_TABLE_COUNT; i++) {
        int ch = atsc_freq_table[i].channel;
        uint32_t freq = atsc_freq_table[i].frequency_hz;

        int fd = dvb_frontend_open(adapter, frontend);
        if (fd < 0) {
            fprintf(stderr, "chan %d: failed to open adapter%d/frontend%d\n", ch, adapter, frontend);
            continue;
        }

        if (dvb_frontend_tune_8vsb(fd, freq) != 0) {
            dvb_frontend_close(fd);
            continue;
        }

        bool locked = dvb_frontend_wait_lock(fd, LOCK_TIMEOUT_MS, NULL, NULL);
        if (!locked) {
            printf("%d,%u,0,NA,NA,NA,NA\n", ch, freq);
            fprintf(stderr, "chan %2d (%u Hz): no lock\n", ch, freq);
            dvb_frontend_close(fd);
            continue;
        }

        usleep(SETTLE_MS * 1000);

        struct dvb_signal_stats stats;
        dvb_frontend_read_stats(fd, &stats);

        double ss_dbm = 0, snr_db = 0;
        bool have_ss = dvb_frontend_read_raw_signal_dbm(fd, &ss_dbm);
        bool have_snr = dvb_frontend_read_raw_snr_db(fd, &snr_db);

        char ss_buf[32], snr_buf[32];
        if (have_ss) snprintf(ss_buf, sizeof(ss_buf), "%.2f", ss_dbm);
        else snprintf(ss_buf, sizeof(ss_buf), "NA");
        if (have_snr) snprintf(snr_buf, sizeof(snr_buf), "%.2f", snr_db);
        else snprintf(snr_buf, sizeof(snr_buf), "NA");

        printf("%d,%u,1,%s,%d,%s,%d\n", ch, freq, ss_buf, stats.signal_strength_pct,
               snr_buf, stats.snr_quality_pct);
        fprintf(stderr, "chan %2d (%u Hz): locked  ss=%s dBm (%d%%)  snr=%s dB (%d%%)\n",
                ch, freq, ss_buf, stats.signal_strength_pct, snr_buf, stats.snr_quality_pct);

        if (have_ss) {
            if (ss_dbm > best_ss)  { best_ss = ss_dbm;  best_ss_ch = ch; }
            if (ss_dbm < worst_ss) { worst_ss = ss_dbm; worst_ss_ch = ch; }
        }
        if (have_snr) {
            if (snr_db > best_snr)  { best_snr = snr_db;  best_snr_ch = ch; }
            if (snr_db < worst_snr) { worst_snr = snr_db; worst_snr_ch = ch; }
        }
        locked_count++;

        dvb_frontend_close(fd);
    }

    fprintf(stderr, "\n=== summary: %d/%d channels locked ===\n", locked_count, ATSC_FREQ_TABLE_COUNT);
    if (locked_count > 0) {
        fprintf(stderr, "signal strength: weakest ch%d = %.2f dBm   strongest ch%d = %.2f dBm\n",
                worst_ss_ch, worst_ss, best_ss_ch, best_ss);
        fprintf(stderr, "CNR/SNR:         worst    ch%d = %.2f dB    best      ch%d = %.2f dB\n",
                worst_snr_ch, worst_snr, best_snr_ch, best_snr);
    } else {
        fprintf(stderr, "no channels locked -- check antenna/adapter\n");
    }
    return 0;
}
