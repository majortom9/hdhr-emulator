#include "config.h"
#include "device_id.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

void config_defaults(struct hdhr_config *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    /* Seed from the current time by default so two emulator instances on
     * the same LAN don't collide; override with device_id=0x... in the
     * config file for a stable identity across restarts (recommended —
     * Plex/Emby key their DVR pairing off this). Checksum-valid per the
     * real HDHomeRun algorithm (see device_id.c), so even the official
     * SiliconDust apps will accept it as genuine. */
    cfg->device_id = hdhr_device_id_generate((uint32_t)time(NULL) & 0x00FFFFFFu);

    snprintf(cfg->friendly_name, sizeof(cfg->friendly_name), "HDHomeRun ATSC");
    snprintf(cfg->model, sizeof(cfg->model), "HDHR3-US");
    snprintf(cfg->firmware_name, sizeof(cfg->firmware_name), "hdhomerun3_atsc");
    snprintf(cfg->firmware_version, sizeof(cfg->firmware_version), "20130117");
    cfg->tuner_count = 2;
    snprintf(cfg->bind_ip, sizeof(cfg->bind_ip), "0.0.0.0");

    for (int i = 0; i < HDHR_MAX_TUNERS; i++) {
        cfg->dvb_adapter[i] = i; /* tunerN -> /dev/dvb/adapterN by default */
    }
    cfg->dvb_frontend = 0;
    cfg->dvb_demux = 0;
    cfg->scan_on_startup = true;
    cfg->debug_signal_stats = false;
    cfg->allow_remote_restart = false;
}

static void trim(char *s)
{
    char *start = s;
    while (*start == ' ' || *start == '\t') start++;
    if (start != s) memmove(s, start, strlen(start) + 1);

    size_t len = strlen(s);
    while (len > 0 && (s[len-1] == '\n' || s[len-1] == '\r' ||
                        s[len-1] == ' '  || s[len-1] == '\t')) {
        s[--len] = '\0';
    }
}

int config_load(const char *path, struct hdhr_config *cfg)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return 0; /* missing config file is not an error — defaults stand */
    }

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';

        char *eq = strchr(line, '=');
        if (!eq) continue;

        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        trim(key);
        trim(val);
        if (key[0] == '\0') continue;

        if (strcmp(key, "device_id") == 0) {
            cfg->device_id = (uint32_t)strtoul(val, NULL, 0);
        } else if (strcmp(key, "friendly_name") == 0) {
            snprintf(cfg->friendly_name, sizeof(cfg->friendly_name), "%s", val);
        } else if (strcmp(key, "model") == 0) {
            snprintf(cfg->model, sizeof(cfg->model), "%s", val);
        } else if (strcmp(key, "firmware_name") == 0) {
            snprintf(cfg->firmware_name, sizeof(cfg->firmware_name), "%s", val);
        } else if (strcmp(key, "firmware_version") == 0) {
            snprintf(cfg->firmware_version, sizeof(cfg->firmware_version), "%s", val);
        } else if (strcmp(key, "tuner_count") == 0) {
            int n = atoi(val);
            if (n < 1) n = 1;
            if (n > HDHR_MAX_TUNERS) n = HDHR_MAX_TUNERS;
            cfg->tuner_count = n;
        } else if (strcmp(key, "bind_ip") == 0) {
            snprintf(cfg->bind_ip, sizeof(cfg->bind_ip), "%s", val);
        } else if (strcmp(key, "dvb_frontend") == 0) {
            cfg->dvb_frontend = atoi(val);
        } else if (strcmp(key, "dvb_demux") == 0) {
            cfg->dvb_demux = atoi(val);
        } else if (strcmp(key, "scan_on_startup") == 0) {
            cfg->scan_on_startup = (atoi(val) != 0);
        } else if (strcmp(key, "debug_signal_stats") == 0) {
            cfg->debug_signal_stats = (atoi(val) != 0);
        } else if (strcmp(key, "allow_remote_restart") == 0) {
            cfg->allow_remote_restart = (atoi(val) != 0);
        } else if (strncmp(key, "tuner", 5) == 0 && strstr(key, "_adapter") != NULL) {
            /* tunerN_adapter=X — override which /dev/dvb/adapterX a given
             * hdhr_tuner slot maps to, for boxes where tuners aren't
             * simply adapter0, adapter1, ... in order. */
            int idx = atoi(key + 5);
            if (idx >= 0 && idx < HDHR_MAX_TUNERS) {
                cfg->dvb_adapter[idx] = atoi(val);
            }
        }
        /* unknown keys are silently ignored so the file stays forward-compatible */
    }

    fclose(f);
    return 0;
}
