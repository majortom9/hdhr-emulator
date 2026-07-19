#include "pid_filter.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

bool pid_filter_parse(const char *str, struct pid_filter *out)
{
    out->count = 0;
    if (!str) return true;

    const char *p = str;
    while (*p) {
        while (*p && isspace((unsigned char)*p)) p++;
        if (!*p) break;

        const char *tok = p;
        while (*p && !isspace((unsigned char)*p)) p++;
        size_t tok_len = (size_t)(p - tok);
        if (tok_len == 0 || tok_len >= 32) return false;

        char buf[32];
        memcpy(buf, tok, tok_len);
        buf[tok_len] = '\0';

        unsigned int start, end;
        char *dash = strchr(buf, '-');
        if (dash) {
            *dash = '\0';
            if (sscanf(buf, "0x%x", &start) != 1) return false;
            if (sscanf(dash + 1, "0x%x", &end) != 1) return false;
        } else {
            if (sscanf(buf, "0x%x", &start) != 1) return false;
            end = start;
        }
        if (start > 0x1FFF || end > 0x1FFF || end < start) return false;

        if (out->count >= PID_FILTER_MAX_RANGES) return false;
        out->ranges[out->count].start = (uint16_t)start;
        out->ranges[out->count].end = (uint16_t)end;
        out->count++;
    }
    return true;
}

int pid_filter_total_count(const struct pid_filter *f)
{
    int total = 0;
    for (int i = 0; i < f->count; i++) {
        total += (int)(f->ranges[i].end - f->ranges[i].start) + 1;
    }
    return total;
}

void pid_filter_format(const uint16_t *pids, int count, char *out, size_t out_len)
{
    uint16_t sorted[64];
    if (count > 64) count = 64; /* defensive; callers never pass more than a handful */
    memcpy(sorted, pids, (size_t)count * sizeof(uint16_t));

    /* insertion sort + in-place dedup -- count is always small (a
     * handful of PMT/PCR/video/audio PIDs), no need for anything
     * fancier */
    for (int i = 1; i < count; i++) {
        uint16_t key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    int n = 0;
    for (int i = 0; i < count; i++) {
        if (n == 0 || sorted[i] != sorted[n - 1]) sorted[n++] = sorted[i];
    }

    size_t off = 0;
    out[0] = '\0';
    int i = 0;
    while (i < n) {
        int j = i;
        while (j + 1 < n && sorted[j + 1] == sorted[j] + 1) j++;

        int w;
        if (j == i) {
            w = snprintf(out + off, out_len - off, "0x%04x ", sorted[i]);
        } else {
            w = snprintf(out + off, out_len - off, "0x%04x-0x%04x ", sorted[i], sorted[j]);
        }
        off += (w > 0) ? (size_t)w : 0;
        if (off >= out_len) { off = out_len - 1; break; }
        i = j + 1;
    }
    /* trim trailing space, matching real firmware's own formatting */
    if (off > 0 && out[off - 1] == ' ') out[off - 1] = '\0';
}
