/*
 * discovery.c — answers HDHomeRun UDP broadcast discovery on port 65001.
 *
 * Wire behaviour (see hdhr_pkt.h for the framing/TLV spec):
 *   Client -> broadcast DISCOVER_REQ { DEVICE_TYPE?, DEVICE_ID? }
 *   Us     -> unicast reply to sender:
 *             DISCOVER_RPY { DEVICE_TYPE, DEVICE_ID, TUNER_COUNT,
 *                             BASE_URL, LINEUP_URL }
 *
 * A request with no tags, or DEVICE_TYPE=WILDCARD / DEVICE_ID=WILDCARD,
 * matches everyone. A request naming a specific device type/id that
 * isn't ours is silently ignored (matches real firmware behaviour —
 * devices don't reply to queries that aren't for them).
 */
#include "discovery.h"
#include "hdhr_pkt.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

static int open_discovery_socket(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("discovery: socket");
        return -1;
    }

    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(HDHR_DISCOVER_UDP_PORT);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("discovery: bind :65001/udp (need root, or setcap CAP_NET_BIND_SERVICE)");
        close(fd);
        return -1;
    }

    return fd;
}

/* Figures out which local IP the kernel would use to reach `peer` — this
 * is more reliable than trusting a single configured bind_ip when the Pi
 * has multiple interfaces, and is what we advertise in BASE_URL/LINEUP_URL
 * so clients can actually reach our HTTP server back. */
static int local_ip_for_peer(const struct sockaddr_in *peer, char *out, size_t outlen)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;

    if (connect(fd, (const struct sockaddr *)peer, sizeof(*peer)) < 0) {
        close(fd);
        return -1;
    }

    struct sockaddr_in local;
    socklen_t len = sizeof(local);
    if (getsockname(fd, (struct sockaddr *)&local, &len) < 0) {
        close(fd);
        return -1;
    }
    close(fd);

    if (!inet_ntop(AF_INET, &local.sin_addr, out, outlen)) {
        return -1;
    }
    return 0;
}

static void handle_discover_req(int fd, const struct hdhr_config *cfg,
                                 const uint8_t *data, size_t len,
                                 const struct sockaddr_in *from)
{
    struct hdhr_pkt in;
    uint16_t type;

    if (hdhr_pkt_open_frame(&in, data, len, &type) != 0) {
        return; /* bad CRC/format — real devices just drop these */
    }
    if (type != HDHR_TYPE_DISCOVER_REQ) {
        return;
    }

    uint32_t req_device_type = HDHR_DEVICE_TYPE_WILDCARD;
    uint32_t req_device_id = HDHR_DEVICE_ID_WILDCARD;

    uint8_t tag;
    const uint8_t *val;
    size_t vlen;
    int r;
    while ((r = hdhr_pkt_read_tlv(&in, &tag, &val, &vlen)) == 1) {
        if (tag == HDHR_TAG_DEVICE_TYPE && vlen == 4) {
            req_device_type = ((uint32_t)val[0] << 24) | ((uint32_t)val[1] << 16) |
                               ((uint32_t)val[2] << 8) | val[3];
        } else if (tag == HDHR_TAG_DEVICE_ID && vlen == 4) {
            req_device_id = ((uint32_t)val[0] << 24) | ((uint32_t)val[1] << 16) |
                             ((uint32_t)val[2] << 8) | val[3];
        }
        /* unknown tags are ignored, per spec */
    }
    if (r < 0) {
        return; /* malformed TLV stream */
    }

    int type_matches = (req_device_type == HDHR_DEVICE_TYPE_WILDCARD) ||
                        (req_device_type == HDHR_DEVICE_TYPE_TUNER);
    int id_matches = (req_device_id == HDHR_DEVICE_ID_WILDCARD) ||
                      (req_device_id == cfg->device_id);

    if (!type_matches || !id_matches) {
        return; /* not addressed to us */
    }

    char base_ip[16];
    const char *advertise_ip = cfg->bind_ip;
    if (strcmp(cfg->bind_ip, "0.0.0.0") == 0) {
        if (local_ip_for_peer(from, base_ip, sizeof(base_ip)) == 0) {
            advertise_ip = base_ip;
        } else {
            advertise_ip = "127.0.0.1";
        }
    }

    char base_url[64];
    char lineup_url[80];
    snprintf(base_url, sizeof(base_url), "http://%s:80", advertise_ip);
    snprintf(lineup_url, sizeof(lineup_url), "http://%s:80/lineup.json", advertise_ip);

    struct hdhr_pkt out;
    hdhr_pkt_start_frame(&out);
    hdhr_pkt_write_tlv_u32(&out, HDHR_TAG_DEVICE_TYPE, HDHR_DEVICE_TYPE_TUNER);
    hdhr_pkt_write_tlv_u32(&out, HDHR_TAG_DEVICE_ID, cfg->device_id);
    /* TUNER_COUNT is a single byte on the wire, unlike DEVICE_TYPE/
     * DEVICE_ID (4 bytes) — real clients reject/ignore a 4-byte value
     * here (len != 1), silently falling back to a tuner count of 1 and
     * never splitting a multi-tuner device into per-tuner "-0"/"-1"
     * entries the way hdhomerun_config_gui does for genuine hardware. */
    uint8_t tuner_count_byte = (uint8_t)cfg->tuner_count;
    hdhr_pkt_write_tlv(&out, HDHR_TAG_TUNER_COUNT, &tuner_count_byte, 1);
    hdhr_pkt_write_tlv_str(&out, HDHR_TAG_BASE_URL, base_url);
    hdhr_pkt_write_tlv_str(&out, HDHR_TAG_LINEUP_URL, lineup_url);
    size_t out_len = hdhr_pkt_seal_frame(&out, HDHR_TYPE_DISCOVER_RPY);

    sendto(fd, out.buffer, out_len, 0, (const struct sockaddr *)from, sizeof(*from));
}

void *discovery_thread_main(void *arg)
{
    struct hdhr_config *cfg = (struct hdhr_config *)arg;

    int fd = open_discovery_socket();
    if (fd < 0) {
        fprintf(stderr, "discovery: failed to start, thread exiting\n");
        return NULL;
    }

    fprintf(stderr, "discovery: listening on udp/%d (device_id=0x%08X, tuners=%d)\n",
            HDHR_DISCOVER_UDP_PORT, cfg->device_id, cfg->tuner_count);

    uint8_t buf[HDHR_MAX_PACKET_SIZE];
    for (;;) {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fromlen);
        if (n <= 0) {
            continue;
        }
        handle_discover_req(fd, cfg, buf, (size_t)n, &from);
    }

    close(fd);
    return NULL;
}
