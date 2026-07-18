#include "hdhr_pkt.h"
#include <string.h>

/* ---- CRC32 (reflected, poly 0xEDB88320, init/xorout 0xFFFFFFFF) ----
 * Bit-for-bit identical results to the nibble-driven implementation in
 * libhdhomerun's hdhomerun_pkt.c — verified against their source. Table
 * form here purely for speed; same algorithm. */
static uint32_t crc32_table[256];
static int crc32_table_ready = 0;

static void crc32_table_init(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        crc32_table[i] = c;
    }
    crc32_table_ready = 1;
}

uint32_t hdhr_crc32(const uint8_t *data, size_t len)
{
    if (!crc32_table_ready) {
        crc32_table_init();
    }
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ---- buffer plumbing ---- */

void hdhr_pkt_init(struct hdhr_pkt *pkt)
{
    memset(pkt->buffer, 0, sizeof(pkt->buffer));
    pkt->start = pkt->buffer;
    pkt->pos = pkt->buffer;
    pkt->end = pkt->buffer;
    pkt->limit = pkt->buffer + sizeof(pkt->buffer);
}

void hdhr_pkt_start_frame(struct hdhr_pkt *pkt)
{
    hdhr_pkt_init(pkt);
    /* reserve 4 bytes for type+length header; filled in by seal_frame */
    pkt->pos = pkt->buffer + 4;
    pkt->end = pkt->pos;
}

void hdhr_pkt_write_u8(struct hdhr_pkt *pkt, uint8_t v)
{
    if (pkt->pos >= pkt->limit) return;
    *pkt->pos++ = v;
    pkt->end = pkt->pos;
}

void hdhr_pkt_write_u32(struct hdhr_pkt *pkt, uint32_t v)
{
    hdhr_pkt_write_u8(pkt, (uint8_t)(v >> 24));
    hdhr_pkt_write_u8(pkt, (uint8_t)(v >> 16));
    hdhr_pkt_write_u8(pkt, (uint8_t)(v >> 8));
    hdhr_pkt_write_u8(pkt, (uint8_t)(v));
}

void hdhr_pkt_write_var_length(struct hdhr_pkt *pkt, size_t len)
{
    if (len <= 127) {
        hdhr_pkt_write_u8(pkt, (uint8_t)len);
    } else {
        hdhr_pkt_write_u8(pkt, (uint8_t)((len & 0x7F) | 0x80));
        hdhr_pkt_write_u8(pkt, (uint8_t)(len >> 7));
    }
}

void hdhr_pkt_write_mem(struct hdhr_pkt *pkt, const void *mem, size_t len)
{
    if (pkt->pos + len > pkt->limit) return;
    memcpy(pkt->pos, mem, len);
    pkt->pos += len;
    pkt->end = pkt->pos;
}

void hdhr_pkt_write_tlv(struct hdhr_pkt *pkt, uint8_t tag, const void *mem, size_t len)
{
    hdhr_pkt_write_u8(pkt, tag);
    hdhr_pkt_write_var_length(pkt, len);
    hdhr_pkt_write_mem(pkt, mem, len);
}

void hdhr_pkt_write_tlv_u32(struct hdhr_pkt *pkt, uint8_t tag, uint32_t v)
{
    uint8_t buf[4] = {
        (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v
    };
    hdhr_pkt_write_tlv(pkt, tag, buf, 4);
}

void hdhr_pkt_write_tlv_str(struct hdhr_pkt *pkt, uint8_t tag, const char *str)
{
    /* per spec, string TLVs include the terminating NUL in the length */
    hdhr_pkt_write_tlv(pkt, tag, str, strlen(str) + 1);
}

size_t hdhr_pkt_seal_frame(struct hdhr_pkt *pkt, uint16_t frame_type)
{
    size_t payload_len = (size_t)(pkt->end - (pkt->buffer + 4));

    pkt->buffer[0] = (uint8_t)(frame_type >> 8);
    pkt->buffer[1] = (uint8_t)(frame_type);
    pkt->buffer[2] = (uint8_t)(payload_len >> 8);
    pkt->buffer[3] = (uint8_t)(payload_len);

    uint32_t crc = hdhr_crc32(pkt->buffer, 4 + payload_len);
    uint8_t *crc_pos = pkt->buffer + 4 + payload_len;
    crc_pos[0] = (uint8_t)(crc);
    crc_pos[1] = (uint8_t)(crc >> 8);
    crc_pos[2] = (uint8_t)(crc >> 16);
    crc_pos[3] = (uint8_t)(crc >> 24);

    return 4 + payload_len + 4;
}

int hdhr_pkt_open_frame(struct hdhr_pkt *pkt, const uint8_t *data, size_t len, uint16_t *ptype)
{
    if (len < 4 + 4) {
        return -1; /* too small to hold header + crc */
    }
    if (len > sizeof(pkt->buffer)) {
        return -1;
    }

    uint16_t type = ((uint16_t)data[0] << 8) | data[1];
    uint16_t payload_len = ((uint16_t)data[2] << 8) | data[3];

    if (4 + (size_t)payload_len + 4 != len) {
        return -1; /* declared length doesn't match datagram size */
    }

    uint32_t calc_crc = hdhr_crc32(data, 4 + payload_len);
    uint32_t wire_crc = (uint32_t)data[4 + payload_len]
                       | ((uint32_t)data[4 + payload_len + 1] << 8)
                       | ((uint32_t)data[4 + payload_len + 2] << 16)
                       | ((uint32_t)data[4 + payload_len + 3] << 24);

    if (calc_crc != wire_crc) {
        return -1;
    }

    memcpy(pkt->buffer, data, len);
    pkt->start = pkt->buffer;
    pkt->pos = pkt->buffer + 4;
    pkt->end = pkt->buffer + 4 + payload_len;
    pkt->limit = pkt->buffer + sizeof(pkt->buffer);

    if (ptype) *ptype = type;
    return 0;
}

int hdhr_pkt_read_tlv(struct hdhr_pkt *pkt, uint8_t *ptag, const uint8_t **pvalue, size_t *plen)
{
    if (pkt->pos >= pkt->end) {
        return 0; /* clean end of payload */
    }
    if (pkt->pos + 1 > pkt->end) {
        return -1;
    }

    uint8_t tag = *pkt->pos++;

    if (pkt->pos >= pkt->end) return -1;
    uint8_t b0 = *pkt->pos++;
    size_t length;
    if (b0 & 0x80) {
        if (pkt->pos >= pkt->end) return -1;
        uint8_t b1 = *pkt->pos++;
        length = (size_t)(b0 & 0x7F) | ((size_t)b1 << 7);
    } else {
        length = b0;
    }

    if (pkt->pos + length > pkt->end) {
        return -1; /* TLV claims more data than remains in payload */
    }

    if (ptag) *ptag = tag;
    if (pvalue) *pvalue = pkt->pos;
    if (plen) *plen = length;

    pkt->pos += length;
    return 1;
}
