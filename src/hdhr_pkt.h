/*
 * hdhr_pkt.h — HDHomeRun wire-format core.
 *
 * Clean-room reimplementation of the packet framing described by
 * Silicondust's libhdhomerun (hdhomerun_pkt.h / hdhomerun_pkt.c), targeting
 * exact on-wire compatibility:
 *
 *   uint16_t  type            (big-endian)
 *   uint16_t  payload length  (big-endian)
 *   uint8_t   payload[len]
 *   uint32_t  CRC32           (little-endian, Ethernet/zlib-style reflected
 *                               CRC-32, poly 0xEDB88320, init/xorout 0xFFFFFFFF,
 *                               computed over type+length+payload)
 *
 * Payload is a sequence of TLVs:
 *   uint8_t   tag
 *   varlen    length   (1 byte if <=127, else 2 bytes: low7|0x80, high bits)
 *   uint8_t   value[length]
 *
 * Tag/type constants below are taken verbatim from the public
 * libhdhomerun header (LGPL, Silicondust USA Inc.) since exact numeric
 * values are required for wire compatibility — only the values are used
 * here, not any of their implementation code.
 */
#ifndef HDHR_PKT_H
#define HDHR_PKT_H

#include <stdint.h>
#include <stddef.h>

#define HDHR_DISCOVER_UDP_PORT   65001
#define HDHR_CONTROL_TCP_PORT    65001
#define HDHR_MAX_PACKET_SIZE     1460
#define HDHR_MAX_PAYLOAD_SIZE    1452
#define HDHR_PKT_BUFFER_SIZE     3074   /* matches libhdhomerun's internal bound */

/* Frame types */
#define HDHR_TYPE_DISCOVER_REQ   0x0002
#define HDHR_TYPE_DISCOVER_RPY   0x0003
#define HDHR_TYPE_GETSET_REQ     0x0004
#define HDHR_TYPE_GETSET_RPY     0x0005
#define HDHR_TYPE_UPGRADE_REQ    0x0006
#define HDHR_TYPE_UPGRADE_RPY    0x0007

/* TLV tags */
#define HDHR_TAG_DEVICE_TYPE       0x01
#define HDHR_TAG_DEVICE_ID         0x02
#define HDHR_TAG_GETSET_NAME       0x03
#define HDHR_TAG_GETSET_VALUE      0x04
#define HDHR_TAG_ERROR_MESSAGE     0x05
#define HDHR_TAG_TUNER_COUNT       0x10
#define HDHR_TAG_GETSET_LOCKKEY    0x15
#define HDHR_TAG_LINEUP_URL        0x27
#define HDHR_TAG_STORAGE_URL       0x28
#define HDHR_TAG_BASE_URL          0x2A
#define HDHR_TAG_DEVICE_AUTH_STR   0x2B
#define HDHR_TAG_STORAGE_ID        0x2C
#define HDHR_TAG_MULTI_TYPE        0x2D

#define HDHR_DEVICE_TYPE_WILDCARD  0xFFFFFFFFu
#define HDHR_DEVICE_TYPE_TUNER     0x00000001u
#define HDHR_DEVICE_TYPE_STORAGE   0x00000005u
#define HDHR_DEVICE_ID_WILDCARD    0xFFFFFFFFu

struct hdhr_pkt {
    uint8_t buffer[HDHR_PKT_BUFFER_SIZE];
    uint8_t *start;  /* start of frame (the type field)         */
    uint8_t *pos;    /* current read/write cursor                */
    uint8_t *end;    /* end of payload, i.e. where CRC begins     */
    uint8_t *limit;  /* end of usable buffer space                */
};

/* CRC32 — Ethernet/zlib-style reflected CRC, as used on the wire. */
uint32_t hdhr_crc32(const uint8_t *data, size_t len);

/* Reset a packet buffer for building a fresh frame. */
void hdhr_pkt_init(struct hdhr_pkt *pkt);

/* --- Writing (building an outgoing frame) --- */
void hdhr_pkt_start_frame(struct hdhr_pkt *pkt);
void hdhr_pkt_write_u8(struct hdhr_pkt *pkt, uint8_t v);
void hdhr_pkt_write_u32(struct hdhr_pkt *pkt, uint32_t v);
void hdhr_pkt_write_var_length(struct hdhr_pkt *pkt, size_t len);
void hdhr_pkt_write_mem(struct hdhr_pkt *pkt, const void *mem, size_t len);
void hdhr_pkt_write_tlv(struct hdhr_pkt *pkt, uint8_t tag, const void *mem, size_t len);
void hdhr_pkt_write_tlv_u32(struct hdhr_pkt *pkt, uint8_t tag, uint32_t v);
void hdhr_pkt_write_tlv_str(struct hdhr_pkt *pkt, uint8_t tag, const char *str);

/* Seals the frame (writes type+length header retroactively isn't needed —
 * header is written up front in start_frame; this appends the CRC).
 * Returns total sealed length (header + payload + crc) to send on the wire. */
size_t hdhr_pkt_seal_frame(struct hdhr_pkt *pkt, uint16_t frame_type);

/* --- Reading (parsing an incoming frame) ---
 * data/len is a raw datagram (UDP) or exactly one framed message (TCP,
 * after the caller has delineated it — see control.c for TCP framing).
 * Validates the CRC. On success sets pkt->pos/pkt->end to the payload
 * bounds so hdhr_pkt_read_tlv() can walk it. */
int hdhr_pkt_open_frame(struct hdhr_pkt *pkt, const uint8_t *data, size_t len, uint16_t *ptype);

/* Returns 1 and fills ptag/pvalue/plen on success, 0 when payload is
 * exhausted, -1 on malformed TLV. pvalue points into pkt->buffer (no copy). */
int hdhr_pkt_read_tlv(struct hdhr_pkt *pkt, uint8_t *ptag, const uint8_t **pvalue, size_t *plen);

#endif /* HDHR_PKT_H */
