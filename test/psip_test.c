/* Standalone test — not part of the daemon build. Encodes synthetic
 * PAT/PMT/TVCT sections by hand (independently of psip.c's own field
 * offsets, i.e. written by directly following the same spec layout
 * rather than copy-pasting psip.c's math) and confirms psip.c parses
 * them back to the expected values. Run with:
 *   gcc -Wall -Wextra -Isrc -o /tmp/psip_test src/psip.c test/psip_test.c && /tmp/psip_test
 */
#include "../src/psip.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

static int g_failures = 0;
#define CHECK(cond) do { \
    if (!(cond)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); g_failures++; } \
} while (0)

static void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v & 0xFF; }
static void put32(uint8_t *p, uint32_t v) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }

/* Builds a section with a placeholder CRC (0s) — psip.c doesn't
 * re-validate CRC (trusts the kernel's DMX_CHECK_CRC), so this is fine
 * for parser testing. */
static size_t finish_section(uint8_t *buf, size_t header_and_body_len)
{
    /* section_length field = everything after the 3-byte header,
     * including the 4-byte CRC we're about to append. */
    uint16_t section_length = (uint16_t)(header_and_body_len - 3 + 4);
    buf[1] = (buf[1] & 0xF0) | ((section_length >> 8) & 0x0F);
    buf[2] = section_length & 0xFF;
    memset(buf + header_and_body_len, 0, 4); /* fake CRC */
    return header_and_body_len + 4;
}

static void test_pat(void)
{
    uint8_t buf[64] = {0};
    buf[0] = 0x00; /* table_id */
    buf[1] = 0x80; /* section_syntax_indicator=1, rest filled by finish_section */
    put16(buf + 3, 0x1234); /* transport_stream_id */
    buf[5] = 0xC1; /* version/current_next */
    buf[6] = 0; buf[7] = 0; /* section_number, last_section_number */

    size_t off = 8;
    put16(buf + off, 0);      put16(buf + off + 2, 0x0010); off += 4; /* network PID entry, must be skipped */
    put16(buf + off, 1001);   put16(buf + off + 2, 0x0100 | 0x2000 /* garbage top bits, must be masked */); off += 4;
    put16(buf + off, 1002);   put16(buf + off + 2, 0x0200); off += 4;

    size_t len = finish_section(buf, off);

    struct pat_entry entries[PAT_MAX_ENTRIES];
    int n = psip_parse_pat(buf, len, entries, PAT_MAX_ENTRIES);
    CHECK(n == 2);
    CHECK(entries[0].program_number == 1001 && entries[0].pmt_pid == 0x0100);
    CHECK(entries[1].program_number == 1002 && entries[1].pmt_pid == 0x0200);
    printf("test_pat: %s\n", (n == 2) ? "ok" : "FAILED");
}

static void test_pmt(void)
{
    uint8_t buf[128] = {0};
    buf[0] = 0x02;
    buf[1] = 0x80;
    put16(buf + 3, 1001); /* program_number */
    buf[5] = 0xC1;
    buf[6] = 0; buf[7] = 0;
    put16(buf + 8, 0x0101 | 0xE000); /* PCR PID, top 3 bits must be masked as reserved */
    put16(buf + 10, 0); /* program_info_length = 0 */

    size_t off = 12;
    buf[off] = 0x02; put16(buf + off + 1, 0x0101); put16(buf + off + 3, 0); off += 5; /* video */
    buf[off] = 0x81; put16(buf + off + 1, 0x0102); put16(buf + off + 3, 0); off += 5; /* AC-3 audio */
    buf[off] = 0x05; put16(buf + off + 1, 0x0103); put16(buf + off + 3, 2); off += 5 + 2; /* registration descriptor, must be skipped over correctly */

    size_t len = finish_section(buf, off);

    struct pmt_info pmt;
    int n = psip_parse_pmt(buf, len, &pmt);
    CHECK(n == 3);
    CHECK(pmt.program_number == 1001);
    CHECK(pmt.pcr_pid == 0x0101);
    CHECK(pmt.streams[0].stream_type == 0x02 && pmt.streams[0].elementary_pid == 0x0101);
    CHECK(pmt.streams[1].stream_type == 0x81 && pmt.streams[1].elementary_pid == 0x0102);
    CHECK(pmt.streams[2].stream_type == 0x05 && pmt.streams[2].elementary_pid == 0x0103);

    uint16_t vpid, apid; uint8_t vtype, atype;
    bool have_video = psip_pmt_pick_av(&pmt, &vpid, &vtype, &apid, &atype);
    CHECK(have_video && vpid == 0x0101 && vtype == 0x02);
    CHECK(apid == 0x0102 && atype == 0x81);
    printf("test_pmt: %s\n", (n == 3 && have_video) ? "ok" : "FAILED");
}

static void test_tvct(void)
{
    uint8_t buf[256] = {0};
    buf[0] = 0xC8; /* TVCT table_id */
    buf[1] = 0x80;
    put16(buf + 3, 0xABCD); /* transport_stream_id */
    buf[5] = 0xC1;
    buf[6] = 0;  /* section_number */
    buf[7] = 0;  /* last_section_number */
    buf[8] = 0;  /* protocol_version */
    buf[9] = 2;  /* num_channels_in_section */

    size_t off = 10;

    /* channel 0: "KABC-HD", 7.1, program_number 3, NOT hidden.
     * Byte 26/27 use realistic values with the two reserved-bit groups
     * set to '1' as the ATSC A/65 spec (section 4.2) mandates — this is
     * the case that exposed a real bug: an earlier version of this
     * parser read one of those reserved bits as "hidden", so every
     * real-world broadcast (which always sets reserved bits to 1) came
     * back hidden=true. Layout verified against A/65:2013 Table 6.4:
     *   ETM_location(2)=01 | access_controlled(1)=0 | hidden(1)=0 |
     *   reserved(2)=11 | hide_guide(1)=0 | [byte27] reserved(3, incl.
     *   the 1 bit carried over from byte26's LSB)=111 | service_type(6)=000010
     * byte26 = 0100 1101 = 0x4D, byte27 = 1100 0010 = 0xC2 */
    {
        uint8_t *e = buf + off;
        const char *name = "KABC-HD";
        for (int i = 0; i < 7; i++) {
            uint16_t unit = (i < (int)strlen(name)) ? (uint16_t)name[i] : 0x0000;
            put16(e + i * 2, unit);
        }
        uint32_t triplet = (0 << 20) | (7 << 10) | 1; /* reserved=0, major=7, minor=1 */
        e[14] = (triplet >> 16) & 0xFF;
        e[15] = (triplet >> 8) & 0xFF;
        e[16] = triplet & 0xFF;
        e[17] = 0x04; /* modulation_mode: 8VSB */
        put32(e + 18, 0); /* carrier_frequency, deprecated/unused */
        put16(e + 22, 0xABCD); /* channel_TSID */
        put16(e + 24, 3);      /* program_number */
        e[26] = 0x4D;          /* NOT hidden, realistic reserved=1 bits */
        e[27] = 0xC2;          /* realistic reserved=1 bits, service_type=2 */
        put16(e + 28, 0x1234); /* source_id */
        put16(e + 30, 0);      /* reserved(6)+descriptors_length(10) = 0 */
        off += 32 + 0;
    }
    /* channel 1: "KABC-SD", 7.2, program_number 4, IS hidden (bit 4 set,
     * same realistic reserved=1 bits elsewhere): byte26 = 0101 1101 = 0x5D */
    {
        uint8_t *e = buf + off;
        const char *name = "KABC-SD";
        for (int i = 0; i < 7; i++) {
            uint16_t unit = (i < (int)strlen(name)) ? (uint16_t)name[i] : 0x0000;
            put16(e + i * 2, unit);
        }
        uint32_t triplet = (0 << 20) | (7 << 10) | 2;
        e[14] = (triplet >> 16) & 0xFF;
        e[15] = (triplet >> 8) & 0xFF;
        e[16] = triplet & 0xFF;
        e[17] = 0x04;
        put32(e + 18, 0);
        put16(e + 22, 0xABCD);
        put16(e + 24, 4);
        e[26] = 0x5D; /* hidden bit (0x10) set, realistic reserved=1 bits elsewhere */
        e[27] = 0xC2; /* realistic reserved=1 bits, service_type=2 */
        put16(e + 28, 0x1235);
        put16(e + 30, 3); /* descriptors_length = 3, must be skipped correctly */
        e[32] = 0xAA; e[33] = 0xBB; e[34] = 0xCC; /* dummy descriptor bytes */
        off += 32 + 3;
    }

    size_t len = finish_section(buf, off);

    struct tvct_entry entries[TVCT_MAX_CHANNELS];
    int sec_num = -1, last_sec_num = -1;
    int n = psip_parse_tvct(buf, len, entries, TVCT_MAX_CHANNELS, &sec_num, &last_sec_num);

    CHECK(n == 2);
    CHECK(sec_num == 0 && last_sec_num == 0);
    CHECK(strcmp(entries[0].short_name, "KABC-HD") == 0);
    CHECK(entries[0].major_channel_number == 7 && entries[0].minor_channel_number == 1);
    CHECK(entries[0].program_number == 3);
    CHECK(entries[0].channel_tsid == 0xABCD);
    CHECK(entries[0].hidden == false);

    CHECK(strcmp(entries[1].short_name, "KABC-SD") == 0);
    CHECK(entries[1].major_channel_number == 7 && entries[1].minor_channel_number == 2);
    CHECK(entries[1].program_number == 4);
    CHECK(entries[1].hidden == true);

    printf("test_tvct: %s\n", (n == 2) ? "ok" : "FAILED");
}

int main(void)
{
    test_pat();
    test_pmt();
    test_tvct();
    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    } else {
        printf("%d CHECK(S) FAILED\n", g_failures);
        return 1;
    }
}
