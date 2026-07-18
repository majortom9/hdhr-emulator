#include "device_id.h"

/* Table and algorithm verified against libhdhomerun's
 * hdhomerun_discover_validate_device_id() — reimplemented independently
 * here (same table values are an intrinsic property of the wire format,
 * not creative expression). */
static const uint8_t nibble_lut[16] = {
    0xA, 0x5, 0xF, 0x6, 0x7, 0xC, 0x1, 0xB,
    0x9, 0x2, 0x8, 0xD, 0x4, 0x3, 0xE, 0x0
};

bool hdhr_device_id_is_valid(uint32_t id)
{
    uint8_t checksum = 0;

    checksum ^= nibble_lut[(id >> 28) & 0xF];
    checksum ^=            (id >> 24) & 0xF;
    checksum ^= nibble_lut[(id >> 20) & 0xF];
    checksum ^=            (id >> 16) & 0xF;
    checksum ^= nibble_lut[(id >> 12) & 0xF];
    checksum ^=            (id >> 8)  & 0xF;
    checksum ^= nibble_lut[(id >> 4)  & 0xF];
    checksum ^=            (id >> 0)  & 0xF;

    return checksum == 0;
}

uint32_t hdhr_device_id_generate(uint32_t seed24)
{
    /* nibble7 = fixed class digit; nibbles 6..1 = 6 nibbles (24 bits) of
     * seed; nibble0 solved so the checksum comes out to zero. */
    uint8_t nib7 = 0x1;
    uint8_t nib6 = (seed24 >> 20) & 0xF;
    uint8_t nib5 = (seed24 >> 16) & 0xF;
    uint8_t nib4 = (seed24 >> 12) & 0xF;
    uint8_t nib3 = (seed24 >> 8)  & 0xF;
    uint8_t nib2 = (seed24 >> 4)  & 0xF;
    uint8_t nib1 = (seed24 >> 0)  & 0xF;

    uint8_t partial = nibble_lut[nib7] ^ nib6 ^ nibble_lut[nib5] ^ nib4 ^
                       nibble_lut[nib3] ^ nib2 ^ nibble_lut[nib1];
    uint8_t nib0 = partial; /* direct XOR position, so this zeroes the sum */

    uint32_t id = ((uint32_t)nib7 << 28) | ((uint32_t)nib6 << 24) |
                   ((uint32_t)nib5 << 20) | ((uint32_t)nib4 << 16) |
                   ((uint32_t)nib3 << 12) | ((uint32_t)nib2 << 8)  |
                   ((uint32_t)nib1 << 4)  | (uint32_t)nib0;
    return id;
}
