#ifndef HDHR_ATSC_FREQ_H
#define HDHR_ATSC_FREQ_H

#include <stdint.h>

/* US terrestrial ATSC channel plan (FCC-assigned center frequencies,
 * public allocation table — not vendor-specific).
 *
 * VHF-Lo (2-6) and VHF-Hi (7-13) have non-uniform gaps (channels 5→6 and
 * 6→7 don't follow the flat 6MHz step), so they're tabulated explicitly.
 * UHF (14-36) is a clean linear series: 473,000,000 + (ch-14)*6,000,000.
 *
 * Table stops at 36 — the FCC's 2017-2020 incentive-auction repack
 * vacated all US broadcast use of channels 37-51 (37 was already
 * reserved for radio astronomy even before that), so scanning them
 * would just burn time on RF that's never going to lock. If you're
 * outside the US repack (or need pre-repack behavior for some other
 * reason), extend the table back to 51 using the linear UHF formula
 * above. */

struct atsc_channel_freq {
    int      channel;      /* RF channel number, 2-36 */
    uint32_t frequency_hz; /* center frequency */
};

#define ATSC_FREQ_TABLE_COUNT 35

extern const struct atsc_channel_freq atsc_freq_table[ATSC_FREQ_TABLE_COUNT];

/* Reverse lookup: frequency -> RF channel number. Returns 0 (not a valid
 * ATSC channel number) if freq_hz isn't an exact match to a table entry
 * — e.g. a client tuned to an arbitrary frequency outside the known
 * plan. Used for the cosmetic "us-bcast:N" /tunerN/channel display. */
int atsc_freq_to_channel(uint32_t freq_hz);

/* Forward lookup: RF channel number -> frequency. Returns 0 if channel
 * isn't in the table (out of range, or in the skipped 37-51 repack
 * range — see the comment above). Used to resolve a "us-bcast:N"
 * /tunerN/channel SET value to an actual tuning frequency. */
uint32_t atsc_channel_to_freq(int channel);

#endif /* HDHR_ATSC_FREQ_H */
