#ifndef HDHR_DEVICE_ID_H
#define HDHR_DEVICE_ID_H

#include <stdint.h>
#include <stdbool.h>

/* Real HDHomeRun device IDs are self-checking: a nibble-XOR checksum over
 * the 8 hex digits must equal zero once each high nibble is passed through
 * a fixed 4-bit lookup table. The official SiliconDust apps (and
 * hdhomerun_config) reject IDs that fail this check as "not a genuine
 * device"; third-party DVR software (Plex/Emby/Jellyfin/Channels DVR)
 * doesn't care either way. We implement it anyway for full fidelity. */

bool hdhr_device_id_is_valid(uint32_t device_id);

/* Takes any 24-bit seed (e.g. derived from the Pi's serial/MAC) and
 * returns a 32-bit device id with a valid checksum, in the same style
 * real devices use: top nibble is a fixed device-class digit (we use 0x1,
 * matching early tuner-class ids), followed by 6 seed nibbles, with the
 * low nibble solved for so the checksum comes out to zero. */
uint32_t hdhr_device_id_generate(uint32_t seed24);

#endif /* HDHR_DEVICE_ID_H */
