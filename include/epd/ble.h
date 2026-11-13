// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Input/work/output must not overlap; no pointers are retained. offset is less
 * than capacity; words is at most capacity. Lengths are sample words, not bytes.
 * phy=0 is LE1M, phy=1 is LE2M; channel is the logical BLE channel, 0..39.
 * sample_rate is 4000000, 8000000 or 16000000 (LE2M requires at least 8000000).
 * An int16_t scratch buffer of words elements is sufficient for either PHY.
 * Separate scratch buffers allow concurrent calls. False means no result;
 * ignore packet contents on failure. Success may describe only a preamble,
 * so check identity_valid before using access_address/prefix/packet_crc.
 * crc_valid verifies an advertising CRC; unknown connection CRCs stay false. */

/* Independent of the native scanner's access address and CRC seed. The input
 * is frozen, circular, signed 10-bit IQ at IF +2 MHz. No samples are modified. */
typedef struct {
    uint32_t access_address, packet_crc;
    uint8_t prefix[32], prefix_length;
    bool crc_valid, identity_valid;
} epd_ble_packet_t;
bool epd_ble_qualify(const uint32_t *iq, unsigned capacity, unsigned offset,
    unsigned words, unsigned sample_rate, unsigned phy, unsigned channel,
    int16_t *scratch, size_t scratch_words, epd_ble_packet_t *packet);

#ifdef __cplusplus
}
#endif
