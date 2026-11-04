// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
