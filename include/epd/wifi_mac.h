// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Bytes required to identify a received MAC header. This is deliberately a
 * prefix check: payload length and FCS evidence belong to the decoder. */
static inline unsigned epd_wifi_mac_header_length(const uint8_t *frame, unsigned bytes) {
    if (!bytes || (frame[0] & 3))
        return 0;
    unsigned type = (frame[0] >> 2) & 3, subtype = frame[0] >> 4;
    unsigned minimum = type == 0 || type == 2                          ? 24
                       : type == 1 && subtype >= 8 && subtype <= 11    ? 16
                       : type == 1 && (subtype == 12 || subtype == 13) ? 10
                                                                       : 0;
    return bytes >= minimum ? minimum : 0;
}

static inline bool epd_wifi_mac_has_transmitter(const uint8_t *frame, unsigned bytes) {
    return epd_wifi_mac_header_length(frame, bytes) >= 16 && !(frame[10] & 1) &&
           (frame[10] | frame[11] | frame[12] | frame[13] | frame[14] | frame[15]);
}
