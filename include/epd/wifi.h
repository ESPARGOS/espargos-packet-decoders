// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    EPD_WIFI_PHY_OFDM = 0,
    EPD_WIFI_PHY_DSSS = 1,
    EPD_WIFI_PHY_HT = 2,
    EPD_WIFI_PHY_EXTENDED_OFDM = 3
};

/* Offsets refer to the supplied, chronologically ordered capture window.
 * A decoded MAC prefix is distinct from a checked and valid whole-frame FCS. */
typedef struct {
    uint16_t frame_start, training_start, training_words, length_bytes;
    uint16_t duration_us;
    int32_t cfo_hz;
    uint8_t phy, rate_half_mbps, mcs, stbc, quality;
    uint8_t prefix_length, prefix[24], transmitter[6];
    bool mac_valid, crc_checked, crc_valid;
} epd_wifi_packet_t;

size_t epd_wifi_qualify_workspace_size(void);
/* 20 MS/s, signed 10-bit I/Q in the low 20 bits of each word. The raw tap's
 * Q polarity is converted here; transmitted samples remain untouched. */
bool epd_wifi_qualify(const uint32_t *ring, uint32_t capacity, uint32_t offset, uint32_t words,
                         void *workspace, epd_wifi_packet_t *packet);

/* Controller references use legacy OFDM. Skip unrelated DSSS traffic leaking
 * into the reference path, without changing antenna reception. */
bool epd_wifi_qualify_reference(const uint32_t *ring, uint32_t capacity, uint32_t offset,
                                   uint32_t words, void *scratch, epd_wifi_packet_t *packet);

/* Native packet metadata is validated separately; this checks its IQ training. */
bool epd_wifi_qualify_native(const uint32_t *samples, uint32_t words, bool dsss,
                               int training_hint, void *scratch, epd_wifi_packet_t *packet);

/* Locate valid L-LTF before spending time decoding an unsupported header. */
bool epd_wifi_qualify_ofdm_training(const uint32_t *samples, uint32_t words,
                                       void *scratch, epd_wifi_packet_t *packet);

/* Validate HT-SIG/repeated L-SIG without decoding a legacy MAC payload. */
bool epd_wifi_qualify_extended_header(const uint32_t *samples, uint32_t words,
                                         void *scratch, epd_wifi_packet_t *packet);

/* Decode at a previously validated L-LTF position; retains all PHY-header
 * checks. Falls back to a full search if that timing does not decode. */
bool epd_wifi_qualify_extended_at(const uint32_t *samples, uint32_t words,
                                    const epd_wifi_packet_t *training,
                                    void *scratch, epd_wifi_packet_t *packet);
