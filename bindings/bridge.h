// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include <stdint.h>
#include <stddef.h>
/* Binding ABI 1: 26 native uint32 words. Fields 0..15 are scalar, bytes 64..95
 * hold the prefix, bytes 96..101 the transmitter. All outputs are zero on failure.
 * Stages: 0 full bounded decode, 1 OFDM training, 2 extended PHY header,
 * 3 native OFDM training at hint, 4 native DSSS training at hint. */
#define EPD_RESULT_WORDS 26
size_t epd_workspace_size(void);
int epd_decode_wifi(const uint32_t *iq, unsigned words, unsigned stage, int hint,
                    void *workspace, uint32_t result[EPD_RESULT_WORDS]);
int epd_decode_ble(const uint32_t *iq, unsigned words, unsigned rate, unsigned phy,
                   unsigned channel, void *workspace, uint32_t result[EPD_RESULT_WORDS]);

int epd_decode_wifi_ofdm(const float *symbols, unsigned count, void *workspace,
                         uint32_t result[EPD_RESULT_WORDS]);
