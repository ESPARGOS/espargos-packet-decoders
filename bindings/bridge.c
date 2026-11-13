// SPDX-License-Identifier: LGPL-3.0-or-later
#include "bridge.h"
#include "epd/wifi.h"
#include "epd/ble.h"
#include <string.h>
size_t epd_workspace_size(void) {
    size_t wifi = epd_wifi_qualify_workspace_size();
    return wifi > 32768 ? wifi : 32768;
}
static int wifi_result(const epd_wifi_packet_t *p, uint32_t *r) {
    r[0] = 1; r[1] = p->phy; r[2] = p->rate_half_mbps; r[3] = p->mcs; r[4] = p->stbc;
    r[5] = p->frame_start; r[6] = p->training_start; r[7] = p->training_words;
    r[8] = p->length_bytes; r[9] = p->duration_us; r[10] = (uint32_t)p->cfo_hz; r[11] = p->quality;
    r[12] = p->mac_valid | (p->crc_checked << 1) | (p->crc_valid << 2); r[13] = p->prefix_length;
    memcpy(r + 16, p->prefix, p->prefix_length);
    memcpy(r + 24, p->transmitter, 6);
    return 1;
}
int epd_decode_wifi(const uint32_t *iq, unsigned words, unsigned stage, int hint,
                    void *work, uint32_t r[EPD_RESULT_WORDS]) {
    if (!r) return 0;
    memset(r, 0, EPD_RESULT_WORDS * sizeof(*r));
    if (!iq || !work || !words || words > 16384) return 0;
    epd_wifi_packet_t p = {0};
    bool valid;
    switch (stage) {
    case 0: valid = epd_wifi_qualify(iq, words, 0, words, work, &p); break;
    case 1: valid = epd_wifi_qualify_ofdm_training(iq, words, work, &p); break;
    case 2: valid = epd_wifi_qualify_extended_header(iq, words, work, &p); break;
    case 3: case 4: valid = epd_wifi_qualify_native(iq, words, stage == 4, hint, work, &p); break;
    default: return 0;
    }
    if (!valid) return 0;
    return wifi_result(&p, r);
}

int epd_decode_wifi_ofdm(const float *symbols, unsigned count, void *work,
                         uint32_t r[EPD_RESULT_WORDS]) {
    if (!r) return 0;
    memset(r, 0, EPD_RESULT_WORDS*sizeof(*r));
    epd_wifi_packet_t p;
    return epd_wifi_decode_ofdm(symbols, count, work, &p) ? wifi_result(&p, r) : 0;
}
int epd_decode_ble(const uint32_t *iq, unsigned words, unsigned rate, unsigned phy,
                   unsigned channel, void *work, uint32_t r[EPD_RESULT_WORDS]) {
    if (!r) return 0;
    memset(r, 0, EPD_RESULT_WORDS * sizeof(*r));
    if (!iq || !work || !words || words > 16384) return 0;
    epd_ble_packet_t p = {0};
    if (!epd_ble_qualify(iq, words, 0, words, rate, phy, channel, work, 16384, &p)) return 0;
    r[0] = 1; r[1] = 100 + phy; r[12] = p.identity_valid | ((p.access_address == 0x8e89bed6u) << 1) | (p.crc_valid << 2);
    r[13] = p.prefix_length; r[14] = p.access_address; r[15] = p.packet_crc;
    memcpy(r + 16, p.prefix, p.prefix_length);
    return 1;
}
