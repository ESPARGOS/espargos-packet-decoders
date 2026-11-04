// SPDX-License-Identifier: LGPL-3.0-or-later
/* Sensor-side admission for packet IQ. Energy only selects local candidates;
 * BLE preamble, access address, header, packet extent and modulation qualify
 * transmission. Advertising CRC is checked; unknown connection CRC seeds are
 * explicitly unverified. Keep signal conventions aligned with ble_decoder.js. */
#include "epd/ble.h"
#include <string.h>
#include <stdlib.h>

static bool address_valid(uint32_t aa)
{
    if (aa == 0x8e89bed6u) return true;
    uint32_t difference = aa ^ 0x8e89bed6u;
    if (!(difference & (difference - 1))) return false;
    unsigned transitions = 0, last = aa & 1, run = 1, upper = 0;
    for (unsigned i = 1; i < 32; i++) {
        unsigned bit = (aa >> i) & 1;
        if (bit == last) { if (++run > 6) return false; }
        else { transitions++; if (i >= 27) upper++; run = 1; }
        last = bit;
    }
    return aa != (aa & 255u) * 0x01010101u && transitions <= 24 && upper >= 2;
}

bool epd_ble_qualify(const uint32_t *iq, unsigned capacity, unsigned offset,
    unsigned words, unsigned rate, unsigned phy, unsigned channel,
    int16_t *work, size_t work_words, epd_ble_packet_t *out)
{
    if (!iq || !work || !out || !capacity || offset >= capacity || words > capacity ||
        words < 128 || phy > 1 || channel > 39 ||
        (rate != 4000000 && rate != 8000000 && rate != 16000000)) return false;
    unsigned target = phy == 1 ? 8 : 4, preamble = phy == 1 ? 16 : 8;
    unsigned step = rate / (target * 1000000u);
    if (!step) return false;
    unsigned count = words / step;
    if (count > work_words || count < (preamble + 72) * 4) return false;
    int hi[4] = {0}, hq[4] = {0}, previous[4] = {0};
    int si = 0, sq = 0, pi = 0, pq = 0, sum = 0;
    unsigned taps = target / 2;
    for (unsigned n = 0; n < count; n++) {
        uint32_t sample = iq[(offset + n * step) % capacity];
        int i = (int)((sample & 1023) ^ 512) - 512;
        int q = (int)(((sample >> 10) & 1023) ^ 512) - 512;
        if (target == 4) { if (n & 1) { i = -i; q = -q; } }
        else {
            int a = i;
            switch (n & 3) {
            case 1: i = q; q = -a; break;
            case 2: i = -i; q = -q; break;
            case 3: i = -q; q = a; break;
            }
        }
        unsigned slot = n & (taps - 1);
        si += i - hi[slot]; sq += q - hq[slot]; hi[slot] = i; hq[slot] = q;
        int power = (si * si + sq * sq + pi * pi + pq * pq) / 2;
        int value = power ? ((pq * si - pi * sq) * 128 / power) * 8 : 0;
        pi = si; pq = sq;
        sum += value - previous[n & 3]; previous[n & 3] = value;
        work[n] = sum / 4;
    }
    if (phy == 0) {
        /* LE Coded has an 80-symbol preamble. Keep this distinct from identity
         * and CRC: the existing host FEC decoder still decodes these captures. */
        static const int pattern[8] = {-1,-1,1,1,1,1,-1,-1};
        for (unsigned start = 8; start < 256 && start + (376 + 43 * 2) * 4 < count; start++) {
            int sx = 0, xy = 0, xx = 0;
            for (unsigned k = 0; k < 80; k++) {
                int x = work[start + k * 4];
                sx += x; xy += x * pattern[k & 7]; xx += x * x;
            }
            int center = sx / 80, amplitude = xy / 80;
            int variance = xx / 80 - center * center;
            if (abs(center) <= 400 && amplitude >= 80 && amplitude <= 600 && variance > 0 &&
                (int64_t)amplitude * amplitude * 100 >= (int64_t)variance * 85) {
                *out = (epd_ble_packet_t){0};
                return true;
            }
        }
    }
    bool found = false;
    for (unsigned start = 8; start < 64 * target && start + (preamble + 72) * 4 < count; start++) {
        int sx = 0, xy = 0, xx = 0;
        for (unsigned k = 0; k < preamble; k++) {
            int x = work[start + k * 4];
            sx += x; xx += x * x; xy += k & 1 ? x : -x;
        }
        int center = sx / (int)preamble, amplitude = xy / (int)preamble;
        int variance = xx / (int)preamble - center * center;
        if (abs(center) > 400 || abs(amplitude) < 80 || abs(amplitude) > 600 ||
            variance <= 0 || (int64_t)amplitude * amplitude * 100 < (int64_t)variance * 90) continue;
        unsigned begin = start + preamble * 4;
        uint32_t aa = 0;
        for (unsigned bit = 0; bit < 32; bit++)
            if (work[begin + bit * 4] > center) aa |= 1u << bit;
        if ((aa & 1) != (unsigned)(amplitude < 0) || !address_valid(aa)) continue;
        uint8_t pdu[261];
        unsigned whitening = channel | 64, size = 2, bytes = 0;
        bool advertising = aa == 0x8e89bed6u;
        while (bytes < size + 3) {
            if (begin + (32 + (bytes + 1) * 8) * 4 >= count) break;
            unsigned value = 0;
            for (unsigned bit = 0; bit < 8; bit++) {
                value |= ((work[begin + (32 + bytes * 8 + bit) * 4] > center) ^ (whitening & 1)) << bit;
                whitening = (whitening >> 1) ^ ((whitening & 1) ? 0x44 : 0);
            }
            pdu[bytes++] = value;
            if (bytes == 2) {
                size = value + 2 + (!advertising && !!(pdu[0] & 0x20));
                if (advertising) {
                    unsigned kind = pdu[0] & 15;
                    if (kind > 7 || (pdu[0] & 16) || (kind != 7 && (value < 6 || value > 37))) break;
                } else if (!(pdu[0] & 3) || (pdu[0] & 0xc0)) break;
            }
        }
        if (bytes != size + 3) continue;
        uint32_t received_crc = pdu[size] | (uint32_t)pdu[size + 1] << 8 | (uint32_t)pdu[size + 2] << 16;
        uint32_t crc = 0xaaaaaa;
        if (advertising) for (unsigned byte = 0; byte < size; byte++)
            for (unsigned bit = 0; bit < 8; bit++)
                crc = (crc >> 1) ^ (((crc ^ (pdu[byte] >> bit)) & 1) ? 0xda6000 : 0);
        bool verified = advertising && crc == received_crc;
        if (!verified) {
            /* Retain identifiable BLE waveforms for cross-antenna recovery,
             * without labeling an unknown or failed CRC as valid. */
            unsigned n = 32 + (size + 3) * 8;
            if (count - (begin + n * 4) > 64 * target) continue;
            uint64_t error = 0;
            for (unsigned k = 0; k < n; k++) {
                int e = abs(work[begin + k * 4] - center) - 330;
                error += e * e;
            }
            if (error / n > 18000) continue;
        }
        if (!found || verified) {
            *out = (epd_ble_packet_t){.access_address = aa, .packet_crc = received_crc,
                .crc_valid = verified, .identity_valid = true, .prefix_length = size < 32 ? size : 32};
            memcpy(out->prefix, pdu, out->prefix_length);
            found = true;
        }
        if (verified) return true;
    }
    return found;
}
