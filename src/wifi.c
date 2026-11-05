// SPDX-License-Identifier: LGPL-3.0-or-later
#include "epd/wifi_mac.h"
#include "epd/wifi.h"
#include "wifi_training.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_WORDS 16384u
#define MAX_BITS 1024u
#define PI 3.14159265358979323846f
typedef struct {
    float r, i;
} cf;
typedef struct {
    cf inverse[64];
    float weights[64];
} channel_t;
typedef struct {
    bool native_header_only, extended_only;
    unsigned dsss_payload_start;
    bool dsss_pbcc;
    int16_t i[MAX_WORDS], q[MAX_WORDS];
    cf cck_samples[4096];
    uint8_t trace[MAX_BITS][64], bits[MAX_BITS];
    int16_t llr[MAX_BITS * 2];
    int32_t symbols_i[MAX_WORDS / 20 + 1], symbols_q[MAX_WORDS / 20 + 1];
    uint8_t serial[MAX_WORDS / 10 + 16], decoded[MAX_WORDS / 10 + 16];
} workspace_t;

size_t epd_wifi_qualify_workspace_size(void) { return sizeof(workspace_t); }
static cf add(cf a, cf b) { return (cf){a.r + b.r, a.i + b.i}; }
static cf mul(cf a, cf b) { return (cf){a.r * b.r - a.i * b.i, a.r * b.i + a.i * b.r}; }
static cf conjugate(cf a) { return (cf){a.r, -a.i}; }
static float power(cf a) { return a.r * a.r + a.i * a.i; }
static unsigned parity(unsigned x) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_parity(x);
#else
    x ^= x >> 16; x ^= x >> 8; x ^= x >> 4;
    return (0x6996u >> (x & 15)) & 1;
#endif
}

static void fft(cf *x) {
    for (unsigned i = 1, j = 0; i < 64; i++) {
        unsigned bit = 32;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            cf tmp = x[i];
            x[i] = x[j];
            x[j] = tmp;
        }
    }
    for (unsigned n = 2; n <= 64; n *= 2) {
        for (unsigned start = 0; start < 64; start += n) {
            for (unsigned j = 0; j < n / 2; j++) {
                unsigned k = j * (64 / n);
                cf phase = {wifi_fft_re[k], wifi_fft_im[k]};
                cf a = x[start + j], b = mul(x[start + j + n / 2], phase);
                x[start + j] = add(a, b);
                x[start + j + n / 2] = (cf){a.r - b.r, a.i - b.i};
            }
        }
    }
}

static void symbol(workspace_t *w, unsigned start, unsigned origin, float omega, cf *out) {
    int32_t si = 0, sq = 0;
    for (unsigned n = 0; n < 64; n++) {
        si += w->i[start + n];
        sq += w->q[start + n];
    }
    cf phase = {cosf(-omega * (start - origin)), sinf(-omega * (start - origin))};
    const cf step = {cosf(-omega), sinf(-omega)};
    for (unsigned n = 0; n < 64; n++) {
        cf sample = {w->i[start + n] - si / 64.0f, w->q[start + n] - sq / 64.0f};
        out[n] = mul(sample, phase);
        phase = mul(phase, step);
    }
    fft(out);
}

static void viterbi(workspace_t *w, unsigned count, bool terminated) {
    int32_t metric[64], next[64];
    for (unsigned s = 0; s < 64; s++)
        metric[s] = s ? -100000000 : 0;
    /* The two predecessors differ only in bit 6. Both generators include
     * that bit, so their branch metrics are exact opposites. Precompute the
     * four possible scores once per bit instead of repeated parity calls. */
    static const uint8_t outputs[64] = {0, 3, 2, 1, 3, 0, 1, 2, 3, 0, 1, 2, 0, 3, 2, 1,
                                        0, 3, 2, 1, 3, 0, 1, 2, 3, 0, 1, 2, 0, 3, 2, 1,
                                        1, 2, 3, 0, 2, 1, 0, 3, 2, 1, 0, 3, 1, 2, 3, 0,
                                        1, 2, 3, 0, 2, 1, 0, 3, 2, 1, 0, 3, 1, 2, 3, 0};
    for (unsigned n = 0; n < count; n++) {
        int a = w->llr[n * 2], b = w->llr[n * 2 + 1];
        const int scores[4] = {-a - b, a - b, -a + b, a + b};
        for (unsigned s = 0; s < 64; s++) {
            unsigned p = s >> 1;
            int score = scores[outputs[s]];
            bool high = metric[p | 32] - score > metric[p] + score;
            w->trace[n][s] = p | (high ? 32 : 0);
            next[s] = high ? metric[p | 32] - score : metric[p] + score;
        }
        memcpy(metric, next, sizeof(metric));
    }
    unsigned state = 0;
    if (!terminated)
        for (unsigned s = 1; s < 64; s++)
            if (metric[s] > metric[state])
                state = s;
    for (unsigned n = count; n--;) {
        w->bits[n] = state & 1;
        state = w->trace[n][state];
    }
}

static unsigned bit_value(const uint8_t *bits, unsigned count) {
    unsigned value = 0;
    for (unsigned n = 0; n < count; n++)
        value |= (unsigned)bits[n] << n;
    return value;
}

static uint32_t crc32(const uint8_t *data, unsigned count) {
    uint32_t crc = 0xffffffff;
    for (unsigned n = 0; n < count; n++) {
        crc ^= data[n];
        for (unsigned bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0);
    }
    return ~crc;
}

static void prefix(epd_wifi_packet_t *p, const uint8_t *bytes, unsigned count, bool complete) {
    p->prefix_length = count < sizeof(p->prefix) ? count : sizeof(p->prefix);
    memcpy(p->prefix, bytes, p->prefix_length);
    if (epd_wifi_mac_has_transmitter(bytes, count)) {
        memcpy(p->transmitter, bytes + 10, 6);
        p->mac_valid = true;
    }
    p->crc_checked = complete && count >= 4;
    if (p->crc_checked) {
        uint32_t expected = (uint32_t)bytes[count - 4] | ((uint32_t)bytes[count - 3] << 8) |
                            ((uint32_t)bytes[count - 2] << 16) | ((uint32_t)bytes[count - 1] << 24);
        p->crc_valid = crc32(bytes, count - 4) == expected;
        if (!p->crc_valid)
            p->mac_valid = false;
    }
}

static void deinterleave(const int16_t *in, int16_t *out, unsigned cbps, unsigned bpsc) {
    unsigned s = bpsc / 2;
    if (!s)
        s = 1;
    for (unsigned j = 0; j < cbps; j++) {
        unsigned first = s * (j / s) + (j + (16 * j) / cbps) % s;
        unsigned second = 16 * first - (cbps - 1) * ((16 * first) / cbps);
        out[second] = in[j];
    }
}

static void equalized(workspace_t *w, unsigned start, unsigned origin, float omega,
                      const channel_t *channel, unsigned pilot_index, cf *out, float *weights) {
    symbol(w, start, origin, omega, out);
    const int pilots[4] = {-21, -7, 7, 21};
    cf pilot = {0, 0};
    for (unsigned k = 0; k < 64; k++) {
        weights[k] = channel->weights[k];
        out[k] = mul(out[k], channel->inverse[k]);
    }
    for (unsigned n = 0; n < 4; n++) {
        unsigned bin = (pilots[n] + 64) % 64;
        cf value = out[bin];
        float sign = (n == 3 ? -1 : 1) * wifi_pilot_polarity[pilot_index % 127] * weights[bin];
        pilot.r += value.r * sign;
        pilot.i += value.i * sign;
    }
    float angle = atan2f(pilot.i, pilot.r);
    cf rotation = {cosf(angle), -sinf(angle)};
    for (unsigned k = 0; k < 64; k++)
        out[k] = mul(out[k], rotation);
}

static void bpsk_soft(const cf *bins, const float *weights, bool rotated, int16_t *out) {
    unsigned n = 0;
    for (int k = -26; k <= 26; k++) {
        if (!k || k == -21 || k == -7 || k == 7 || k == 21)
            continue;
        unsigned bin = (k + 64) % 64;
        float value = (rotated ? bins[bin].i : bins[bin].r) * weights[bin] * 1024;
        if (value > 4095)
            value = 4095;
        if (value < -4095)
            value = -4095;
        out[n++] = (int16_t)value;
    }
}

static bool ht_crc(const uint8_t *bits) {
    unsigned state = 255;
    for (unsigned n = 0; n < 34; n++) {
        unsigned feedback = ((state >> 7) & 1) ^ bits[n];
        state = ((state << 1) & 255) ^ (feedback ? 7 : 0);
    }
    for (unsigned n = 0; n < 8; n++)
        if (bits[34 + n] != (((state >> (7 - n)) & 1) ^ 1))
            return false;
    return true;
}

static int16_t soft_limit(float value) {
    if (value > 4095)
        value = 4095;
    if (value < -4095)
        value = -4095;
    return (int16_t)value;
}

static void constellation_soft(cf value, float weight, unsigned bpsc, int16_t *out) {
    if (bpsc == 1) {
        out[0] = soft_limit(value.r * weight * 1024);
        return;
    }
    /* IEEE 802.11-2012 18.3.5.8: the earliest bit controls I, not Q.
     * Axis tables below index that earliest/sign bit in the least significant
     * position, matching the serialized bit stream. */
    if (bpsc == 2) {
        out[0] = soft_limit(value.r * weight * 1024);
        out[1] = soft_limit(value.i * weight * 1024);
        return;
    }
    static const int8_t levels4[4] = {-3, 3, -1, 1};
    static const int8_t levels8[8] = {-7, 7, -1, 1, -5, 5, -3, 3};
    const int8_t *levels = bpsc == 4 ? levels4 : levels8;
    unsigned axis_bits = bpsc / 2, points = 1u << axis_bits;
    float norm = bpsc == 4 ? sqrtf(10) : sqrtf(42);
    for (unsigned axis = 0; axis < 2; axis++) {
        float coordinate = (axis ? value.i : value.r) * norm;
        for (unsigned bit = 0; bit < axis_bits; bit++) {
            float best0 = 1e30f, best1 = 1e30f;
            for (unsigned p = 0; p < points; p++) {
                float delta = coordinate - levels[p], distance = delta * delta;
                if ((p >> bit) & 1) {
                    if (distance < best1)
                        best1 = distance;
                } else if (distance < best0)
                    best0 = distance;
            }
            out[axis * axis_bits + bit] =
                soft_limit((best0 - best1) * weight * 256 / (norm * norm));
        }
    }
}

static void legacy_prefix(workspace_t *w, unsigned count, unsigned t, float omega,
                          const channel_t *channel, epd_wifi_packet_t *packet) {
    unsigned rate = packet->rate_half_mbps / 2;
    unsigned bpsc = rate < 12 ? 1 : rate < 24 ? 2 : rate < 48 ? 4 : 6;
    unsigned dbps = rate * 4, cbps = 48 * bpsc;
    unsigned available = (count - t - 208) / 80;
    /* Decode complete short packets when possible. Long packets need only
     * their 24-byte MAC prefix plus Viterbi traceback; decoding 96 bytes here
     * adds dead time without providing a full-packet FCS verdict. */
    unsigned desired = 16 + 8 * (packet->length_bytes <= 96 ? packet->length_bytes : 24) + 36;
    unsigned symbols = (desired + dbps - 1) / dbps;
    if (symbols > available)
        symbols = available;
    unsigned nbits = symbols * dbps;
    if (!symbols || nbits > MAX_BITS)
        return;
    unsigned coding = rate == 48                                              ? 2
                      : (rate == 9 || rate == 18 || rate == 36 || rate == 54) ? 3
                                                                              : 1;
    static const uint8_t patterns[3][6] = {
        {1, 1, 0, 0, 0, 0}, {1, 1, 1, 0, 0, 0}, {1, 1, 1, 0, 0, 1}};
    unsigned period = coding * 2;
    unsigned encoded = 0;
    cf bins[64];
    float weights[64];
    int16_t interleaved[288], plain[288];
    for (unsigned s = 0; s < symbols; s++) {
        equalized(w, t + 224 + s * 80, t, omega, channel, s + 1, bins, weights);
        unsigned position = 0;
        for (int k = -26; k <= 26; k++) {
            if (!k || k == -21 || k == -7 || k == 7 || k == 21)
                continue;
            unsigned bin = (k + 64) % 64;
            constellation_soft(bins[bin], weights[bin], bpsc, interleaved + position);
            position += bpsc;
        }
        deinterleave(interleaved, plain, cbps, bpsc);
        for (unsigned n = 0; n < cbps; n++) {
            while (!patterns[coding - 1][encoded % period])
                w->llr[encoded++] = 0;
            w->llr[encoded++] = plain[n];
        }
    }
    while (encoded < nbits * 2)
        w->llr[encoded++] = 0;
    unsigned tail = 16 + packet->length_bytes * 8 + 6;
    bool complete = nbits >= tail;
    if (complete)
        nbits = tail;
    viterbi(w, nbits, complete);
    unsigned seed = 0;
    for (unsigned candidate = 1; candidate < 128; candidate++) {
        unsigned state = candidate;
        bool zero = true;
        for (unsigned n = 0; n < 16; n++) {
            unsigned feedback = ((state >> 6) ^ (state >> 3)) & 1;
            if ((w->bits[n] ^ feedback) != 0)
                zero = false;
            state = ((state << 1) & 127) | feedback;
        }
        if (zero) {
            seed = candidate;
            break;
        }
    }
    if (!seed)
        return;
    unsigned state = seed;
    for (unsigned n = 0; n < nbits; n++) {
        unsigned feedback = ((state >> 6) ^ (state >> 3)) & 1;
        w->bits[n] ^= feedback;
        state = ((state << 1) & 127) | feedback;
    }
    unsigned margin = complete ? 6 : 30;
    if (nbits < 16 + margin)
        return;
    unsigned bytes = (nbits - 16 - margin) / 8;
    if (bytes > packet->length_bytes)
        bytes = packet->length_bytes;
    uint8_t payload[104];
    if (bytes > sizeof(payload))
        bytes = sizeof(payload);
    for (unsigned n = 0; n < bytes; n++)
        payload[n] = bit_value(w->bits + 16 + n * 8, 8);
    prefix(packet, payload, bytes, complete && bytes == packet->length_bytes);
}

static bool ofdm_header(workspace_t *w, unsigned count, unsigned t, float omega,
                        epd_wifi_packet_t *p) {
    if (t + 208 > count)
        return false;
    channel_t channel;
    cf second[64], bins[64];
    float weights[64];
    int16_t soft[48];
    symbol(w, t, t, omega, channel.inverse);
    symbol(w, t + 64, t, omega, second);
    float maximum = 1;
    for (unsigned k = 0; k < 64; k++) {
        float sign = wifi_ltf_sign[(k + 32) % 64];
        cf h = {(channel.inverse[k].r + second[k].r) * .5f * sign,
                (channel.inverse[k].i + second[k].i) * .5f * sign};
        channel.weights[k] = power(h);
        if (channel.weights[k] > maximum)
            maximum = channel.weights[k];
        float denominator = channel.weights[k] > 1 ? channel.weights[k] : 1;
        channel.inverse[k] = (cf){h.r / denominator, -h.i / denominator};
    }
    /* Channel inversion is shared by every symbol of this packet. Software
     * float division is expensive on C61; do it once per subcarrier. */
    for (unsigned k = 0; k < 64; k++)
        channel.weights[k] /= maximum;
    equalized(w, t + 144, t, omega, &channel, 0, bins, weights);
    bpsk_soft(bins, weights, false, soft);
    deinterleave(soft, w->llr, 48, 1);
    viterbi(w, 24, true);
    unsigned rate = 0, par = 0, errors = 0, state = 0;
    for (unsigned n = 0; n < 4; n++)
        rate = (rate << 1) | w->bits[n];
    for (unsigned n = 0; n < 18; n++)
        par ^= w->bits[n];
    for (unsigned n = 0; n < 24; n++) {
        state = ((state << 1) | w->bits[n]) & 127;
        errors += parity(state & 0155) != (w->llr[n * 2] > 0);
        errors += parity(state & 0117) != (w->llr[n * 2 + 1] > 0);
    }
    static const uint8_t rates[16] = {0, 48, 0, 0, 0, 12, 0, 0, 0, 24, 0, 0, 0, 6, 0, 0};
    unsigned mbps = rates[rate];
    if (rate == 3)
        mbps = 54;
    if (rate == 7)
        mbps = 18;
    if (rate == 11)
        mbps = 36;
    if (rate == 15)
        mbps = 9;
    unsigned length = bit_value(w->bits + 5, 12);
    if (!mbps || !length || w->bits[4] || par || errors > 12)
        return false;
    uint8_t legacy_signal[24];
    memcpy(legacy_signal, w->bits, 24);
    *p = (epd_wifi_packet_t){.frame_start = t >= 192 ? t - 192 : UINT16_MAX,
                                .training_start = t,
                                .training_words = 128,
                                .length_bytes = length,
                                .phy = EPD_WIFI_PHY_OFDM,
                                .rate_half_mbps = mbps * 2,
                                .cfo_hz = (int32_t)(omega * 20000000 / (2 * PI)),
                                .quality = 255 - errors * 10};
    if (w->native_header_only)
        return true;
    if (t + 368 <= count) {
        equalized(w, t + 224, t, omega, &channel, 1, bins, weights);
        bpsk_soft(bins, weights, false, soft);
        deinterleave(soft, w->llr, 48, 1);
        viterbi(w, 24, true);
        if (!memcmp(w->bits, legacy_signal, 24)) {
            /* HE's repeated L-SIG is not a legacy MAC payload. Its common
             * training remains useful even without decoding HE-SIG/payload. */
            p->phy = EPD_WIFI_PHY_EXTENDED_OFDM;
            return true;
        }
        float real_energy = 0, rotated_energy = 0;
        for (unsigned n = 0; n < 2; n++) {
            equalized(w, t + 224 + 80 * n, t, omega, &channel, n + 1, bins, weights);
            for (int k = -26; k <= 26; k++) {
                if (!k || k == -21 || k == -7 || k == 7 || k == 21)
                    continue;
                unsigned bin = (k + 64) % 64;
                real_energy += bins[bin].r * bins[bin].r * weights[bin];
                rotated_energy += bins[bin].i * bins[bin].i * weights[bin];
            }
            bpsk_soft(bins, weights, true, soft);
            deinterleave(soft, w->llr + 48 * n, 48, 1);
        }
        viterbi(w, 48, true);
        /* CRC alone can accidentally match a legacy payload. HT-SIG must
         * also exhibit its rotated BPSK modulation and a defined MCS. */
        if (rotated_energy > 2 * real_energy && bit_value(w->bits, 7) <= 76 && w->bits[26] &&
            ht_crc(w->bits)) {
            p->phy = EPD_WIFI_PHY_HT;
            p->mcs = bit_value(w->bits, 7);
            p->stbc = bit_value(w->bits + 28, 2);
            p->length_bytes = bit_value(w->bits + 8, 16);
            p->prefix_length = 6;
            for (unsigned n = 0; n < 6; n++)
                p->prefix[n] = bit_value(w->bits + n * 8, 8);
            return true;
        }
    }
    if (w->extended_only) return false;
    legacy_prefix(w, count, t, omega, &channel, p);
    return true;
}

static float ltf_score(workspace_t *w, unsigned t, int64_t e) {
    int64_t r = 0, i = 0, reference_energy = 0;
    for (unsigned n = 0; n < 64; n++) {
        r += (int32_t)w->i[t + n] * wifi_ltf_i[n] + (int32_t)w->q[t + n] * wifi_ltf_q[n];
        i += (int32_t)w->q[t + n] * wifi_ltf_i[n] - (int32_t)w->i[t + n] * wifi_ltf_q[n];
        reference_energy +=
            (int32_t)wifi_ltf_i[n] * wifi_ltf_i[n] + (int32_t)wifi_ltf_q[n] * wifi_ltf_q[n];
    }
    return e > 0 ? (float)(r * r + i * i) / ((float)e * reference_energy) : 0;
}

static bool ofdm(workspace_t *w, unsigned count, epd_wifi_packet_t *packet) {
    unsigned required = w->native_header_only ? 128 : 208;
    if (count < required)
        return false;
    /* Each rolling sum covers only 64 signed 10-bit complex samples and
     * fits int32. Widen products below, not every addition in the RV32 scan. */
    int32_t cross_r = 0, cross_i = 0, e0 = 0, e1 = 0, si0 = 0, sq0 = 0, si1 = 0, sq1 = 0;
    for (unsigned n = 0; n < 64; n++) {
        int32_t a = w->i[n], b = w->q[n], c = w->i[n + 64], d = w->q[n + 64];
        cross_r += (int64_t)a * c + (int64_t)b * d;
        cross_i += (int64_t)a * d - (int64_t)b * c;
        e0 += (int64_t)a * a + (int64_t)b * b;
        e1 += (int64_t)c * c + (int64_t)d * d;
        si0 += a;
        sq0 += b;
        si1 += c;
        sq1 += d;
    }
    struct {
        unsigned t;
        float score, omega;
    } candidates[8] = {0};
    unsigned used = 0;
    for (unsigned t = 0; t + required <= count; t++) {
        int64_t a = e0 - ((int64_t)si0 * si0 + (int64_t)sq0 * sq0) / 64,
                b = e1 - ((int64_t)si1 * si1 + (int64_t)sq1 * sq1) / 64;
        int64_t r = cross_r - ((int64_t)si0 * si1 + (int64_t)sq0 * sq1) / 64,
                i = cross_i - ((int64_t)si0 * sq1 - (int64_t)sq0 * si1) / 64;
        if (a > 64 && b > 64 && (r * r + i * i) * 2 > a * b) {
            float score = ltf_score(w, t, a);
            if (score > .22f && ltf_score(w, t + 64, b) > .16f) {
                unsigned slot = used;
                for (unsigned n = 0; n < used; n++)
                    if (t - candidates[n].t < 16) {
                        slot = n;
                        break;
                    }
                if (slot == used && used == 8) {
                    slot = 0;
                    for (unsigned n = 1; n < 8; n++)
                        if (candidates[n].score < candidates[slot].score)
                            slot = n;
                }
                if (slot == used || score > candidates[slot].score) {
                    candidates[slot].t = t;
                    candidates[slot].score = score;
                    candidates[slot].omega = atan2f((float)i, (float)r) / 64;
                    if (slot == used)
                        used++;
                }
            }
        }
        if (t + required == count)
            break;
        int32_t ai = w->i[t], aq = w->q[t], bi = w->i[t + 64], bq = w->q[t + 64],
                ci = w->i[t + 128], cq = w->q[t + 128];
        cross_r += (int64_t)bi * ci + (int64_t)bq * cq - (int64_t)ai * bi - (int64_t)aq * bq;
        cross_i += (int64_t)bi * cq - (int64_t)bq * ci - (int64_t)ai * bq + (int64_t)aq * bi;
        e0 += (int64_t)bi * bi + (int64_t)bq * bq - (int64_t)ai * ai - (int64_t)aq * aq;
        e1 += (int64_t)ci * ci + (int64_t)cq * cq - (int64_t)bi * bi - (int64_t)bq * bq;
        si0 += bi - ai;
        sq0 += bq - aq;
        si1 += ci - bi;
        sq1 += cq - bq;
    }
    if (w->native_header_only) {
        if (!used) return false;
        unsigned best = 0;
        for (unsigned n = 1; n < used; n++) if (candidates[n].score > candidates[best].score) best = n;
        unsigned t = candidates[best].t;
        *packet = (epd_wifi_packet_t){.phy = EPD_WIFI_PHY_OFDM,
            .frame_start = t >= 192 ? t - 192 : UINT16_MAX,
            .training_start = t, .training_words = 128,
            .cfo_hz = (int32_t)(candidates[best].omega * 20000000 / (2*PI)),
            .quality = (uint8_t)(255 * fminf(1, candidates[best].score))};
        return true;
    }
    bool valid = false;
    for (unsigned n = 0; n < used; n++) {
        epd_wifi_packet_t p;
        if (ofdm_header(w, count, candidates[n].t, candidates[n].omega, &p) &&
            (!valid || p.training_start > packet->training_start + 128 ||
             (abs((int)p.training_start - packet->training_start) < 128 &&
              p.quality > packet->quality))) {
            *packet = p;
            valid = true;
        }
    }
    return valid;
}

static bool plcp_crc(const uint8_t *bits) {
    unsigned crc = 65535;
    for (unsigned n = 0; n < 32; n++) {
        unsigned feedback = ((crc >> 15) & 1) ^ bits[n];
        crc = ((crc << 1) & 65535) ^ (feedback ? 0x1021 : 0);
    }
    for (unsigned n = 0; n < 16; n++)
        if (bits[32 + n] != (((crc >> (15 - n)) & 1) ^ 1))
            return false;
    return true;
}

/* Differential 2 Mbps symbols, with the scrambler continuing across PLCP. */
static unsigned dqpsk(workspace_t *w, unsigned first, unsigned symbols, float rotation,
                      unsigned state, uint8_t *out) {
    cf correction = {cosf(rotation), -sinf(rotation)};
    for (unsigned k = 0; k < symbols; k++) {
        unsigned pos = first + k;
        cf a = {w->symbols_i[pos], w->symbols_q[pos]},
           b = {w->symbols_i[pos + 1], w->symbols_q[pos + 1]};
        cf delta = mul(mul(conjugate(a), b), correction);
        unsigned quadrant =
            fabsf(delta.r) > fabsf(delta.i) ? (delta.r >= 0 ? 0 : 2) : (delta.i >= 0 ? 1 : 3);
        static const uint8_t dibits[4] = {0, 2, 3, 1};
        for (unsigned bit = 0; bit < 2; bit++) {
            unsigned coded = (dibits[quadrant] >> bit) & 1;
            out[k * 2 + bit] = coded ^ ((state >> 3) & 1) ^ ((state >> 6) & 1);
            state = ((state << 1) | coded) & 127;
        }
    }
    return state;
}

/* Fit a short complex equalizer to the known PLCP Barker chips before
 * correlating CCK codewords. This compensates front-end and multipath ISI;
 * regularization bounds amplification when the training matrix is ill-conditioned. */
#define CCK_TAPS 7
static cf cdiv(cf a, cf b) {
    float e = power(b);
    return e > 1e-12f ? mul(a, (cf){b.r / e, -b.i / e}) : (cf){0};
}
static cf cck_filter(workspace_t *w, unsigned n, const cf *weights) {
    cf y = {0};
    for (unsigned j = 0; j < CCK_TAPS; j++)
        y = add(y, mul(weights[j], w->cck_samples[n + j - CCK_TAPS / 2]));
    return y;
}
static void cck_equalizer(workspace_t *w, unsigned start, unsigned end, float omega, cf *weights) {
    const unsigned origin = start - 180;
    cf carrier = {cosf(omega * 180), sinf(omega * 180)}, step = {cosf(-omega), sinf(-omega)};
    for (unsigned n = origin; n < end; n++) {
        w->cck_samples[n - origin] = mul((cf){w->i[n], w->q[n]}, carrier);
        carrier = mul(carrier, step);
    }
    cf matrix[CCK_TAPS][CCK_TAPS + 1] = {0};
    float template_energy = 0;
    for (unsigned k = 0; k < 20; k++)
        template_energy += (float)wifi_barker20[k] * wifi_barker20[k];
    for (unsigned symbol = 1; symbol < 8; symbol++) {
        unsigned offset = 20 + symbol * 20;
        cf phase = {0};
        for (unsigned k = 0; k < 20; k++) {
            phase.r += w->cck_samples[offset + k].r * wifi_barker20[k];
            phase.i += w->cck_samples[offset + k].i * wifi_barker20[k];
        }
        phase.r /= template_energy;
        phase.i /= template_energy;
        for (unsigned k = 0; k < 20; k++) {
            cf target = {phase.r * wifi_barker20[k], phase.i * wifi_barker20[k]};
            for (unsigned j = 0; j < CCK_TAPS; j++) {
                cf x = conjugate(w->cck_samples[offset + k + j - CCK_TAPS / 2]);
                for (unsigned t = 0; t < CCK_TAPS; t++)
                    matrix[j][t] =
                        add(matrix[j][t], mul(x, w->cck_samples[offset + k + t - CCK_TAPS / 2]));
                matrix[j][CCK_TAPS] = add(matrix[j][CCK_TAPS], mul(x, target));
            }
        }
    }
    float ridge = 0;
    for (unsigned j = 0; j < CCK_TAPS; j++)
        ridge += matrix[j][j].r;
    for (unsigned j = 0; j < CCK_TAPS; j++)
        matrix[j][j].r += ridge * .001f / CCK_TAPS;
    for (unsigned j = 0; j < CCK_TAPS; j++) {
        unsigned pivot = j;
        for (unsigned k = j + 1; k < CCK_TAPS; k++)
            if (power(matrix[k][j]) > power(matrix[pivot][j]))
                pivot = k;
        if (pivot != j)
            for (unsigned k = j; k <= CCK_TAPS; k++) {
                cf z = matrix[j][k];
                matrix[j][k] = matrix[pivot][k];
                matrix[pivot][k] = z;
            }
        cf diagonal = matrix[j][j];
        for (unsigned k = j; k <= CCK_TAPS; k++)
            matrix[j][k] = cdiv(matrix[j][k], diagonal);
        for (unsigned row = 0; row < CCK_TAPS; row++)
            if (row != j) {
                cf factor = matrix[row][j];
                for (unsigned k = j; k <= CCK_TAPS; k++) {
                    cf v = mul(factor, matrix[j][k]);
                    matrix[row][k].r -= v.r;
                    matrix[row][k].i -= v.i;
                }
            }
    }
    for (unsigned j = 0; j < CCK_TAPS; j++)
        weights[j] = matrix[j][CCK_TAPS];
}

/* Decode CCK by correlating each eight-chip symbol with the 4/64 basic
 * codewords, then differentially decode its common phase. IEEE 802.11b
 * 18.4.6.5: the first payload symbol is even; the scrambler spans PLCP. */
static void cck_prefix(workspace_t *w, unsigned count, unsigned first, unsigned phase,
                       float rotation, unsigned state, epd_wifi_packet_t *packet) {
    unsigned bps = packet->rate_half_mbps == 11 ? 4 : 8;
    // Differential preamble coherence also peaks at incorrect chip phases.
    // Resolve chip timing with Barker matched-filter energy across PLCP.
    int shift = 0;
    float strongest = -1;
    for (int candidate = -10; candidate <= 10; candidate++) {
        float energy = 0;
        for (unsigned symbol = first - 7; symbol <= first; symbol++) {
            int t = (int)(phase + 20 * symbol) + candidate;
            if (t < 0 || t + 20 > (int)count)
                continue;
            int32_t i = 0, q = 0;
            for (unsigned k = 0; k < 20; k++) {
                i += w->i[t + k] * wifi_barker20[k];
                q += w->q[t + k] * wifi_barker20[k];
            }
            energy += (float)i * i + (float)q * q;
        }
        if (energy > strongest) {
            strongest = energy;
            shift = candidate;
        }
    }
    unsigned start = phase + 20 * (first + 1) + shift;
    if (start + 32 >= count)
        return;
    unsigned symbols = (count - start - 8) * 11 / 160;
    unsigned bytes = symbols * bps / 8;
    if (bytes > packet->length_bytes)
        bytes = packet->length_bytes;
    if (bytes > 104)
        bytes = 104;
    if (!bytes)
        return;
    symbols = bytes * 8 / bps;
    uint8_t payload[104] = {0};
    const float omega = rotation / 20;
    cf weights[CCK_TAPS];
    unsigned end = start + (symbols * 160 + 10) / 11 + 8;
    if (end > count)
        end = count;
    cck_equalizer(w, start, end, omega, weights);
    cf previous = {0};
    for (unsigned k = 0; k < 20; k++) {
        cf x = cck_filter(w, 160 + k, weights);
        previous.r += x.r * wifi_barker20[k];
        previous.i += x.i * wifi_barker20[k];
    }
    static const uint8_t dibits[4] = {0, 2, 3, 1};

    for (unsigned s = 0; s < symbols; s++) {
        cf chips[8];
        for (unsigned k = 0; k < 8; k++) {
            float t = 180 + (s * 8 + k) * (20.0f / 11);
            unsigned n = (unsigned)t;
            float f = t - n;
            cf a = cck_filter(w, n, weights), b = cck_filter(w, n + 1, weights);
            chips[k] = (cf){a.r + f * (b.r - a.r), a.i + f * (b.i - a.i)};
        }
        float best = -1;
        unsigned word = 0;
        cf common = {0};
        for (unsigned code = 0; code < (bps == 4 ? 4 : 64); code++) {
            unsigned p2 = bps == 4 ? 1 + 2 * (code & 1) : ((code & 1) << 1) | ((code >> 1) & 1);
            unsigned p3 = bps == 4 ? 0 : ((code >> 1) & 2) | ((code >> 3) & 1);
            unsigned p4 = bps == 4 ? code & 2 : ((code >> 3) & 2) | ((code >> 5) & 1);
            unsigned phases[8] = {p2 + p3 + p4, p3 + p4, p2 + p4, p4 + 2, p2 + p3, p3, p2 + 2, 0};
            cf sum = {0};
            for (unsigned k = 0; k < 8; k++) {
                cf x = chips[k];
                switch (phases[k] & 3) {
                case 0:
                    sum.r += x.r;
                    sum.i += x.i;
                    break;
                case 1:
                    sum.r += x.i;
                    sum.i -= x.r;
                    break;
                case 2:
                    sum.r -= x.r;
                    sum.i -= x.i;
                    break;
                case 3:
                    sum.r -= x.i;
                    sum.i += x.r;
                    break;
                }
            }
            float score = power(sum);
            if (score > best) {
                best = score;
                word = code;
                common = sum;
            }
        }
        cf delta = mul(conjugate(previous), common);
        if (s & 1) {
            delta.r = -delta.r;
            delta.i = -delta.i;
        }
        unsigned quadrant =
            fabsf(delta.r) > fabsf(delta.i) ? (delta.r >= 0 ? 0 : 2) : (delta.i >= 0 ? 1 : 3);
        unsigned coded = dibits[quadrant] | (word << 2);
        for (unsigned b = 0; b < bps; b++) {
            unsigned bit = (coded >> b) & 1;
            unsigned decoded = bit ^ ((state >> 3) & 1) ^ ((state >> 6) & 1);
            unsigned pos = s * bps + b;
            payload[pos / 8] |= decoded << (pos & 7);
            state = ((state << 1) | bit) & 127;
        }
        previous = common;
    }
    prefix(packet, payload, bytes, bytes == packet->length_bytes);
}

static bool dsss(workspace_t *w, unsigned count, epd_wifi_packet_t *packet) {
    if (count < 1920)
        return false;
    bool valid = false;
    float best = 0;
    /* Visit separated timing phases first; a CRC-qualified high-coherence
     * header does not need an exhaustive sub-sample timing search. */
    static const uint8_t phases[20] = {0, 10, 5, 15, 2, 12, 7, 17, 1, 11,
                                       6, 16, 3, 13, 8, 18, 4, 14, 9, 19};
    for (unsigned attempt = 0; attempt < 20; attempt++) {
        unsigned phase = phases[attempt];
        unsigned symbols = (count - phase) / 20;
        for (unsigned s = 0; s < symbols; s++) {
            int32_t i = 0, q = 0;
            unsigned t = phase + s * 20;
            for (unsigned n = 0; n < 20; n++) {
                i += w->i[t + n] * wifi_barker20[n];
                q += w->q[t + n] * wifi_barker20[n];
            }
            w->symbols_i[s] = i;
            w->symbols_q[s] = q;
            if (s) {
                int64_t cross = (int64_t)i * w->symbols_i[s - 1] + (int64_t)q * w->symbols_q[s - 1];
                w->serial[s - 1] = cross < 0;
            }
        }
        unsigned bits = symbols - 1;
        memset(w->decoded, 0, 7);
        for (unsigned n = 7; n < bits; n++)
            w->decoded[n] = w->serial[n] ^ w->serial[n - 4] ^ w->serial[n - 7];
        uint16_t sfd = bit_value(w->decoded + 47, 16);
        for (unsigned n = 48; n + 40 < bits; n++) {
            sfd = (sfd >> 1) | ((uint16_t)w->decoded[n + 15] << 15);
            const bool short_header = sfd == 0x05cf;
            if (sfd != 0xf3a0 && !short_header)
                continue;
            if (!short_header && n + 64 >= bits)
                continue;
            unsigned ones = 0;
            for (unsigned k = n - 40; k < n; k++)
                ones += w->decoded[k];
            if (short_header ? ones > 4 : ones < 36)
                continue;
            cf sum = {0, 0};
            for (unsigned k = n - 40; k < n; k++) {
                float a = w->symbols_i[k], b = w->symbols_q[k], c = w->symbols_i[k + 1],
                      d = w->symbols_q[k + 1];
                cf delta = {a * c + b * d, a * d - b * c};
                float e = power(delta);
                if (e > 0) {
                    sum.r += (delta.r * delta.r - delta.i * delta.i) / e;
                    sum.i += 2 * delta.r * delta.i / e;
                }
            }
            float quality = sqrtf(power(sum)) / 40;
            if (quality < .75f || (valid && quality <= best))
                continue;
            float rotation = atan2f(sum.i, sum.r) / 2;
            uint8_t short_bits[48];
            unsigned payload_state = 0;
            const uint8_t *header = w->decoded + n + 16;
            if (short_header) {
                for (unsigned k = n + 9; k < n + 16; k++)
                    payload_state = ((payload_state << 1) | w->serial[k]) & 127;
                payload_state = dqpsk(w, n + 16, 24, rotation, payload_state, short_bits);
                header = short_bits;
            }
            if (!plcp_crc(header))
                continue;
            unsigned signal = bit_value(header, 8), duration = bit_value(header + 16, 16);
            unsigned rate = signal == 10    ? 2
                            : signal == 20  ? 4
                            : signal == 55  ? 11
                            : signal == 110 ? 22
                                            : 0;
            if (!rate || !duration || (short_header && rate == 2))
                continue;
            epd_wifi_packet_t p = {.phy = EPD_WIFI_PHY_DSSS,
                                      .rate_half_mbps = rate,
                                      .duration_us = duration,
                                      .length_bytes = (duration * rate) / 16,
                                      .cfo_hz = (int32_t)(rotation * 1000000 / (2 * PI)),
                                      .quality = (uint8_t)(quality * 255)};
            if (rate == 22 && (bit_value(header + 8, 8) & 128) && p.length_bytes)
                p.length_bytes--;
            /* PBCC adds one octet to the PLCP length calculation. Its payload
             * is not CCK and must not be passed to the CCK correlator. */
            if ((rate == 11 || rate == 22) && header[11] && p.length_bytes)
                p.length_bytes--;
            int start = (int)phase + 20 * ((int)n + 1 - (short_header ? 56 : 128));
            p.frame_start = start >= 0 ? start : UINT16_MAX;
            p.training_start = phase + 20 * (n - 40);
            p.training_words = 800;
            if (rate == 2) {
                unsigned available = (bits - n - 64) / 8,
                         bytes = available < p.length_bytes ? available : p.length_bytes;
                uint8_t payload[104];
                if (bytes > sizeof(payload))
                    bytes = sizeof(payload);
                for (unsigned k = 0; k < bytes; k++)
                    payload[k] = bit_value(w->decoded + n + 64 + 8 * k, 8);
                prefix(&p, payload, bytes, bytes == p.length_bytes);
            } else if (rate == 4) {
                unsigned first = n + (short_header ? 40 : 64), available = (bits - first) * 2;
                unsigned bytes = available / 8;
                if (bytes > p.length_bytes)
                    bytes = p.length_bytes;
                uint8_t payload[104] = {0};
                if (bytes > sizeof(payload))
                    bytes = sizeof(payload);
                unsigned state = payload_state;
                if (!short_header)
                    for (unsigned k = first - 7; k < first; k++)
                        state = ((state << 1) | w->serial[k]) & 127;
                uint8_t decoded[104 * 8];
                dqpsk(w, first, bytes * 4, rotation, state, decoded);
                for (unsigned k = 0; k < bytes; k++)
                    payload[k] = bit_value(decoded + 8 * k, 8);
                prefix(&p, payload, bytes, bytes == p.length_bytes);
            }
            if ((rate == 11 || rate == 22) && !(header[11])) {
                unsigned first = n + (short_header ? 40 : 64), state = payload_state;
                if (!short_header)
                    for (unsigned k = first - 7; k < first; k++)
                        state = ((state << 1) | w->serial[k]) & 127;
                cck_prefix(w, count, first, phase, rotation, state, &p);
            }
            w->dsss_payload_start = phase + 20 * (n + (short_header ? 41 : 65));
            w->dsss_pbcc = header[11];
            *packet = p;
            best = quality;
            valid = true;
            if (p.crc_valid || (!p.crc_checked && quality > .90f))
                return true;
        }
    }
    return valid;
}

static bool qualify(const uint32_t *ring, uint32_t capacity, uint32_t offset, uint32_t words,
                    void *scratch, epd_wifi_packet_t *packet, bool ofdm_only, bool native_header_only, bool extended_only) {
    if (!ring || !scratch || !packet || !capacity || capacity > MAX_WORDS || offset >= capacity ||
        words > capacity || words < 128)
        return false;
    workspace_t *w = scratch;
    w->native_header_only = native_header_only;
    w->extended_only = extended_only;
    *packet = (epd_wifi_packet_t){0};
    for (unsigned n = 0; n < words; n++) {
        uint32_t raw = ring[(offset + n) % capacity];
        int i = raw & 1023, q = (raw >> 10) & 1023;
        if (i & 512)
            i -= 1024;
        if (q & 512)
            q -= 1024;
        w->i[n] = i;
        w->q[n] = -q;
    }
    return ofdm(w, words, packet) || (!ofdm_only && dsss(w, words, packet));
}

bool epd_wifi_qualify(const uint32_t *ring, uint32_t capacity, uint32_t offset, uint32_t words,
                         void *scratch, epd_wifi_packet_t *packet) {
    return qualify(ring, capacity, offset, words, scratch, packet, false, false, false);
}
bool epd_wifi_qualify_reference(const uint32_t *ring, uint32_t capacity, uint32_t offset,
                                   uint32_t words, void *scratch, epd_wifi_packet_t *packet) {
    return qualify(ring, capacity, offset, words, scratch, packet, true, false, false);
}

/* Native MAC validation supplies the packet identity and FCS. OFDM still
 * checks the known L-LTF around the native timestamp; it does not decode
 * the PHY header or payload again. DSSS validates the Barker training and estimates CFO before exporting
 * it. The caller supplies an approximate, common preamble position derived
 * from the hardware receive timestamp, not an independently selected symbol. */
bool epd_wifi_qualify_native(const uint32_t *samples, uint32_t words, bool dsss,
                               int training_hint, void *scratch, epd_wifi_packet_t *packet) {
    if (!dsss) {
        if (training_hint < -64 || training_hint > (int)words) return false;
        /* Native timing usually locates the L-LTF closely. Try that bounded
         * neighborhood first with the same waveform checks; retain the wider
         * search for unusual recorder/timestamp alignment. */
        unsigned near_begin = training_hint > 32 ? training_hint - 32 : 0;
        unsigned near_end = training_hint + 32 + 128;
        if (near_end > words) near_end = words;
        if (near_end >= near_begin + 128 &&
            qualify(samples, words, near_begin, near_end-near_begin, scratch, packet, true, true, false)) {
            packet->training_start += near_begin;
            packet->frame_start = packet->training_start >= 192 ? packet->training_start - 192 : UINT16_MAX;
            return true;
        }
        unsigned begin = training_hint > 192 ? training_hint - 192 : 0;
        unsigned end = training_hint + 192 + 128;
        if (end > words) end = words;
        if (end < begin + 128 || !qualify(samples, words, begin, end-begin, scratch, packet, true, true, false)) return false;
        packet->training_start += begin;
        packet->frame_start = packet->training_start >= 192 ? packet->training_start - 192 : UINT16_MAX;
        return true;
    }
    if (!samples || !scratch || !packet || words > MAX_WORDS || training_hint < 10 ||
        (uint32_t)training_hint + 250 > words)
        return false;
    unsigned symbols = (words - training_hint - 10) / 20;
    if (symbols > 40) symbols = 40;
    workspace_t *w = scratch;
    int best_start = training_hint;
    int64_t best_energy = 0;
    /* Locate the chip phase, preserving the common whole-symbol anchor. */
    for (int shift = -10; shift <= 10; shift++) {
        int64_t energy = 0;
        for (unsigned k = 0; k < symbols; k++) {
            int32_t r = 0, q = 0;
            for (unsigned n = 0; n < 20; n++) {
                uint32_t raw = samples[training_hint + shift + k * 20 + n];
                int i = ((raw & 1023) ^ 512) - 512;
                int j = (((raw >> 10) & 1023) ^ 512) - 512;
                r += i * wifi_barker20[n];
                q -= j * wifi_barker20[n];
            }
            energy += (int64_t)r * r + (int64_t)q * q;
            w->symbols_i[k] = r;
            w->symbols_q[k] = q;
        }
        if (energy > best_energy) { best_energy = energy; best_start = training_hint + shift; }
    }
    cf sum = {0};
    for (unsigned k = 0; k < symbols; k++) {
        int32_t r = 0, q = 0;
        for (unsigned n = 0; n < 20; n++) {
            uint32_t raw = samples[best_start + k * 20 + n];
            r += (int)(((raw & 1023) ^ 512) - 512) * wifi_barker20[n];
            q -= (int)((((raw >> 10) & 1023) ^ 512) - 512) * wifi_barker20[n];
        }
        w->symbols_i[k] = r; w->symbols_q[k] = q;
        if (k) {
            float a = w->symbols_i[k-1], b = w->symbols_q[k-1];
            cf delta = {a*r+b*q, a*q-b*r};
            float e = power(delta);
            if (e > 0) { sum.r += (delta.r*delta.r-delta.i*delta.i)/e; sum.i += 2*delta.r*delta.i/e; }
        }
    }
    float quality = sqrtf(power(sum)) / (symbols - 1);
    if (quality < .75f) return false;
    *packet = (epd_wifi_packet_t){.phy = EPD_WIFI_PHY_DSSS, .frame_start = UINT16_MAX,
        .training_start = best_start, .training_words = symbols * 20,
        .cfo_hz = (int32_t)(atan2f(sum.i,sum.r)*1000000/(4*PI)),
        .quality = (uint8_t)(quality*255)};
    return true;
}

bool epd_wifi_qualify_ofdm_training(const uint32_t *samples, uint32_t words,
                                       void *scratch, epd_wifi_packet_t *packet) {
    return qualify(samples, words, 0, words, scratch, packet, true, true, false);
}

bool epd_wifi_qualify_extended_header(const uint32_t *samples, uint32_t words,
                                         void *scratch, epd_wifi_packet_t *packet) {
    return qualify(samples, words, 0, words, scratch, packet, true, false, true);
}

bool epd_wifi_qualify_extended_at(const uint32_t *samples, uint32_t words,
                                    const epd_wifi_packet_t *training,
                                    void *scratch, epd_wifi_packet_t *packet) {
    if (!samples || !training || !scratch || !packet || words > MAX_WORDS)
        return false;
    workspace_t *w = scratch;
    unsigned start = training->training_start;
    float omega = training->cfo_hz * (2 * PI / 20000000);
    w->native_header_only = false;
    w->extended_only = true;
    for (unsigned n = 0; n < words; n++) {
        w->i[n] = (int)((samples[n] & 1023) ^ 512) - 512;
        w->q[n] = -((int)(((samples[n] >> 10) & 1023) ^ 512) - 512);
    }
    if (ofdm_header(w, words, start, omega, packet))
        return true;
    return epd_wifi_qualify_extended_header(samples, words, scratch, packet);
}

unsigned epd_wifi_dsss_symbols(const uint32_t *samples, unsigned words, unsigned kind,
                              unsigned hint, unsigned length, int cfo_hz, void *scratch,
                              float *out, unsigned capacity, unsigned *rate) {
    if (!samples || !scratch || !out || !rate || !capacity || words > MAX_WORDS ||
        kind > 1 || words < 200) return 0;
    workspace_t *w = scratch;
    // Crops can start inside SYNC. Leave search/descrambler history before
    // the real samples; no returned point is taken from this idle padding.
    unsigned pad = kind == 0 && words <= MAX_WORDS - 400 ? 400 : 0;
    memset(w->i, 0, pad * sizeof(w->i[0]));
    memset(w->q, 0, pad * sizeof(w->q[0]));
    for (unsigned n = 0; n < words; n++) {
        w->i[n + pad] = ((samples[n] & 1023) ^ 512) - 512;
        w->q[n + pad] = -((int)(((samples[n] >> 10) & 1023) ^ 512) - 512);
    }
    words += pad;
    unsigned start, end;
    if (kind == 0) {
        epd_wifi_packet_t p;
        if (!dsss(w, words, &p) || w->dsss_pbcc) return 0;
        start = w->dsss_payload_start;
        cfo_hz = p.cfo_hz;
        *rate = p.rate_half_mbps;
        unsigned duration = (p.length_bytes * 16u + *rate - 1) / *rate;
        end = start + duration * 20;
        if (end > words) end = words;
    } else {
        if (hint > words || length > words - hint || length < 200) return 0;
        start = hint + 180;
        end = hint + length;
        *rate = 2;
    }
    // Resolve the chip phase against known Barker symbols before the field.
    int shift = 0;
    float best = -1;
    for (int s = -10; s <= 10; s++) {
        if ((int)start + s < 180 || (int)end + s > (int)words) continue;
        float energy = 0;
        for (unsigned symbol = 1; symbol < 8; symbol++) {
            int t = (int)start + s - 180 + symbol * 20;
            cf z = {0};
            for (unsigned k = 0; k < 20; k++) {
                z.r += w->i[t+k] * wifi_barker20[k];
                z.i += w->q[t+k] * wifi_barker20[k];
            }
            energy += power(z);
        }
        if (energy > best) { best = energy; shift = s; }
    }
    if (best <= 0) return 0;
    start += shift; end += shift;
    if (end > start + 3800) end = start + 3800;
    if (end <= start + 24 || start < 180) return 0;
    cf weights[CCK_TAPS];
    cck_equalizer(w, start, end, cfo_hz * (2 * PI / 20000000), weights);
    cf previous = {0};
    for (unsigned k = 0; k < 20; k++) {
        cf x = cck_filter(w, 160+k, weights);
        previous.r += x.r * wifi_barker20[k];
        previous.i += x.i * wifi_barker20[k];
    }
    float phase = atan2f(previous.i, previous.r);
    cf carrier = {cosf(-phase), sinf(-phase)};
    unsigned count = 0;
    if (*rate <= 4) {
        for (unsigned n = 180; n + 23 < end - start + 180 && count < capacity; n += 20) {
            cf z = {0};
            for (unsigned k = 0; k < 20; k++) {
                cf x = cck_filter(w, n+k, weights);
                z.r += x.r * wifi_barker20[k]; z.i += x.i * wifi_barker20[k];
            }
            cf delta = mul(z, conjugate(previous));
            out[2*count] = delta.r; out[2*count+1] = delta.i;
            previous = z; count++;
        }
    } else {
        // CCK chips carry QPSK phases; keep the measured amplitudes and
        // phase errors rather than replacing them with decoded codewords.
        for (unsigned chip = 0; count < capacity; chip++) {
            float t = 180 + chip * (20.0f / 11);
            unsigned n = (unsigned)t;
            if (n + 4 >= end - start + 180) break;
            float f = t - n;
            cf a = cck_filter(w, n, weights), b = cck_filter(w, n+1, weights);
            cf z = mul((cf){a.r + f*(b.r-a.r), a.i + f*(b.i-a.i)}, carrier);
            out[2*count] = z.r; out[2*count+1] = z.i; count++;
        }
    }
    if (*rate > 4 && count) {
        // CCK has no payload pilots. Estimate one common residual carrier
        // phase from the fourth moment; retain the harmless quadrant ambiguity.
        // Never rotate individual decision points.
        cf fourth = {0};
        for (unsigned n = 0; n < count; n++) {
            cf z = {out[2*n], out[2*n+1]};
            float e = power(z);
            if (e > 1e-12f) {
                cf square = mul(z,z), x = mul(square,square);
                fourth.r += x.r/(e*e); fourth.i += x.i/(e*e);
            }
        }
        float angle = -atan2f(fourth.i, fourth.r)/4;
        cf correction = {cosf(angle), sinf(angle)};
        for (unsigned n = 0; n < count; n++) {
            cf z = mul((cf){out[2*n],out[2*n+1]},correction);
            out[2*n] = z.r; out[2*n+1] = z.i;
        }
    }
    float energy = 0;
    for (unsigned n = 0; n < 2*count; n++) energy += out[n]*out[n];
    if (!count || !(energy > 1e-12f) || !isfinite(energy)) return 0;
    float scale = sqrtf(count / energy);
    for (unsigned n = 0; n < 2*count; n++) out[n] *= scale;
    return count;
}

bool epd_wifi_qualify_dsss_training(const uint32_t *samples, unsigned words, int hint,
                                  void *scratch, epd_wifi_packet_t *packet) {
    if (!epd_wifi_qualify_native(samples, words, true, hint, scratch, packet)) return false;
    double matched = 0, total = 0, template_power = 0;
    for (unsigned k = 0; k < 20; k++) template_power += wifi_barker20[k] * wifi_barker20[k];
    for (unsigned s = 0; s < packet->training_words; s += 20) {
        double r = 0, q = 0, energy = 0;
        for (unsigned k = 0; k < 20; k++) {
            uint32_t raw = samples[packet->training_start + s + k];
            int i = (int)((raw & 1023) ^ 512) - 512;
            int j = (int)(((raw >> 10) & 1023) ^ 512) - 512;
            r += i * wifi_barker20[k]; q += j * wifi_barker20[k];
            energy += i*i + j*j;
        }
        matched += r*r + q*q;
        total += energy * template_power;
    }
    return total > 0 && matched > .55 * total;
}
