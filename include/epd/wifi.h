// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Common contract: input samples are read-only. Workspaces must have at least
 * epd_wifi_qualify_workspace_size() bytes and malloc-compatible alignment.
 * Do not share a workspace between concurrent calls or overlap it with input
 * or output. No pointer is retained after a call. A false return means no valid
 * result; packet contents must then be ignored. Success does not imply FCS.
 * Raw input capacity/length is at most 16384 sample words. Offsets and lengths
 * are in sample words, not bytes; circular offset must be less than capacity.
 * Contiguous-input functions use words as both buffer length and capacity. */

#ifdef __cplusplus
extern "C" {
#endif

enum {
    EPD_WIFI_PHY_OFDM = 0,
    EPD_WIFI_PHY_DSSS = 1,
    EPD_WIFI_PHY_HT = 2,
    EPD_WIFI_PHY_EXTENDED_OFDM = 3
};

/* Offsets refer to the supplied, chronologically ordered capture window.
 * UINT16_MAX means an unavailable frame/training offset. training_words=0
 * means no training was established. cfo_hz is in Hz; duration_us in us;
 * rate_half_mbps is in units of 0.5 Mbps. Only prefix_length bytes of prefix
 * are meaningful; transmitter requires mac_valid. crc_valid requires
 * crc_checked. A decoded prefix does not imply a verified whole-frame FCS. */
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

/* Soft 11b constellation samples, interleaved I,Q floats (never decisions).
 * kind=0: payload, requiring captured CRC-valid PLCP; kind=1: validated Barker
 * training at hint/length. Payload CCK returns equalized chip samples; DSSS
 * returns differential despread symbols. rate reports half-Mbps (2 for Barker).
 * Output capacity counts complex points. Returns zero if unavailable/invalid. */
unsigned epd_wifi_dsss_symbols(const uint32_t *samples, unsigned words, unsigned kind,
                              unsigned hint, unsigned length, int cfo_hz, void *scratch,
                              float *out, unsigned capacity, unsigned *rate);

/* Standalone Barker admission, additionally requiring the known chip waveform
 * (native_dsss alone relies on the modem's packet verdict). */
bool epd_wifi_qualify_dsss_training(const uint32_t *samples, unsigned words, int hint,
                                  void *scratch, epd_wifi_packet_t *packet);

/* Decode associated, equalized OFDM symbols without re-quantizing into IQ.
 * Each symbol has 64 FFT-order triples (real, imaginary, inverse noise variance).
 * The first symbol is L-SIG, followed by the consecutive 80-sample legacy
 * symbols (including HT-SIG when present). Pilots retain their transmitted
 * polarity; residual common phase is tracked by the decoder. Zero weight is
 * an erasure. All triples must be finite; |real| and |imaginary| <= 1e6.
 * Relative weights share one scale across the whole packet. This API validates
 * headers/FCS but supplies no training, timing or CFO evidence. */
#define EPD_WIFI_OFDM_MAX_SYMBOLS 200u
bool epd_wifi_decode_ofdm(const float *symbols, unsigned count, void *workspace,
                          epd_wifi_packet_t *packet);

#ifdef __cplusplus
}
#endif
