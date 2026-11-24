// SPDX-License-Identifier: LGPL-3.0-or-later
#ifndef EPD_SPATIAL_H
#define EPD_SPATIAL_H

/* HT20 (phy=2) and HE SU20 (phy=4). Offsets are relative to the first
 * legacy LTF FFT. All spectra use natural FFT bin order, without scaling. */
typedef struct {
    int phy, paths, ltf_count, training_fft, training_offset, training_stride;
    int data_fft, data_offset, data_stride;
} epd_spatial_plan;

int epd_spatial_layout(int phy, int paths, int ltf, int gi, epd_spatial_plan *plan);
typedef struct {
    double re, im;
} epd_complex;

/* Use the unmodified plan returned by epd_spatial_layout.
 * For each receiver, spectra contains all training FFTs followed by up to
 * eight payload FFTs. A missing payload FFT has NaN in its first bin.
 * noise is one legacy FFT noise power per receiver. channels is indexed by
 * receiver, FFT bin, space-time path and contains only measured training.
 * symbols is indexed by symbol, FFT bin, spatial stream (one stream for HT
 * Alamouti). Unsupported bins are NaN. Pilots occupy their own FFT bins.
 * stats receives counts of rank and pilot rejections. Returns 0 on success. */
int epd_spatial_decode(const epd_spatial_plan *plan, int nr, int nsym, int alamouti, const double *noise,
                       const epd_complex *spectra, epd_complex *channels, epd_complex *symbols,
                       int stats[2]);
#endif
