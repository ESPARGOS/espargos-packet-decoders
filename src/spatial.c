// SPDX-License-Identifier: LGPL-3.0-or-later
#include "epd/spatial.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

typedef epd_complex Z;
static Z add(Z a, Z b) { return (Z){a.re + b.re, a.im + b.im}; }
static Z sub(Z a, Z b) { return (Z){a.re - b.re, a.im - b.im}; }
static Z mul(Z a, Z b) { return (Z){a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re}; }
static Z scale(Z a, double x) { return (Z){a.re * x, a.im * x}; }
static Z conjz(Z a) { return (Z){a.re, -a.im}; }
static double power(Z a) { return a.re * a.re + a.im * a.im; }
static Z missing(void) { return (Z){NAN, NAN}; }
static void clear_bins(Z *z, size_t n) {
    for (size_t i = 0; i < n; i++)
        z[i] = missing();
}
static const signed char ht_sign[] = {
    1,  1,  1,  1,  -1, -1, 1,  1,  -1, 1,  -1, 1,  1, 1, 1,  1, 1,  -1, -1,
    1,  1,  -1, 1,  -1, 1,  1,  1,  1,  1,  -1, -1, 1, 1, -1, 1, -1, 1,  -1,
    -1, -1, -1, -1, 1,  1,  -1, -1, 1,  -1, 1,  -1, 1, 1, 1,  1, -1, -1,
};
static const signed char he_sign4[] = {
    -1, -1, 1,  -1, 1,  -1, 1,  1,  1,  -1, 1,  1,  1,  -1, -1, 1,  -1, -1, -1, -1, -1, 1,  1,  -1, -1,
    -1, -1, 1,  1,  -1, 1,  -1, 1,  1,  1,  1,  -1, 1,  -1, -1, 1,  1,  -1, 1,  1,  1,  1,  -1, -1, 1,
    -1, -1, -1, 1,  1,  1,  1,  -1, 1,  1,  -1, -1, -1, -1, 1,  -1, -1, 1,  1,  -1, 1,  -1, -1, -1, -1,
    1,  -1, 1,  -1, -1, -1, -1, -1, -1, 1,  1,  -1, -1, -1, -1, -1, 1,  -1, -1, 1,  1,  1,  -1, 1,  1,
    1,  -1, 1,  -1, 1,  -1, -1, -1, -1, -1, 1,  1,  1,  -1, -1, -1, 1,  -1, 1,  1,  1,  0,  0,  0,  -1,
    1,  -1, 1,  -1, 1,  1,  -1, 1,  1,  1,  -1, -1, 1,  -1, -1, 1,  -1, 1,  -1, 1,  1,  1,  -1, 1,  1,
    1,  -1, -1, 1,  -1, -1, -1, -1, -1, 1,  1,  -1, -1, -1, -1, -1, -1, 1,  -1, 1,  -1, -1, -1, -1, 1,
    -1, 1,  1,  -1, -1, 1,  -1, -1, -1, -1, 1,  1,  -1, 1,  1,  1,  1,  1,  1,  1,  -1, 1,  1,  -1, -1,
    -1, -1, 1,  -1, -1, 1,  1,  -1, 1,  -1, -1, -1, -1, 1,  -1, 1,  -1, -1, 1,  1,  1,  1,  -1, -1, 1,
    1,  1,  1,  1,  -1, 1,  1,  -1, -1, -1, 1,  -1, -1, -1, 1,  -1, 1,  -1, 1,  1,
};
static const signed char he_sign2[] = {
    -1, -1, -1, 1,  1,  -1, 1,  -1, -1, -1, -1, 1,  -1, 1,  -1, -1, 1,  1,  -1, 1,  1, 1,  1,  1,  -1,
    1,  -1, 1,  -1, -1, 1,  1,  -1, 1,  -1, -1, -1, -1, 1,  -1, 1,  1,  1,  -1, -1, 1, -1, -1, -1, -1,
    -1, 1,  -1, -1, -1, 1,  1,  1,  -1, -1, 1,  0,  1,  -1, 1,  1,  -1, 1,  1,  -1, 1, 1,  -1, -1, 1,
    -1, 1,  1,  1,  1,  -1, 1,  -1, 1,  1,  -1, -1, 1,  -1, -1, -1, -1, -1, 1,  -1, 1, 1,  -1, -1, 1,
    1,  -1, 1,  -1, -1, -1, -1, 1,  -1, 1,  1,  1,  -1, -1, 1,  -1, -1, -1, -1, -1, 1, -1, 1,
};
static const signed char he_sign1[] = {
    -1, 1, 1,  -1, 1,  -1, 1,  1,  1,  1,  -1, -1, 1,  1, 1,  -1, -1, -1, 1,  -1, -1,
    1,  1, -1, -1, 1,  -1, -1, 1,  -1, 0,  -1, 1,  1,  1, 1,  1,  1,  -1, -1, -1, -1,
    -1, 1, -1, -1, -1, 1,  -1, -1, 1,  -1, -1, 1,  -1, 1, -1, -1, -1, -1, -1,
};

int epd_spatial_layout(int phy, int paths, int ltf, int gi, epd_spatial_plan *p) {
    if (!p || paths < 1 || paths > 8)
        return -1;
    int count = paths == 1 ? 1 : 2 * ((paths + 1) / 2);
    if (phy == 2 && paths <= 4 && ltf == 1 && (gi == 8 || gi == 16)) {
        *p = (epd_spatial_plan){phy, paths, count, 64, 464, 80, 64, 464 + 80 * count + gi - 16, 64 + gi};
    } else if (phy == 4 && (ltf == 1 || ltf == 2 || ltf == 4) && (gi == 16 || gi == 32 || gi == 64)) {
        int size = 64 * ltf;
        *p = (epd_spatial_plan){
            phy, paths, count, size, 528 + gi, size + gi, 256, 528 + gi + count * (size + gi), 256 + gi};
    } else
        return -1;
    return 0;
}

static const int ht_pilots[] = {-21, -7, 7, 21}, he_pilots[] = {-116, -90, -48, -22, 22, 48, 90, 116};
static const int pilot_patterns[4][4][4] = {
    {{1, 1, 1, -1}},
    {{1, 1, -1, -1}, {1, -1, -1, 1}},
    {{1, 1, -1, -1}, {1, -1, 1, -1}, {-1, 1, 1, -1}},
    {{1, 1, 1, -1}, {1, 1, -1, 1}, {1, -1, 1, 1}, {-1, 1, 1, 1}}};
static int pilot_sign(int phy, int n, int path, int j, int symbol) {
    static const int he[] = {1, 1, 1, -1, -1, 1, 1, 1};
    static const int polarity[] = {1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, -1};
    if (phy == 4)
        return he[(symbol + j) % 8] * polarity[symbol + 1];
    return pilot_patterns[n - 1][path][(symbol + j) % 4] * polarity[symbol + 3];
}
static int bin(int k, int size) { return (k + size) % size; }
static Z code(int row, int col, int count) {
    static const int p[4][4] = {{1, -1, 1, 1}, {1, 1, -1, 1}, {1, 1, 1, -1}, {-1, 1, 1, 1}};
    if (count == 6) {
        static const int signs[] = {1, -1, 1, 1, 1, -1};
        double angle = -6.2831853071795864769 * row * col / 6;
        return (Z){signs[col] * cos(angle), signs[col] * sin(angle)};
    }
    return (Z){p[row % 4][col % 4] * (row >= 4 && col >= 4 ? -1 : 1), 0};
}
typedef struct {
    Z *h, pilot[8], rot[2];
    double noise;
    int valid[2];
} receiver;

static void training(const epd_spatial_plan *p, const Z *fft, double noise, receiver *r, Z *measured) {
    int n = p->paths, size = p->training_fft, count = p->ltf_count, he = p->phy == 4;
    int step = p->data_fft / size, edge = he ? (step == 4 ? 120 : 122) : 28;
    const signed char *signs = size == 256 ? he_sign4 : size == 128 ? he_sign2 : he_sign1;
    Z matrix[8][8];
    for (int s = 0; s < n; s++)
        for (int j = 0; j < count; j++)
            matrix[s][j] = conjz(code(s, j, count));
    clear_bins(r->h, p->data_fft * n);
    clear_bins(r->pilot, 8);
    for (int k = -edge; k <= edge; k += step) {
        if (!k)
            continue;
        int sign = he ? signs[(k + edge) / step] : ht_sign[k + 28 - (k > 0)];
        if (!sign)
            continue;
        int pilot = -1;
        if (he)
            for (int j = 0; j < 8; j++)
                if (he_pilots[j] == k)
                    pilot = j;
        Z *h = r->h + bin(k, p->data_fft) * n;
        for (int s = 0; s < (pilot >= 0 ? 1 : n); s++) {
            Z sum = {0};
            for (int j = 0; j < count; j++)
                sum = add(sum, mul(fft[j * size + bin(k / step, size)], matrix[s][j]));
            Z value = scale(sum, sign * sqrt(step) / count);
            if (pilot >= 0)
                r->pilot[pilot] = value;
            else
                h[s] = value;
        }
    }
    for (int i = 0; i < p->data_fft * n; i++)
        measured[i] = r->h[i];
    r->noise = noise;
    if (!he)
        return;
    double guard = 0;
    int bins = 0;
    for (int j = 0; j < count; j++)
        for (int k = -size / 2; k < size / 2; k++)
            if (abs(k) * step > 122) {
                guard += power(fft[j * size + bin(k, size)]);
                bins++;
            }
    r->noise = fmax(4 * noise, step * guard / bins);
    // Interpolate between measured tones only, separately on each side of DC.
    int last = 0;
    for (int k = -edge; k <= edge; k++) {
        if (!k) {
            last = 0;
            continue;
        }
        Z *right = r->h + bin(k, 256) * n;
        if (!isfinite(right->re))
            continue;
        if (last)
            for (int i = last + 1; i < k; i++) {
                Z *left = r->h + bin(last, 256) * n, *out = r->h + bin(i, 256) * n;
                double f = (double)(i - last) / (k - last);
                for (int s = 0; s < n; s++)
                    out[s] = add(scale(left[s], 1 - f), scale(right[s], f));
            }
        last = k;
    }
    if (step == 4)
        for (int j = 0; j < 8; j++) {
            Z sum = {0}, *h = r->h + bin(he_pilots[j], 256) * n;
            for (int s = 0; s < n; s++)
                sum = add(sum, h[s]);
            r->pilot[j] = sum;
        }
}

// Noise-weighted least squares. Keep the existing two-stream HT conditioning
// criterion and reject dimensions below the training noise floor for HE.
static int solve(Z a[8][9], int n, int rows, int count, int ht, Z *out) {
    if (rows < n)
        return 0;
    double maximum = 0;
    for (int i = 0; i < n; i++)
        maximum = fmax(maximum, a[i][i].re);
    if (!(maximum > 0))
        return 0;
    if (ht && n <= 2) {
        double x = a[0][0].re;
        if (x < 1)
            return 0;
        if (n == 1) {
            out[0] = scale(a[0][1], 1 / x);
            return 1;
        }
        double y = a[1][1].re, det = x * y - power(a[0][1]);
        if (y < 1 || det < .01 * x * y)
            return 0;
        out[0] = scale(sub(scale(a[0][2], y), mul(a[0][1], a[1][2])), 1 / det);
        out[1] = scale(sub(scale(a[1][2], x), mul(a[1][0], a[0][2])), 1 / det);
        return 1;
    }
    double floor = fmax(maximum * .001, n > 1 ? 10. * rows / count : 0);
    for (int i = 0; i < n; i++) {
        int pivot = i;
        for (int j = i + 1; j < n; j++)
            if (power(a[j][i]) > power(a[pivot][i]))
                pivot = j;
        if (power(a[pivot][i]) < floor * floor)
            return 0;
        for (int j = i; j <= n; j++) {
            Z tmp = a[i][j];
            a[i][j] = a[pivot][j];
            a[pivot][j] = tmp;
        }
        Z inv = scale(conjz(a[i][i]), 1 / power(a[i][i]));
        for (int j = i; j <= n; j++)
            a[i][j] = mul(a[i][j], inv);
        for (int row = 0; row < n; row++)
            if (row != i) {
                Z factor = a[row][i];
                for (int j = i; j <= n; j++)
                    a[row][j] = sub(a[row][j], mul(factor, a[i][j]));
            }
    }
    for (int i = 0; i < n; i++)
        out[i] = a[i][n];
    return 1;
}

static int phase(Z cross, double magnitude, double fraction, Z *rotation) {
    double len = sqrt(power(cross));
    if (!(len > 0 && len >= fraction * magnitude))
        return 0;
    *rotation = scale(conjz(cross), 1 / len);
    return 1;
}

static int antenna_phase(const epd_spatial_plan *p, const receiver *r, const Z *y, int sym, Z *rotation,
                         Z *pilot_points) {
    const int he = p->phy == 4, np = he ? 8 : 4;
    const int *pilots = he ? he_pilots : ht_pilots;
    Z cross = {0}, ratios[8];
    double magnitude = 0;
    int used = 0;
    for (int j = 0; j < np; j++) {
        int k = bin(pilots[j], p->data_fft);
        Z expected = {0};
        if (he)
            expected = scale(r->pilot[j], pilot_sign(4, 1, 0, j, sym));
        else
            for (int s = 0; s < p->paths; s++)
                expected =
                    add(expected, scale(r->h[k * p->paths + s], pilot_sign(2, p->paths, s, j, sym)));
        ratios[j] = missing();
        if (!isfinite(expected.re))
            continue;
        Z z = mul(conjz(expected), y[k]);
        cross = add(cross, z);
        magnitude += sqrt(power(z));
        used++;
        if (power(expected) > r->noise)
            ratios[j] = scale(z, pilot_sign(p->phy, p->paths, 0, j, sym) / power(expected));
    }
    if (used < (he ? 6 : 3) || !phase(cross, magnitude, he ? .65 : .7, rotation))
        return 0;
    for (int j = 0; j < np; j++)
        if (isfinite(ratios[j].re))
            pilot_points[bin(pilots[j], p->data_fft)] = mul(ratios[j], *rotation);
    return 1;
}

int epd_spatial_decode(const epd_spatial_plan *p, int nr, int nsym, int alamouti, const double *noise,
                       const Z *spectra, Z *channels, Z *symbols, int stats[2]) {
    if (!p || nr < 1 || nsym < 0 || nsym > 8 || !noise || !spectra || !channels || !stats ||
        (nsym && !symbols))
        return -1;
    const int n = p->paths, size = p->data_fft, he = p->phy == 4, streams = alamouti ? 1 : n;
    if (alamouti && (he || n != 2))
        return -1;
    int nt = p->training_fft * p->ltf_count, stride = nt + nsym * size;
    size_t channel_size = (size_t)n * size;
    if ((size_t)nr > SIZE_MAX / (channel_size * sizeof(Z)) || (size_t)nr > SIZE_MAX / (stride * sizeof(Z)))
        return -1;
    receiver *rx = calloc(nr, sizeof(*rx));
    Z *storage = malloc(nr * channel_size * sizeof(Z));
    if (!rx || !storage) {
        free(storage);
        free(rx);
        return -1;
    }
    for (int r = 0; r < nr; r++) {
        rx[r].h = storage + r * channel_size;
        training(p, spectra + (size_t)r * stride, noise[r], rx + r, channels + r * channel_size);
    }
    clear_bins(symbols, nsym * size * streams);
    stats[0] = stats[1] = 0;
    for (int sym = 0; sym < nsym; sym++) {
        int half = sym % 2, usable = 0;
        Z *out = symbols + sym * size * streams;
        for (int r = 0; r < nr; r++) {
            const Z *y = spectra + (size_t)r * stride + nt + sym * size;
            rx[r].rot[half] = (Z){1, 0};
            rx[r].valid[half] = isfinite(y[0].re);
            if (rx[r].valid[half] && (he || alamouti)) {
                Z pilots[256];
                clear_bins(pilots, size);
                rx[r].valid[half] = antenna_phase(p, rx + r, y, sym, &rx[r].rot[half], pilots);
                if (!rx[r].valid[half])
                    stats[1]++;
                else {
                    const int *keys = he ? he_pilots : ht_pilots;
                    for (int j = 0; j < (he ? 8 : 4); j++) {
                        int k = bin(keys[j], size);
                        if (isfinite(pilots[k].re) && !isfinite(out[k * streams].re))
                            out[k * streams] = pilots[k];
                    }
                }
            }
            if (rx[r].valid[half] && (!alamouti || rx[r].valid[0]))
                usable++;
        }
        if (usable < streams || (alamouti && !half))
            continue;
        int solved = 0;
        for (int tone = he ? -122 : -28; tone <= (he ? 122 : 28); tone++) {
            if (abs(tone) < (he ? 2 : 1))
                continue;
            int k = bin(tone, size);

            int pilot = 0;
            if (he || alamouti) {
                const int *keys = he ? he_pilots : ht_pilots;
                for (int j = 0; j < (he ? 8 : 4); j++)
                    if (keys[j] == tone)
                        pilot = 1;
                if (pilot)
                    continue;
            }
            Z gram[8][9] = {0}, x = {0}, v = {0};
            double weight = 0;
            int rows = 0;
            for (int r = 0; r < nr; r++) {
                if (!rx[r].valid[half] || (alamouti && !rx[r].valid[0]))
                    continue;
                const Z *h = rx[r].h + k * n;
                if (!isfinite(h[0].re))
                    continue;
                const Z *fft = spectra + (size_t)r * stride + nt + sym * size;
                Z y = mul(fft[k], rx[r].rot[half]);
                double w = 1 / rx[r].noise;
                if (alamouti) {
                    Z u = mul(fft[k - size], rx[r].rot[0]);
                    x = add(x, scale(add(mul(conjz(h[0]), u), mul(h[1], conjz(y))), w));
                    v = add(v, scale(sub(mul(conjz(h[0]), y), mul(h[1], conjz(u))), w));
                    weight += (power(h[0]) + power(h[1])) * w;
                } else
                    for (int i = 0; i < n; i++) {
                        Z g = scale(conjz(h[i]), w);
                        for (int j = 0; j < n; j++)
                            gram[i][j] = add(gram[i][j], mul(g, h[j]));
                        gram[i][n] = add(gram[i][n], mul(g, y));
                    }
                rows++;
            }
            if (alamouti) {
                if (weight >= 1) {
                    out[k - size] = scale(x, 1 / weight);
                    out[k] = scale(v, 1 / weight);
                }
            } else if (rows && solve(gram, n, rows, p->ltf_count, !he, out + k * n))
                solved++;
            else if (rows)
                stats[0]++;
        }
        if (he && n > 1 && solved < 117)
            clear_bins(out, size * n);
        if (!he && !alamouti) {
            Z cross = {0}, rot;
            double magnitude = 0;
            int count = 0;
            for (int j = 0; j < 4; j++)
                for (int s = 0; s < n; s++) {
                    Z z = out[bin(ht_pilots[j], size) * n + s];
                    if (!isfinite(z.re))
                        continue;
                    cross = add(cross, scale(z, pilot_sign(2, n, s, j, sym)));
                    magnitude += sqrt(power(z));
                    count++;
                }
            if (count < 3 * n || !phase(cross, magnitude, .7, &rot)) {
                clear_bins(out, size * n);
                stats[1]++;
            } else
                for (int i = 0; i < size * n; i++)
                    out[i] = mul(out[i], rot);
        }
    }
    free(storage);
    free(rx);
    return 0;
}
