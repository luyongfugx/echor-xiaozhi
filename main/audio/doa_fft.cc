#include "doa_fft.h"
#include <cmath>
#include <vector>
#include <cstring>
#include <cstdlib>
#include "esp_dsp.h"

struct fft_doa_handle_t
{
    int sample_rate;
    float mic_distance;
    int frame_samples;
    int N; // FFT size (power of 4 chosen)
    float c;
    // buffers
    float *x1;
    float *x2;
};

static int next_power_of_four(int n)
{
    int p = 1;
    while (p < n)
        p <<= 1;
    // ensure power of 4: if power of 2 but not 4, multiply by 2
    // we require power-of-4 for dsps_fft4r_fc32 which expects N=4^k; approximate by using closest pow2 and ensure divisible by 4
    while ((p & (p - 1)) != 0)
        p <<= 1; // ensure power of two
    while (p % 4 != 0)
        p <<= 1;
    return p;
}

fft_doa_handle_t *fft_doa_create(int sample_rate, float /*max_angle_deg*/, float mic_distance, int frame_samples)
{
    fft_doa_handle_t *h = (fft_doa_handle_t *)malloc(sizeof(fft_doa_handle_t));
    if (!h)
        return nullptr;
    h->sample_rate = sample_rate;
    h->mic_distance = mic_distance;
    h->frame_samples = frame_samples;
    h->c = 343.0f;
    // choose FFT size (at least 2*frame_samples for zero-padding correlation)
    int required = frame_samples * 2;
    int N = 1;
    // pick a power-of-4 N >= required
    while (N < required)
        N <<= 1;
    while (N % 4 != 0)
        N <<= 1;
    h->N = N; // N = number of complex bins (FFT size in complex samples)
    // allocate interleaved complex arrays: 2 * N floats (real,imag pairs)
    h->x1 = (float *)malloc(sizeof(float) * (2 * h->N));
    h->x2 = (float *)malloc(sizeof(float) * (2 * h->N));
    if (!h->x1 || !h->x2)
    {
        free(h->x1);
        free(h->x2);
        free(h);
        return nullptr;
    }
    memset(h->x1, 0, sizeof(float) * 2 * h->N);
    memset(h->x2, 0, sizeof(float) * 2 * h->N);
    return h;
}

// Helper functions: compute FFT, cross-power, apply PHAT weighting, IFFT to get cross-correlation
// x1, x2: interleaved complex arrays length == 2*M floats (M complex bins)
// M: number of complex samples (FFT size)
static void computeFFTandCrossPowerSpectrum(float *x1, float *x2, int M)
{
    // perform forward FFT on both signals (in-place)
    dsps_fft4r_fc32(x1, M);
    dsps_bit_rev4r_fc32(x1, M);
    dsps_fft4r_fc32(x2, M);
    dsps_bit_rev4r_fc32(x2, M);

    float *p1 = x1;
    float *p2 = x2;
    // compute cross-power spectrum: G = X1 * conj(X2)
    for (int i = 0; i < 2 * M; i += 2)
    {
        float real1 = p1[0];
        float imag1 = p1[1];
        float real2 = p2[0];
        float imag2 = p2[1];
        // G = X1 * conj(X2) = (a+jb)*(c-jd) = (ac+bd) + j(bc-ad)
        p2[0] = real1 * real2 + imag1 * imag2; // real
        p2[1] = imag1 * real2 - real1 * imag2; // imag
        p1 += 2;
        p2 += 2;
    }
}

static void applyPHATandIFFT(float *G, int M)
{
    // PHAT: normalize by magnitude
    for (int i = 0; i < 2 * M; i += 2)
    {
        float re = G[i];
        float im = G[i + 1];
        float mag = sqrtf(re * re + im * im) + 1e-10f;
        G[i] = re / mag;
        G[i + 1] = im / mag;
    }
    // IFFT via conjugation trick: conjugate (negate imag), forward FFT, scale by 1/M, conjugate
    for (int i = 1; i < 2 * M; i += 2)
        G[i] = -G[i];
    dsps_fft4r_fc32(G, M);
    dsps_bit_rev4r_fc32(G, M);
    // scale by 1/M (apply to both real and imag entries)
    for (int i = 0; i < 2 * M; ++i)
        G[i] *= (1.0f / (float)M);
    for (int i = 1; i < 2 * M; i += 2)
        G[i] = -G[i];
}

// Find peak index and return also peak and second_peak magnitudes (on complex interleaved array R length 2*M)
static int findDelayIndex(float *R, int M, float &out_peak, float &out_second)
{
    float maxVal = -INFINITY;
    float secondVal = -INFINITY;
    int idx = 0;
    for (int i = 0; i < 2 * M; i += 2)
    {
        float val = R[i] * R[i] + R[i + 1] * R[i + 1];
        if (val > maxVal)
        {
            secondVal = maxVal;
            maxVal = val;
            idx = i >> 1; // complex bin index
        }
        else if (val > secondVal)
        {
            secondVal = val;
        }
    }
    out_peak = (maxVal < 0.0f) ? 0.0f : sqrtf(maxVal);
    out_second = (secondVal < 0.0f) ? 0.0f : sqrtf(secondVal);
    return idx;
}

float fft_doa_process(fft_doa_handle_t *h, const int16_t *left, const int16_t *right, float *out_conf)
{
    if (!h || !left || !right)
    {
        if (out_conf)
            *out_conf = 0.0f;
        return NAN;
    }
    int M = h->N; // number of complex bins (FFT size)
    // prepare interleaved complex arrays (real, imag)
    for (int i = 0; i < M; ++i)
    {
        if (i < h->frame_samples)
        {
            h->x1[2 * i] = (float)left[i];
            h->x1[2 * i + 1] = 0.0f;
            h->x2[2 * i] = (float)right[i];
            h->x2[2 * i + 1] = 0.0f;
        }
        else
        {
            h->x1[2 * i] = 0.0f;
            h->x1[2 * i + 1] = 0.0f;
            h->x2[2 * i] = 0.0f;
            h->x2[2 * i + 1] = 0.0f;
        }
    }

    // compute cross-power spectrum with M complex bins
    computeFFTandCrossPowerSpectrum(h->x1, h->x2, M);
    // cross-power stored in x2, apply PHAT and perform IFFT (result in x2)
    applyPHATandIFFT(h->x2, M);

    float peak_mag = 0.0f;
    float second_mag = 0.0f;
    int idx = findDelayIndex(h->x2, M, peak_mag, second_mag);
    // map idx [0..M-1] to signed lag in bins: if idx > M/2 then idx -= M
    int signed_idx = idx;
    if (idx > M / 2)
        signed_idx = idx - M;
    float delay = (float)signed_idx / (float)h->sample_rate; // seconds
    float arg = delay * h->c / h->mic_distance;

    // compute confidence from peak vs second_peak
    float conf = 0.0f;
    if (peak_mag <= 0.0f)
    {
        conf = 0.0f;
    }
    else if (second_mag <= 0.0f)
    {
        conf = 1.0f;
    }
    else
    {
        float ratio = peak_mag / (second_mag + 1e-9f);
        conf = ratio / (ratio + 1.0f);
    }
    if (out_conf)
        *out_conf = conf;
    if (arg > 1.0f)
        arg = 1.0f;
    if (arg < -1.0f)
        arg = -1.0f;
    return acosf(arg) * 180.0f / M_PI;
}

void fft_doa_destroy(fft_doa_handle_t *h)
{
    if (!h)
        return;
    free(h->x1);
    free(h->x2);
    free(h);
}
