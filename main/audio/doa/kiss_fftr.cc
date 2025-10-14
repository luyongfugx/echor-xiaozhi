/*
 * A naive DFT-based implementation that matches the small subset of the kiss_fftr interface
 * used by the DOA example. This is O(N^2) and intended for correctness/testing.
 * Replace with real kissFFT (optimized) for production.
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "kiss_fftr.h"

typedef struct {
    int nfft;
    int inverse;
} naive_cfg;

kiss_fftr_cfg kiss_fftr_alloc(int nfft, int inverse_fft, void *mem, size_t *lenmem) {
    naive_cfg *c = (naive_cfg*)malloc(sizeof(naive_cfg));
    if (!c) return NULL;
    c->nfft = nfft;
    c->inverse = inverse_fft;
    if (lenmem) *lenmem = sizeof(naive_cfg);
    return (kiss_fftr_cfg)c;
}

// timdata: real input, length nfft
// freqdata: complex output length nfft/2+1
void kiss_fftr(kiss_fftr_cfg cfg, const float *timedata, kiss_fft_cpx *freqdata) {
    naive_cfg *c = (naive_cfg*)cfg;
    int n = c->nfft;
    int ncpx = n/2 + 1;
    for (int k=0;k<ncpx;k++){
        float re = 0.0f, im = 0.0f;
        for (int nidx=0;nidx<n;nidx++){
            float phase = -2.0f * M_PI * k * nidx / (float)n;
            re += timedata[nidx] * cosf(phase);
            im += timedata[nidx] * sinf(phase);
        }
        freqdata[k].r = re;
        freqdata[k].i = im;
    }
}

// inverse: freqdata (n/2+1) -> timedata (n)
void kiss_fftri(kiss_fftr_cfg cfg, const kiss_fft_cpx *freqdata, float *timedata) {
    naive_cfg *c = (naive_cfg*)cfg;
    int n = c->nfft;
    int ncpx = n/2 + 1;
    for (int t=0;t<n;t++){
        float sum = 0.0f;
        for (int k=0;k<ncpx;k++){
            float phase = 2.0f * M_PI * k * t / (float)n;
            // k=0 and k=n/2 (if present) are unique, others have conjugate pairs
            if (k==0 || (k==n/2 && n%2==0)) {
                sum += freqdata[k].r * cosf(phase) - freqdata[k].i * sinf(phase);
            } else {
                // include both k and n-k as conjugate pairs
                float re_k = freqdata[k].r;
                float im_k = freqdata[k].i;
                float re_nk = freqdata[k].r; // for real-input FFT symmetry it's same magnitude
                float im_nk = -freqdata[k].i;
                // contribution from k
                sum += re_k * cosf(phase) - im_k * sinf(phase);
                // contribution from n-k
                float phase_nk = 2.0f * M_PI * (n - k) * t / (float)n;
                sum += re_nk * cosf(phase_nk) - im_nk * sinf(phase_nk);
            }
        }
        // normalization: naive IDFT sum needs division by n
        timedata[t] = sum / (float)n;
    }
}
