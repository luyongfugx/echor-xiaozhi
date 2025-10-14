#ifndef KISS_FFTR_H
#define KISS_FFTR_H
#include "kiss_fft.h"
typedef void* kiss_fftr_cfg;
kiss_fftr_cfg kiss_fftr_alloc(int nfft, int inverse_fft, void *mem, size_t *lenmem);
void kiss_fftr(kiss_fftr_cfg cfg, const float *timedata, kiss_fft_cpx *freqdata);
void kiss_fftri(kiss_fftr_cfg cfg, const kiss_fft_cpx *freqdata, float *timedata);
#endif // KISS_FFTR_H
