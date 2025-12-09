// FFT-based DOA using GCC-PHAT (light wrapper)
#pragma once

#include <cstdint>

typedef struct fft_doa_handle_t fft_doa_handle_t;

// create handle: sample_rate, max_angle_deg (unused), mic_distance (m), frame_samples (samples per channel)
fft_doa_handle_t *fft_doa_create(int sample_rate, float max_angle_deg, float mic_distance, int frame_samples);

// process one frame (left/right arrays length == frame_samples)
// returns angle in degrees or NAN
// If out_conf != nullptr, writes a confidence estimate in [0,1].
float fft_doa_process(fft_doa_handle_t *h, const int16_t *left, const int16_t *right, float *out_conf = nullptr);

void fft_doa_destroy(fft_doa_handle_t *h);
