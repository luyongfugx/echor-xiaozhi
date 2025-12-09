// Lightweight time-domain DOA (TDOA) helper
#pragma once

#include <cstdint>

typedef struct simple_doa_handle_t simple_doa_handle_t;

// Create a DOA handle
// sample_rate: sampling rate in Hz
// max_angle_deg: unused placeholder for compatibility
// mic_distance: distance between two mics in meters
// frame_samples: number of samples per channel per frame
simple_doa_handle_t *simple_doa_create(int sample_rate, float max_angle_deg, float mic_distance, int frame_samples);

// Process one frame of interleaved (left,right) int16 samples (each array length == frame_samples)
// Returns angle in degrees. If invalid, returns NAN.
// If out_conf != nullptr, writes a confidence value in range [0,1] indicating reliability.
float simple_doa_process(simple_doa_handle_t *h, const int16_t *left, const int16_t *right, float *out_conf = nullptr);

// Destroy handle
void simple_doa_destroy(simple_doa_handle_t *h);
