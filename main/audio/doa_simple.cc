#include "doa_simple.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <vector>

struct simple_doa_handle_t
{
    int sample_rate;
    float mic_distance;
    int frame_samples;
    int max_lag;   // in samples (positive)
    float c;       // speed of sound
    float *window; // Hamming window coefficients
};

simple_doa_handle_t *simple_doa_create(int sample_rate, float /*max_angle_deg*/, float mic_distance, int frame_samples)
{
    simple_doa_handle_t *h = (simple_doa_handle_t *)malloc(sizeof(simple_doa_handle_t));
    if (!h)
        return nullptr;
    h->sample_rate = sample_rate;
    h->mic_distance = mic_distance;
    h->frame_samples = frame_samples;
    h->c = 343.0f;
    // compute maximal expected lag in samples: when source is at 0 deg, tau = d/c; add small margin
    float max_delay = mic_distance / h->c;
    h->max_lag = std::max(1, (int)ceilf(max_delay * sample_rate) + 4);

    // Generate Hamming window coefficients: w(n) = 0.54 - 0.46 * cos(2πn/(N-1))
    h->window = (float *)malloc(frame_samples * sizeof(float));
    if (!h->window)
    {
        free(h);
        return nullptr;
    }
    for (int i = 0; i < frame_samples; ++i)
    {
        h->window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (frame_samples - 1));
    }

    return h;
}

static inline float clampf01(float v)
{
    if (v > 1.0f)
        return 1.0f;
    if (v < -1.0f)
        return -1.0f;
    return v;
}

// Parabolic interpolation around peak: returns sub-sample offset (peak pos + delta)
static float parabola_peak_adjust(float ym, float y0, float yp)
{
    // y(x) = ax^2 + bx + c; peak offset = 0.5*(ym - yp) / (ym - 2*y0 + yp)
    float denom = (ym - 2.0f * y0 + yp);
    if (fabsf(denom) < 1e-8f)
        return 0.0f;
    return 0.5f * (ym - yp) / denom;
}

float simple_doa_process(simple_doa_handle_t *h, const int16_t *left, const int16_t *right, float *out_conf)
{
    if (!h || !left || !right)
    {
        if (out_conf)
            *out_conf = 0.0f;
        return NAN;
    }

    int N = h->frame_samples;
    int maxLag = h->max_lag;
    int len = 2 * maxLag + 1;

    // Check signal energy to detect silence
    // Compute RMS for both channels
    long long sum_sq_left = 0;
    long long sum_sq_right = 0;
    for (int i = 0; i < N; ++i)
    {
        sum_sq_left += (long long)left[i] * left[i];
        sum_sq_right += (long long)right[i] * right[i];
    }
    float rms_left = sqrtf((float)sum_sq_left / N);
    float rms_right = sqrtf((float)sum_sq_right / N);

    // Silence threshold: if both channels RMS < 50, consider it silence
    const float SILENCE_THRESHOLD = 80.0f;
    if (rms_left < SILENCE_THRESHOLD && rms_right < SILENCE_THRESHOLD)
    {
        if (out_conf)
            *out_conf = 0.0f;
        return NAN; // No sound detected, no direction output
    }

    // Apply Hamming window to both channels
    std::vector<float> left_windowed(N);
    std::vector<float> right_windowed(N);
    for (int i = 0; i < N; ++i)
    {
        left_windowed[i] = left[i] * h->window[i];
        right_windowed[i] = right[i] * h->window[i];
    }

    // compute cross-correlation for lags [-maxLag, maxLag]
    // r[k] = sum_n left_windowed[n] * right_windowed[n + lag]
    std::vector<float> r(len);
    for (int lag = -maxLag; lag <= maxLag; ++lag)
    {
        double acc = 0.0;
        int count = 0;
        for (int n = 0; n < N; ++n)
        {
            int rn = n + lag;
            if (rn < 0 || rn >= N)
                continue;
            acc += left_windowed[n] * right_windowed[rn];
            ++count;
        }
        float val = (count > 0) ? (acc / count) : 0.0f;
        r[lag + maxLag] = val;
    }

    // find peak
    int idx = 0;
    float maxv = -INFINITY;
    for (int i = 0; i < len; ++i)
    {
        if (r[i] > maxv)
        {
            maxv = r[i];
            idx = i;
        }
    }

    if (maxv == -INFINITY)
    {
        if (out_conf)
            *out_conf = 0.0f;
        return NAN;
    }

    // compute second largest (absolute) value for confidence
    float second_max = -INFINITY;
    for (int i = 0; i < len; ++i)
    {
        if (i == idx)
            continue;
        if (r[i] > second_max)
            second_max = r[i];
    }

    float conf = 0.0f;
    if (second_max <= -INFINITY || second_max <= 0.0f)
    {
        // if no meaningful secondary peak, set high confidence when maxv > 0
        conf = (maxv > 0.0f) ? 1.0f : 0.0f;
    }
    else
    {
        float ratio = maxv / (second_max + 1e-9f);
        conf = ratio / (ratio + 1.0f); // in (0,1)
    }
    if (out_conf)
        *out_conf = conf;

    // refine with parabolic interpolation if neighbors exist
    float delta = 0.0f;
    if (idx > 0 && idx + 1 < len)
    {
        float ym = r[idx - 1];
        float y0 = r[idx];
        float yp = r[idx + 1];
        delta = parabola_peak_adjust(ym, y0, yp);
        // clamp delta reasonable
        if (delta > 1.0f)
            delta = 1.0f;
        if (delta < -1.0f)
            delta = -1.0f;
    }

    float lag_est = (float)(idx - maxLag) + delta; // in samples
    float tau = lag_est / (float)h->sample_rate;   // seconds (can be negative)

    // cos(theta) = tau * c / d
    float arg = tau * h->c / h->mic_distance;
    arg = clampf01(arg);
    if (fabsf(arg) > 1.0f)
    {
        return NAN;
    }
    float theta = acosf(arg) * 180.0f / M_PI;

    // Quantize to nearest discrete direction: every 15° from 0° to 180°
    // Directions: 0°, 15°, 30°, 45°, 60°, 75°, 90°, 105°, 120°, 135°, 150°, 165°, 180°
    static const float directions[] = {
        0.0f, 15.0f, 30.0f, 45.0f, 60.0f, 75.0f, 90.0f,
        105.0f, 120.0f, 135.0f, 150.0f, 165.0f, 180.0f};
    float min_diff = INFINITY;
    float quantized = theta;
    for (int i = 0; i < 13; ++i)
    {
        float diff = fabsf(theta - directions[i]);
        if (diff < min_diff)
        {
            min_diff = diff;
            quantized = directions[i];
        }
    }

    return quantized;
}

void simple_doa_destroy(simple_doa_handle_t *h)
{
    if (h)
    {
        if (h->window)
            free(h->window);
        free(h);
    }
}
