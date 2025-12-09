#include <iostream>
#include <cmath>
#include <vector>
#include "doa_simple.h"
#include <complex>

// A simple host-side GCC-PHAT implementation (direct DFT, O(N^2)).
// This is only for testing on host and not optimized.
static float fft_doa_host(const int16_t *left, const int16_t *right, int frame_samples, int sample_rate, float mic_distance, float *out_conf)
{
    if (!left || !right || frame_samples <= 0)
    {
        if (out_conf)
            *out_conf = 0.0f;
        return NAN;
    }
    const double c = 343.0;
    int N = frame_samples * 2; // zero-pad to 2*N for better resolution (use 2x)
    std::vector<std::complex<double>> x1(N), x2(N);
    for (int i = 0; i < N; ++i)
    {
        double v1 = 0.0, v2 = 0.0;
        if (i < frame_samples)
        {
            v1 = (double)left[i];
            v2 = (double)right[i];
        }
        x1[i] = std::complex<double>(v1, 0.0);
        x2[i] = std::complex<double>(v2, 0.0);
    }

    // DFT X[k]
    std::vector<std::complex<double>> X1(N), X2(N);
    const double TWO_PI = 2.0 * M_PI;
    for (int k = 0; k < N; ++k)
    {
        std::complex<double> sum1(0.0, 0.0), sum2(0.0, 0.0);
        for (int n = 0; n < N; ++n)
        {
            double angle = -TWO_PI * k * n / (double)N;
            std::complex<double> w(cos(angle), sin(angle));
            sum1 += x1[n] * w;
            sum2 += x2[n] * w;
        }
        X1[k] = sum1;
        X2[k] = sum2;
    }

    // Cross-power G = X1 * conj(X2)
    std::vector<std::complex<double>> G(N);
    for (int k = 0; k < N; ++k)
    {
        G[k] = X1[k] * std::conj(X2[k]);
    }

    // Peak magnitude for confidence (use abs of G)
    double peakG = 0.0, secondG = 0.0;
    for (int k = 0; k < N; ++k)
    {
        double mag = std::abs(G[k]);
        if (mag > peakG)
        {
            secondG = peakG;
            peakG = mag;
        }
        else if (mag > secondG)
        {
            secondG = mag;
        }
    }

    // PHAT normalize
    for (int k = 0; k < N; ++k)
    {
        double mag = std::abs(G[k]);
        if (mag > 1e-12)
            G[k] /= mag;
        else
            G[k] = std::complex<double>(0.0, 0.0);
    }

    // IDFT to get cross-correlation r[n] = (1/N) sum_k G[k] * exp(j*2pi*k*n/N)
    std::vector<std::complex<double>> r(N);
    for (int n = 0; n < N; ++n)
    {
        std::complex<double> sum(0.0, 0.0);
        for (int k = 0; k < N; ++k)
        {
            double angle = TWO_PI * k * n / (double)N;
            std::complex<double> w(cos(angle), sin(angle));
            sum += G[k] * w;
        }
        r[n] = sum / (double)N;
    }

    // find peak index in r (use real part magnitude)
    double peakR = -1e300;
    int idx = 0;
    double secondR = -1e300;
    for (int n = 0; n < N; ++n)
    {
        double val = std::abs(r[n]);
        if (val > peakR)
        {
            secondR = peakR;
            peakR = val;
            idx = n;
        }
        else if (val > secondR)
        {
            secondR = val;
        }
    }

    // parabolic interpolation around idx (real part)
    int im1 = (idx - 1 + N) % N;
    int ip1 = (idx + 1) % N;
    double ym = std::abs(r[im1]);
    double y0 = std::abs(r[idx]);
    double yp = std::abs(r[ip1]);
    double delta = 0.0;
    double denom = (ym - 2.0 * y0 + yp);
    if (fabs(denom) > 1e-12)
        delta = 0.5 * (ym - yp) / denom;

    double peak_pos = (double)idx + delta;
    // map to signed lag
    int Mhalf = N / 2;
    int signed_idx = (int)round(peak_pos);
    if (signed_idx > Mhalf)
        signed_idx -= N;
    double delay = (double)signed_idx / (double)sample_rate; // seconds
    double arg = delay * c / mic_distance;
    if (arg > 1.0)
        arg = 1.0;
    if (arg < -1.0)
        arg = -1.0;
    double theta = acos(arg) * 180.0 / M_PI;

    // confidence from r peak ratio
    double conf = 0.0;
    if (secondR <= 0.0)
        conf = (peakR > 0.0) ? 1.0 : 0.0;
    else
    {
        double ratio = peakR / (secondR + 1e-12);
        conf = ratio / (ratio + 1.0);
    }
    if (out_conf)
        *out_conf = (float)conf;
    return (float)theta;
}

int main()
{
    const int sample_rate = 16000;
    const int frame_samples = 1024;
    const float mic_distance = 0.045f; // 45 mm
    const float c = 343.0f;
    const float TEST_FREQ = 1000.0f; // 1 kHz test tone

    simple_doa_handle_t *h = simple_doa_create(sample_rate, 20.0f, mic_distance, frame_samples);
    if (!h)
    {
        std::cerr << "Failed to create simple DOA handle\n";
        return 1;
    }

    std::cout << "Angle,ExpectedAngle(Input),Measured_simple,Recovered_simple,Conf_simple,Measured_fft,Recovered_fft,Conf_fft" << std::endl;

    std::vector<int16_t> left(frame_samples);
    std::vector<int16_t> right(frame_samples);

    for (int ang = -80; ang <= 80; ang += 10)
    {
        float angle_deg = (float)ang;
        float angle_rad = angle_deg * M_PI / 180.0f;
        // According to the test signal generation used elsewhere: delay = d * sin(angle)/c
        float tau = (mic_distance * sinf(angle_rad)) / c; // seconds

        // Generate continuous-time samples with fractional delay by shifting time inside sine
        for (int n = 0; n < frame_samples; ++n)
        {
            float t = (float)n / (float)sample_rate;
            float left_v = sinf(2.0f * M_PI * TEST_FREQ * t);
            float right_v = sinf(2.0f * M_PI * TEST_FREQ * (t - tau));
            left[n] = (int16_t)(left_v * 10000.0f);
            right[n] = (int16_t)(right_v * 10000.0f);
        }

        float conf_simple = 0.0f;
        float measured_simple = simple_doa_process(h, left.data(), right.data(), &conf_simple);

        float conf_fft = 0.0f;
        float measured_fft = fft_doa_host(left.data(), right.data(), frame_samples, sample_rate, mic_distance, &conf_fft);

        // print line with both results
        std::cout << ang << "," << angle_deg << ",";
        if (std::isnan(measured_simple))
        {
            std::cout << "NaN,NaN," << conf_simple << ",";
        }
        else
        {
            float recovered = 90.0f - measured_simple;
            std::cout << measured_simple << "," << recovered << "," << conf_simple << ",";
        }

        if (std::isnan(measured_fft))
        {
            std::cout << "NaN,NaN," << conf_fft;
        }
        else
        {
            float recovered_fft = 90.0f - measured_fft;
            std::cout << measured_fft << "," << recovered_fft << "," << conf_fft;
        }
        std::cout << std::endl;
    }

    simple_doa_destroy(h);
    return 0;
}
