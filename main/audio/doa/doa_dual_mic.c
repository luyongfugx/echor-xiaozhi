/*
 * doa_dual_mic.c
 * A DOA example using GCC-PHAT and a simple FFT implementation provided
 * in kiss_fft.c / kiss_fftr.c (naive DFT). Replace with optimized kissFFT
 * for production use.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2s.h"

#include "kiss_fft.h"
#include "kiss_fftr.h"

static const char *TAG = "DOA";

#define SAMPLE_RATE     16000
#define FRAME_SIZE      512
#define MIC_DISTANCE_M  0.06f
#define SPEED_SOUND     343.0f

typedef struct {
    int16_t samples_left[FRAME_SIZE];
    int16_t samples_right[FRAME_SIZE];
} audio_frame_t;

static QueueHandle_t xFrameQueue = NULL;

static float window[FRAME_SIZE];
static void init_window() {
    for (int n = 0; n < FRAME_SIZE; n++) {
        window[n] = 0.5f * (1.0f - cosf(2.0f * M_PI * n / (FRAME_SIZE - 1)));
    }
}

static void remove_dc(float *buf, int N) {
    float mean = 0;
    for (int i = 0; i < N; i++) mean += buf[i];
    mean /= N;
    for (int i = 0; i < N; i++) buf[i] -= mean;
}

static float parabolic_interp(float left, float center, float right) {
    float denom = (left - 2.0f * center + right);
    if (denom == 0.0f) return 0.0f;
    return 0.5f * (left - right) / denom;
}

static float compute_tdoa_gcc_phat(const float *x, const float *y, int N) {
    int nfft = N;
    // allocate naive fft cfgs (not used internally but to match interface)
    void *fwd = kiss_fftr_alloc(nfft, 0, NULL, NULL);
    void *inv = kiss_fftr_alloc(nfft, 1, NULL, NULL);
    if (!fwd || !inv) {
        ESP_LOGE(TAG, "fft alloc fail");
        if (fwd) free(fwd);
        if (inv) free(inv);
        return 0;
    }

    int ncpx = nfft/2 + 1;
    kiss_fft_cpx *X = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx)*ncpx);
    kiss_fft_cpx *Y = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx)*ncpx);
    kiss_fft_cpx *R = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx)*ncpx);
    float *cc = (float*)malloc(sizeof(float)*nfft);
    float *xw = (float*)malloc(sizeof(float)*nfft);
    float *yw = (float*)malloc(sizeof(float)*nfft);
    if (!X || !Y || !R || !cc || !xw || !yw) {
        ESP_LOGE(TAG, "malloc fail fft temps");
        free(X); free(Y); free(R); free(cc); free(xw); free(yw);
        free(fwd); free(inv);
        return 0;
    }

    for (int i=0;i<N;i++){
        xw[i] = x[i]*window[i];
        yw[i] = y[i]*window[i];
    }

    kiss_fftr(fwd, xw, X);
    kiss_fftr(fwd, yw, Y);

    for (int k=0;k<ncpx;k++){
        float Rre = X[k].r * Y[k].r + X[k].i * Y[k].i;
        float Rim = X[k].i * Y[k].r - X[k].r * Y[k].i;
        float mag = sqrtf(Rre*Rre + Rim*Rim);
        if (mag < 1e-8f) {
            R[k].r = 0; R[k].i = 0;
        } else {
            R[k].r = Rre / mag;
            R[k].i = Rim / mag;
        }
    }

    kiss_fftri(inv, R, cc);

    int max_idx = 0;
    float max_val = cc[0];
    for (int i=1;i<nfft;i++){
        if (cc[i] > max_val) { max_val = cc[i]; max_idx = i; }
    }

    int mid = nfft/2;
    int tau_idx = max_idx;
    if (max_idx > mid) tau_idx = max_idx - nfft;

    int idx0 = (max_idx - 1 + nfft) % nfft;
    int idx2 = (max_idx + 1) % nfft;
    float interp = parabolic_interp(cc[idx0], cc[max_idx], cc[idx2]);
    float tdoa_samples = (float)tau_idx + interp;

    free(X); free(Y); free(R); free(cc); free(xw); free(yw);
    free(fwd); free(inv);
    return tdoa_samples;
}

static float compute_doa_from_frame(const int16_t *left, const int16_t *right) {
    float x[FRAME_SIZE], y[FRAME_SIZE];
    for (int i=0;i<FRAME_SIZE;i++){
        x[i] = (float)left[i];
        y[i] = (float)right[i];
    }
    remove_dc(x, FRAME_SIZE);
    remove_dc(y, FRAME_SIZE);

    float tdoa = compute_tdoa_gcc_phat(x, y, FRAME_SIZE);
    float tau = tdoa / SAMPLE_RATE;
    float val = SPEED_SOUND * tau / MIC_DISTANCE_M;
    if (val > 1.0f) val = 1.0f;
    if (val < -1.0f) val = -1.0f;
    float angle = asinf(val) * 180.0f / M_PI;
    return angle;
}

static void i2s_reader_task(void *arg) {
    size_t bytes_read;
    int16_t *tmp = malloc(sizeof(int16_t)*FRAME_SIZE*2);
    if (!tmp) vTaskDelete(NULL);
    while (1) {
        i2s_read(I2S_NUM_0, tmp, FRAME_SIZE*2*sizeof(int16_t), &bytes_read, portMAX_DELAY);
        audio_frame_t *frame = malloc(sizeof(audio_frame_t));
        if (!frame) continue;
        for (int i=0;i<FRAME_SIZE;i++){
            frame->samples_left[i] = tmp[2*i];
            frame->samples_right[i] = tmp[2*i+1];
        }
        if (xFrameQueue) xQueueSend(xFrameQueue, &frame, 0);
    }
    free(tmp);
}

static void doa_task(void *arg) {
    audio_frame_t *frame = NULL;
    while (1) {
        if (xFrameQueue && xQueueReceive(xFrameQueue, &frame, portMAX_DELAY) == pdPASS) {
            float angle = compute_doa_from_frame(frame->samples_left, frame->samples_right);
            ESP_LOGI(TAG, "DOA Angle: %.2f deg", angle);
            free(frame);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void app_doa_init(void) {
    init_window();
    xFrameQueue = xQueueCreate(4, sizeof(audio_frame_t*));

    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = FRAME_SIZE,
    };
    i2s_pin_config_t pin_cfg = {0}; // TODO: 填写你的 I2S 引脚配置结构
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    // i2s_set_pin(I2S_NUM_0, &pin_cfg);

    xTaskCreatePinnedToCore(i2s_reader_task, "i2s_reader", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(doa_task, "doa_task", 8192, NULL, 5, NULL, 1);
}
