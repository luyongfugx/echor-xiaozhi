#ifndef AFE_AUDIO_PROCESSOR_H
#define AFE_AUDIO_PROCESSOR_H

#include <esp_afe_sr_models.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_afe_sr_iface.h>

#include <string>
#include <vector>
#include <functional>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <thread>

#include "audio_processor.h"
#include "audio_codec.h"

#include "esp_doa.h"
#include <math.h>

#include <cmath>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "../doa/kiss_fft.h"
#include "../doa/kiss_fftr.h"

#define _USE_MATH_DEFINES

class AfeAudioProcessor : public AudioProcessor {
public:
    AfeAudioProcessor();
    ~AfeAudioProcessor();

    void Initialize(AudioCodec* codec, int frame_duration_ms, srmodel_list_t* models_list) override;
    void Feed(std::vector<int16_t>&& data) override;
    void Start() override;
    void Stop() override;
    bool IsRunning() override;
    void OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) override;
    void OnVadStateChange(std::function<void(bool speaking)> callback) override;
    size_t GetFeedSize() override;
    void EnableDeviceAec(bool enable) override;
    void generate_test_frame(int16_t *left, int16_t *right, int frame_size, float angle_deg, int sample_rate);
    void test_doa();
    void generate_test_frame1(float *left, float *right, int frame_size, float angle_deg);
    void doa_sim_task();
    void ProcessSoundSourceLocalization(const afe_fetch_result_t* res);
private:
    EventGroupHandle_t event_group_ = nullptr;
    esp_afe_sr_iface_t* afe_iface_ = nullptr;
    esp_afe_sr_data_t* afe_data_ = nullptr;
    std::function<void(std::vector<int16_t>&& data)> output_callback_;
    std::function<void(bool speaking)> vad_state_change_callback_;
    AudioCodec* codec_ = nullptr;
    int frame_samples_ = 0;
    bool is_speaking_ = false;
    std::vector<int16_t> output_buffer_;
    void AudioProcessorTask();

    // void TestDoaFunctionality();
    doa_handle_t* doa_handle_ = nullptr;
    int doa_sample_rate_ = 16000;
    int doa_frame_samples_ = 1024;
    int total_channels_ = 0;  // 添加总通道数成员
    std::vector<int16_t> doa_buffer_;  // DOA 数据累积缓冲区
    std::vector<int16_t> pre_wake_word_buffer_;  // 唤醒词前音频数据缓冲区
    size_t pre_wake_word_buffer_size_ = 0;  // 预存储缓冲区大小（0.2秒数据）
    std::function<void(float angle)> doa_callback_;  // DOA 角度回调
    bool doa_enabled_ = false;  // DOA检测启用标志
    uint32_t last_buffer_check_ = 0;  // 上次缓冲区检查时间
    uint32_t buffer_full_count_ = 0;  // 缓冲区满计数
    uint32_t consecutive_timeouts_ = 0;  // 连续超时计数
    uint32_t speech_start_time_ = 0;  // 开始说话的时间
    bool should_do_delayed_doa_ = false;  // 是否应该执行延迟的DOA检测
    uint32_t delayed_doa_time_ = 0;  // 延迟DOA执行的时间
    

};

#endif 