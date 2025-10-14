#include "afe_audio_processor.h"
#include <esp_log.h>
#include <math.h>

#define PROCESSOR_RUNNING 0x01

#define TAG "AfeAudioProcessor"




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
    if (!buf || N <= 0 || N > 8192) {
        ESP_LOGE(TAG, "Invalid buffer or size: buf=%p N=%d", buf, N);
        return;
    }
    
    // 检查缓冲区是否可访问
    if ((uintptr_t)buf % 4 != 0) {
        ESP_LOGE(TAG, "Buffer not aligned: %p", buf);
        return;
    }
    
    float mean = 0;
    for (int i = 0; i < N; i++) {
        if (!isfinite(buf[i])) {
            ESP_LOGE(TAG, "Non-finite value at index %d: %f", i, buf[i]);
            buf[i] = 0.0f;
        }
        mean += buf[i];
    }
    
    if (N > 0) {
        mean /= N;
        // 检查均值是否合理
        if (!isfinite(mean)) {
            ESP_LOGE(TAG, "Non-finite mean: %f", mean);
            mean = 0.0f;
        }
        
        for (int i = 0; i < N; i++) {
            buf[i] -= mean;
            // 确保结果值合理
            if (!isfinite(buf[i])) {
                buf[i] = 0.0f;
            }
        }
    }
}

static float parabolic_interp(float left, float center, float right) {
    float denom = (left - 2.0f * center + right);
    if (denom == 0.0f) return 0.0f;
    return 0.5f * (left - right) / denom;
}

static float compute_tdoa_gcc_phat(const float *x, const float *y, int N) {
    // 参数安全检查
    if (!x || !y || N <= 0 || N > 8192) {
        ESP_LOGE(TAG, "Invalid parameters: x=%p y=%p N=%d", x, y, N);
        return 0.0f;
    }
    
    // 检查内存对齐
    if (((uintptr_t)x % 4 != 0) || ((uintptr_t)y % 4 != 0)) {
        ESP_LOGE(TAG, "Unaligned buffers: x=%p y=%p", x, y);
        return 0.0f;
    }
    
    int nfft = N;
    // allocate naive fft cfgs (not used internally but to match interface)
    void *fwd = kiss_fftr_alloc(nfft, 0, NULL, NULL);
    void *inv = kiss_fftr_alloc(nfft, 1, NULL, NULL);
    if (!fwd || !inv) {
        ESP_LOGE(TAG, "fft alloc fail");
        if (fwd) free(fwd);
        if (inv) free(inv);
        return 0.0f;
    }

    int ncpx = nfft/2 + 1;
    kiss_fft_cpx *X = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx)*ncpx);
    kiss_fft_cpx *Y = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx)*ncpx);
    kiss_fft_cpx *R = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx)*ncpx);
    float *cc = (float*)malloc(sizeof(float)*nfft);
    float *xw = (float*)malloc(sizeof(float)*nfft);
    float *yw = (float*)malloc(sizeof(float)*nfft);
    
    // 检查所有内存分配
    if (!X || !Y || !R || !cc || !xw || !yw) {
        ESP_LOGE(TAG, "malloc fail fft temps");
        if (X) free(X);
        if (Y) free(Y);
        if (R) free(R);
        if (cc) free(cc);
        if (xw) free(xw);
        if (yw) free(yw);
        free(fwd);
        free(inv);
        return 0.0f;
    }
    
    // 初始化内存
    memset(X, 0, sizeof(kiss_fft_cpx)*ncpx);
    memset(Y, 0, sizeof(kiss_fft_cpx)*ncpx);
    memset(R, 0, sizeof(kiss_fft_cpx)*ncpx);
    memset(cc, 0, sizeof(float)*nfft);
    memset(xw, 0, sizeof(float)*nfft);
    memset(yw, 0, sizeof(float)*nfft);

    for (int i=0;i<N;i++){
        // 检查输入值是否合理
        if (!isfinite(x[i]) || !isfinite(y[i])) {
            ESP_LOGW(TAG, "Non-finite input at index %d: x=%f y=%f", i, x[i], y[i]);
            xw[i] = 0.0f;
            yw[i] = 0.0f;
        } else {
            xw[i] = x[i]*window[i];
            yw[i] = y[i]*window[i];
        }
    }

    // 执行FFT
    kiss_fftr(fwd, xw, X);
    kiss_fftr(fwd, yw, Y);

    for (int k=0;k<ncpx;k++){
        // 检查FFT结果是否合理
        if (!isfinite(X[k].r) || !isfinite(X[k].i) || 
            !isfinite(Y[k].r) || !isfinite(Y[k].i)) {
            ESP_LOGW(TAG, "Non-finite FFT result at bin %d", k);
            R[k].r = 0.0f;
            R[k].i = 0.0f;
            continue;
        }
        
        float Rre = X[k].r * Y[k].r + X[k].i * Y[k].i;
        float Rim = X[k].i * Y[k].r - X[k].r * Y[k].i;
        
        // 检查计算结果
        if (!isfinite(Rre) || !isfinite(Rim)) {
            ESP_LOGW(TAG, "Non-finite cross-correlation at bin %d", k);
            R[k].r = 0.0f;
            R[k].i = 0.0f;
            continue;
        }
        
        float mag = sqrtf(Rre*Rre + Rim*Rim);
        if (mag < 1e-8f) {
            R[k].r = 0.0f; 
            R[k].i = 0.0f;
        } else {
            R[k].r = Rre / mag;
            R[k].i = Rim / mag;
            
            // 检查归一化结果
            if (!isfinite(R[k].r) || !isfinite(R[k].i)) {
                ESP_LOGW(TAG, "Non-finite normalized result at bin %d", k);
                R[k].r = 0.0f;
                R[k].i = 0.0f;
            }
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
    // 参数安全检查
    if (!left || !right) {
        ESP_LOGE(TAG, "Invalid input buffers in compute_doa_from_frame");
        return 0.0f;
    }
    
    // 使用堆内存避免栈溢出
    float* x = (float*)malloc(FRAME_SIZE * sizeof(float));
    float* y = (float*)malloc(FRAME_SIZE * sizeof(float));
    
    if (!x || !y) {
        ESP_LOGE(TAG, "Failed to allocate memory for DOA computation");
        if (x) free(x);
        if (y) free(y);
        return 0.0f;
    }
    
    // 转换数据类型
    for (int i=0;i<FRAME_SIZE;i++){
        x[i] = (float)left[i];
        y[i] = (float)right[i];
    }
    
    // 移除直流分量
    remove_dc(x, FRAME_SIZE);
    remove_dc(y, FRAME_SIZE);

    // 计算TDOA
    float tdoa = compute_tdoa_gcc_phat(x, y, FRAME_SIZE);
    
    // 计算角度
    float tau = tdoa / SAMPLE_RATE;
    float val = SPEED_SOUND * tau / MIC_DISTANCE_M;
    
    // 限制值范围
    if (val > 1.0f) val = 1.0f;
    if (val < -1.0f) val = -1.0f;
    
    // 安全检查asin输入
    float angle = 0.0f;
    if (isnan(val) || !isfinite(val)) {
        ESP_LOGE(TAG, "Invalid value for asin: %f", val);
    } else {
        angle = asinf(val) * 180.0f / M_PI;
        // 检查角度结果
        if (!isfinite(angle)) {
            ESP_LOGE(TAG, "Invalid angle result: %f", angle);
            angle = 0.0f;
        }
    }
    
    free(x);
    free(y);
    
    return angle;
}

AfeAudioProcessor::AfeAudioProcessor()
    : afe_data_(nullptr) {
    event_group_ = xEventGroupCreate();
}

void AfeAudioProcessor::Initialize(AudioCodec* codec, int frame_duration_ms, srmodel_list_t* models_list) {
    codec_ = codec;
    frame_samples_ = frame_duration_ms * 16000 / 1000;

    // Pre-allocate output buffer capacity
    output_buffer_.reserve(frame_samples_);

    int ref_num = codec_->input_reference() ? 1 : 0;
    int total_channels = codec_->input_channels();
    
     ESP_LOGI(TAG, "Audio channels: total=%d, reference=%d", total_channels, ref_num);

    std::string input_format;
    for (int i = 0; i < total_channels - ref_num; i++) {
        input_format.push_back('M');
    }
    for (int i = 0; i < ref_num; i++) {
        input_format.push_back('R');
    }
    
    ESP_LOGI(TAG, "Input format: %s", input_format.c_str());
    
    // 保存通道数供后续使用
    total_channels_ = total_channels;
    ESP_LOGI(TAG, "Audio channels saved: total=%d (from codec=%d), reference=%d, input_channels=%d", 
             total_channels_, total_channels, ref_num, codec_->input_channels());
    
    // 安全检查：确保保存的值合理
    if (total_channels_ <= 0 || total_channels_ > 8) {
        ESP_LOGW(TAG, "Warning: total_channels_ has unexpected value: %d, resetting to 2", total_channels_);
        total_channels_ = 2;
    }

    srmodel_list_t *models;
    if (models_list == nullptr) {
        models = esp_srmodel_init("model");
    } else {
        models = models_list;
    }

    char* ns_model_name = esp_srmodel_filter(models, ESP_NSNET_PREFIX, NULL);
    char* vad_model_name = esp_srmodel_filter(models, ESP_VADN_PREFIX, NULL);
    
    afe_config_t* afe_config = afe_config_init(input_format.c_str(), NULL, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    afe_config->aec_mode = AEC_MODE_VOIP_HIGH_PERF;
    afe_config->vad_mode = VAD_MODE_0;
    afe_config->vad_min_noise_ms = 100;
    if (vad_model_name != nullptr) {
        afe_config->vad_model_name = vad_model_name;
    }

    if (ns_model_name != nullptr) {
        afe_config->ns_init = true;
        afe_config->ns_model_name = ns_model_name;
        afe_config->afe_ns_mode = AFE_NS_MODE_NET;
    } else {
        afe_config->ns_init = false;
    }

    afe_config->agc_init = false;
    afe_config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;

#ifdef CONFIG_USE_DEVICE_AEC
    afe_config->aec_init = true;
    afe_config->vad_init = false;
#else
    afe_config->aec_init = false;
    afe_config->vad_init = true;
#endif


    afe_iface_ = esp_afe_handle_from_config(afe_config);
    afe_data_ = afe_iface_->create_from_config(afe_config);
    ESP_LOGI(TAG, "AFE initialized successfully");
    
    // 初始化窗口函数
    init_window();
    ESP_LOGI(TAG, "Window function initialized");
    
    // 创建音频处理任务
    xTaskCreate([](void* arg) {
        auto this_ = (AfeAudioProcessor*)arg;
        ESP_LOGI(TAG, "Audio processor task started");
        this_->AudioProcessorTask();
        vTaskDelete(NULL);
    }, "audio_communication", 4096, this, 3, NULL);
    
    // 延迟启动DOA模拟任务，避免初始化时资源竞争
    // xTaskCreate([](void* arg) {
    //     vTaskDelay(pdMS_TO_TICKS(2000)); // 延迟2秒启动
    //     auto this_ = (AfeAudioProcessor*)arg;
    //     ESP_LOGI(TAG, "Starting DOA simulation task");
    //     this_->doa_sim_task();
    //     vTaskDelete(NULL);
    // }, "doa_sim_start", 2048, this, 2, NULL);

}

AfeAudioProcessor::~AfeAudioProcessor() {
    // 清理简单的DOA处理器（静态变量）
    static doa_handle_t* simple_doa_handle = nullptr;
    if (simple_doa_handle != nullptr) {
        esp_doa_destroy(simple_doa_handle);
        simple_doa_handle = nullptr;
        ESP_LOGI(TAG, "Simple DOA destroyed");
    }
    
    if (afe_data_ != nullptr) {
        afe_iface_->destroy(afe_data_);
    }
    vEventGroupDelete(event_group_);
}

size_t AfeAudioProcessor::GetFeedSize() {
    if (afe_data_ == nullptr) {
        return 0;
    }
    return afe_iface_->get_feed_chunksize(afe_data_);
}

void AfeAudioProcessor::Feed(std::vector<int16_t>&& data) {
    if (afe_data_ == nullptr) {
        return;
    }
    afe_iface_->feed(afe_data_, data.data());
}

void AfeAudioProcessor::Start() {
    xEventGroupSetBits(event_group_, PROCESSOR_RUNNING);
}

void AfeAudioProcessor::Stop() {
    xEventGroupClearBits(event_group_, PROCESSOR_RUNNING);
    if (afe_data_ != nullptr) {
        afe_iface_->reset_buffer(afe_data_);
    }
}

bool AfeAudioProcessor::IsRunning() {
    return xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING;
}

void AfeAudioProcessor::OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) {
    output_callback_ = callback;
}

void AfeAudioProcessor::OnVadStateChange(std::function<void(bool speaking)> callback) {
    vad_state_change_callback_ = callback;
}

void AfeAudioProcessor::AudioProcessorTask() {
    auto fetch_size = afe_iface_->get_fetch_chunksize(afe_data_);
    auto feed_size = afe_iface_->get_feed_chunksize(afe_data_);
    ESP_LOGI(TAG, "Audio communication task started, feed size: %d fetch size: %d",
        feed_size, fetch_size);

    while (true) {
        xEventGroupWaitBits(event_group_, PROCESSOR_RUNNING, pdFALSE, pdTRUE, portMAX_DELAY);

        // 动态调整超时时间，基于缓冲区状态
        TickType_t timeout;
        
        if (consecutive_timeouts_ > 10) {
            // 如果连续超时次数多，增加超时时间
            timeout = pdMS_TO_TICKS(100);
        } else if (buffer_full_count_ > 5) {
            // 如果缓冲区经常满，减少超时时间以加快处理
            timeout = pdMS_TO_TICKS(20);
        } else {
            // 正常情况使用中等超时时间
            timeout = pdMS_TO_TICKS(50);
        }
        
        auto res = afe_iface_->fetch_with_delay(afe_data_, timeout);
        if ((xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING) == 0) {
            continue;
        }
        if (res == nullptr || res->ret_value == ESP_FAIL) {
            if (res != nullptr) {
                if (res->ret_value == ESP_ERR_TIMEOUT) {
                    // 超时是正常情况
                    consecutive_timeouts_++;
                    if (consecutive_timeouts_ % 50 == 0) { // 每50次超时记录一次
                        ESP_LOGD(TAG, "AFE fetch timeout (consecutive: %u)", consecutive_timeouts_);
                    }
                } else {
                    // 其他错误才记录
                    consecutive_timeouts_ = 0; // 重置连续超时计数
                    ESP_LOGI(TAG, "AFE fetch error: %d", res->ret_value);
                }
            }
            continue;
        }

        // 成功获取数据，重置计数器
        consecutive_timeouts_ = 0;
        buffer_full_count_ = 0;
        
        // 记录AFE输出数据信息（用于调试）
        static uint32_t fetch_count = 0;
        fetch_count++;
        if (fetch_count % 200 == 0) { // 每200次记录一次
            ESP_LOGI(TAG, "AFE fetch: data_size=%d, wakeup_state=%d", 
                     res->data_size, res->wakeup_state);
            
            // 详细检查数据大小
            size_t samples = res->data_size / sizeof(int16_t);
            ESP_LOGI(TAG, "AFE data details: data_size=%d, sizeof(int16_t)=%zu, samples=%zu", 
                     res->data_size, sizeof(int16_t), samples);
        }

      
           
     //   ESP_LOGI(TAG, "DOA Check:============");
        // 调试：检查 total_channels_ 的值
       // ESP_LOGI(TAG, "DOA Check: total_channels_=%d (should be 2)", total_channels_);
        
       // 简单的声源定位检测 - 在主线程中初始化DOA处理器
        static uint32_t last_doa_time = 0;
        static doa_handle_t* simple_doa_handle = nullptr;
        static std::vector<int16_t> accumulated_audio_data;  // 累积的音频数据
        
      //  在主线程中初始化DOA处理器（确保线程安全）
        if (simple_doa_handle == nullptr) {
            int sample_rate = 16000;
            int frame_samples = 256;  // 减小帧大小，减少内存使用
            float mic_distance = 0.045f;  // 45毫米
            ESP_LOGI(TAG, "Initializing DOA: sample_rate=%d, frame_samples=%d, mic_distance=%.3f", 
                     sample_rate, frame_samples, mic_distance);
            simple_doa_handle = esp_doa_create(sample_rate, mic_distance, 0.06f, frame_samples);
            if (simple_doa_handle) {
                ESP_LOGI(TAG, "Simple DOA initialized successfully");
            } else {
                ESP_LOGE(TAG, "Failed to initialize simple DOA");
                simple_doa_handle = nullptr;
            }
        }
        
        //累积AFE输出数据
        size_t current_samples = res->data_size / sizeof(int16_t);
        accumulated_audio_data.insert(accumulated_audio_data.end(), res->data, res->data + current_samples);
        
        // 每10秒进行一次DOA检测
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_doa_time > 10000 && simple_doa_handle != nullptr) {
           // ESP_LOGI(TAG, "DOA Check:============xxxxxxxxx");
            last_doa_time = current_time;
            
            // 安全检查：如果保存的通道数不合理，使用默认值2
            int total_channels = 2;
            size_t required_samples = 256 * total_channels;  // 使用256帧大小
            
            ESP_LOGI(TAG, "DOA Check: accumulated_samples=%zu, total_channels=%d (saved=%d), required_samples=%zu", 
                     accumulated_audio_data.size(), total_channels, total_channels_, required_samples);
            
            // 确保有足够的数据和至少2个通道
            if (accumulated_audio_data.size() >= required_samples && total_channels >= 2) {
               /// ESP_LOGI(TAG, "DOA Check:============xxxxxxxxxqweqeqeqwewqewqw");
                ESP_LOGI(TAG, "Starting DOA detection in background thread");
                
                // 复制累积的音频数据到堆上
                std::vector<int16_t> audio_data_copy = accumulated_audio_data;
                
                // 在单独线程中执行DOA计算
                std::thread([this, audio_data_copy = std::move(audio_data_copy), total_channels, simple_doa_handle]() {
                    // 提取左右声道数据（使用累积数据的前256个样本）
                    std::vector<int16_t> left_channel(256);
                    std::vector<int16_t> right_channel(256);
                    
                    for (int i = 0; i < 256; i++) {
                        if (i * total_channels + 1 < audio_data_copy.size()) {
                            left_channel[i] = audio_data_copy[i * total_channels];  // 第一个通道
                            right_channel[i] = audio_data_copy[i * total_channels + 1];  // 第二个通道
                        }
                    }
                    
                    // 进行DOA计算
                    float angle = esp_doa_process(simple_doa_handle, left_channel.data(), right_channel.data());
                    
                    // 打印角度信息
                    ESP_LOGI(TAG, "Sound Source Direction: %.1f degrees", angle);
                }).detach(); // 分离线程，让它独立运行
                
                // 清空累积的数据，重新开始累积
                accumulated_audio_data.clear();
            } else {
                ESP_LOGW(TAG, "DOA conditions not met: accumulated_samples=%zu, channels=%d", 
                         accumulated_audio_data.size(), total_channels);
            }
        }

        // VAD state change
        if (vad_state_change_callback_) {
            if (res->vad_state == VAD_SPEECH && !is_speaking_) {
                is_speaking_ = true;
                vad_state_change_callback_(true);

            } else if (res->vad_state == VAD_SILENCE && is_speaking_) {
                is_speaking_ = false;
                vad_state_change_callback_(false);
            }
        }

        if (output_callback_) {
            size_t samples = res->data_size / sizeof(int16_t);
            
            // Add data to buffer
            output_buffer_.insert(output_buffer_.end(), res->data, res->data + samples);
            
            // Output complete frames when buffer has enough data
            while (output_buffer_.size() >= frame_samples_) {
                if (output_buffer_.size() == frame_samples_) {
                    // If buffer size equals frame size, move the entire buffer
                    output_callback_(std::move(output_buffer_));
                    output_buffer_.clear();
                    output_buffer_.reserve(frame_samples_);
                } else {
                    // If buffer size exceeds frame size, copy one frame and remove it
                    output_callback_(std::vector<int16_t>(output_buffer_.begin(), output_buffer_.begin() + frame_samples_));
                    output_buffer_.erase(output_buffer_.begin(), output_buffer_.begin() + frame_samples_);
                }
            }
        }
    }
}

void AfeAudioProcessor::EnableDeviceAec(bool enable) {
    if (enable) {
#if CONFIG_USE_DEVICE_AEC
        afe_iface_->disable_vad(afe_data_);
        afe_iface_->enable_aec(afe_data_);
#else
        ESP_LOGE(TAG, "Device AEC is not supported");
#endif
    } else {
        afe_iface_->disable_aec(afe_data_);
        afe_iface_->enable_vad(afe_data_);
    }
}


void AfeAudioProcessor::generate_test_frame1(float *left, float *right, int frame_size, float angle_deg) {
    float freq = 1000.0f;
    float phase = 0.0f;
    float d = MIC_DISTANCE_M;
    float c = SPEED_SOUND;
    float delta_t = (d * sinf(angle_deg * M_PI / 180.0f)) / c;
    float delta_phase = 2.0f * M_PI * freq * delta_t;
    ESP_LOGI(TAG, " generate_test_frame1 1111111====");
    for (int i = 0; i < frame_size; i++) {
        float t = (float)i / SAMPLE_RATE;
        left[i] = sinf(2.0f * M_PI * freq * t + phase);
        right[i] = sinf(2.0f * M_PI * freq * t + phase + delta_phase);
    }
    ESP_LOGI(TAG, " generate_test_frame1 2222222====");
}

static void doa_simulation_task(void* arg) {
    auto this_ = (AfeAudioProcessor*)arg;
    
    // 使用堆内存而不是栈内存，避免栈溢出
    float* left = (float*)malloc(FRAME_SIZE * sizeof(float));
    float* right = (float*)malloc(FRAME_SIZE * sizeof(float));
    
    if (!left || !right) {
        ESP_LOGE(TAG, "Failed to allocate memory for DOA simulation");
        if (left) free(left);
        if (right) free(right);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "DOA simulation task started");
    
    while (1) {
        float test_angle = 30.0f; // simulate source from +30°
        
        // 生成测试帧
        this_->generate_test_frame1(left, right, FRAME_SIZE, test_angle);
        ESP_LOGI(TAG, "Generated test frames");
        
        // 移除直流分量
        remove_dc(left, FRAME_SIZE);
        ESP_LOGI(TAG, "Removed DC from left channel");
        
        remove_dc(right, FRAME_SIZE);
        ESP_LOGI(TAG, "Removed DC from right channel");
        
        // 计算TDOA
        float tdoa = compute_tdoa_gcc_phat(left, right, FRAME_SIZE);
        ESP_LOGI(TAG, "Computed TDOA: %.2f samples", tdoa);
        
        // 计算角度
        float tau = tdoa / SAMPLE_RATE;
        float val = SPEED_SOUND * tau / MIC_DISTANCE_M;
        
        // 限制值范围
        if (val > 1.0f) val = 1.0f;
        if (val < -1.0f) val = -1.0f;
        
        // 安全检查asin输入
        if (isnan(val) || !isfinite(val)) {
            ESP_LOGE(TAG, "Invalid value for asin: %f", val);
            val = 0.0f;
        }
        
        float angle = asinf(val) * 180.0f / M_PI;
        
        ESP_LOGI(TAG, "True=%.2f°, Estimated=%.2f°", test_angle, angle);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    free(left);
    free(right);
}

void AfeAudioProcessor::doa_sim_task() {
    // 创建单独的任务来运行DOA模拟，避免阻塞主线程
    xTaskCreate(doa_simulation_task, 
                "doa_sim", 
                4096,  // 更大的栈空间
                this, 
                2,     // 较低的优先级
                NULL);
}

void AfeAudioProcessor::generate_test_frame(int16_t *left, int16_t *right, int frame_size, float angle_deg, int sample_rate)
{
    int TEST_FREQ = 1000;
    static float phase = 0.0f;
    const float d = 0.06f;
    const float c = 343.0f;

    float theta = angle_deg * M_PI / 180.0f;
    float tau = d * cosf(theta) / c;

    int delay_samples = (int)roundf(tau * sample_rate);
    printf("Angle: %f, Delay: %d samples\n", angle_deg, delay_samples);

    for (int i = 0; i < frame_size; i++) {
        float t = (float)(i + phase) / sample_rate;
        left[i] = (int16_t)(sinf(2 * M_PI * TEST_FREQ * t) * 32767);

        int delayed_index = i - delay_samples;
        right[i] = (int16_t)(sinf(2 * M_PI * TEST_FREQ * (delayed_index + phase) / sample_rate) * 32767);
    }
    phase += frame_size;
}

void AfeAudioProcessor::test_doa(){
    ESP_LOGE(TAG, "in AfeAudioProcessor test_doa call");
      // 初始化DOA估计器
      int frame_samples = 1024;
      int sample_rate = 16000;
      int16_t *left = (int16_t *)malloc(frame_samples * sizeof(int16_t));
      int16_t *right = (int16_t *)malloc(frame_samples * sizeof(int16_t));
      int start_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      doa_handle_t *doa = esp_doa_create(sample_rate, 20.0f, 0.06f, frame_samples);
  
      uint32_t c0, c1, t_doa = 0;
      int angle = 10;
      for (int f = 0; f < angle; f++) { // 1秒多帧
          generate_test_frame(left, right, frame_samples, f*1.0, sample_rate);
          c0 = esp_timer_get_time();
          float est_angle = esp_doa_process(doa, left, right);
          c1 = esp_timer_get_time();
          t_doa += c1 - c0;
         // ESP_LOGE(TAG, "doa memory size:%d, cpu loading:%f\n", doa_mem_size, (t_doa * 1.0 / 1000000 * sample_rate) / (angle * frame_samples));
   
          ESP_LOGE(TAG,"%.1f\t\t%.1f\n", f*1.0, est_angle); // memory leak
      }
      int doa_mem_size = start_size - heap_caps_get_free_size(MALLOC_CAP_8BIT);
     // ESP_LOGE(TAG, "in AfeAudioProcessor test_doa call");
      ESP_LOGE(TAG, "doa memory size:%d, cpu loading:%f\n", doa_mem_size, (t_doa * 1.0 / 1000000 * sample_rate) / (angle * frame_samples));
   
      esp_doa_destroy(doa);
      int end_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  
      // create & destroy 5 times
      for (int i = 0; i < 5; i++) {
          doa = esp_doa_create(sample_rate, 20.0f, 0.06f, frame_samples);
          esp_doa_process(doa, left, right);
          esp_doa_destroy(doa);
      }
  
      int last_end_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        ESP_LOGE(TAG, "memory leak:%d\n", start_size - end_size);

      free(left);
      free(right);
      // return 0;
      ESP_LOGE(TAG, "TEST DONE\n\n");
}