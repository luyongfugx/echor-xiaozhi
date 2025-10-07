#include "afe_audio_processor.h"
#include <esp_log.h>

#define PROCESSOR_RUNNING 0x01

#define TAG "AfeAudioProcessor"

AfeAudioProcessor::AfeAudioProcessor()
    : afe_data_(nullptr) {
    event_group_ = xEventGroupCreate();
    // DOA 缓冲区将在 Initialize 方法中预分配
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
   
    doa_sample_rate_ = 16000;   // 与麦克风采样率一致
    doa_frame_samples_ = 1024;  // 每帧采样点数

    ESP_LOGI(TAG, "Initializing DOA - sample_rate: %d, frame_samples: %d", 
            doa_sample_rate_, doa_frame_samples_);
    
    doa_handle_ = esp_doa_create(
        doa_sample_rate_,
        343.0f,   // 声速（m/s）修正为实际声速
        0.06f,    // 麦间距，单位米（根据硬件）
        doa_frame_samples_
    );

    if (doa_handle_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create DOA handle!");
    } else {
        ESP_LOGI(TAG, "DOA initialized successfully (fs=%d, spacing=%.2fm, frame=%d)", 
                doa_sample_rate_, 0.06f, doa_frame_samples_);
    }

    // 预分配 DOA 缓冲区容量（2倍所需大小，避免频繁重分配）
    size_t buffer_capacity = doa_frame_samples_ * total_channels_ * 2;
    doa_buffer_.reserve(buffer_capacity);
    ESP_LOGI(TAG, "DOA buffer pre-allocated with capacity: %zu samples", buffer_capacity);

    xTaskCreate([](void* arg) {
        auto this_ = (AfeAudioProcessor*)arg;
        this_->AudioProcessorTask();
        vTaskDelete(NULL);
    }, "audio_communication", 4096, this, 3, NULL);
}

AfeAudioProcessor::~AfeAudioProcessor() {
    if (doa_handle_) {
        esp_doa_destroy(doa_handle_);
        doa_handle_ = nullptr;
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
    // 清空 DOA 缓冲区
    doa_buffer_.clear();
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

        auto res = afe_iface_->fetch_with_delay(afe_data_, portMAX_DELAY);
        if ((xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING) == 0) {
            continue;
        }
        if (res == nullptr || res->ret_value == ESP_FAIL) {
            if (res != nullptr) {
                ESP_LOGI(TAG, "Error code: %d", res->ret_value);
            }
            continue;
        }

   
        // VAD state change
        if (vad_state_change_callback_) {
            if (res->vad_state == VAD_SPEECH && !is_speaking_) {
                is_speaking_ = true;
                vad_state_change_callback_(true);
                
                // ✅ 声源定位 DOA - 使用循环缓冲区累积数据
                if (doa_handle_ && total_channels_ >= 2) {
                    size_t current_samples = res->data_size / sizeof(int16_t);
                    size_t required_samples = doa_frame_samples_ * total_channels_;
                    
                    ESP_LOGI(TAG, "DOA Buffer - current: %zu samples, required: %zu samples, buffer: %zu samples", 
                            current_samples, required_samples, doa_buffer_.size());
                    
                    // 将当前数据添加到缓冲区
                    doa_buffer_.insert(doa_buffer_.end(), res->data, res->data + current_samples);
                    
                    // 当缓冲区有足够数据时进行 DOA 处理
                    while (doa_buffer_.size() >= required_samples) {
                        ESP_LOGI(TAG, "DOA processing - buffer has %zu samples, processing %zu samples", 
                                doa_buffer_.size(), required_samples);
                        
                        // 直接使用缓冲区中的数据指针，避免数据复制
                        int16_t* left_channel = doa_buffer_.data();
                        int16_t* right_channel = doa_buffer_.data() + 1;
                        
                        // 打印前几个样本值验证数据
                        if (doa_buffer_.size() >= 8) {
                            ESP_LOGI(TAG, "DOA data - L[0-3]: %d,%d,%d,%d R[0-3]: %d,%d,%d,%d",
                                    left_channel[0], left_channel[2], left_channel[4], left_channel[6],
                                    right_channel[0], right_channel[2], right_channel[4], right_channel[6]);
                        }
                        
                        // 调用 DOA 处理 - 注意：这里需要交错步长为2
                        float doa_angle = esp_doa_process(doa_handle_, left_channel, right_channel);
                        
                        // 检查 DOA 角度是否有效
                        if (doa_angle >= -180.0f && doa_angle <= 180.0f) {
                            ESP_LOGI(TAG, "DOA result: %.1f°", doa_angle);
                        } else {
                            ESP_LOGW(TAG, "DOA returned invalid angle: %.1f°", doa_angle);
                        }
                        
                        // 移除已处理的数据（滑动窗口）
                        doa_buffer_.erase(doa_buffer_.begin(), doa_buffer_.begin() + required_samples);
                        ESP_LOGI(TAG, "DOA buffer after processing: %zu samples", doa_buffer_.size());
                    }
                } else {
                    if (!doa_handle_) {
                        ESP_LOGE(TAG, "DOA handle is NULL!");
                    }
                    if (total_channels_ < 2) {
                        ESP_LOGW(TAG, "Insufficient channels for DOA: %d channels available", total_channels_);
                    }
                }


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
