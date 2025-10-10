#include "afe_audio_processor.h"
#include <esp_log.h>

#define PROCESSOR_RUNNING 0x01

#define TAG "AfeAudioProcessor"

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
    // total_channels_ = total_channels;

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

    xTaskCreate([](void* arg) {
        auto this_ = (AfeAudioProcessor*)arg;
        //this_->test_doa();
        ESP_LOGE(TAG, "xTaskCreate test_doa call");
        this_->AudioProcessorTask();
     
        vTaskDelete(NULL);
    }, "audio_communication", 4096, this, 3, NULL);

}

AfeAudioProcessor::~AfeAudioProcessor() {
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