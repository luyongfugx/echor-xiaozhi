#include "audio_service.h"
#include <esp_log.h>
#include <cstring>
#include <cmath>

#include "doa_simple.h"
#include "doa_fft.h"

#if CONFIG_USE_AUDIO_PROCESSOR
#include "processors/afe_audio_processor.h"
#else
#include "processors/no_audio_processor.h"
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#include "wake_words/afe_wake_word.h"
#include "wake_words/custom_wake_word.h"
#else
#include "wake_words/esp_wake_word.h"
#endif

#define TAG "AudioService"
#define AUDIO_SERVICE_INPUT_SAMPLE_RATE 16000

AudioService::AudioService()
{
    event_group_ = xEventGroupCreate();
}

AudioService::~AudioService()
{
    if (event_group_ != nullptr)
    {
        vEventGroupDelete(event_group_);
    }
}

void AudioService::Initialize(AudioCodec *codec)
{
    codec_ = codec;
    codec_->Start();

    /* Setup the audio codec */
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(codec->output_sample_rate(), 1, OPUS_FRAME_DURATION_MS);
    opus_encoder_ = std::make_unique<OpusEncoderWrapper>(AUDIO_SERVICE_INPUT_SAMPLE_RATE, 1, OPUS_FRAME_DURATION_MS);
    opus_encoder_->SetComplexity(0);

    if (codec->input_sample_rate() != AUDIO_SERVICE_INPUT_SAMPLE_RATE)
    {
        input_resampler_.Configure(codec->input_sample_rate(), AUDIO_SERVICE_INPUT_SAMPLE_RATE);
        reference_resampler_.Configure(codec->input_sample_rate(), AUDIO_SERVICE_INPUT_SAMPLE_RATE);
    }

#if CONFIG_USE_AUDIO_PROCESSOR
    audio_processor_ = std::make_unique<AfeAudioProcessor>();
#else
    audio_processor_ = std::make_unique<NoAudioProcessor>();
#endif

    audio_processor_->OnOutput([this](std::vector<int16_t> &&data)
                               { PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue, std::move(data)); });

    audio_processor_->OnVadStateChange([this](bool speaking)
                                       {
        voice_detected_ = speaking;
        if (callbacks_.on_vad_change) {
            callbacks_.on_vad_change(speaking);
        } });

    esp_timer_create_args_t audio_power_timer_args = {
        .callback = [](void *arg)
        {
            AudioService *audio_service = (AudioService *)arg;
            audio_service->CheckAndUpdateAudioPowerState();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "audio_power_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&audio_power_timer_args, &audio_power_timer_);
}

void AudioService::Start()
{
    service_stopped_ = false;
    xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING | AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING);

    esp_timer_start_periodic(audio_power_timer_, 1000000);

#if CONFIG_USE_AUDIO_PROCESSOR
    /* Start the audio input task */
    xTaskCreatePinnedToCore([](void *arg)
                            {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioInputTask();
        vTaskDelete(NULL); }, "audio_input", 2048 * 3, this, 8, &audio_input_task_handle_, 0);

    /* Start the audio output task */
    xTaskCreate([](void *arg)
                {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioOutputTask();
        vTaskDelete(NULL); }, "audio_output", 2048 * 2, this, 4, &audio_output_task_handle_);
#else
    /* Start the audio input task */
    xTaskCreate([](void *arg)
                {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioInputTask();
        vTaskDelete(NULL); }, "audio_input", 2048 * 2, this, 8, &audio_input_task_handle_);

    /* Start the audio output task */
    xTaskCreate([](void *arg)
                {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioOutputTask();
        vTaskDelete(NULL); }, "audio_output", 2048, this, 4, &audio_output_task_handle_);
#endif

    /* Start the opus codec task */
    xTaskCreate([](void *arg)
                {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->OpusCodecTask();
        vTaskDelete(NULL); }, "opus_codec", 2048 * 13, this, 2, &opus_codec_task_handle_);

    // Continuous DOA task is disabled - we only do DOA on wake word detection
    // DOA will be triggered by OnWakeWordDetected callback
}

void AudioService::Stop()
{
    esp_timer_stop(audio_power_timer_);
    service_stopped_ = true;
    xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING |
                                         AS_EVENT_WAKE_WORD_RUNNING |
                                         AS_EVENT_AUDIO_PROCESSOR_RUNNING);

    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    audio_encode_queue_.clear();
    audio_decode_queue_.clear();
    audio_playback_queue_.clear();
    audio_testing_queue_.clear();
    audio_queue_cv_.notify_all();
}

bool AudioService::ReadAudioData(std::vector<int16_t> &data, int sample_rate, int samples)
{
    if (!codec_->input_enabled())
    {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableInput(true);
    }

    if (codec_->input_sample_rate() != sample_rate)
    {
        data.resize(samples * codec_->input_sample_rate() / sample_rate * codec_->input_channels());
        if (!codec_->InputData(data))
        {
            return false;
        }
        if (codec_->input_channels() == 2)
        {
            auto mic_channel = std::vector<int16_t>(data.size() / 2);
            auto reference_channel = std::vector<int16_t>(data.size() / 2);
            for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2)
            {
                mic_channel[i] = data[j];
                reference_channel[i] = data[j + 1];
            }
            auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
            auto resampled_reference = std::vector<int16_t>(reference_resampler_.GetOutputSamples(reference_channel.size()));
            input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
            reference_resampler_.Process(reference_channel.data(), reference_channel.size(), resampled_reference.data());
            data.resize(resampled_mic.size() + resampled_reference.size());
            for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2)
            {
                data[j] = resampled_mic[i];
                data[j + 1] = resampled_reference[i];
            }
        }
        else
        {
            auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
            input_resampler_.Process(data.data(), data.size(), resampled.data());
            data = std::move(resampled);
        }
    }
    else
    {
        data.resize(samples * codec_->input_channels());
        if (!codec_->InputData(data))
        {
            return false;
        }
    }

    /* Update the last input time */
    last_input_time_ = std::chrono::steady_clock::now();
    debug_statistics_.input_count++;

#if CONFIG_USE_AUDIO_DEBUGGER
    // 音频调试：发送原始音频数据
    if (audio_debugger_ == nullptr)
    {
        audio_debugger_ = std::make_unique<AudioDebugger>();
    }
    audio_debugger_->Feed(data);
#endif

    return true;
}

void AudioService::AudioInputTask()
{
    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING | AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING,
                                               pdFALSE, pdFALSE, portMAX_DELAY);

        if (service_stopped_)
        {
            break;
        }
        if (audio_input_need_warmup_)
        {
            audio_input_need_warmup_ = false;
            vTaskDelay(pdMS_TO_TICKS(120));
            continue;
        }

        /* Used for audio testing in NetworkConfiguring mode by clicking the BOOT button */
        if (bits & AS_EVENT_AUDIO_TESTING_RUNNING)
        {

            if (audio_testing_queue_.size() >= AUDIO_TESTING_MAX_DURATION_MS / OPUS_FRAME_DURATION_MS)
            {
                ESP_LOGW(TAG, "Audio testing queue is full, stopping audio testing");
                EnableAudioTesting(false);
                continue;
            }

            std::vector<int16_t> data;
            int samples = OPUS_FRAME_DURATION_MS * AUDIO_SERVICE_INPUT_SAMPLE_RATE / 1000;
            if (ReadAudioData(data, AUDIO_SERVICE_INPUT_SAMPLE_RATE, samples))
            {
                if (codec_->input_channels() == 2)
                {
                    auto mono_data = std::vector<int16_t>(data.size() / 2);
                    for (size_t i = 0, j = 0; i < mono_data.size(); ++i, j += 2)
                    {
                        mono_data[i] = data[j];
                    }
                    data = std::move(mono_data);
                }
                PushTaskToEncodeQueue(kAudioTaskTypeEncodeToTestingQueue, std::move(data));
                continue;
            }
        }

        /* Feed the wake word */
        if (bits & AS_EVENT_WAKE_WORD_RUNNING)
        {
            std::vector<int16_t> data;
            int samples = wake_word_->GetFeedSize();
            if (samples > 0)
            {
                if (ReadAudioData(data, AUDIO_SERVICE_INPUT_SAMPLE_RATE, samples))
                {
                    // 先喂给唤醒词检测器,避免AFE缓冲区溢出
                    wake_word_->Feed(data);

                    // 然后将音频数据累积到doa_buffer_中，用于唤醒词触发时的DOA检测
                    // 保留最近1秒的音频数据,当检测到唤醒词时使用这些数据进行声源定位
                    {
                        std::lock_guard<std::mutex> lock(audio_queue_mutex_);

                        size_t one_second_samples = static_cast<size_t>(AUDIO_SERVICE_INPUT_SAMPLE_RATE * 1) * codec_->input_channels();

                        // 优化:使用预分配+覆盖而不是insert+erase,避免频繁内存操作
                        if (doa_buffer_.capacity() < one_second_samples)
                        {
                            doa_buffer_.reserve(one_second_samples);
                        }

                        if (doa_buffer_.size() < one_second_samples)
                        {
                            // 缓冲区未满,直接追加
                            doa_buffer_.insert(doa_buffer_.end(), data.begin(), data.end());
                        }
                        else
                        {
                            // 缓冲区已满,使用环形覆盖方式:删除旧数据并追加新数据
                            // 计算需要删除的数量
                            size_t overflow = doa_buffer_.size() + data.size() - one_second_samples;
                            if (overflow > 0)
                            {
                                // 删除最旧的数据
                                doa_buffer_.erase(doa_buffer_.begin(), doa_buffer_.begin() + overflow);
                            }
                            // 追加新数据
                            doa_buffer_.insert(doa_buffer_.end(), data.begin(), data.end());
                        }
                    }

                    continue;
                }
            }
        }

        /* Feed the audio processor */
        if (bits & AS_EVENT_AUDIO_PROCESSOR_RUNNING)
        {
            std::vector<int16_t> data;
            int samples = audio_processor_->GetFeedSize();
            if (samples > 0)
            {
                if (ReadAudioData(data, AUDIO_SERVICE_INPUT_SAMPLE_RATE, samples))
                {
                    audio_processor_->Feed(std::move(data));
                    continue;
                }
            }
        }

        ESP_LOGE(TAG, "Should not be here, bits: %lx", bits);
        break;
    }

    ESP_LOGW(TAG, "Audio input task stopped");
}

void AudioService::AudioOutputTask()
{
    while (true)
    {
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);
        audio_queue_cv_.wait(lock, [this]()
                             { return !audio_playback_queue_.empty() || service_stopped_; });
        if (service_stopped_)
        {
            break;
        }

        auto task = std::move(audio_playback_queue_.front());
        audio_playback_queue_.pop_front();
        audio_queue_cv_.notify_all();
        lock.unlock();

        if (!codec_->output_enabled())
        {
            esp_timer_stop(audio_power_timer_);
            esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
            codec_->EnableOutput(true);
        }
        codec_->OutputData(task->pcm);

        /* Update the last output time */
        last_output_time_ = std::chrono::steady_clock::now();
        debug_statistics_.playback_count++;

#if CONFIG_USE_SERVER_AEC
        /* Record the timestamp for server AEC */
        if (task->timestamp > 0)
        {
            lock.lock();
            timestamp_queue_.push_back(task->timestamp);
        }
#endif
    }

    ESP_LOGW(TAG, "Audio output task stopped");
}

void AudioService::OpusCodecTask()
{
    while (true)
    {
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);
        audio_queue_cv_.wait(lock, [this]()
                             { return service_stopped_ ||
                                      (!audio_encode_queue_.empty() && audio_send_queue_.size() < MAX_SEND_PACKETS_IN_QUEUE) ||
                                      (!audio_decode_queue_.empty() && audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE); });
        if (service_stopped_)
        {
            break;
        }

        /* Decode the audio from decode queue */
        if (!audio_decode_queue_.empty() && audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE)
        {
            auto packet = std::move(audio_decode_queue_.front());
            audio_decode_queue_.pop_front();
            audio_queue_cv_.notify_all();
            lock.unlock();

            auto task = std::make_unique<AudioTask>();
            task->type = kAudioTaskTypeDecodeToPlaybackQueue;
            task->timestamp = packet->timestamp;

            SetDecodeSampleRate(packet->sample_rate, packet->frame_duration);
            if (opus_decoder_->Decode(std::move(packet->payload), task->pcm))
            {
                // Resample if the sample rate is different
                if (opus_decoder_->sample_rate() != codec_->output_sample_rate())
                {
                    int target_size = output_resampler_.GetOutputSamples(task->pcm.size());
                    std::vector<int16_t> resampled(target_size);
                    output_resampler_.Process(task->pcm.data(), task->pcm.size(), resampled.data());
                    task->pcm = std::move(resampled);
                }

                lock.lock();
                audio_playback_queue_.push_back(std::move(task));
                audio_queue_cv_.notify_all();
            }
            else
            {
                ESP_LOGE(TAG, "Failed to decode audio");
                lock.lock();
            }
            debug_statistics_.decode_count++;
        }

        /* Encode the audio to send queue */
        if (!audio_encode_queue_.empty() && audio_send_queue_.size() < MAX_SEND_PACKETS_IN_QUEUE)
        {
            auto task = std::move(audio_encode_queue_.front());
            audio_encode_queue_.pop_front();
            audio_queue_cv_.notify_all();
            lock.unlock();

            auto packet = std::make_unique<AudioStreamPacket>();
            packet->frame_duration = OPUS_FRAME_DURATION_MS;
            packet->sample_rate = AUDIO_SERVICE_INPUT_SAMPLE_RATE;
            packet->timestamp = task->timestamp;
            if (!opus_encoder_->Encode(std::move(task->pcm), packet->payload))
            {
                ESP_LOGE(TAG, "Failed to encode audio");
                continue;
            }

            if (task->type == kAudioTaskTypeEncodeToSendQueue)
            {
                {
                    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
                    audio_send_queue_.push_back(std::move(packet));
                }
                if (callbacks_.on_send_queue_available)
                {
                    callbacks_.on_send_queue_available();
                }
            }
            else if (task->type == kAudioTaskTypeEncodeToTestingQueue)
            {
                std::lock_guard<std::mutex> lock(audio_queue_mutex_);
                audio_testing_queue_.push_back(std::move(packet));
            }
            debug_statistics_.encode_count++;
            lock.lock();
        }
    }

    ESP_LOGW(TAG, "Opus codec task stopped");
}

void AudioService::SetDecodeSampleRate(int sample_rate, int frame_duration)
{
    if (opus_decoder_->sample_rate() == sample_rate && opus_decoder_->duration_ms() == frame_duration)
    {
        return;
    }

    opus_decoder_.reset();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(sample_rate, 1, frame_duration);

    auto codec = Board::GetInstance().GetAudioCodec();
    if (opus_decoder_->sample_rate() != codec->output_sample_rate())
    {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", opus_decoder_->sample_rate(), codec->output_sample_rate());
        output_resampler_.Configure(opus_decoder_->sample_rate(), codec->output_sample_rate());
    }
}

void AudioService::PushTaskToEncodeQueue(AudioTaskType type, std::vector<int16_t> &&pcm)
{
    auto task = std::make_unique<AudioTask>();
    task->type = type;
    task->pcm = std::move(pcm);

    /* Push the task to the encode queue */
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);

    /* If the task is to send queue, we need to set the timestamp */
    if (type == kAudioTaskTypeEncodeToSendQueue && !timestamp_queue_.empty())
    {
        if (timestamp_queue_.size() <= MAX_TIMESTAMPS_IN_QUEUE)
        {
            task->timestamp = timestamp_queue_.front();
        }
        else
        {
            ESP_LOGW(TAG, "Timestamp queue (%u) is full, dropping timestamp", timestamp_queue_.size());
        }
        timestamp_queue_.pop_front();
    }

    audio_queue_cv_.wait(lock, [this]()
                         { return audio_encode_queue_.size() < MAX_ENCODE_TASKS_IN_QUEUE; });
    audio_encode_queue_.push_back(std::move(task));
    audio_queue_cv_.notify_all();
}

bool AudioService::PushPacketToDecodeQueue(std::unique_ptr<AudioStreamPacket> packet, bool wait)
{
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);
    if (audio_decode_queue_.size() >= MAX_DECODE_PACKETS_IN_QUEUE)
    {
        if (wait)
        {
            audio_queue_cv_.wait(lock, [this]()
                                 { return audio_decode_queue_.size() < MAX_DECODE_PACKETS_IN_QUEUE; });
        }
        else
        {
            return false;
        }
    }
    audio_decode_queue_.push_back(std::move(packet));
    audio_queue_cv_.notify_all();
    return true;
}

std::unique_ptr<AudioStreamPacket> AudioService::PopPacketFromSendQueue()
{
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    if (audio_send_queue_.empty())
    {
        return nullptr;
    }
    auto packet = std::move(audio_send_queue_.front());
    audio_send_queue_.pop_front();
    audio_queue_cv_.notify_all();
    return packet;
}

void AudioService::EncodeWakeWord()
{
    if (wake_word_)
    {
        wake_word_->EncodeWakeWordData();
    }
}

const std::string &AudioService::GetLastWakeWord() const
{
    return wake_word_->GetLastDetectedWakeWord();
}

std::unique_ptr<AudioStreamPacket> AudioService::PopWakeWordPacket()
{
    auto packet = std::make_unique<AudioStreamPacket>();
    if (wake_word_->GetWakeWordOpus(packet->payload))
    {
        return packet;
    }
    return nullptr;
}

void AudioService::EnableWakeWordDetection(bool enable)
{
    if (!wake_word_)
    {
        return;
    }

    ESP_LOGD(TAG, "%s wake word detection", enable ? "Enabling" : "Disabling");
    if (enable)
    {
        if (!wake_word_initialized_)
        {
            if (!wake_word_->Initialize(codec_, models_list_))
            {
                ESP_LOGE(TAG, "Failed to initialize wake word");
                return;
            }
            wake_word_initialized_ = true;
        }
        wake_word_->Start();
        xEventGroupSetBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
    }
    else
    {
        wake_word_->Stop();
        xEventGroupClearBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
    }
}

void AudioService::EnableVoiceProcessing(bool enable)
{
    ESP_LOGD(TAG, "%s voice processing", enable ? "Enabling" : "Disabling");
    if (enable)
    {
        if (!audio_processor_initialized_)
        {
            audio_processor_->Initialize(codec_, OPUS_FRAME_DURATION_MS, models_list_);
            audio_processor_initialized_ = true;
        }

        /* We should make sure no audio is playing */
        ResetDecoder();
        audio_input_need_warmup_ = true;
        audio_processor_->Start();
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_PROCESSOR_RUNNING);
    }
    else
    {
        audio_processor_->Stop();
        xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_PROCESSOR_RUNNING);
    }
}

void AudioService::EnableAudioTesting(bool enable)
{
    ESP_LOGI(TAG, "%s audio testing", enable ? "Enabling" : "Disabling");
    if (enable)
    {
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING);
    }
    else
    {
        xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING);
        /* Copy audio_testing_queue_ to audio_decode_queue_ */
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        audio_decode_queue_ = std::move(audio_testing_queue_);
        audio_queue_cv_.notify_all();
    }
}

void AudioService::EnableDeviceAec(bool enable)
{
    ESP_LOGE(TAG, "%s device AEC", enable ? "Enabling" : "Disabling");
    if (!audio_processor_initialized_)
    {
        audio_processor_->Initialize(codec_, OPUS_FRAME_DURATION_MS, models_list_);
        audio_processor_initialized_ = true;
    }

    audio_processor_->EnableDeviceAec(enable);
}

void AudioService::SetCallbacks(AudioServiceCallbacks &callbacks)
{
    callbacks_ = callbacks;
}

void AudioService::PlaySound(const std::string_view &ogg)
{
    if (!codec_->output_enabled())
    {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableOutput(true);
    }

    const uint8_t *buf = reinterpret_cast<const uint8_t *>(ogg.data());
    size_t size = ogg.size();
    size_t offset = 0;

    auto find_page = [&](size_t start) -> size_t
    {
        for (size_t i = start; i + 4 <= size; ++i)
        {
            if (buf[i] == 'O' && buf[i + 1] == 'g' && buf[i + 2] == 'g' && buf[i + 3] == 'S')
                return i;
        }
        return static_cast<size_t>(-1);
    };

    bool seen_head = false;
    bool seen_tags = false;
    int sample_rate = AUDIO_SERVICE_INPUT_SAMPLE_RATE; // 默认值

    while (true)
    {
        size_t pos = find_page(offset);
        if (pos == static_cast<size_t>(-1))
            break;
        offset = pos;
        if (offset + 27 > size)
            break;

        const uint8_t *page = buf + offset;
        uint8_t page_segments = page[26];
        size_t seg_table_off = offset + 27;
        if (seg_table_off + page_segments > size)
            break;

        size_t body_size = 0;
        for (size_t i = 0; i < page_segments; ++i)
            body_size += page[27 + i];

        size_t body_off = seg_table_off + page_segments;
        if (body_off + body_size > size)
            break;

        // Parse packets using lacing
        size_t cur = body_off;
        size_t seg_idx = 0;
        while (seg_idx < page_segments)
        {
            size_t pkt_len = 0;
            size_t pkt_start = cur;
            bool continued = false;
            do
            {
                uint8_t l = page[27 + seg_idx++];
                pkt_len += l;
                cur += l;
                continued = (l == 255);
            } while (continued && seg_idx < page_segments);

            if (pkt_len == 0)
                continue;
            const uint8_t *pkt_ptr = buf + pkt_start;

            if (!seen_head)
            {
                // 解析OpusHead包
                if (pkt_len >= 19 && std::memcmp(pkt_ptr, "OpusHead", 8) == 0)
                {
                    seen_head = true;

                    // OpusHead结构：[0-7] "OpusHead", [8] version, [9] channel_count, [10-11] pre_skip
                    // [12-15] input_sample_rate, [16-17] output_gain, [18] mapping_family
                    if (pkt_len >= 12)
                    {
                        uint8_t version = pkt_ptr[8];
                        uint8_t channel_count = pkt_ptr[9];

                        if (pkt_len >= 16)
                        {
                            // 读取输入采样率 (little-endian)
                            sample_rate = pkt_ptr[12] | (pkt_ptr[13] << 8) |
                                          (pkt_ptr[14] << 16) | (pkt_ptr[15] << 24);
                            ESP_LOGI(TAG, "OpusHead: version=%d, channels=%d, sample_rate=%d",
                                     version, channel_count, sample_rate);
                        }
                    }
                }
                continue;
            }
            if (!seen_tags)
            {
                // Expect OpusTags in second packet
                if (pkt_len >= 8 && std::memcmp(pkt_ptr, "OpusTags", 8) == 0)
                {
                    seen_tags = true;
                }
                continue;
            }

            // Audio packet (Opus)
            auto packet = std::make_unique<AudioStreamPacket>();
            packet->sample_rate = sample_rate;
            packet->frame_duration = 60;
            packet->payload.resize(pkt_len);
            std::memcpy(packet->payload.data(), pkt_ptr, pkt_len);
            PushPacketToDecodeQueue(std::move(packet), true);
        }

        offset = body_off + body_size;
    }
}

bool AudioService::IsIdle()
{
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    return audio_encode_queue_.empty() && audio_decode_queue_.empty() && audio_playback_queue_.empty() && audio_testing_queue_.empty();
}

void AudioService::ResetDecoder()
{
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    opus_decoder_->ResetState();
    timestamp_queue_.clear();
    audio_decode_queue_.clear();
    audio_playback_queue_.clear();
    audio_testing_queue_.clear();
    audio_queue_cv_.notify_all();
}

void AudioService::CheckAndUpdateAudioPowerState()
{
    auto now = std::chrono::steady_clock::now();
    auto input_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_input_time_).count();
    auto output_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_output_time_).count();
    if (input_elapsed > AUDIO_POWER_TIMEOUT_MS && codec_->input_enabled())
    {
        codec_->EnableInput(false);
    }
    if (output_elapsed > AUDIO_POWER_TIMEOUT_MS && codec_->output_enabled())
    {
        codec_->EnableOutput(false);
    }
    if (!codec_->input_enabled() && !codec_->output_enabled())
    {
        esp_timer_stop(audio_power_timer_);
    }
}

void AudioService::SetDoaMode(bool use_fft)
{
    doa_enabled_ = use_fft;
    ESP_LOGI(TAG, "Set DOA mode: %s", use_fft ? "FFT (GCC-PHAT)" : "Simple (TDOA)");
}

void AudioService::SetModelsList(srmodel_list_t *models_list)
{
    models_list_ = models_list;

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
    if (esp_srmodel_filter(models_list_, ESP_MN_PREFIX, NULL) != nullptr)
    {
        wake_word_ = std::make_unique<CustomWakeWord>();
    }
    else if (esp_srmodel_filter(models_list_, ESP_WN_PREFIX, NULL) != nullptr)
    {
        wake_word_ = std::make_unique<AfeWakeWord>();
    }
    else
    {
        wake_word_ = nullptr;
    }
#else
    if (esp_srmodel_filter(models_list_, ESP_WN_PREFIX, NULL) != nullptr)
    {
        wake_word_ = std::make_unique<EspWakeWord>();
    }
    else
    {
        wake_word_ = nullptr;
    }
#endif

    if (wake_word_)
    {
        wake_word_->OnWakeWordDetected([this](const std::string &wake_word)
                                       {
                                           ESP_LOGI(TAG, "Wake word detected: %s, performing DOA detection", wake_word.c_str());

                                           if (callbacks_.on_wake_word_detected)
                                           {
                                               callbacks_.on_wake_word_detected(wake_word);
                                           }
                                           // 在唤醒词检测到时执行DOA检测
                                           PerformDOADetection(); });
    }
}

bool AudioService::IsAfeWakeWord()
{
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
    return wake_word_ != nullptr && dynamic_cast<AfeWakeWord *>(wake_word_.get()) != nullptr;
#else
    return false;
#endif
}
void AudioService::PerformDOADetection()
{
    static simple_doa_handle_t *simple_doa_handle = nullptr;
    static fft_doa_handle_t *fft_doa_handle = nullptr;
    const int frame_samples = 1024;
    int sample_rate = AUDIO_SERVICE_INPUT_SAMPLE_RATE;
    float mic_distance = 0.045f; // 45毫米

    // 获取实际的编解码器通道数 (2或3，取决于是否启用AEC)
    int total_channels = codec_->input_channels();
    ESP_LOGI(TAG, "Audio codec input channels: %d", total_channels);
    // 初始化DOA处理器（按当前模式只初始化对应实现）
    if (simple_doa_handle == nullptr && fft_doa_handle == nullptr)
    {
        ESP_LOGI(TAG, "Initializing DOA for wake word: sample_rate=%d, frame_samples=%d, mic_distance=%.3f",
                 sample_rate, frame_samples, mic_distance);
        if (doa_enabled_)
        {
            fft_doa_handle = fft_doa_create(sample_rate, 20.0f, mic_distance, frame_samples);
            if (fft_doa_handle)
            {
                ESP_LOGI(TAG, "FFT DOA initialized successfully");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to initialize FFT DOA");
                return;
            }
        }
        else
        {
            simple_doa_handle = simple_doa_create(sample_rate, 20.0f, mic_distance, frame_samples);
            if (simple_doa_handle)
            {
                ESP_LOGI(TAG, "Simple DOA initialized successfully");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to initialize simple DOA");
                simple_doa_handle = nullptr;
                return;
            }
        }
    }

    // 只有真正的双麦克风配置才适合DOA检测
    if (total_channels < 2)
    {
        ESP_LOGW(TAG, "DOA detection requires at least 2 input channels, but only %d available", total_channels);
        return;
    }

    const int samples_per_frame = frame_samples * total_channels; // 每帧需要的样本数

    // 计算可进行的DOA检测次数
    int available_frames = doa_buffer_.size() / samples_per_frame;

    ESP_LOGI(TAG, "=== Wake Word Triggered DOA Detection ===");
    ESP_LOGI(TAG, "DOA Detection: buffer_size=%lu, total_channels=%d, samples_per_frame=%d, available_frames=%d",
             (unsigned long)doa_buffer_.size(), total_channels, samples_per_frame, available_frames);

    if (available_frames > 0)
    {
        // 在进入后台处理前在互斥下准备好所需的数据副本，避免后台线程与音频输入并发访问同一缓冲区导致内存/堆损坏
        std::vector<int16_t> proc_buf;
        int frames_to_process = 0;

        {
            std::lock_guard<std::mutex> lock(audio_queue_mutex_);
            if (doa_detection_running_)
            {
                ESP_LOGI(TAG, "DOA detection already running, skipping new request");
                return;
            }
            // 重新计算可用帧以保证一致性
            frames_to_process = static_cast<int>(doa_buffer_.size() / samples_per_frame);
            if (frames_to_process <= 0)
            {
                ESP_LOGW(TAG, "Not enough data for DOA after recheck: buffer_size=%lu", (unsigned long)doa_buffer_.size());
                return;
            }

            // 我们希望使用最近约 1 秒的数据进行统计：desired_frames = 1s / (frame_samples / sample_rate)
            int desired_frames = static_cast<int>(1.0 * sample_rate / frame_samples + 0.5); // 约 15 帧
            if (desired_frames <= 0)
                desired_frames = frames_to_process;

            // 只处理最多 desired_frames（最近的帧），避免一次处理过多历史数据
            if (frames_to_process > desired_frames)
            {
                frames_to_process = desired_frames;
            }

            // 只拷贝要处理的最新数据片段（尾部），避免拷贝过多早期数据
            size_t copy_elems = static_cast<size_t>(frames_to_process) * samples_per_frame;
            proc_buf.reserve(copy_elems);
            if (doa_buffer_.size() >= copy_elems)
            {
                size_t start_off = doa_buffer_.size() - copy_elems;
                proc_buf.insert(proc_buf.end(), doa_buffer_.begin() + start_off, doa_buffer_.begin() + start_off + copy_elems);
            }
            else
            {
                proc_buf.insert(proc_buf.end(), doa_buffer_.begin(), doa_buffer_.end());
            }

            // 标记正在运行，后续在后台线程结束时清除
            doa_detection_running_ = true;
        }

        // copy static pointers/flags to automatic storage so we can safely capture them in the thread
        simple_doa_handle_t *simple_ptr = simple_doa_handle;
        fft_doa_handle_t *fft_ptr = fft_doa_handle;
        bool use_fft = doa_enabled_;

        // 在单独线程中执行多次DOA计算，线程只访问 proc_buf（不再访问共享 doa_buffer_）
        std::thread([this, frames_to_process, total_channels, simple_ptr, fft_ptr, use_fft, frame_samples, samples_per_frame, proc = std::move(proc_buf)]() mutable
                    {
            ESP_LOGI(TAG, "DOA Thread: total_channels=%d, frames_to_process=%d, proc_buf.size()=%zu",
                     total_channels, frames_to_process, proc.size());
            
            // 存储每帧的角度与权重（能量）
            std::vector<std::pair<float, double>> angle_weights;

            // 对每个可用帧进行DOA检测
            for (int frame_idx = 0; frame_idx < frames_to_process; frame_idx++)
            {
                // 计算当前帧的起始位置
                size_t start_index = static_cast<size_t>(frame_idx) * samples_per_frame;

                // 提取当前帧的左右声道数据
                std::vector<int16_t> left_channel(frame_samples);
                std::vector<int16_t> right_channel(frame_samples);
                
                // Debug: Print first frame's first few samples
                if (frame_idx == 0)
                {
                    ESP_LOGI(TAG, "First frame raw data (interleaved, total_channels=%d): [%d,%d,%d,%d,%d,%d]",
                             total_channels,
                             proc.size() > 0 ? proc[0] : 0,
                             proc.size() > 1 ? proc[1] : 0,
                             proc.size() > 2 ? proc[2] : 0,
                             proc.size() > 3 ? proc[3] : 0,
                             proc.size() > 4 ? proc[4] : 0,
                             proc.size() > 5 ? proc[5] : 0);
                }

                for (int i = 0; i < frame_samples; i++)
                {
                    size_t data_index = start_index + static_cast<size_t>(i) * static_cast<size_t>(total_channels);
                    if (data_index + total_channels - 1 < proc.size())
                    {
                        left_channel[i] = proc[data_index];      // MIC1 (索引0)
                        
                        // MIC3 (索引1) - 硬件增益偏低,需要软件补偿约20倍
                        int32_t right_raw = proc[data_index + 1];
                        int32_t right_amplified = right_raw * 20; // 软件增益补偿
                        
                        // 限幅防止溢出
                        if (right_amplified > 32767) right_amplified = 32767;
                        if (right_amplified < -32768) right_amplified = -32768;
                        
                        right_channel[i] = static_cast<int16_t>(right_amplified);
                    }
                    else
                    {
                        // 如果数据不足，填充0
                        left_channel[i] = 0;
                        right_channel[i] = 0;
                    }
                }
                
                // Debug: Print first frame's extracted channels and first 10 samples of each channel
                if (frame_idx == 0)
                {
                    ESP_LOGI(TAG, "First frame extracted: left[0-2]=[%d,%d,%d], right[0-2]=[%d,%d,%d]",
                             left_channel[0], left_channel[1], left_channel[2],
                             right_channel[0], right_channel[1], right_channel[2]);
                    
                    // 打印前10个样本,检查右声道是否真的有数据
                    ESP_LOGI(TAG, "Left channel [0-9]: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                             left_channel[0], left_channel[1], left_channel[2], left_channel[3], left_channel[4],
                             left_channel[5], left_channel[6], left_channel[7], left_channel[8], left_channel[9]);
                    ESP_LOGI(TAG, "Right channel [0-9]: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                             right_channel[0], right_channel[1], right_channel[2], right_channel[3], right_channel[4],
                             right_channel[5], right_channel[6], right_channel[7], right_channel[8], right_channel[9]);
                }

                // 计算帧能量作为权重（RMS）
                double energy_L = 0.0, energy_R = 0.0;
                for (int k = 0; k < frame_samples; ++k)
                {
                    energy_L += (double)left_channel[k] * (double)left_channel[k];
                    energy_R += (double)right_channel[k] * (double)right_channel[k];
                }
                double rms_L = sqrt(energy_L / frame_samples);
                double rms_R = sqrt(energy_R / frame_samples);
                double weight = sqrt((energy_L + energy_R) / (2.0 * frame_samples)); // RMS across both channels

                // 执行DOA检测（根据选择使用轻量或FFT实现），并尝试获取置信度
                float angle = NAN;
                float conf = 0.0f;
                if (use_fft)
                {
                    if (fft_ptr)
                        angle = fft_doa_process(fft_ptr, left_channel.data(), right_channel.data(), &conf);
                }
                else
                {
                    if (simple_ptr)
                        angle = simple_doa_process(simple_ptr, left_channel.data(), right_channel.data(), &conf);
                }

                // 仅根据置信度/权重来过滤帧（不再根据角度值本身进行过滤）
                double eff_weight = 0.0;
                if (conf > 1e-6f)
                {
                    eff_weight = conf; // 使用 DOA 算法返回的置信度
                }
                else
                {
                    // 回退到基于能量的归一化权重，参考值为 10000 (可调)
                    eff_weight = std::min(1.0, weight / 10000.0);
                }

                // 判定是否保留本帧：置信度必须 > 0.5 且角度不为 NaN
                bool pass_conf = (eff_weight > 0.5);

                if (pass_conf && !std::isnan(angle))
                {
                    angle_weights.emplace_back(angle, eff_weight);
                    ESP_LOGI(TAG, "DOA Frame %d/%d: angle=%.1f°, conf=%.3f, weight=%.3f, RMS_L=%.1f, RMS_R=%.1f",
                             frame_idx + 1, frames_to_process, angle, conf, eff_weight, rms_L, rms_R);
                }
                else
                {
                    const char *filter_reason = std::isnan(angle) ? "NaN angle" : "low confidence (≤0.5)";
                    ESP_LOGI(TAG, "DOA Frame %d/%d: angle=%.1f° (filtered: %s), conf=%.3f, RMS_L=%.1f, RMS_R=%.1f",
                             frame_idx + 1, frames_to_process, angle, filter_reason, conf, rms_L, rms_R);
                }
            }
            // 计算能量加权的圆形平均（如果有有效帧）
            if (!angle_weights.empty())
            {
                double sum_sin = 0.0;
                double sum_cos = 0.0;
                double sum_w = 0.0;
                for (auto &p : angle_weights)
                {
                    double ang_deg = p.first;
                    double w = p.second;
                    double ang_rad = ang_deg * M_PI / 180.0;
                    sum_sin += w * sin(ang_rad);
                    sum_cos += w * cos(ang_rad);
                    sum_w += w;
                }
                float average_angle = 0.0f;
                if (sum_w > 0.0)
                {
                    double mean_rad = atan2(sum_sin / sum_w, sum_cos / sum_w);
                    average_angle = (float)(mean_rad * 180.0 / M_PI);
                    // make result positive in [0,180] if necessary
                    if (average_angle < 0.0f)
                        average_angle += 360.0f;
                }

                // 归一化到15度间隔: 0°, 15°, 30°, 45°, 60°, 75°, 90°, 105°, 120°, 135°, 150°, 165°, 180°
                float quantized_angle = roundf(average_angle / 15.0f) * 15.0f;
                
                // 确保角度在 [0, 180] 范围内
                if (quantized_angle > 180.0f)
                    quantized_angle = 180.0f;
                if (quantized_angle < 0.0f)
                    quantized_angle = 0.0f;

                ESP_LOGE(TAG, "DOA Detection Complete: %d frames processed, average angle = %.1f° → quantized = %.0f° ================",
                         frames_to_process, average_angle, quantized_angle);

                // 这里可以添加回调函数，将归一化后的角度传递给其他模块
                if (callbacks_.on_doa_detected)
                {
                    callbacks_.on_doa_detected(quantized_angle);
                }
            }

            // mark detection finished 在互斥下清除运行标志
            {
                std::lock_guard<std::mutex> lock(this->audio_queue_mutex_);
                this->doa_detection_running_ = false;
            } })
            .detach(); // 分离线程，让它独立运行
    }
    else
    {
        ESP_LOGW(TAG, "Not enough data for DOA detection: buffer_size=%lu, required_per_frame=%d",
                 (unsigned long)doa_buffer_.size(), samples_per_frame);
    }
}

void AudioService::ContinuousDOATask()
{
    const int sample_rate = AUDIO_SERVICE_INPUT_SAMPLE_RATE;
    const int frame_samples = 1024; // samples per channel per read

    // Calculate samples needed for 1 second of audio
    // 1 second * 16000 Hz = 16000 samples per channel
    const int total_samples_needed = sample_rate * 1; // 16000 samples per channel

    // Initialize simple DOA - use 1s worth of samples for each detection
    float mic_distance = 0.045f; // 45 mm
    simple_doa_handle_t *simple = simple_doa_create(sample_rate, 20.0f, mic_distance, total_samples_needed);
    if (!simple)
    {
        ESP_LOGE(TAG, "Failed to create simple DOA handle");
        return;
    }
    ESP_LOGI(TAG, "Continuous DOA initialized (1s accumulation: %d samples)", total_samples_needed);

    // Get codec channels configuration
    const int codec_channels = codec_->input_channels();                           // 2 or 3 depending on AEC
    const int total_samples_with_channels = total_samples_needed * codec_channels; // Total samples including all channels

    std::vector<int16_t> left_ch(total_samples_needed);  // MIC1 (ch0)
    std::vector<int16_t> right_ch(total_samples_needed); // MIC3 (ch2)

    ESP_LOGI(TAG, "Starting continuous DOA processing loop (codec_channels=%d, samples_needed=%d)",
             codec_channels, total_samples_needed);

    while (!service_stopped_ && continuous_doa_enabled_)
    {
        // Wait for enough data in doa_buffer_
        // doa_buffer_ is filled by AudioInputTask when wake word detection is running
        {
            std::lock_guard<std::mutex> lock(audio_queue_mutex_);

            // Check if we have enough data (1 second worth)
            // doa_buffer_ contains interleaved channels data
            if (doa_buffer_.size() < static_cast<size_t>(total_samples_with_channels))
            {
                // Not enough data yet
            }
            else
            {
                // Extract the most recent 1 second of data
                size_t start_idx = doa_buffer_.size() - total_samples_with_channels;

                // De-interleave: extract MIC1 and MIC3
                // doa_buffer_ format depends on codec_channels:
                // 2-channel: [MIC1, MIC3, MIC1, MIC3, ...]
                // 3-channel: [MIC1, MIC3, REF, MIC1, MIC3, REF, ...]
                for (int i = 0; i < total_samples_needed; i++)
                {
                    left_ch[i] = doa_buffer_[start_idx + i * codec_channels + 0];  // MIC1
                    right_ch[i] = doa_buffer_[start_idx + i * codec_channels + 1]; // MIC3
                }

                // Debug: Print first few samples to verify data
                if (start_idx == doa_buffer_.size() - total_samples_with_channels)
                {
                    ESP_LOGI(TAG, "DOA buffer first 10 samples: [%d,%d] [%d,%d] [%d,%d] [%d,%d] [%d,%d]",
                             left_ch[0], right_ch[0], left_ch[1], right_ch[1],
                             left_ch[2], right_ch[2], left_ch[3], right_ch[3],
                             left_ch[4], right_ch[4]);
                }

                // Process DOA outside the lock
            }
        }

        // Check if we have valid data to process
        if (left_ch[0] == 0 && right_ch[0] == 0)
        {
            // No data yet, wait a bit
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Calculate RMS energy for diagnostics
        int64_t sum_L = 0, sum_R = 0;
        for (int i = 0; i < total_samples_needed; i++)
        {
            sum_L += static_cast<int64_t>(left_ch[i]) * left_ch[i];
            sum_R += static_cast<int64_t>(right_ch[i]) * right_ch[i];
        }
        float rms_L = std::sqrt(static_cast<float>(sum_L) / total_samples_needed);
        float rms_R = std::sqrt(static_cast<float>(sum_R) / total_samples_needed);

        // Process DOA on 1 second of accumulated data
        float conf = 0.0f;
        float angle = simple_doa_process(simple, left_ch.data(), right_ch.data(), &conf);

        // Only log if not silence (angle is not NAN)
        if (!std::isnan(angle))
        {
            ESP_LOGI(TAG, "=== DOA Result (1s) === RMS: L=%.1f R=%.1f | angle=%.1f°, conf=%.3f",
                     rms_L, rms_R, angle, conf);

            // Trigger callback if registered
            if (doa_callback_)
            {
                doa_callback_(angle);
            }
            if (callbacks_.on_doa_detected)
            {
                callbacks_.on_doa_detected(angle);
            }
        }

        // Wait 1 second before next detection
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Cleanup
    simple_doa_destroy(simple);
    ESP_LOGI(TAG, "Continuous DOA task stopped");
}
