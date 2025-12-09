#include "audio_debugger.h"
#include "sdkconfig.h"

#if CONFIG_USE_AUDIO_DEBUGGER
#include <esp_log.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <string>
#endif

#define TAG "AudioDebugger"

AudioDebugger::AudioDebugger()
{
#if CONFIG_USE_AUDIO_DEBUGGER
    udp_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sockfd_ >= 0)
    {
        // 解析配置的服务器地址 "IP:PORT"
        std::string server_addr = CONFIG_AUDIO_DEBUG_UDP_SERVER;
        size_t colon_pos = server_addr.find(':');

        if (colon_pos != std::string::npos)
        {
            std::string ip = server_addr.substr(0, colon_pos);
            int port = std::stoi(server_addr.substr(colon_pos + 1));

            memset(&udp_server_addr_, 0, sizeof(udp_server_addr_));
            udp_server_addr_.sin_family = AF_INET;
            udp_server_addr_.sin_port = htons(port);
            inet_pton(AF_INET, ip.c_str(), &udp_server_addr_.sin_addr);

            ESP_LOGI(TAG, "Initialized server address: %s", CONFIG_AUDIO_DEBUG_UDP_SERVER);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid server address: %s, should be IP:PORT", CONFIG_AUDIO_DEBUG_UDP_SERVER);
            close(udp_sockfd_);
            udp_sockfd_ = -1;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Failed to create UDP socket: %d", errno);
    }

    // start sender thread
    running_.store(true);
    sender_thread_ = std::thread(&AudioDebugger::SenderThread, this);
#endif
}

AudioDebugger::~AudioDebugger()
{
#if CONFIG_USE_AUDIO_DEBUGGER
    // stop sender thread
    running_.store(false);
    queue_cv_.notify_all();
    if (sender_thread_.joinable())
        sender_thread_.join();

    if (udp_sockfd_ >= 0)
    {
        close(udp_sockfd_);
        ESP_LOGI(TAG, "Closed UDP socket");
    }
#endif
}

void AudioDebugger::Feed(const std::vector<int16_t> &data)
{
#if CONFIG_USE_AUDIO_DEBUGGER
    // queue data for background sending to avoid blocking audio task or overwhelming lwIP
    if (udp_sockfd_ >= 0 && running_.load())
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        // keep queue bounded to avoid memory growth
        if (send_queue_.size() > 8)
        {
            send_queue_.pop_front();
        }
        send_queue_.emplace_back(data);
        queue_cv_.notify_one();
    }
#endif
}

void AudioDebugger::SenderThread()
{
#if CONFIG_USE_AUDIO_DEBUGGER
    while (running_.load())
    {
        std::vector<int16_t> item;
        {
            std::unique_lock<std::mutex> lk(queue_mutex_);
            queue_cv_.wait_for(lk, std::chrono::milliseconds(50), [this]
                               { return !send_queue_.empty() || !running_.load(); });
            if (!send_queue_.empty())
            {
                item = std::move(send_queue_.front());
                send_queue_.pop_front();
            }
        }
        if (!item.empty() && udp_sockfd_ >= 0)
        {
            ssize_t sent = sendto(udp_sockfd_, item.data(), item.size() * sizeof(int16_t), 0,
                                  (struct sockaddr *)&udp_server_addr_, sizeof(udp_server_addr_));
            if (sent < 0)
            {
                ESP_LOGD(TAG, "AudioDebugger sendto failed: %d", errno);
            }
        }
    }
#endif
}
