#ifndef AUDIO_DEBUGGER_H
#define AUDIO_DEBUGGER_H

#include <vector>
#include <cstdint>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>

#include <sys/socket.h>
#include <netinet/in.h>

class AudioDebugger
{
public:
    AudioDebugger();
    ~AudioDebugger();

    // non-blocking: queue the data for background sending
    void Feed(const std::vector<int16_t> &data);

private:
    void SenderThread();

    int udp_sockfd_ = -1;
    struct sockaddr_in udp_server_addr_;

    std::thread sender_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<std::vector<int16_t>> send_queue_;
    std::atomic<bool> running_{false};
};

#endif