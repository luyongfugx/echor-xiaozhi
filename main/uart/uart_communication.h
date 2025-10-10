#ifndef _UART_COMMUNICATION_H_
#define _UART_COMMUNICATION_H_

#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string>
#include <functional>
#include <vector>
#include <memory>
#include <mutex>

class UartCommunication {
public:
    /**
     * @brief UART配置结构体
     */
    struct Config {
        uart_port_t port = UART_NUM_1;           // UART端口号
        int baud_rate = 115200;                  // 波特率
        uart_word_length_t data_bits = UART_DATA_8_BITS;  // 数据位
        uart_parity_t parity = UART_PARITY_DISABLE;       // 校验位
        uart_stop_bits_t stop_bits = UART_STOP_BITS_1;    // 停止位
        gpio_num_t tx_pin = GPIO_NUM_17;         // TX引脚
        gpio_num_t rx_pin = GPIO_NUM_18;         // RX引脚
        gpio_num_t rts_pin = (gpio_num_t)-1; // RTS引脚
        gpio_num_t cts_pin = (gpio_num_t)-1; // CTS引脚
        int rx_buffer_size = 1024;               // 接收缓冲区大小
        int tx_buffer_size = 1024;               // 发送缓冲区大小
        int event_queue_size = 32;               // 事件队列大小
        int read_timeout_ms = 100;               // 读取超时时间(ms)
    };

    /**
     * @brief 接收数据回调函数类型
     */
    using DataReceivedCallback = std::function<void(const std::vector<uint8_t>& data)>;

    /**
     * @brief 获取单例实例
     */
    static UartCommunication& GetInstance();

    // 删除拷贝构造函数和赋值运算符
    UartCommunication(const UartCommunication&) = delete;
    UartCommunication& operator=(const UartCommunication&) = delete;

    /**
     * @brief 初始化UART通信
     * @param config UART配置
     * @return 是否初始化成功
     */
    bool Initialize(const Config& config);

    /**
     * @brief 启动UART通信
     * @return 是否启动成功
     */
    bool Start();

    /**
     * @brief 停止UART通信
     */
    void Stop();

    /**
     * @brief 发送数据
     * @param data 要发送的数据
     * @param timeout_ms 发送超时时间(ms)
     * @return 实际发送的字节数，-1表示失败
     */
    int SendData(const std::vector<uint8_t>& data, int timeout_ms = 1000);

    /**
     * @brief 发送字符串数据
     * @param data 要发送的字符串
     * @param timeout_ms 发送超时时间(ms)
     * @return 实际发送的字节数，-1表示失败
     */
    int SendString(const std::string& data, int timeout_ms = 1000);

    /**
     * @brief 设置数据接收回调函数
     * @param callback 回调函数
     */
    void SetDataReceivedCallback(DataReceivedCallback callback);

    /**
     * @brief 检查UART是否正在运行
     * @return 是否正在运行
     */
    bool IsRunning() const { return is_running_; }

    /**
     * @brief 获取UART配置
     * @return UART配置
     */
    const Config& GetConfig() const { return config_; }

    /**
     * @brief 获取接收到的数据统计
     * @return 接收字节数
     */
    size_t GetReceivedBytes() const { return received_bytes_; }

    /**
     * @brief 获取发送的数据统计
     * @return 发送字节数
     */
    size_t GetSentBytes() const { return sent_bytes_; }

private:
    UartCommunication();
    ~UartCommunication();

    /**
     * @brief UART事件处理任务
     */
    static void EventTask(void* arg);

    /**
     * @brief 处理UART事件
     */
    void HandleUartEvents();

    Config config_;
    bool is_initialized_ = false;
    bool is_running_ = false;
    QueueHandle_t uart_queue_ = nullptr;
    TaskHandle_t event_task_handle_ = nullptr;
    DataReceivedCallback data_received_callback_;
    
    // 统计信息
    mutable std::mutex stats_mutex_;
    size_t received_bytes_ = 0;
    size_t sent_bytes_ = 0;
};

#endif // _UART_COMMUNICATION_H_
