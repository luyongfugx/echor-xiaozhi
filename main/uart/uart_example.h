#ifndef _UART_EXAMPLE_H_
#define _UART_EXAMPLE_H_

#include "uart_communication.h"
#include <esp_log.h>
#include <string>

/**
 * @brief UART使用示例类
 * 
 * 这个类展示了如何在Application中使用UART通信
 */
class UartExample {
public:
    static UartExample& GetInstance() {
        static UartExample instance;
        return instance;
    }

    // 删除拷贝构造函数和赋值运算符
    UartExample(const UartExample&) = delete;
    UartExample& operator=(const UartExample&) = delete;

    /**
     * @brief 初始化UART通信
     */
    void Initialize() {
        auto& uart = UartCommunication::GetInstance();
        
        // 配置UART参数
        UartCommunication::Config config;
        config.port = UART_NUM_1;           // 使用UART1
        config.baud_rate = 115200;          // 波特率115200
        config.tx_pin = (GPIO_NUM_6);        // TX引脚
        config.rx_pin = (GPIO_NUM_5);        // RX引脚
        config.rx_buffer_size = 2048;       // 接收缓冲区大小
        config.tx_buffer_size = 2048;       // 发送缓冲区大小
        
        // 设置数据接收回调
        uart.SetDataReceivedCallback([this](const std::vector<uint8_t>& data) {
            OnUartDataReceived(data);
        });
        
        // 初始化并启动UART
        if (uart.Initialize(config)) {
            if (uart.Start()) {
                ESP_LOGI(TAG, "UART communication initialized successfully");
                
                // 发送欢迎消息
                std::string welcome_msg = "UART Communication Ready!\r\n";
                uart.SendString(welcome_msg);
            } else {
                ESP_LOGE(TAG, "Failed to start UART communication");
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize UART communication");
        }
    }

    /**
     * @brief 发送消息到UART
     * @param message 要发送的消息
     */
    void SendMessage(const std::string& message) {
        auto& uart = UartCommunication::GetInstance();
        if (uart.IsRunning()) {
            std::string formatted_msg = message + "\r\n";
            int bytes_sent = uart.SendString(formatted_msg);
            if (bytes_sent > 0) {
                ESP_LOGI(TAG, "Sent %d bytes to UART: %s", bytes_sent, message.c_str());
            } else {
                ESP_LOGE(TAG, "Failed to send message to UART");
            }
        }
    }

    /**
     * @brief 设置UART数据接收回调
     * @param callback 回调函数
     */
    void SetDataReceivedCallback(std::function<void(const std::string&)> callback) {
        data_received_callback_ = std::move(callback);
    }

    /**
     * @brief 获取UART统计信息
     */
    void PrintStats() {
        auto& uart = UartCommunication::GetInstance();
        if (uart.IsRunning()) {
            ESP_LOGI(TAG, "UART Stats - Received: %zu bytes, Sent: %zu bytes", 
                     uart.GetReceivedBytes(), uart.GetSentBytes());
        }
    }

private:
    UartExample() = default;
    ~UartExample() = default;

    static constexpr const char* TAG = "UartExample";
    std::function<void(const std::string&)> data_received_callback_;

    /**
     * @brief UART数据接收回调函数
     * @param data 接收到的数据
     */
    void OnUartDataReceived(const std::vector<uint8_t>& data) {
        // 将接收到的数据转换为字符串
        std::string received_data(data.begin(), data.end());
        
        // 移除换行符
        if (!received_data.empty() && received_data.back() == '\n') {
            received_data.pop_back();
            if (!received_data.empty() && received_data.back() == '\r') {
                received_data.pop_back();
            }
        }
        
        ESP_LOGI(TAG, "Received from UART: %s", received_data.c_str());
        
        // 调用外部回调函数
        if (data_received_callback_) {
            data_received_callback_(received_data);
        }
        
        // 这里可以添加自定义的数据处理逻辑
        // 例如：解析命令、触发事件等
        
        // 示例：回显接收到的数据
        if (!received_data.empty()) {
            std::string echo_msg = "Echo: " + received_data;
            SendMessage(echo_msg);
        }
    }
};

#endif // _UART_EXAMPLE_H_
