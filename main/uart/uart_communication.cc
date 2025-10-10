#include "uart_communication.h"
#include <esp_log.h>
#include <cstring>

#define TAG "UartCommunication"

UartCommunication& UartCommunication::GetInstance() {
    static UartCommunication instance;
    return instance;
}

UartCommunication::UartCommunication() {
    // 构造函数
}

UartCommunication::~UartCommunication() {
    Stop();
}

bool UartCommunication::Initialize(const Config& config) {
    if (is_initialized_) {
        ESP_LOGW(TAG, "UART already initialized");
        return true;
    }

    config_ = config;

    // 配置UART参数
    // uart_config_t uart_config = {
    //     .baud_rate = config_.baud_rate,
    //     .data_bits = config_.data_bits,
    //     .parity = config_.parity,
    //     .stop_bits = config_.stop_bits,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .rx_flow_ctrl_thresh = 122,
    //     .source_clk = UART_SCLK_DEFAULT,
    // };
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装UART驱动程序
    esp_err_t ret = uart_driver_install(config_.port, 
                                       config_.rx_buffer_size, 
                                       config_.tx_buffer_size,
                                       config_.event_queue_size, 
                                       &uart_queue_, 
                                       0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return false;
    }

    // 配置UART参数
    ret = uart_param_config(config_.port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(config_.port);
        return false;
    }

    // 设置UART引脚
    ret = uart_set_pin(config_.port, config_.tx_pin, config_.rx_pin, 
                      config_.rts_pin, config_.cts_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(config_.port);
        return false;
    }

    // 设置读取超时
    uart_set_rx_timeout(config_.port, config_.read_timeout_ms / portTICK_PERIOD_MS);

    is_initialized_ = true;
    ESP_LOGI(TAG, "UART%d initialized: baud=%d, tx=%d, rx=%d", 
             config_.port, config_.baud_rate, config_.tx_pin, config_.rx_pin);
    
    return true;
}

bool UartCommunication::Start() {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "UART not initialized");
        return false;
    }

    if (is_running_) {
        ESP_LOGW(TAG, "UART already running");
        return true;
    }

    // 创建事件处理任务
    BaseType_t ret = xTaskCreate(EventTask, 
                                "uart_event_task", 
                                4096, 
                                this, 
                                5, 
                                &event_task_handle_);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART event task");
        return false;
    }

    is_running_ = true;
    ESP_LOGI(TAG, "UART communication started");
    return true;
}

void UartCommunication::Stop() {
    if (!is_running_) {
        return;
    }

    is_running_ = false;

    // 停止事件处理任务
    if (event_task_handle_ != nullptr) {
        vTaskDelete(event_task_handle_);
        event_task_handle_ = nullptr;
    }

    // 删除UART驱动程序
    if (is_initialized_) {
        uart_driver_delete(config_.port);
        is_initialized_ = false;
    }

    ESP_LOGI(TAG, "UART communication stopped");
}

int UartCommunication::SendData(const std::vector<uint8_t>& data, int timeout_ms) {
    if (!is_running_ || data.empty()) {
        return -1;
    }

    int bytes_written = uart_write_bytes(config_.port, 
                                        reinterpret_cast<const char*>(data.data()), 
                                        data.size());
    
    if (bytes_written > 0) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        sent_bytes_ += bytes_written;
    }

    return bytes_written;
}

int UartCommunication::SendString(const std::string& data, int timeout_ms) {
    if (!is_running_ || data.empty()) {
        return -1;
    }

    int bytes_written = uart_write_bytes(config_.port, data.c_str(), data.length());
    
    if (bytes_written > 0) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        sent_bytes_ += bytes_written;
    }

    return bytes_written;
}

void UartCommunication::SetDataReceivedCallback(DataReceivedCallback callback) {
    data_received_callback_ = std::move(callback);
}

void UartCommunication::EventTask(void* arg) {
    UartCommunication* uart = static_cast<UartCommunication*>(arg);
    uart->HandleUartEvents();
}

void UartCommunication::HandleUartEvents() {
    uart_event_t event;
    
    while (is_running_) {
        // 等待UART事件
        if (xQueueReceive(uart_queue_, (void*)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    // 数据接收事件
                    uint8_t buffer[256];
                    int length = 0;
                    
                    // 读取所有可用数据
                    while ((length = uart_read_bytes(config_.port, buffer, sizeof(buffer), 0)) > 0) {
                        std::vector<uint8_t> data(buffer, buffer + length);
                        
                        // 更新统计信息
                        {
                            std::lock_guard<std::mutex> lock(stats_mutex_);
                            received_bytes_ += length;
                        }
                        
                        // 调用回调函数
                        if (data_received_callback_) {
                            data_received_callback_(data);
                        }
                        
                        ESP_LOGD(TAG, "Received %d bytes from UART", length);
                    }
                    break;
                }
                
                case UART_FIFO_OVF:
                    ESP_LOGE(TAG, "UART FIFO overflow");
                    uart_flush_input(config_.port);
                    xQueueReset(uart_queue_);
                    break;
                    
                case UART_BUFFER_FULL:
                    ESP_LOGE(TAG, "UART buffer full");
                    uart_flush_input(config_.port);
                    xQueueReset(uart_queue_);
                    break;
                    
                case UART_BREAK:
                    ESP_LOGI(TAG, "UART break detected");
                    break;
                    
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "UART parity error");
                    break;
                    
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "UART frame error");
                    break;
                    
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "UART pattern detected");
                    break;
                    
                default:
                    ESP_LOGD(TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }
    
    vTaskDelete(NULL);
}
