# UART通信模块使用指南

## 概述

这个UART通信模块为ESP32设备提供了完整的串口通信功能，包括异步数据收发、事件处理和JSON消息发送。

## 快速开始

### 1. 基本初始化

在 `Application::Start()` 方法中已经自动初始化了UART通信：

```cpp
// 在application.cc的Start()方法中已经包含：
UartExample::GetInstance().Initialize();
```

### 2. 发送消息

使用Application类的接口发送消息：

```cpp
// 发送字符串消息
Application::GetInstance().SendUartMessage("Hello UART!");

// 发送JSON消息（示例）
Application::GetInstance().SendPeriodicUartData();
```

### 3. 检查状态

```cpp
if (Application::GetInstance().IsUartRunning()) {
    ESP_LOGI("APP", "UART通信正常运行");
}
```

## 配置说明

### 默认配置

- **UART端口**: UART_NUM_1
- **波特率**: 115200
- **TX引脚**: GPIO 17
- **RX引脚**: GPIO 18
- **数据位**: 8位
- **停止位**: 1位
- **校验位**: 无

### 修改配置

如果需要修改配置，编辑 `UartExample::Initialize()` 方法：

```cpp
UartCommunication::Config config;
config.port = UART_NUM_2;      // 改为UART2
config.baud_rate = 9600;       // 改为9600波特率
config.tx_pin = GPIO_NUM_4;    // 改为GPIO4
config.rx_pin = GPIO_NUM_5;    // 改为GPIO5
```

## 数据格式

### 周期性数据

模块会自动每20秒发送一次JSON格式的系统数据：

```json
{
  "device_id": "ESP32_01",
  "message_id": 1,
  "angle": 45.5,
  "status": true,
  "timestamp": 1736505600,
  "free_heap": 123456,
  "min_free_heap": 120000,
  "device_state": "idle",
  "uart_received_bytes": 1024,
  "uart_sent_bytes": 512
}
```

### 自定义数据发送

创建自定义JSON消息：

```cpp
void SendCustomUartData() {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "custom");
    cJSON_AddNumberToObject(root, "value", 123);
    cJSON_AddStringToObject(root, "message", "Hello from ESP32");
    
    char *json_str = cJSON_PrintUnformatted(root);
    Application::GetInstance().SendUartMessage(json_str);
    free(json_str);
    cJSON_Delete(root);
}
```

## 接收数据处理

### 设置接收回调

在 `UartExample::Initialize()` 方法中设置数据接收回调：

```cpp
uart.SetDataReceivedCallback([](const std::vector<uint8_t>& data) {
    // 处理接收到的数据
    std::string received(data.begin(), data.end());
    ESP_LOGI("UART", "Received: %s", received.c_str());
    
    // 解析JSON数据示例
    cJSON *root = cJSON_Parse(received.c_str());
    if (root) {
        // 处理JSON数据
        cJSON_Delete(root);
    }
});
```

### 命令解析示例

```cpp
void ParseUartCommand(const std::string& command) {
    if (command == "reboot") {
        Application::GetInstance().Reboot();
    } else if (command == "status") {
        Application::GetInstance().SendPeriodicUartData();
    } else if (command.find("set_angle:") == 0) {
        // 解析设置角度的命令
        float angle = std::stof(command.substr(10));
        // 处理角度设置
    }
}
```

## 调试和监控

### 日志输出

模块会输出以下日志信息：

- `UartCommunication` - UART通信状态和错误
- `UartExample` - 使用示例相关日志
- `UartDataTask` - 数据发送任务日志
- `UartPeriodic` - 周期性数据发送日志

### 统计信息

可以通过以下方法获取统计信息：

```cpp
auto& uart = UartCommunication::GetInstance();
ESP_LOGI("STATS", "Received: %zu, Sent: %zu", 
         uart.GetReceivedBytes(), uart.GetSentBytes());
```

## 故障排除

### 常见问题

1. **UART初始化失败**
   - 检查引脚配置是否正确
   - 确认引脚没有被其他功能占用

2. **数据接收异常**
   - 检查波特率设置是否匹配
   - 确认硬件连接正确

3. **内存泄漏**
   - 确保正确释放cJSON对象
   - 使用 `cJSON_Delete()` 释放JSON对象

### 调试技巧

启用详细日志：

```cpp
// 在uart_communication.cc中修改日志级别
ESP_LOGD(TAG, "Received %d bytes from UART", length);
```

## 扩展功能

### 添加新的数据字段

在 `SendPeriodicUartData()` 方法中添加自定义数据：

```cpp
// 添加传感器数据
cJSON_AddNumberToObject(root, "temperature", read_temperature());
cJSON_AddNumberToObject(root, "humidity", read_humidity());

// 添加网络状态
cJSON_AddNumberToObject(root, "wifi_rssi", get_wifi_strength());
```

### 创建自定义定时任务

```cpp
// 创建新的定时器
esp_timer_handle_t custom_timer;
esp_timer_create_args_t timer_args = {
    .callback = [](void* arg) {
        // 自定义任务
    },
    .arg = this,
    .name = "custom_timer"
};
esp_timer_create(&timer_args, &custom_timer);
esp_timer_start_periodic(custom_timer, 5000000); // 5秒
```

## 性能考虑

1. **内存使用**: JSON序列化会消耗内存，注意及时释放
2. **任务优先级**: UART事件任务优先级为5，可根据需要调整
3. **缓冲区大小**: 根据数据量调整接收和发送缓冲区大小

## 安全注意事项

1. 验证接收数据的格式和长度
2. 限制接收缓冲区大小防止溢出
3. 对关键命令进行身份验证

这个模块为您的ESP32项目提供了完整的UART通信解决方案，可以根据具体需求进行定制和扩展。
