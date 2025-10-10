# UART通信模块

这个模块提供了ESP32的UART通信功能，支持异步数据收发和事件处理。

## 文件结构

- `uart_communication.h` - UART通信类头文件
- `uart_communication.cc` - UART通信类实现
- `uart_example.h` - 使用示例类
- `README.md` - 说明文档

## 主要功能

### UartCommunication类

核心UART通信类，提供以下功能：

1. **异步数据收发** - 支持后台数据接收和发送
2. **事件驱动** - 基于FreeRTOS队列的事件处理
3. **回调机制** - 数据接收回调函数
4. **统计信息** - 收发字节数统计
5. **错误处理** - 完善的错误检测和处理

### 配置选项

```cpp
struct Config {
    uart_port_t port = UART_NUM_1;           // UART端口号
    int baud_rate = 115200;                  // 波特率
    uart_word_length_t data_bits = UART_DATA_8_BITS;  // 数据位
    uart_parity_t parity = UART_PARITY_DISABLE;       // 校验位
    uart_stop_bits_t stop_bits = UART_STOP_BITS_1;    // 停止位
    gpio_num_t tx_pin = GPIO_NUM_17;         // TX引脚
    gpio_num_t rx_pin = GPIO_NUM_18;         // RX引脚
    gpio_num_t rts_pin = UART_PIN_NO_CHANGE; // RTS引脚
    gpio_num_t cts_pin = UART_PIN_NO_CHANGE; // CTS引脚
    int rx_buffer_size = 1024;               // 接收缓冲区大小
    int tx_buffer_size = 1024;               // 发送缓冲区大小
    int event_queue_size = 32;               // 事件队列大小
    int read_timeout_ms = 100;               // 读取超时时间(ms)
};
```

## 使用方法

### 1. 基本使用

```cpp
#include "uart_communication.h"

// 获取UART实例
auto& uart = UartCommunication::GetInstance();

// 配置UART参数
UartCommunication::Config config;
config.port = UART_NUM_1;
config.baud_rate = 115200;
config.tx_pin = GPIO_NUM_17;
config.rx_pin = GPIO_NUM_18;

// 设置数据接收回调
uart.SetDataReceivedCallback([](const std::vector<uint8_t>& data) {
    // 处理接收到的数据
    std::string received_str(data.begin(), data.end());
    ESP_LOGI("UART", "Received: %s", received_str.c_str());
});

// 初始化并启动
if (uart.Initialize(config)) {
    uart.Start();
}

// 发送数据
std::string message = "Hello UART!";
uart.SendString(message);
```

### 2. 在Application中使用

```cpp
#include "uart/uart_example.h"

// 在Application::Start()中初始化
UartExample::GetInstance().Initialize();

// 发送消息
UartExample::GetInstance().SendMessage("Hello from Application!");

// 或者使用Application的接口
Application::GetInstance().SendUartMessage("Hello from Application!");
```

### 3. 通过Application接口使用

```cpp
// 发送UART消息
Application::GetInstance().SendUartMessage("Hello World!");

// 检查UART状态
if (Application::GetInstance().IsUartRunning()) {
    ESP_LOGI("APP", "UART is running");
}
```

## 引脚配置

默认配置使用以下引脚：
- **TX**: GPIO 17
- **RX**: GPIO 18

可以根据实际硬件修改这些引脚配置。

## 事件处理

UART模块会自动处理以下事件：
- `UART_DATA` - 数据接收
- `UART_FIFO_OVF` - FIFO溢出
- `UART_BUFFER_FULL` - 缓冲区满
- `UART_BREAK` - Break信号
- `UART_PARITY_ERR` - 奇偶校验错误
- `UART_FRAME_ERR` - 帧错误

## 调试信息

模块会输出以下日志信息：
- `UartCommunication` - UART通信相关日志
- `UartExample` - 使用示例相关日志

## 注意事项

1. **引脚冲突** - 确保使用的UART引脚不与其他功能冲突
2. **缓冲区大小** - 根据数据量调整缓冲区大小
3. **任务优先级** - UART事件任务优先级为5，可根据需要调整
4. **资源释放** - 在程序退出时调用Stop()释放资源

## 扩展功能

可以根据需要扩展以下功能：
- 自定义协议解析
- 数据分包处理
- 流控支持
- 多UART实例支持
