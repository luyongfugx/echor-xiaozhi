# UART触发表情功能使用指南

## 功能概述

现在您的ESP32设备可以通过UART接收数据来自动播放不同的表情和声音效果。当设备通过UART接收到特定关键词时，会自动显示相应的表情并播放声音。

## 支持的表情触发关键词

### 1. 开心表情 😊
**触发关键词**: `happy`, `smile`, `good`
- **表情**: 微笑 (smile)
- **声音**: 成功音效 (OGG_SUCCESS)
- **示例命令**: 
  ```
  happy
  I'm feeling good today
  This is a smile message
  ```

### 2. 悲伤表情 😢
**触发关键词**: `sad`, `bad`, `error`
- **表情**: 悲伤 (sad)
- **声音**: 感叹音效 (OGG_EXCLAMATION)
- **示例命令**:
  ```
  sad
  Something bad happened
  Error detected
  ```

### 3. 问候表情 😉
**触发关键词**: `hello`, `hi`, `greeting`
- **表情**: 眨眼 (wink)
- **声音**: 弹出音效 (OGG_POPUP)
- **示例命令**:
  ```
  hello
  Hi there!
  Greeting message
  ```

### 4. 思考表情 🤔
**触发关键词**: `question`, `?`
- **表情**: 思考 (thinking)
- **声音**: 无
- **示例命令**:
  ```
  question
  What is this?
  Can you help me?
  ```

### 5. 惊讶表情 😲
**触发关键词**: `surprise`, `wow`
- **表情**: 惊讶 (surprised)
- **声音**: 振动音效 (OGG_VIBRATION)
- **示例命令**:
  ```
  surprise
  Wow, amazing!
  This is surprising
  ```

### 6. 默认表情 😐
**其他任何消息**
- **表情**: 中性 (neutral)
- **声音**: 无
- **示例命令**:
  ```
  test message
  any other text
  ```

## 使用示例

### 通过串口工具发送命令

1. **打开串口工具** (如PuTTY, Arduino Serial Monitor等)
2. **设置参数**:
   - 端口: 对应的UART端口
   - 波特率: 115200
   - 数据位: 8
   - 停止位: 1
   - 校验位: 无

3. **发送测试命令**:
   ```
   hello
   ```
   设备会显示眨眼表情并播放弹出音效

   ```
   How are you today? This is a happy message
   ```
   设备会显示微笑表情并播放成功音效

### 通过代码发送命令

#### Python示例
```python
import serial
import time

# 连接串口
ser = serial.Serial('COM3', 115200, timeout=1)

# 发送不同的消息触发表情
messages = [
    "hello",           # 触发问候表情
    "I'm happy today", # 触发开心表情
    "This is bad",     # 触发悲伤表情
    "What is this?",   # 触发思考表情
    "Wow, surprise!",  # 触发惊讶表情
]

for msg in messages:
    ser.write((msg + '\r\n').encode())
    print(f"Sent: {msg}")
    time.sleep(5)  # 等待3秒表情播放 + 2秒间隔

ser.close()
```

#### Arduino示例
```cpp
void setup() {
  Serial.begin(115200);
}

void loop() {
  // 发送不同的消息
  Serial.println("hello");
  delay(5000);
  
  Serial.println("happy message");
  delay(5000);
  
  Serial.println("This is a question?");
  delay(5000);
}
```

## 技术细节

### 数据处理流程
1. **数据接收**: UART异步接收数据
2. **文本转换**: 将二进制数据转换为字符串
3. **关键词匹配**: 不区分大小写匹配关键词
4. **表情触发**: 通过Schedule机制在主线程中安全更新UI
5. **自动恢复**: 3秒后自动恢复中性表情

### 线程安全
- 所有UI更新都在主事件循环中执行
- 使用Schedule机制确保线程安全
- 表情变化不会影响其他功能

### 自定义扩展

#### 添加新的表情触发
在 `PlayEmotionForUartData()` 方法中添加新的条件分支：

```cpp
else if (lower_data.find("love") != std::string::npos || 
         lower_data.find("heart") != std::string::npos) {
    // 添加爱心表情
    ESP_LOGI(EMOTION_TAG, "Playing love emotion");
    display->SetEmotion("heart");
    audio_service_.PlaySound(Lang::Sounds::OGG_SUCCESS);
}
```

#### 修改表情持续时间
修改延迟时间：
```cpp
// 将3秒改为5秒
vTaskDelay(pdMS_TO_TICKS(5000));
```

## 调试信息

系统会输出以下日志信息：
- `UartReceive`: 接收到UART数据
- `UartEmotion`: 表情触发信息

## 注意事项

1. **关键词匹配**: 关键词匹配不区分大小写
2. **表情优先级**: 第一个匹配的关键词会被触发
3. **声音资源**: 确保音频文件存在且可播放
4. **内存使用**: 大量数据接收时注意内存管理
5. **性能影响**: 表情更新不会影响主要功能性能

这个功能为您的设备增加了丰富的交互体验，可以通过简单的UART命令控制设备的表情显示！
