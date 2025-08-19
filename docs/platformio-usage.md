# PlatformIO 使用指南

本文档详细说明如何在PlatformIO环境中使用AXP173驱动库。

## 库配置文件

项目根目录的 `library.json` 文件包含了PlatformIO库的元数据配置：

- **支持的框架**: Arduino, ESP-IDF
- **支持的平台**: ESP32, STM32
- **导出的头文件**: `axp173/axp173.hpp`, `i2c_manager/i2c_manager.hpp`
- **包含目录**: `include`
- **排除文件**: `docs`, `cmake`, `CMakeLists.txt`

## 使用方法

### 1. 本地库使用

将库复制到你的PlatformIO项目的 `lib` 目录：

```bash
cd your_project/lib
git clone https://github.com/your-username/axp173-drive.git
```

### 2. Git依赖

在 `platformio.ini` 中添加Git依赖：

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    https://github.com/your-username/axp173-drive.git
```

### 3. PlatformIO Registry

库发布到Registry后，可以直接使用库名：

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    AXP173-Driver
```

## 代码示例

### 基本使用

```cpp
#include <Arduino.h>
#include <axp173/axp173.hpp>
#include <i2c_manager/i2c_manager.hpp>

I2C i2c;
Axp173<I2C> axp(i2c, 0x34);

void setup() {
    Serial.begin(115200);
    
    // 初始化I2C
    if (!i2c.init(21, 22, 400000)) {
        Serial.println("I2C初始化失败");
        return;
    }
    
    // 初始化AXP173
    if (!axp.init()) {
        Serial.println("AXP173初始化失败");
        return;
    }
    
    // 配置电源输出
    axp.setVoltage(VOLT_DC1, 3300);
    axp.enableOutput(EN_DC1);
}

void loop() {
    float voltage;
    if (axp.getBatteryVoltage(voltage)) {
        Serial.printf("电池电压: %.2f mV\n", voltage);
    }
    delay(1000);
}
```

## 完整示例项目

查看 `examples/platformio_example/` 目录获取完整的PlatformIO示例项目，包含：

- `platformio.ini`: 项目配置文件
- `src/main.cpp`: 完整的使用示例

## 支持的开发板

- ESP32 DevKit
- ESP32-S3 DevKit
- 其他支持I2C的ESP32系列开发板

## 注意事项

1. 确保你的开发板支持I2C通信
2. 正确连接AXP173的SDA和SCL引脚
3. 根据你的硬件配置调整I2C引脚定义
4. 检查AXP173的I2C地址（通常为0x34）

## 故障排除

### 编译错误

- 确保包含了正确的头文件
- 检查PlatformIO平台和框架版本兼容性

### 运行时错误

- 检查I2C连接
- 验证AXP173电源供应
- 使用I2C扫描器确认设备地址

### 性能优化

- 调整I2C时钟频率（100kHz-400kHz）
- 合理设置读取间隔
- 避免频繁的电源状态切换