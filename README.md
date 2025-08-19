# AXP 电源管理芯片驱动库

这是一个用于ESP32系列微控制器的AXP系列电源管理芯片驱动库，支持AXP173、AXP192、AXP2101等芯片，提供电池管理、电压调节、温度监测等功能。

## 项目结构

```
axp173-drive/
├── include/                    # 头文件目录
│   ├── axp173/                # AXP173驱动相关头文件
│   │   ├── axp173.hpp         # AXP173主驱动类
│   │   ├── axp173_reg.h       # 寄存器定义
│   │   ├── axp173_cm.h        # 常量和枚举定义
│   │   ├── axp192.hpp         # AXP192驱动类（扩展支持）
│   │   └── i2c_device_base.hpp # I2C设备基类
│   └── i2c_manager/           # I2C管理器相关头文件
│       ├── i2c_manager.hpp    # I2C管理器主文件
│       ├── i2c_impl_esp32.h   # ESP32 Arduino实现
│       ├── i2c_impl_esp32_idf.h # ESP32 IDF实现
│       └── i2c_impl_stm32.h   # STM32 HAL实现
├── examples/                   # 使用示例
│   ├── basic_example.cpp      # 基础使用示例
│   ├── axp173_example.cpp     # 完整功能示例
│   ├── usage_examples.cpp     # 多平台使用示例
│   ├── platformio_example/    # PlatformIO示例项目
│   │   ├── platformio.ini     # PlatformIO配置
│   │   └── src/main.cpp       # PlatformIO示例代码
│   └── CMakeLists.txt         # 示例构建配置
├── docs/                      # 文档目录
│   ├── axp-driver.md         # AXP驱动文档
│   ├── i2c-manager.md        # I2C管理器文档
│   └── platformio-usage.md   # PlatformIO使用指南
├── cmake/                     # CMake配置文件
│   └── axp173_driver-config.cmake.in
├── library.json               # PlatformIO库配置
├── CMakeLists.txt            # 主CMake配置
└── README.md                 # 项目说明
```

## 功能特性

### 🔋 电池管理
- 电池电压、电流监测
- 电池百分比计算
- 充电状态检测
- 充电电流设置（100mA-1320mA）
- 充电终止电压设置
- 电池温度监测与保护

### ⚡ 电源输出控制
- DC1输出：700-3500mV，25mV步进
- LDO2/LDO3输出：1800-3300mV，100mV步进
- LDO4输出：700-3500mV，25mV步进
- 独立的输出开关控制

### 🌡️ 温度保护
- 电池充电温度保护
- 可配置温度阈值
- 自动充电控制
- 温度报警寄存器支持

### 📊 监测功能
- VBUS电压/电流监测
- 内部温度监测
- 电源状态检测
- 库仑计功能

## 硬件连接

### ESP32-S3 连接示例
```
ESP32-S3    AXP173
--------    ------
GPIO21  --> SDA
GPIO22  --> SCL
3.3V    --> VCC
GND     --> GND
```

## 安装和使用

### 方法1: 直接克隆使用

```bash
git clone https://github.com/your-username/axp173-drive.git
```

在你的项目中包含头文件：

```cpp
#include "axp173-drive/include/axp173/axp173.hpp"
#include "axp173-drive/include/i2c_manager/i2c_manager.hpp"
```

### 方法2: 使用CMake

在你的CMakeLists.txt中添加：

```cmake
# 添加子目录
add_subdirectory(axp173-drive)

# 链接到你的目标
target_link_libraries(your_target axp173_driver)
```

### 方法3: 作为CMake包安装

```bash
cd axp173-drive
mkdir build && cd build
cmake ..
make install
```

然后在你的CMakeLists.txt中：

```cmake
find_package(axp173_driver REQUIRED)
target_link_libraries(your_target axp173_driver::axp173_driver)
```

### 方法4: PlatformIO平台使用

#### 4.1 从本地使用

将库克隆到你的PlatformIO项目的`lib`目录下：

```bash
cd your_platformio_project/lib
git clone https://github.com/your-username/axp173-drive.git
```

#### 4.2 从GitHub直接引用

在你的`platformio.ini`文件中添加：

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    https://github.com/your-username/axp173-drive.git
```

#### 4.3 发布到PlatformIO Registry后

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    AXP173-Driver
```

在你的代码中使用：

```cpp
#include <axp173/axp173.hpp>
#include <i2c_manager/i2c_manager.hpp>

I2C i2c;
Axp173<I2C> axp(i2c, 0x34);
```

## 快速开始

### 平台宏定义说明

库使用预处理器宏来自动选择合适的I2C实现：

```cpp
// 便捷的类型定义
#ifdef ARDUINO
  using I2C = I2CManager<I2CImplESP32>;
#elif defined(ESP_PLATFORM)
  using I2C = I2CManager<I2CImplESP32IDF>;
#elif defined(STM32)
  using I2C = I2CManager<I2CImplSTM32>;
#endif
```

**宏定义自动设置情况**：

- **ARDUINO**: Arduino IDE和PlatformIO的Arduino框架会自动定义
- **ESP_PLATFORM**: ESP-IDF构建系统会自动设置为1
- **STM32**: 需要在STM32项目中手动定义或通过构建系统设置

### 1. 基本初始化

#### 方法1: 使用自动类型定义（推荐）

```cpp
#include "axp173/axp173.hpp"
#include "i2c_manager/i2c_manager.hpp"

// 使用自动选择的I2C类型
I2C i2c;
Axp173<I2C> axp(i2c, 0x34);

void setup() {
    // 初始化I2C
    if (!i2c.init(21, 22, 400000)) {
        // I2C初始化失败处理
        return;
    }
    
    // 初始化AXP173
    if (!axp.init()) {
        // AXP173初始化失败处理
        return;
    }
    
    // 配置电源输出
    axp.setVoltage(VOLT_DC1, 3300);  // 设置DC1为3.3V
    axp.enableOutput(EN_DC1);        // 启用DC1输出
}
```

#### 方法2: 手动指定I2C实现

```cpp
#include "axp173/axp173.hpp"
#include "i2c_manager/i2c_manager.hpp"

// 手动指定I2C实现类型
#ifdef ARDUINO
  I2CManager<I2CImplESP32> i2c;
#elif defined(ESP_PLATFORM)
  I2CManager<I2CImplESP32IDF> i2c;
#elif defined(STM32)
  I2CManager<I2CImplSTM32> i2c;
#endif

Axp173<decltype(i2c)> axp(i2c, 0x34);

// 其余初始化代码相同...
```

#### 方法3: 特定平台初始化

**Arduino/PlatformIO:**
```cpp
#include "axp173/axp173.hpp"
#include "i2c_manager/i2c_manager.hpp"

I2C i2c;  // 自动解析为 I2CManager<I2CImplESP32>
Axp173<I2C> axp(i2c, 0x34);
```

**ESP-IDF:**
```cpp
#include "axp173/axp173.hpp"
#include "i2c_manager/i2c_manager.hpp"

I2C i2c;  // 自动解析为 I2CManager<I2CImplESP32IDF>
Axp173<I2C> axp(i2c, 0x34);
```

**STM32 HAL:**
```cpp
#include "axp173/axp173.hpp"
#include "i2c_manager/i2c_manager.hpp"

// 确保定义了STM32宏
#ifndef STM32
#define STM32
#endif

I2C i2c;  // 自动解析为 I2CManager<I2CImplSTM32>
Axp173<I2C> axp(i2c, 0x34);
```

### 2. 默认配置

`initWithDefaults()` 会自动配置以下默认值：
- DC1: 3.3V (启用)
- LDO2: 3.3V (启用)
- LDO3: 3.0V (启用)
- LDO4: 2.8V (启用)
- 充电电流: 700mA

### 3. 状态监测

```cpp
void loop() {
  // 读取电池信息
  float batteryVoltage;
  if (axp.getBatteryVoltage(batteryVoltage)) {
    float batteryPercentage = axp.getBatteryPercentage();
    Serial.printf("电池: %.3fV (%.1f%%)\n", 
                  batteryVoltage / 1000.0f, batteryPercentage);
  }
  
  // 读取充电状态
  bool isCharging = axp.isCharging();
  float chargeCurrent;
  if (axp.getChargeCurrent(chargeCurrent)) {
    Serial.printf("充电: %s (%.1fmA)\n", 
                  isCharging ? "是" : "否", chargeCurrent);
  }
  
  // 读取温度
  float temperature;
  if (axp.getBatteryTemperature(temperature)) {
    Serial.printf("温度: %.1f°C\n", temperature);
  }
  
  delay(5000);
}
```

## API 参考

### 初始化函数

```cpp
// 基本初始化
bool init();

// 使用默认配置初始化
bool initWithDefaults();
```

### 电池管理

```cpp
// 获取电池电压（毫伏）
bool getBatteryVoltage(float& voltage_mv);

// 获取电池电流（毫安）
bool getBatteryCurrent(float& current_ma);

// 获取电池百分比
float getBatteryPercentage();

// 获取电池温度
bool getBatteryTemperature(float& temp_celsius);

// 充电控制
bool enableCharging();
bool disableCharging();
bool isCharging();
bool setChargeCurrent(charge_current_t current);
```

### 电源输出控制

```cpp
// 设置输出电压
bool setVoltage(voltage_type_t type, uint16_t millivolts);

// 启用/禁用输出
bool enableOutput(output_channel_t channel);
bool disableOutput(output_channel_t channel);
```

### 温度保护

```cpp
// 启用温度保护
bool enableBatteryTemperatureProtection();

// 设置温度阈值
bool setBatteryTemperatureThresholds(float low_temp, float high_temp);

// 检查温度保护状态
bool checkBatteryTemperatureProtection(bool& should_charge);
```

### 监测功能

```cpp
// VBUS监测
bool getVBUSVoltage(float& voltage_mv);
bool getVBUSCurrent(float& current_ma);

// 内部温度
bool getInternalTemperature(float& temp_celsius);

// 电源状态
bool getPowerStatus(uint8_t& status);
bool getChargeStatus(uint8_t& status);
```

## 充电电流设置

支持的充电电流值：
```cpp
enum charge_current_t {
  CHARGE_100mA = 0,   // 100mA
  CHARGE_190mA = 1,   // 190mA
  CHARGE_280mA = 2,   // 280mA
  CHARGE_360mA = 3,   // 360mA
  CHARGE_450mA = 4,   // 450mA
  CHARGE_550mA = 5,   // 550mA
  CHARGE_630mA = 6,   // 630mA
  CHARGE_700mA = 7,   // 700mA (默认)
  CHARGE_780mA = 8,   // 780mA
  CHARGE_880mA = 9,   // 880mA
  CHARGE_960mA = 10,  // 960mA
  CHARGE_1000mA = 11, // 1000mA
  CHARGE_1080mA = 12, // 1080mA
  CHARGE_1160mA = 13, // 1160mA
  CHARGE_1240mA = 14, // 1240mA
  CHARGE_1320mA = 15  // 1320mA
};
```

## 电压输出类型

```cpp
enum voltage_type_t {
  DC2_SET_VOLT = 0x23,  // DC2输出电压设置
  DC1_SET_VOLT = 0x26,  // DC1输出电压设置
  LDO4_SET_VOLT = 0x27, // LDO4输出电压设置
  LDO2_SET_VOLT = 0x28, // LDO2输出电压设置
  LDO3_SET_VOLT = 0x29, // LDO3输出电压设置
  VOFF_SET_VOLT = 0x31, // 关机电压设置
  VHOLD_SET_VOLT = 0x32 // 保持电压设置
};
```

## 输出通道控制

```cpp
enum output_channel_t {
  EN_OP_EXTEN = 0x01,  // EXTEN输出
  EN_OP_DC2 = 0x02,    // DC2输出
  EN_OP_LDO4 = 0x04,   // LDO4输出
  EN_OP_LDO3 = 0x08,   // LDO3输出
  EN_OP_LDO2 = 0x10,   // LDO2输出
  EN_OP_DC1 = 0x20     // DC1输出
};
```

## 温度保护功能

### 启用温度保护
```cpp
// 启用温度保护并设置阈值
axp.enableBatteryTemperatureProtection();
axp.setBatteryTemperatureThresholds(-5.0f, 45.0f); // -5°C到45°C

// 在主循环中检查温度保护
bool should_charge;
if (axp.checkBatteryTemperatureProtection(should_charge)) {
  if (!should_charge) {
    Serial.println("温度超出安全范围，已停止充电");
  }
}
```

## 故障排除

### 常见问题

1. **I2C通信失败**
   - 检查SDA/SCL引脚连接
   - 确认I2C地址正确（AXP173: 0x34）
   - 检查上拉电阻

2. **电池电压读取为0**
   - 确认电池已连接
   - 检查电池连接极性
   - 验证AXP173供电正常

3. **充电不工作**
   - 检查VBUS连接
   - 确认充电功能已启用
   - 检查温度保护设置

4. **温度读取异常**
   - 确认TS引脚连接正确
   - 检查热敏电阻规格（10kΩ NTC）
   - 验证ADC配置

## I2C 多平台架构设计

本项目采用模板化设计，支持多个平台的I2C通信实现，通过统一的接口提供跨平台兼容性。

### 🏗️ 架构概览

```
I2CManager<Impl> (模板类)
    ↓
┌─────────────────┬─────────────────┬─────────────────┐
│  Arduino ESP32  │   ESP-IDF       │    STM32 HAL    │
│  Wire库封装     │   原生驱动      │    HAL库封装    │
└─────────────────┴─────────────────┴─────────────────┘
```

### 📱 支持的平台

#### 1. Arduino 框架 (ESP32)
- **实现文件**: `i2c_impl_esp32.h`
- **依赖库**: Arduino Wire库
- **特点**: 
  - 简单易用，适合快速原型开发
  - 自动处理I2C初始化和时钟配置
  - 内置错误处理和超时机制
- **使用场景**: Arduino IDE开发，快速验证

```cpp
// Arduino平台使用示例
#include "i2c_manager/i2c_manager.hpp"

I2C i2c;  // 自动选择Arduino实现
i2c.init(21, 22, 100000);  // SDA=21, SCL=22, 100kHz
```

#### 2. ESP-IDF 框架
- **实现文件**: `i2c_impl_esp32_idf.h`
- **依赖库**: ESP-IDF I2C驱动
- **特点**:
  - 更底层的控制，更好的性能
  - 详细的错误信息和日志输出
  - 支持多I2C端口配置
  - 可配置的超时和重试机制
- **使用场景**: 生产环境，需要精确控制的应用

```cpp
// ESP-IDF平台使用示例
#include "i2c_manager/i2c_manager.hpp"

// 使用默认端口I2C_NUM_0
I2C i2c;
i2c.init(21, 22, 100000);

// 或指定特定端口
I2CManager<I2CImplESP32IDF> i2c_port1(I2C_NUM_1);
i2c_port1.init(18, 19, 400000);
```

#### 3. STM32 HAL 库
- **实现文件**: `i2c_impl_stm32.h`
- **依赖库**: STM32 HAL库
- **特点**:
  - 支持STM32全系列微控制器
  - 需要CubeMX预配置I2C外设
  - 支持设备就绪检测
  - 可配置超时时间
- **使用场景**: STM32项目，工业级应用

```cpp
// STM32平台使用示例
#include "i2c_manager/i2c_manager.hpp"
extern I2C_HandleTypeDef hi2c1;  // CubeMX生成

I2CImplSTM32 impl(&hi2c1, 1000);  // 1秒超时
I2CManager<I2CImplSTM32> i2c;
i2c.init(0, 0, 0);  // STM32中参数由CubeMX配置

// 检查设备是否在线
if (impl.isDeviceReady(0x34)) {
    // 设备响应正常
}
```

### 🔧 统一接口设计

所有平台实现都遵循相同的接口规范：

```cpp
template<typename Impl>
class I2CManager {
public:
    bool init(int sda, int scl, int hz);                    // 初始化I2C
    bool write(uint8_t addr, const uint8_t* data, size_t len);  // 写入数据
    bool read(uint8_t addr, uint8_t* data, size_t len);         // 读取数据
    bool writeRead(uint8_t addr, const uint8_t* w, size_t wlen, 
                   uint8_t* r, size_t rlen);                    // 写后读
};
```

### 🎯 自动平台检测

编译器根据预定义宏自动选择合适的实现：

```cpp
// 平台检测逻辑
#ifdef ARDUINO
  #include "i2c_impl_esp32.h"     // Arduino框架
  using I2C = I2CManager<I2CImplESP32>;
#elif defined(ESP_PLATFORM)
  #include "i2c_impl_esp32_idf.h" // ESP-IDF框架
  using I2C = I2CManager<I2CImplESP32IDF>;
#elif defined(STM32)
  #include "i2c_impl_stm32.h"     // STM32 HAL库
  using I2C = I2CManager<I2CImplSTM32>;
#endif
```

### 🚀 性能对比

| 平台 | 初始化复杂度 | 运行性能 | 错误处理 | 调试支持 |
|------|-------------|----------|----------|----------|
| Arduino | ⭐⭐⭐ | ⭐⭐ | ⭐⭐ | ⭐⭐⭐ |
| ESP-IDF | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| STM32 | ⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |

### 🔄 扩展新平台

要添加新平台支持，只需：

1. **创建实现文件** `i2c_impl_xxx.h`
2. **实现统一接口**:
   ```cpp
   struct I2CImplXXX {
       bool init(int sda, int scl, int hz);
       bool write(uint8_t addr, const uint8_t* data, size_t len);
       bool read(uint8_t addr, uint8_t* data, size_t len);
       // 可选：bool writeRead(...); 
   };
   ```
3. **添加条件编译** 在 `i2c_manager.hpp` 中
4. **添加类型定义** `using I2C = I2CManager<I2CImplXXX>;`

## 项目结构

```
src/
├── AXP173/
│   ├── axp173.hpp          # 主驱动文件
│   ├── axp173_reg.h        # 寄存器定义
│   ├── axp173_cm.h         # 通用宏定义
│   └── i2c_device_base.hpp # I2C设备基类
├── i2c_manager/
│   ├── i2c_manager.hpp     # I2C管理器模板类
│   ├── i2c_impl_esp32.h    # Arduino ESP32实现
│   ├── i2c_impl_esp32_idf.h # ESP-IDF实现
│   ├── i2c_impl_stm32.h    # STM32 HAL实现
│   └── README.md           # I2C架构详细文档
└── main.cpp                # 示例代码
```

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 更新日志

### v1.0.0
- 初始版本发布
- 支持基本的电池管理功能
- 支持电源输出控制
- 支持温度监测与保护
- 修复电池百分比计算错误
- 添加默认配置初始化功能