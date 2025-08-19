# I2C Manager 多平台支持

这个I2C管理器提供了统一的接口，支持多个平台的I2C通信实现。

## 支持的平台

### 1. Arduino 框架 (ESP32)
- **文件**: `i2c_impl_esp32.h`
- **依赖**: Arduino Wire库
- **特点**: 简单易用，适合快速原型开发

### 2. ESP-IDF 框架
- **文件**: `i2c_impl_esp32_idf.h`
- **依赖**: ESP-IDF I2C驱动
- **特点**: 更底层的控制，更好的性能和错误处理

### 3. STM32 HAL 库
- **文件**: `i2c_impl_stm32.h`
- **依赖**: STM32 HAL库
- **特点**: 支持STM32系列微控制器，需要CubeMX配置

## 使用方法

### 基本用法

```cpp
#include "i2c_manager.hpp"

// 使用便捷的类型定义
I2C i2c;

// 初始化I2C
if (!i2c.init(21, 22, 100000)) {  // SDA=21, SCL=22, 100kHz
    // 初始化失败处理
}

// 写入数据
uint8_t data[] = {0x01, 0x02, 0x03};
i2c.write(0x34, data, sizeof(data));

// 读取数据
uint8_t buffer[3];
i2c.read(0x34, buffer, sizeof(buffer));

// 写入寄存器地址然后读取值
uint8_t reg_addr = 0x10;
uint8_t reg_value;
i2c.writeRead(0x34, &reg_addr, 1, &reg_value, 1);
```

### 平台特定配置

#### Arduino (ESP32)
```cpp
// 自动检测Arduino环境，无需额外配置
I2C i2c;
i2c.init(21, 22, 100000);
```

#### ESP-IDF
```cpp
// 自动检测ESP-IDF环境
I2C i2c;  // 默认使用I2C_NUM_0
// 或者指定端口
I2CManager<I2CImplESP32IDF> i2c_custom(I2C_NUM_1);
i2c.init(21, 22, 100000);
```

#### STM32
```cpp
// 需要先在CubeMX中配置I2C，然后传入句柄
extern I2C_HandleTypeDef hi2c1;

I2CImplSTM32 impl(&hi2c1, 1000);  // 1秒超时
I2CManager<I2CImplSTM32> i2c;

// STM32中init参数不使用，但仍需调用
i2c.init(0, 0, 0);

// 检查设备是否在线
if (impl.isDeviceReady(0x34)) {
    // 设备响应正常
}
```

## API 接口

### 核心方法

- `bool init(int sda, int scl, int hz)` - 初始化I2C
- `bool write(uint8_t addr, const uint8_t* data, size_t len)` - 写入数据
- `bool read(uint8_t addr, uint8_t* data, size_t len)` - 读取数据
- `bool writeRead(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen)` - 写入后读取

### STM32 特有方法

- `void setHandle(I2C_HandleTypeDef* handle)` - 设置HAL句柄
- `void setTimeout(uint32_t timeout_ms)` - 设置超时时间
- `bool isDeviceReady(uint8_t addr, uint32_t trials = 3)` - 检查设备是否响应

## 编译配置

编译器会根据预定义宏自动选择合适的实现：

- `ARDUINO` - Arduino框架
- `ESP_PLATFORM` - ESP-IDF框架
- `STM32` - STM32 HAL库

## 示例代码

详细的使用示例请参考 `usage_examples.cpp` 文件，包含：
- 各平台的基本使用方法
- AXP173电源管理芯片的操作示例
- 通用模板函数的使用

## 注意事项

1. **Arduino**: 使用Wire库，自动处理I2C初始化
2. **ESP-IDF**: 提供更详细的错误信息和日志输出
3. **STM32**: 需要在CubeMX中预先配置I2C外设，代码中只处理数据传输
4. **超时处理**: 所有实现都包含超时机制，避免程序卡死
5. **错误处理**: 建议检查所有I2C操作的返回值

## 扩展支持

要添加新平台支持，只需：
1. 创建新的实现文件 `i2c_impl_xxx.h`
2. 实现相同的接口方法
3. 在 `i2c_manager.hpp` 中添加条件编译
4. 添加对应的类型定义