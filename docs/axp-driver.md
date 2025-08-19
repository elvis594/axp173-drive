# AXP系列电源管理芯片驱动架构

## 概述

本项目实现了一个可扩展的AXP系列电源管理芯片驱动架构，通过抽象出通用的I2C操作基类，使得不同的AXP芯片（如AXP173、AXP192等）可以共享相同的底层I2C操作逻辑。

## 架构设计

### 类层次结构

```
I2CDeviceBase<Bus>          // 通用I2C设备基类
├── Axp173<Bus>             // AXP173具体实现
└── Axp192<Bus>             // AXP192具体实现（示例）
```

### 核心组件

#### 1. I2CDeviceBase 基类 (`i2c_device_base.hpp`)

通用I2C设备操作基类，提供以下功能：

- **基础I2C操作**
  - `writeReg()` - 写入单个寄存器
  - `readReg()` - 读取单个寄存器
  - `readRegs()` - 读取多个连续寄存器

- **多字节数据读取**
  - `readU16BE()` - 读取16位大端序数据
  - `readU24BE()` - 读取24位大端序数据
  - `readU32BE()` - 读取32位大端序数据

- **ADC数据读取**
  - `read12Bit()` - 读取12位ADC数据
  - `read13Bit()` - 读取13位ADC数据

- **位操作**
  - `setBit()` - 设置寄存器中的某一位
  - `clearBit()` - 清除寄存器中的某一位
  - `getBit()` - 获取寄存器中某一位的状态
  - `setRegBits()` - 设置寄存器中的多个位

#### 2. AXP173 驱动类 (`axp173.hpp`)

继承自`I2CDeviceBase`，实现AXP173特有的功能：

- **电源管理**
  - 输出控制（DC1、LDO2、LDO3等）
  - 电压设置
  - 充电控制

- **ADC数据读取**
  - 电池电压/电流
  - VBUS电压/电流
  - 内部温度

- **充电管理**
  - 充电电流设置
  - 充电终止电压设置
  - 充电百分比限制

#### 3. AXP192 驱动类 (`axp192.hpp`) - 示例实现

展示如何基于`I2CDeviceBase`实现其他AXP系列芯片的驱动。

## 使用方法

### 1. 使用AXP173

```cpp
#include "axp173.hpp"

// 创建I2C总线和AXP173实例
I2CManager<I2CImplESP32> i2c_bus;
Axp173<I2CManager<I2CImplESP32>> axp173(i2c_bus, 0x34);

// 初始化
if (axp173.init()) {
    Serial.println("AXP173 initialized successfully");
    
    // 启用输出
    axp173.enableOutput(AXP173_DC1_ENABLE);
    
    // 设置电压
    axp173.setVoltage(AXP173_LDO2_VOLTAGE, 3300);
    
    // 读取电池电压
    float voltage;
    if (axp173.getBatteryVoltage(voltage)) {
        Serial.printf("Battery voltage: %.2fV\n", voltage / 1000.0f);
    }
}
```

### 2. 使用AXP192

```cpp
#include "axp192.hpp"

// 创建AXP192实例
Axp192<I2CManager<I2CImplESP32>> axp192(i2c_bus, 0x34);

// 初始化和使用
if (axp192.init()) {
    // 设置DCDC1电压
    axp192.setDCDC1Voltage(3300);
    
    // 设置LDO电压
    axp192.setLDOVoltage(2, 3300); // LDO2设置为3.3V
    
    // 启用充电
    axp192.setChargingEnabled(true);
}
```

### 3. 扩展新的AXP芯片

要支持新的AXP系列芯片，只需：

1. 创建新的头文件（如`axp202.hpp`）
2. 继承`I2CDeviceBase<Bus>`
3. 实现芯片特有的功能

```cpp
template<class Bus>
class Axp202 : public I2CDeviceBase<Bus> {
public:
    Axp202(Bus& bus, uint8_t addr = 0x35) : I2CDeviceBase<Bus>(bus, addr) {}
    
    // 实现AXP202特有的功能
    bool init() {
        // 初始化逻辑
        return true;
    }
    
    // 其他AXP202特有的方法...
};
```

## 设计优势

1. **代码复用**：通用I2C操作逻辑只需实现一次
2. **易于扩展**：新增AXP芯片支持只需实现特有功能
3. **类型安全**：使用模板确保编译时类型检查
4. **性能优化**：模板实例化避免虚函数调用开销
5. **维护性**：清晰的层次结构便于代码维护

## 注意事项

1. **模板依赖查找**：在派生类中调用基类方法需要使用`this->`前缀
2. **I2C总线兼容性**：确保I2C总线类实现了`write()`和`writeRead()`方法
3. **寄存器定义**：每个芯片需要定义自己的寄存器地址常量
4. **ADC精度**：不同芯片的ADC位数可能不同，需要相应调整

## 文件结构

```
src/AXP173/
├── i2c_device_base.hpp     # I2C设备基类
├── axp173.hpp              # AXP173驱动实现
├── axp173_reg.h            # AXP173寄存器定义
├── axp173_cm.h             # AXP173常量和枚举
├── axp192.hpp              # AXP192驱动示例
└── README.md               # 本文档
```

这种架构设计使得AXP系列芯片的驱动开发更加高效和规范化，同时保持了良好的可扩展性和维护性。