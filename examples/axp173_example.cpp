#include <Arduino.h>
#include "../include/i2c_manager/i2c_manager.hpp"
#include "../include/i2c_manager/i2c_impl_esp32.h"
#include "../include/axp173/axp173.hpp"

// 函数声明
void basicFunctionTest();
void powerManagementTest();
void batteryMonitorTest();
void chargeControlTest();
void temperatureMonitorTest();
void monitorBatteryStatus();
void advancedFeaturesExample();
void errorHandlingExample();

// AXP173 使用示例

// 创建I2C管理器实例
I2CManager<I2CImplESP32> i2c;

// 创建AXP173实例
Axp173<I2CManager<I2CImplESP32>> axp(i2c, 0x34);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== AXP173 电源管理芯片测试 ===");
    
    // 初始化I2C (SDA=21, SCL=22, 400kHz)
    if (!i2c.init(21, 22, 400000)) {
        Serial.println("I2C 初始化失败!");
        return;
    }
    
    Serial.println("I2C 初始化成功!");
    
    // 初始化AXP173
    if (axp.init()) {
        Serial.println("AXP173 初始化成功!");
    } else {
        Serial.println("AXP173 初始化失败!");
        return;
    }
    
    // 基础功能测试
    basicFunctionTest();
    
    // 电源管理测试
    powerManagementTest();
    
    // 电池监测测试
    batteryMonitorTest();
    
    // 充电控制测试
    chargeControlTest();
    
    // 温度监测测试
    temperatureMonitorTest();
}

void loop() {
    // 持续监测电池状态
    monitorBatteryStatus();
    delay(5000);
}

// 基础功能测试
void basicFunctionTest() {
    Serial.println("\n--- 基础功能测试 ---");
    
    // 读取电源状态
    uint8_t power_status;
    if (axp.getPowerStatus(power_status)) {
        Serial.printf("电源状态寄存器: 0x%02X\n", power_status);
        Serial.printf("VBUS连接状态: %s\n", (power_status & 0x20) ? "已连接" : "未连接");
        Serial.printf("ACIN连接状态: %s\n", (power_status & 0x80) ? "已连接" : "未连接");
        Serial.printf("电池连接状态: %s\n", (power_status & 0x08) ? "已连接" : "未连接");
    }
    
    // 读取充电状态
    uint8_t charge_status;
    if (axp.getChargeStatus(charge_status)) {
        Serial.printf("充电状态寄存器: 0x%02X\n", charge_status);
        Serial.printf("充电状态: %s\n", (charge_status & 0x40) ? "充电中" : "未充电");
    }
}

// 电源管理测试
void powerManagementTest() {
    Serial.println("\n--- 电源管理测试 ---");
    
    // 设置各路电源电压
    Serial.println("设置电源电压:");
    axp.setVoltage(DC1_SET_VOLT, 3300);   // DC1: 3.3V
    axp.setVoltage(DC2_SET_VOLT, 1200);   // DC2: 1.2V
    axp.setVoltage(LDO2_SET_VOLT, 3000);  // LDO2: 3.0V
    axp.setVoltage(LDO3_SET_VOLT, 2800);  // LDO3: 2.8V
    axp.setVoltage(LDO4_SET_VOLT, 1800);  // LDO4: 1.8V
    
    delay(100);
    
    Serial.println("电压设置完成");
    
    // 电源开关控制
    Serial.println("\n电源开关控制:");
    axp.enableOutput(EN_OP_DC1);   // 开启DC1
    axp.enableOutput(EN_OP_LDO4);  // 开启LDO4
    axp.enableOutput(EN_OP_LDO2);  // 开启LDO2
    axp.enableOutput(EN_OP_LDO3);  // 开启LDO3
    axp.enableOutput(EN_OP_DC2);   // 开启DC2
    axp.enableOutput(EN_OP_EXTEN); // 开启EXTEN
    
    delay(100);
    
    // 检查电源开关状态
    bool enabled;
    if (axp.isOutputEnabled(EN_OP_DC1, enabled)) {
        Serial.printf("DC1状态: %s\n", enabled ? "开启" : "关闭");
    }
    Serial.printf("LDO4状态: %s\n", axp.getPowerOutput(1) ? "开启" : "关闭");
    Serial.printf("LDO2状态: %s\n", axp.getPowerOutput(2) ? "开启" : "关闭");
    Serial.printf("LDO3状态: %s\n", axp.getPowerOutput(3) ? "开启" : "关闭");
    Serial.printf("DC2状态: %s\n", axp.getPowerOutput(4) ? "开启" : "关闭");
    Serial.printf("EXTEN状态: %s\n", axp.getPowerOutput(6) ? "开启" : "关闭");
}

// 电池监测测试
void batteryMonitorTest() {
    Serial.println("\n--- 电池监测测试 ---");
    
    // 检查电池连接状态
    uint8_t power_status;
    if (axp.getPowerStatus(power_status)) {
        if (!(power_status & 0x08)) {
            Serial.println("未检测到电池连接");
            return;
        }
    }
    
    // 读取电池参数
    float batVoltage;
    if (axp.getBatteryVoltage(batVoltage)) {
        Serial.printf("电池电压: %.3fV\n", batVoltage / 1000.0f); // 转换为V
    }
    
    float batCurrent;
    if (axp.getBatteryCurrent(batCurrent)) {
        Serial.printf("电池电流: %.1fmA\n", batCurrent);
    }
    
    float vbusVoltage;
    if (axp.getVBUSVoltage(vbusVoltage)) {
        Serial.printf("VBUS电压: %.3fV\n", vbusVoltage / 1000.0f); // 转换为V
    }
    
    float temperature;
    if (axp.getInternalTemperature(temperature)) {
        Serial.printf("内部温度: %.1f°C\n", temperature);
    }
    
    // 计算电池电量百分比（简单估算）
    if (batVoltage > 0) {
        float batVoltageV = batVoltage / 1000.0f;
        float batteryPercent = 0;
        if (batVoltageV > 3.0f) {
            batteryPercent = (batVoltageV - 3.0f) / (4.2f - 3.0f) * 100.0f;
            if (batteryPercent > 100.0f) batteryPercent = 100.0f;
        }
        Serial.printf("电池电量估算: %.1f%%\n", batteryPercent);
    }
}

// 充电控制测试
void chargeControlTest() {
    Serial.println("\n--- 充电控制测试 ---");
    
    // 启用充电
    if (axp.enableOutput(EN_CHARGE)) {
        Serial.println("充电功能已启用");
    } else {
        Serial.println("充电功能启用失败");
    }
    
    // 启用库仑计
    if (axp.enableOutput(EN_COLUMB)) {
        Serial.println("库仑计已启用");
    } else {
        Serial.println("库仑计启用失败");
    }
    
    Serial.println("充电控制设置完成");
}

// 温度监测测试
void temperatureMonitorTest() {
    Serial.println("\n--- 温度监测测试 ---");
    
    float internalTemp;
    if (axp.getInternalTemperature(internalTemp)) {
        Serial.printf("AXP173内部温度: %.1f°C\n", internalTemp);
        
        // 温度报警检查
        if (internalTemp > 80.0f) {
            Serial.println("⚠️ 警告: AXP173温度过高!");
        } else if (internalTemp < -10.0f) {
            Serial.println("⚠️ 警告: AXP173温度过低!");
        } else {
            Serial.println("✓ AXP173温度正常");
        }
    } else {
        Serial.println("温度读取失败");
    }
    
    // 读取TS引脚ADC值（电池温度传感器）
    uint32_t tsADC;
    if (axp.readADCData(DATA_TS_ADC, tsADC)) {
        Serial.printf("TS引脚ADC值: %u (电池温度传感器)\n", tsADC);
    }
}

// 持续监测电池状态
void monitorBatteryStatus() {
    Serial.println("\n=== 电池状态监测 ===");
    
    // 读取电源状态
    uint8_t powerStatus;
    if (axp.getPowerStatus(powerStatus)) {
        bool vbusConnected = (powerStatus & 0x20) != 0;
        bool acConnected = (powerStatus & 0x80) != 0;
        bool batteryConnected = (powerStatus & 0x08) != 0;
        
        Serial.printf("电源输入: VBUS=%s, ACIN=%s\n", 
                      vbusConnected ? "√" : "×",
                      acConnected ? "√" : "×");
        
        // 电池状态
        if (batteryConnected) {
            float voltage, current;
            if (axp.getBatteryVoltage(voltage) && axp.getBatteryCurrent(current)) {
                Serial.printf("电池: %.3fV, %.1fmA\n", voltage, current);
                
                // 电压报警
                if (voltage < 3.2f) {
                    Serial.println("⚠️ 警告: 电池电压过低!");
                } else if (voltage > 4.3f) {
                    Serial.println("⚠️ 警告: 电池电压过高!");
                }
            }
        } else {
            Serial.println("电池: 未连接");
        }
        
        // VBUS电压
        if (vbusConnected) {
            float vbusVoltage;
            if (axp.getVBUSVoltage(vbusVoltage)) {
                Serial.printf("VBUS: %.3fV\n", vbusVoltage);
            }
        }
    }
    
    // 温度
    float temperature;
    if (axp.getInternalTemperature(temperature)) {
        Serial.printf("温度: %.1f°C\n", temperature);
    }
}

// 高级功能示例
void advancedFeaturesExample() {
    Serial.println("\n--- 高级功能示例 ---");
    
    // 设置GPIO功能
    Serial.println("配置GPIO功能:");
    if (axp.setGPIOMode(0, GPIO_OUTPUT_LOW)) {
        Serial.println("GPIO0设置为输出低电平");
    }
    
    // 设置LDO电压
    Serial.println("设置LDO电压:");
    if (axp.setVoltage(LDO2, 3300)) {
        Serial.println("LDO2电压设置为3.3V");
    }
    
    // 读取各种ADC数据
    Serial.println("读取ADC数据:");
    uint32_t adcValue;
    if (axp.readADCData(DATA_VBUS_VOL, adcValue)) {
        Serial.printf("VBUS ADC: %u\n", adcValue);
    }
    if (axp.readADCData(DATA_BAT_VOL, adcValue)) {
        Serial.printf("电池电压 ADC: %u\n", adcValue);
    }
}

// 错误处理示例
void errorHandlingExample() {
    Serial.println("\n--- 错误处理示例 ---");
    
    // 测试寄存器读写
    uint8_t testValue;
    if (!axp.readReg(0x00, testValue)) {
        Serial.println("错误: AXP173通信失败");
        return;
    }
    Serial.printf("AXP173通信正常，芯片ID: 0x%02X\n", testValue);
    
    // 检查电池电压
    float batVoltage;
    if (axp.getBatteryVoltage(batVoltage)) {
        if (batVoltage < 2.5f) {
            Serial.println("错误: 电池电压异常低，可能损坏");
        } else if (batVoltage > 4.5f) {
            Serial.println("错误: 电池电压异常高，存在安全风险");
        } else {
            Serial.printf("电池电压正常: %.3fV\n", batVoltage);
        }
    } else {
        Serial.println("错误: 无法读取电池电压");
    }
    
    // 检查温度
    float internalTemp;
    if (axp.getInternalTemperature(internalTemp)) {
        if (internalTemp > 85.0f) {
            Serial.println("警告: 芯片温度过高");
        } else {
            Serial.printf("芯片温度正常: %.1f°C\n", internalTemp);
        }
    } else {
        Serial.println("错误: 无法读取芯片温度");
    }
}