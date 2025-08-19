/**
 * PlatformIO AXP173 使用示例
 * 适用于ESP32开发板
 */

#include <Arduino.h>
#include <axp173/axp173.hpp>
#include <i2c_manager/i2c_manager.hpp>

// 创建I2C管理器和AXP173实例
I2C i2c;
Axp173<I2C> axp(i2c, 0x34);

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== PlatformIO AXP173 示例 ===");
    
    // 初始化I2C (SDA=21, SCL=22, 400kHz)
    if (!i2c.init(21, 22, 400000)) {
        Serial.println("I2C 初始化失败!");
        return;
    }
    
    // 初始化AXP173
    if (!axp.init()) {
        Serial.println("AXP173 初始化失败!");
        return;
    }
    
    Serial.println("AXP173 初始化成功!");
    
    // 配置电源输出
    axp.setVoltage(VOLT_DC1, 3300);   // DC1 = 3.3V
    axp.enableOutput(EN_DC1);
    
    axp.setVoltage(VOLT_LDO2, 3300);  // LDO2 = 3.3V
    axp.enableOutput(EN_LDO2);
    
    Serial.println("电源输出已配置");
}

void loop() {
    // 读取电池信息
    float battery_voltage;
    if (axp.getBatteryVoltage(battery_voltage)) {
        Serial.printf("电池电压: %.2f mV\n", battery_voltage);
    }
    
    float battery_current = axp.getBatCurrent();
    Serial.printf("电池电流: %.2f mA\n", battery_current);
    
    float battery_percentage = axp.getBatteryPercentage();
    Serial.printf("电池百分比: %.1f%%\n", battery_percentage);
    
    // 读取内部温度
    float internal_temp;
    if (axp.getInternalTemperature(internal_temp)) {
        Serial.printf("内部温度: %.1f°C\n", internal_temp);
    }
    
    // 检查充电状态
    Serial.printf("充电状态: %s\n", axp.isCharging() ? "充电中" : "未充电");
    Serial.printf("电池存在: %s\n", axp.isBatExist() ? "是" : "否");
    
    Serial.println("---");
    delay(3000);
}