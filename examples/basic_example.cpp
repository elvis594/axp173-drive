/**
 * AXP173 基础使用示例
 * 展示如何初始化和使用AXP173电源管理芯片
 */

#include <Arduino.h>
#include "../include/i2c_manager/i2c_manager.hpp"
#include "../include/axp173/axp173.hpp"

// 创建I2C管理器和AXP173实例
I2C i2c;
Axp173<I2C> axp(i2c, 0x34);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== AXP173 基础示例 ===");
    
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
    
    // 启用DC1输出 (3.3V)
    axp.setVoltage(VOLT_DC1, 3300);
    axp.enableOutput(EN_DC1);
    
    // 启用LDO2输出 (3.3V)
    axp.setVoltage(VOLT_LDO2, 3300);
    axp.enableOutput(EN_LDO2);
    
    Serial.println("电源输出已启用");
}

void loop() {
    // 读取电池电压
    float battery_voltage;
    if (axp.getBatteryVoltage(battery_voltage)) {
        Serial.printf("电池电压: %.2f mV\n", battery_voltage);
    }
    
    // 读取电池电流
    float battery_current = axp.getBatCurrent();
    Serial.printf("电池电流: %.2f mA\n", battery_current);
    
    // 读取电池百分比
    float battery_percentage = axp.getBatteryPercentage();
    Serial.printf("电池百分比: %.1f%%\n", battery_percentage);
    
    // 检查充电状态
    if (axp.isCharging()) {
        Serial.println("状态: 充电中");
    } else {
        Serial.println("状态: 未充电");
    }
    
    Serial.println("---");
    delay(2000);
}