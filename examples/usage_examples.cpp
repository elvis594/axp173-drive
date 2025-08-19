/*
 * I2C Manager 使用示例
 * 支持 Arduino (ESP32)、ESP-IDF、STM32 HAL 三种平台
 */

#include "../include/i2c_manager/i2c_manager.hpp"

// ============================================================================
// Arduino 框架示例 (ESP32)
// ============================================================================
#ifdef ARDUINO

void arduino_example() {
  // 使用便捷类型定义
  I2C i2c;
  
  // 初始化I2C (SDA=21, SCL=22, 100kHz)
  if (!i2c.init(21, 22, 100000)) {
    Serial.println("I2C初始化失败");
    return;
  }
  
  // 写入数据到设备地址0x34
  uint8_t write_data[] = {0x01, 0x02, 0x03};
  if (i2c.write(0x34, write_data, sizeof(write_data))) {
    Serial.println("写入成功");
  }
  
  // 从设备读取数据
  uint8_t read_data[3];
  if (i2c.read(0x34, read_data, sizeof(read_data))) {
    Serial.println("读取成功");
  }
  
  // 写入寄存器地址然后读取
  uint8_t reg_addr = 0x10;
  uint8_t reg_value;
  if (i2c.writeRead(0x34, &reg_addr, 1, &reg_value, 1)) {
    Serial.printf("寄存器0x%02X的值: 0x%02X\n", reg_addr, reg_value);
  }
}

#endif

// ============================================================================
// ESP-IDF 框架示例
// ============================================================================
#ifdef ESP_PLATFORM

#include "esp_log.h"

static const char* TAG = "I2C_EXAMPLE";

void esp_idf_example() {
  // 使用便捷类型定义
  I2C i2c;
  
  // 初始化I2C (SDA=21, SCL=22, 100kHz)
  if (!i2c.init(21, 22, 100000)) {
    ESP_LOGE(TAG, "I2C初始化失败");
    return;
  }
  
  // 写入数据到设备地址0x34
  uint8_t write_data[] = {0x01, 0x02, 0x03};
  if (i2c.write(0x34, write_data, sizeof(write_data))) {
    ESP_LOGI(TAG, "写入成功");
  }
  
  // 从设备读取数据
  uint8_t read_data[3];
  if (i2c.read(0x34, read_data, sizeof(read_data))) {
    ESP_LOGI(TAG, "读取成功");
  }
  
  // 写入寄存器地址然后读取
  uint8_t reg_addr = 0x10;
  uint8_t reg_value;
  if (i2c.writeRead(0x34, &reg_addr, 1, &reg_value, 1)) {
    ESP_LOGI(TAG, "寄存器0x%02X的值: 0x%02X", reg_addr, reg_value);
  }
}

#endif

// ============================================================================
// STM32 HAL 框架示例
// ============================================================================
#ifdef STM32

// 假设已经在main.c中定义了hi2c1
extern I2C_HandleTypeDef hi2c1;

void stm32_example() {
  // 创建I2C管理器并设置HAL句柄
  I2CManager<I2CImplSTM32> i2c;
  
  // 获取底层实现并设置HAL句柄
  // 注意：这里需要访问私有成员，实际使用时可能需要添加公共接口
  // 或者在构造时传入句柄
  
  // 方法1: 使用模板特化的构造函数
  I2CImplSTM32 impl(&hi2c1, 1000);  // 1秒超时
  I2CManager<I2CImplSTM32> i2c_mgr;
  
  // 初始化检查 (STM32中通常在CubeMX中已完成硬件初始化)
  if (!i2c_mgr.init(0, 0, 0)) {  // 参数在STM32中不使用
    // 处理初始化失败
    return;
  }
  
  // 检查设备是否在线
  if (!impl.isDeviceReady(0x34)) {
    // 设备未响应
    return;
  }
  
  // 写入数据到设备地址0x34
  uint8_t write_data[] = {0x01, 0x02, 0x03};
  if (i2c_mgr.write(0x34, write_data, sizeof(write_data))) {
    // 写入成功
  }
  
  // 从设备读取数据
  uint8_t read_data[3];
  if (i2c_mgr.read(0x34, read_data, sizeof(read_data))) {
    // 读取成功
  }
  
  // 写入寄存器地址然后读取
  uint8_t reg_addr = 0x10;
  uint8_t reg_value;
  if (i2c_mgr.writeRead(0x34, &reg_addr, 1, &reg_value, 1)) {
    // 寄存器读取成功
  }
}

#endif

// ============================================================================
// 通用示例 - 使用模板
// ============================================================================

template<typename I2CImpl>
void generic_axp173_example(I2CManager<I2CImpl>& i2c) {
  const uint8_t AXP173_ADDR = 0x34;
  const uint8_t AXP173_CHIP_ID_REG = 0x03;
  
  // 读取芯片ID
  uint8_t reg_addr = AXP173_CHIP_ID_REG;
  uint8_t chip_id;
  
  if (i2c.writeRead(AXP173_ADDR, &reg_addr, 1, &chip_id, 1)) {
    if (chip_id == 0x73) {
      // AXP173 检测成功
#ifdef ARDUINO
      Serial.println("AXP173 检测成功");
#elif defined(ESP_PLATFORM)
      ESP_LOGI("AXP173", "芯片检测成功");
#endif
    }
  }
  
  // 启用电池电压ADC
  uint8_t adc_enable_cmd[] = {0x82, 0xFF};
  i2c.write(AXP173_ADDR, adc_enable_cmd, 2);
  
  // 读取电池电压 (寄存器0x78-0x79)
  uint8_t bat_voltage_reg = 0x78;
  uint8_t voltage_data[2];
  if (i2c.writeRead(AXP173_ADDR, &bat_voltage_reg, 1, voltage_data, 2)) {
    uint16_t voltage_raw = (voltage_data[0] << 4) | (voltage_data[1] & 0x0F);
    float voltage = voltage_raw * 1.1f / 1000.0f;  // 转换为伏特
    
#ifdef ARDUINO
    Serial.printf("电池电压: %.3fV\n", voltage);
#elif defined(ESP_PLATFORM)
    ESP_LOGI("AXP173", "电池电压: %.3fV", voltage);
#endif
  }
}