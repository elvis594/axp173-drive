#pragma once
#include <cstdint>
#include <cstddef>
#include "i2c_device_base.hpp"

// AXP192寄存器定义（示例）
#define AXP192_POWER_STATUS     0x00
#define AXP192_CHARGE_STATUS    0x01
#define AXP192_LDO23_VOLTAGE    0x28
#define AXP192_DCDC1_VOLTAGE    0x26
#define AXP192_DCDC2_VOLTAGE    0x23
#define AXP192_DCDC3_VOLTAGE    0x27
#define AXP192_CHARGE_CONTROL_1 0x33
#define AXP192_CHARGE_CONTROL_2 0x34
#define AXP192_BAT_VOLTAGE      0x78
#define AXP192_BAT_CURRENT      0x7A
#define AXP192_INTERNAL_TEMP    0x5E

/**
 * @brief AXP192电源管理芯片驱动类
 * @tparam Bus I2C总线类型
 */
template<class Bus>
class Axp192 : public I2CDeviceBase<Bus> {
public:
    /**
     * @brief 构造函数
     * @param bus I2C总线引用
     * @param addr 设备I2C地址（AXP192默认为0x34）
     */
    Axp192(Bus& bus, uint8_t addr = 0x34) : I2CDeviceBase<Bus>(bus, addr) {}

    /**
     * @brief 初始化AXP192
     * @return 成功返回true，失败返回false
     */
    bool init() {
        // 检查设备是否存在
        uint8_t status;
        if (!this->readReg(AXP192_POWER_STATUS, status)) {
            return false;
        }
        
        // 执行基本初始化配置
        // 这里可以添加AXP192特有的初始化代码
        
        return true;
    }

    /**
     * @brief 获取电池电压
     * @param voltage_mv 电压值（毫伏）
     * @return 成功返回true，失败返回false
     */
    bool getBatteryVoltage(float& voltage_mv) {
        uint16_t val;
        if (!this->readU16BE(AXP192_BAT_VOLTAGE, val)) return false;
        // AXP192电池电压：1.1mV/bit
        voltage_mv = (val >> 4) * 1.1f;
        return true;
    }

    /**
     * @brief 获取电池电流
     * @param current_ma 电流值（毫安）
     * @return 成功返回true，失败返回false
     */
    bool getBatteryCurrent(float& current_ma) {
        uint16_t val;
        if (!this->readU16BE(AXP192_BAT_CURRENT, val)) return false;
        // AXP192电池电流：0.5mA/bit
        current_ma = (val >> 3) * 0.5f;
        return true;
    }

    /**
     * @brief 获取内部温度
     * @param temp_celsius 温度值（摄氏度）
     * @return 成功返回true，失败返回false
     */
    bool getInternalTemperature(float& temp_celsius) {
        uint16_t val;
        if (!this->readU16BE(AXP192_INTERNAL_TEMP, val)) return false;
        // AXP192内部温度：-144.7 + val * 0.1
        temp_celsius = -144.7f + (val >> 4) * 0.1f;
        return true;
    }

    /**
     * @brief 设置DCDC1电压
     * @param millivolts 电压值（毫伏），范围700-3500mV
     * @return 成功返回true，失败返回false
     */
    bool setDCDC1Voltage(uint16_t millivolts) {
        if (millivolts < 700 || millivolts > 3500) return false;
        uint8_t val = (millivolts - 700) / 25; // 25mV/step
        return this->writeReg(AXP192_DCDC1_VOLTAGE, val);
    }

    /**
     * @brief 设置LDO2/LDO3电压
     * @param ldo_num LDO编号（2或3）
     * @param millivolts 电压值（毫伏），范围1800-3300mV
     * @return 成功返回true，失败返回false
     */
    bool setLDOVoltage(uint8_t ldo_num, uint16_t millivolts) {
        if (ldo_num < 2 || ldo_num > 3) return false;
        if (millivolts < 1800 || millivolts > 3300) return false;
        
        uint8_t val = (millivolts - 1800) / 100; // 100mV/step
        uint8_t reg_val;
        if (!this->readReg(AXP192_LDO23_VOLTAGE, reg_val)) return false;
        
        if (ldo_num == 2) {
            reg_val = (reg_val & 0x0F) | (val << 4); // LDO2在高4位
        } else {
            reg_val = (reg_val & 0xF0) | val; // LDO3在低4位
        }
        
        return this->writeReg(AXP192_LDO23_VOLTAGE, reg_val);
    }

    /**
     * @brief 启用/禁用充电功能
     * @param enable true启用，false禁用
     * @return 成功返回true，失败返回false
     */
    bool setChargingEnabled(bool enable) {
        if (enable) {
            return this->setBit(AXP192_CHARGE_CONTROL_1, 7);
        } else {
            return this->clearBit(AXP192_CHARGE_CONTROL_1, 7);
        }
    }

    /**
     * @brief 检查是否正在充电
     * @return 正在充电返回true，否则返回false
     */
    bool isCharging() {
        uint8_t status;
        if (!this->readReg(AXP192_CHARGE_STATUS, status)) return false;
        return (status & 0x40) != 0; // bit 6表示充电状态
    }

    /**
     * @brief 获取电源状态
     * @param status 状态值
     * @return 成功返回true，失败返回false
     */
    bool getPowerStatus(uint8_t& status) {
        return this->readReg(AXP192_POWER_STATUS, status);
    }
};