#pragma once
#include <cstdint>
#include <cstddef>

/**
 * @brief 通用I2C设备操作基类
 * @tparam Bus I2C总线类型
 */
template<class Bus>
class I2CDeviceBase {
public:
    /**
     * @brief 构造函数
     * @param bus I2C总线引用
     * @param addr 设备I2C地址
     */
    I2CDeviceBase(Bus& bus, uint8_t addr) : bus_(bus), addr_(addr) {}

    /**
     * @brief 写入单个寄存器
     * @param reg 寄存器地址
     * @param val 要写入的值
     * @return 成功返回true，失败返回false
     */
    bool writeReg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = {reg, val};
        return bus_.write(addr_, buf, 2);
    }

    /**
     * @brief 读取单个寄存器
     * @param reg 寄存器地址
     * @param out 读取的值
     * @return 成功返回true，失败返回false
     */
    bool readReg(uint8_t reg, uint8_t& out) {
        return bus_.writeRead(addr_, &reg, 1, &out, 1);
    }

    /**
     * @brief 读取多个连续寄存器
     * @param reg 起始寄存器地址
     * @param dst 目标缓冲区
     * @param n 要读取的字节数
     * @return 成功返回true，失败返回false
     */
    bool readRegs(uint8_t reg, uint8_t* dst, size_t n) {
        return bus_.writeRead(addr_, &reg, 1, dst, n);
    }

    /**
     * @brief 读取16位大端序数据
     * @param reg 寄存器地址
     * @param val 读取的值
     * @return 成功返回true，失败返回false
     */
    bool readU16BE(uint8_t reg, uint16_t& val) {
        uint8_t buf[2];
        if (!readRegs(reg, buf, 2)) return false;
        val = (buf[0] << 8) | buf[1];
        return true;
    }

    /**
     * @brief 读取24位大端序数据
     * @param reg 寄存器地址
     * @param val 读取的值
     * @return 成功返回true，失败返回false
     */
    bool readU24BE(uint8_t reg, uint32_t& val) {
        uint8_t buf[3];
        if (!readRegs(reg, buf, 3)) return false;
        val = (buf[0] << 16) | (buf[1] << 8) | buf[2];
        return true;
    }

    /**
     * @brief 读取32位大端序数据
     * @param reg 寄存器地址
     * @param val 读取的值
     * @return 成功返回true，失败返回false
     */
    bool readU32BE(uint8_t reg, uint32_t& val) {
        uint8_t buf[4];
        if (!readRegs(reg, buf, 4)) return false;
        val = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
        return true;
    }

    /**
     * @brief 读取12位ADC数据
     * @param reg 寄存器地址
     * @return 12位ADC值
     */
    uint16_t read12Bit(uint8_t reg) {
        uint16_t val;
        if (!readU16BE(reg, val)) return 0;
        return val >> 4; // 12位数据在高12位
    }

    /**
     * @brief 读取13位ADC数据
     * @param reg 寄存器地址
     * @return 13位ADC值
     */
    uint16_t read13Bit(uint8_t reg) {
        uint16_t val;
        if (!readU16BE(reg, val)) return 0;
        return val >> 3; // 13位数据在高13位
    }

    /**
     * @brief 设置寄存器中的某一位
     * @param reg 寄存器地址
     * @param bit 位号(0-7)
     * @return 成功返回true，失败返回false
     */
    bool setBit(uint8_t reg, uint8_t bit) {
        uint8_t val;
        if (!readReg(reg, val)) return false;
        val |= (1 << bit);
        return writeReg(reg, val);
    }

    /**
     * @brief 清除寄存器中的某一位
     * @param reg 寄存器地址
     * @param bit 位号(0-7)
     * @return 成功返回true，失败返回false
     */
    bool clearBit(uint8_t reg, uint8_t bit) {
        uint8_t val;
        if (!readReg(reg, val)) return false;
        val &= ~(1 << bit);
        return writeReg(reg, val);
    }

    /**
     * @brief 获取寄存器中某一位的状态
     * @param reg 寄存器地址
     * @param bit 位号(0-7)
     * @param state 位状态
     * @return 成功返回true，失败返回false
     */
    bool getBit(uint8_t reg, uint8_t bit, bool& state) {
        uint8_t val;
        if (!readReg(reg, val)) return false;
        state = (val & (1 << bit)) != 0;
        return true;
    }

    /**
     * @brief 设置寄存器中的多个位
     * @param reg 寄存器地址
     * @param mask 位掩码
     * @param value 要设置的值
     * @return 成功返回true，失败返回false
     */
    bool setRegBits(uint8_t reg, uint8_t mask, uint8_t value) {
        uint8_t val;
        if (!readReg(reg, val)) return false;
        val = (val & ~mask) | (value & mask);
        return writeReg(reg, val);
    }

protected:
    Bus& bus_;     ///< I2C总线引用
    uint8_t addr_; ///< 设备I2C地址
};