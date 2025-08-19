#pragma once
#ifdef STM32
#include "stm32f1xx_hal.h"  // 根据具体STM32系列调整，如stm32f4xx_hal.h
#include <cstdint>

struct I2CImplSTM32 {
  I2C_HandleTypeDef* hi2c;
  uint32_t timeout;
  
  I2CImplSTM32(I2C_HandleTypeDef* i2c_handle = nullptr, uint32_t timeout_ms = 1000) 
    : hi2c(i2c_handle), timeout(timeout_ms) {}
  
  bool init(int sda, int scl, int hz) {
    // STM32 HAL库中I2C初始化通常在CubeMX生成的代码中完成
    // 这里只做基本的检查和配置验证
    if (hi2c == nullptr) {
      return false;
    }
    
    // 检查I2C是否已经初始化
    if (hi2c->State == HAL_I2C_STATE_RESET) {
      return false;
    }
    
    return true;
  }
  
  bool write(uint8_t addr, const uint8_t* data, size_t len) {
    if (hi2c == nullptr || data == nullptr || len == 0) {
      return false;
    }
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
      hi2c, 
      (uint16_t)(addr << 1),  // 7位地址左移1位
      (uint8_t*)data, 
      (uint16_t)len, 
      timeout
    );
    
    return status == HAL_OK;
  }
  
  bool read(uint8_t addr, uint8_t* data, size_t len) {
    if (hi2c == nullptr || data == nullptr || len == 0) {
      return false;
    }
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
      hi2c, 
      (uint16_t)(addr << 1),  // 7位地址左移1位
      data, 
      (uint16_t)len, 
      timeout
    );
    
    return status == HAL_OK;
  }
  
  bool writeRead(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen) {
    if (hi2c == nullptr) {
      return false;
    }
    
    // 先写后读
    if (wlen > 0 && w != nullptr) {
      HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        hi2c, 
        (uint16_t)(addr << 1), 
        (uint8_t*)w, 
        (uint16_t)wlen, 
        timeout
      );
      
      if (status != HAL_OK) {
        return false;
      }
    }
    
    // 读取数据
    if (rlen > 0 && r != nullptr) {
      HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
        hi2c, 
        (uint16_t)(addr << 1), 
        r, 
        (uint16_t)rlen, 
        timeout
      );
      
      if (status != HAL_OK) {
        return false;
      }
    }
    
    return true;
  }
  
  // 设置I2C句柄
  void setHandle(I2C_HandleTypeDef* handle) {
    hi2c = handle;
  }
  
  // 设置超时时间
  void setTimeout(uint32_t timeout_ms) {
    timeout = timeout_ms;
  }
  
  // 检查I2C设备是否响应
  bool isDeviceReady(uint8_t addr, uint32_t trials = 3) {
    if (hi2c == nullptr) {
      return false;
    }
    
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(
      hi2c, 
      (uint16_t)(addr << 1), 
      trials, 
      timeout
    );
    
    return status == HAL_OK;
  }
};

#endif // STM32