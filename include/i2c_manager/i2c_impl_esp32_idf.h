#pragma once
#include "driver/i2c.h"
#include "esp_log.h"
#include <cstdint>

static const char* TAG = "I2C_IMPL_IDF";

struct I2CImplESP32IDF {
  i2c_port_t port;
  
  I2CImplESP32IDF(i2c_port_t i2c_port = I2C_NUM_0) : port(i2c_port) {}
  
  bool init(int sda, int scl, int hz) {
    i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = (gpio_num_t)sda,
      .scl_io_num = (gpio_num_t)scl,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = {
        .clk_speed = (uint32_t)hz
      },
      .clk_flags = 0
    };
    
    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
      return false;
    }
    
    err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
      return false;
    }
    
    ESP_LOGI(TAG, "I2C initialized on port %d, SDA=%d, SCL=%d, freq=%dHz", port, sda, scl, hz);
    return true;
  }
  
  bool write(uint8_t addr, const uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "I2C write failed: %s", esp_err_to_name(err));
      return false;
    }
    return true;
  }
  
  bool read(uint8_t addr, uint8_t* data, size_t len) {
    if (len == 0) return true;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
      i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "I2C read failed: %s", esp_err_to_name(err));
      return false;
    }
    return true;
  }
  
  bool writeRead(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen) {
    if (wlen == 0 && rlen == 0) return true;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Write phase
    if (wlen > 0) {
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
      i2c_master_write(cmd, w, wlen, true);
    }
    
    // Read phase
    if (rlen > 0) {
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
      
      if (rlen > 1) {
        i2c_master_read(cmd, r, rlen - 1, I2C_MASTER_ACK);
      }
      i2c_master_read_byte(cmd, r + rlen - 1, I2C_MASTER_NACK);
    }
    
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "I2C write-read failed: %s", esp_err_to_name(err));
      return false;
    }
    return true;
  }
  
  ~I2CImplESP32IDF() {
    i2c_driver_delete(port);
  }
};