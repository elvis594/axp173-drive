#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

struct I2CImplESP32 {
  bool init(int sda, int scl, int hz) {
    Wire.begin(sda, scl, hz);
    return true;
  }
  bool write(uint8_t addr, const uint8_t* data, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(data, len);
    return Wire.endTransmission() == 0;
  }
  bool read(uint8_t addr, uint8_t* data, size_t len) {
    size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
    if (got != len) return false;
    for (size_t i=0;i<len;i++) data[i] = Wire.read();
    return true;
  }
  bool writeRead(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen) {
    if (!write(addr, w, wlen)) return false;
    return read(addr, r, rlen);
  }
};
