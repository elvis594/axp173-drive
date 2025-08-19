#include <cstdint>
#include <cstddef>

// 平台特定的I2C实现
#ifdef ARDUINO
  #include "i2c_impl_esp32.h"     // Arduino框架 (ESP32)
#elif defined(ESP_PLATFORM)
  #include "i2c_impl_esp32_idf.h" // ESP-IDF框架
#elif defined(STM32)
  #include "i2c_impl_stm32.h"     // STM32 HAL库
#endif

template<typename Impl>
class I2CManager {
public:
  bool init(int sda, int scl, int hz)                    { return impl.init(sda, scl, hz); }
  bool write(uint8_t addr, const uint8_t* d, size_t n)   { return impl.write(addr, d, n); }
  bool read (uint8_t addr, uint8_t* d, size_t n)         { return impl.read(addr, d, n); }
  bool writeRead(uint8_t a, const uint8_t* w, size_t wn, uint8_t* r, size_t rn) {
    if constexpr (requires(Impl i){ i.writeRead(a,w,wn,r,rn); }) {
      return impl.writeRead(a,w,wn,r,rn);
    } else {
      // 退化：没有提供 writeRead 就用两步法
      return impl.write(a,w,wn) && impl.read(a,r,rn);
    }
  }
private:
  Impl impl;
};

// 便捷的类型定义
#ifdef ARDUINO
  using I2C = I2CManager<I2CImplESP32>;
#elif defined(ESP_PLATFORM)
  using I2C = I2CManager<I2CImplESP32IDF>;
#elif defined(STM32)
  using I2C = I2CManager<I2CImplSTM32>;
#endif
