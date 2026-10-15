#pragma once
#include "Arduino.h"
void sim_pwm_write(uint8_t ch, uint16_t off);
class TwoWire {
  uint8_t buf[16]; uint8_t n = 0;
 public:
  void begin() {}
  void setClock(long) {}
  void setWireTimeout(unsigned long = 25000, bool = false) {}
  bool getWireTimeoutFlag() { return false; }
  void clearWireTimeoutFlag() {}
  void beginTransmission(uint8_t) { n = 0; }
  size_t write(uint8_t b) { if (n < 16) buf[n++] = b; return 1; }
  uint8_t endTransmission() {
    // PCA9685: [reg=0x06+4ch, onL, onH, offL, offH]
    if (n == 5 && buf[0] >= 0x06 && buf[0] < 0x06 + 64) sim_pwm_write((buf[0] - 6) / 4, buf[3] | (buf[4] << 8));
    return 0;
  }
};
extern TwoWire Wire;
