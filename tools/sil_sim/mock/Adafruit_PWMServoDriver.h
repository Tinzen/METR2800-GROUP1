#pragma once
#include "Arduino.h"
#include "Wire.h"
void sim_pwm_write(uint8_t ch, uint16_t off);
class Adafruit_PWMServoDriver {
 public:
  bool begin(uint8_t = 0) { return true; }
  void setPWMFreq(float) {}
  uint8_t setPWM(uint8_t ch, uint16_t on, uint16_t off) { sim_pwm_write(ch, off); return 0; }
};
