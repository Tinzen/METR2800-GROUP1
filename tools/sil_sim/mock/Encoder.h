#pragma once
#include "Arduino.h"
long sim_encoder_read();
void sim_encoder_write(long v);
class Encoder {
 public:
  Encoder(uint8_t, uint8_t) {}
  long read() { return sim_encoder_read(); }
  void write(long v) { sim_encoder_write(v); }
};
