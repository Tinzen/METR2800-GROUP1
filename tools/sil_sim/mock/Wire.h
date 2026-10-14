#pragma once
#include "Arduino.h"
class TwoWire {
 public:
  void begin() {}
  void setClock(long) {}
  void setWireTimeout(unsigned long = 25000, bool = false) {}
  bool getWireTimeoutFlag() { return false; }
  void clearWireTimeoutFlag() {}
};
extern TwoWire Wire;
