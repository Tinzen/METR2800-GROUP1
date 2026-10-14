// Minimal Arduino API mock for native software-in-the-loop builds.
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
typedef uint8_t byte;
typedef bool boolean;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LED_BUILTIN 13
#define DEC 10
template <class A, class B> inline auto min(A a, B b) -> decltype(a + b) { return a < b ? a : b; }
template <class A, class B> inline auto max(A a, B b) -> decltype(a + b) { return a > b ? a : b; }
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int val);
int digitalRead(int pin);
void analogWrite(int pin, int val);
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
class SerialMock {
 public:
  void begin(unsigned long) {}
  int available();
  int read();
  size_t write(const char* s);
  size_t print(const char* s) { return write(s); }
  size_t print(const __FlashStringHelper* s) { return write((const char*)s); }
  size_t print(char c) { char b[2] = {c, 0}; return write(b); }
  size_t print(int v) { char b[24]; snprintf(b, sizeof b, "%d", v); return write(b); }
  size_t print(unsigned int v) { char b[24]; snprintf(b, sizeof b, "%u", v); return write(b); }
  size_t print(long v) { char b[24]; snprintf(b, sizeof b, "%ld", v); return write(b); }
  size_t print(unsigned long v) { char b[24]; snprintf(b, sizeof b, "%lu", v); return write(b); }
  size_t print(unsigned char v) { return print((unsigned int)v); }
  size_t print(double v, int d = 2) { char b[40]; snprintf(b, sizeof b, "%.*f", d, v); return write(b); }
  template <typename T> size_t println(T v) { size_t n = print(v); return n + write("\r\n"); }
  size_t println(double v, int d) { size_t n = print(v, d); return n + write("\r\n"); }
  size_t println() { return write("\r\n"); }
};
extern SerialMock Serial;
