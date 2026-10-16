#pragma once
#include <string.h>
#define PROGMEM
#define memcpy_P memcpy
#define pgm_read_byte(p) (*(const unsigned char*)(p))
