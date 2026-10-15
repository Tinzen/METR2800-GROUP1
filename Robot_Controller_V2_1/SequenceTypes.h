// Types and keywords used by Sequence.h. You shouldn't need to edit this file.
#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>

#define HOLD      9999.0f    // axis keeps its current target (a move in progress keeps going)
#define NONE      9999.0f    // no via point
#define INSTANT   (-1)       // MOVE: send the target at once, servo moves at its own speed (V2 style)
#define PROFILED  0          // MOVE: smooth trapezoid at ARM_SPEED / SCOOP_SPEED

enum WaitType : uint8_t { W_MS = 0, W_ARRIVED = 1, W_ARM = 2, W_SLIDE = 3 };
#define WAIT_MS(ms)       W_MS, (ms)        // next step after ms
#define WAIT_ARRIVED(ms)  W_ARRIVED, (ms)   // when every axis this step moved has arrived, plus ms
#define WAIT_ARM(pct)     W_ARM, (pct)      // when the arm is pct % of the way through its move
#define WAIT_SLIDE(pct)   W_SLIDE, (pct)    // when the slides are pct % of the way

struct Step {
  char     name[13];
  float    arm;          // deg or HOLD
  float    scoop;        // deg or HOLD
  float    slide;        // mm from fully retracted, or HOLD
  float    slideSpeed;   // mm/s (0 = SLIDE_MAX)
  int16_t  move;         // INSTANT, PROFILED, or ms for a timed move (arm + scoop finish together)
  float    armVia;       // deg or NONE: timed moves go start -> via -> target (first half / second half)
  float    vibrate;      // +/- deg on the scoop while in this step (0 = off)
  uint8_t  waitType;
  int16_t  waitValue;
};
