// Types and keywords used by Sequence.h. You shouldn't need to edit this file.
#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>

#define HOLD      9999.0f    // axis keeps its current target (a move in progress keeps going)

enum WaitType : uint8_t { W_MS = 0, W_ARRIVED = 1, W_ARM = 2, W_SLIDE = 3,
                          W_ARM_DONE = 4, W_SCOOP_DONE = 5, W_SLIDE_DONE = 6, W_SCOOP = 7 };
#define WAIT_MS(ms)              W_MS, (ms)           // next step after ms
// "arrived" guards: + ms = wait that long after arriving; - ms = go that long BEFORE arriving (by the plan)
#define WAIT_ARRIVED(ms)         W_ARRIVED, (ms)      // when EVERY axis has stopped (arm, scoop, slides; moves from earlier steps count), plus ms
#define WAIT_ARM_ARRIVED(ms)     W_ARM_DONE, (ms)     // when the arm has arrived, plus ms (its move may have started earlier)
#define WAIT_SCOOP_ARRIVED(ms)   W_SCOOP_DONE, (ms)   // when the scoop has arrived, plus ms
#define WAIT_SLIDE_ARRIVED(ms)   W_SLIDE_DONE, (ms)   // when the slides have arrived, plus ms
#define WAIT_ARM(pct)            W_ARM, (pct)         // when the arm is pct % of the way through its move
#define WAIT_SCOOP(pct)          W_SCOOP, (pct)       // when the scoop is pct % of the way
#define WAIT_SLIDE(pct)          W_SLIDE, (pct)       // when the slides are pct % of the way

// Every move is a trapezoid profile: speeds up at the axis's ACCEL, slows down at its DECEL.
// Servos: the row's speed (0 = ARM_SPEED / SCOOP_SPEED) with ARM_ / SCOOP_ ACCEL and DECEL.
// A new target interrupts a running move smoothly. Slides: a named slide speed with
// SLIDE_ACCEL / SLIDE_DECEL (SLIDE_RETRACT_SPEED uses SLIDE_RETRACT_ACCEL / _DECEL).
// OR AFTER (optional): the step ends after this many ms even if its NEXT WHEN
// hasn't happened (something stuck). 0 = no limit. The robot logs E,STEP_LIMIT,<step>.
// ACC% / DEC% (optional, last four): this row's arm and scoop moves speed up / slow down at
// that % of ARM_ / SCOOP_ ACCEL and DECEL (1-255; 100 or 0 / left out = the axis value).
struct Step {
  char     name[13];
  float    arm;          // deg or HOLD
  float    armSpeed;     // deg/s (0 = ARM_SPEED)
  float    scoop;        // deg or HOLD
  float    scoopSpeed;   // deg/s (0 = SCOOP_SPEED)
  float    slide;        // mm from fully retracted, or HOLD
  float    slideSpeed;   // mm/s, one of the named slide speeds (0 = SLIDE_FAST)
  float    vibrate;      // +/- deg on the scoop while in this step (0 = off)
  uint8_t  waitType;
  int16_t  waitValue;    // ms (arrived guards: negative = before arriving) or %
  uint16_t limitMs;      // OR AFTER: end the step after this long regardless (0 = no limit; missing = 0)
  uint8_t  armAccPct, armDecPct, scoopAccPct, scoopDecPct;   // % of the axis accel / decel (0 = 100)
};
