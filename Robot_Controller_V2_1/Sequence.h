// =============================================================================
//  DOVER SEQUENCE  -  the robot's state machine. One row = one step.
//
//  Edit here, or open this file in tools/sequence_editor.html (it shows the
//  3D twin and the timing, and saves straight back to this file).
//  Whichever was edited last wins: see AGENTS.md "Sequence sync".
//
//  ANGLES   ARM:   degrees forward of vertical (+ = out toward the intake floor)
//           SCOOP: degrees relative to the arm (0 = CAD pose: blade flat on top, arm vertical)
//  SLIDES   mm from fully retracted.   SPEEDS deg/s, mm/s.   TIMES ms.
// =============================================================================
#pragma once
#include "SequenceTypes.h"
#define PROGRAM_NAME "Max speed"   // the editor's Program menu: which version of the sequence this is

// ---- ARM ANGLES (deg) -------------------------------------------------------
const float A_START    =  -31.6f;  // power-on pose (V2 940)
const float A_DOWN     =  100.3f;  // V2 200
const float A_PEAK     =   62.3f;  // sweep lifts to here halfway (V2 350)
const float A_SCOOP    =  110.3f;  // sweep ends here: scoop flat on the floor (V2 150)
const float A_TRANSFER =  -40.2f;  // over the bucket (V2 920)
const float A_MOVE     =   35.9f;  // clear of the slides (V2 500)
const float A_RESET    =    35.9f;   // Reset / STEP_STOP: the arm waits here while the slides come in (added: = A_MOVE)

// ---- SCOOP ANGLES (deg) -----------------------------------------------------
const float S_START    =    5.4f;  // power-on pose (V2 2300 us)
const float S_DOWN     = -250.0f;  // V2 700 us
const float S_SCOOP    =  -79.6f;  // end of sweep: blade flat on the floor (V2 1900 us)
const float S_PICKUP   =  -33.7f;  // lip up (V2 2100 us)
const float S_CARRY    =   -89.6f;  // during the lift: keeps the rocks in (V2 transfer pose, 1800 us)
const float S_POUR     = -80.2f;  // at the top: tip further to pour (V1 tipped ~17 deg more here)
const float S_RESET    =  -250.0f;   // Reset / STEP_STOP: the scoop waits here while the slides come in (added: = S_DOWN)

// ---- SLIDE POSITIONS (mm) ---------------------------------------------------
const float X_CREEP    =  1665.0f;  // slow down here, just before the bucket touches the pole (was 3000 at the old 0.5317 mm/count scale)
const float X_TIP      =  1780.0f;  // bucket fully tipped (was 3220). PAST the end stop (~1694 mm, 15 Oct logs): CREEP ends in a STALL
const float X_NEW1     =   200.0f;

// ---- SPEEDS -----------------------------------------------------------------
// Servo moves: a step's speed (0 = the default below), speeding up at ACCEL and slowing down
// at DECEL. Above about the servo's own top speed (FAST) the servo can't keep up and
// "arrived" comes early.
const float ARM_SPEED   =  300.0f; // deg/s     default arm speed
const float ARM_FAST    =  358.3f; // deg/s     arm servos' own top speed (430 / 1.2 gearing): the old INSTANT
const float ARM_ACCEL   = 1600.0f; // deg/s^2   every arm move
const float ARM_DECEL   = 1600.0f; // deg/s^2   every arm move, slowing down
const float SCOOP_SPEED =  450.0f; // deg/s     default scoop speed
const float SCOOP_FAST  =  430.0f; // deg/s     scoop servo's own top speed: the old INSTANT
const float SCOOP_ACCEL = 2250.0f; // deg/s^2   every scoop move
const float SCOOP_DECEL = 2250.0f; // deg/s^2   every scoop move, slowing down
// Slide moves pick one of these speeds (not a number), so they stay fixed and tuned.
// ACCEL = speeding up, DECEL = slowing down (the retract speed brings its own pair).
const float SLIDE_FAST  =  800.0f; // mm/s      (was 1000 at the old scale)
const float SLIDE_CREEP =  130.0f; // mm/s      bucket tips slowly at this speed (was 200)
const float SLIDE_ACCEL = 3000.0f; // mm/s^2    (was 5000)
const float SLIDE_DECEL = 3000.0f; // mm/s^2    slowing down
const float SLIDE_RETRACT_SPEED = 2000.0f; // mm/s    RETRACT (was SLIDE_MAX). Free speed ~824 at 12 V, ~1150 on a full 4S: PWM saturates
const float SLIDE_RETRACT_ACCEL = 4000.0f; // mm/s^2  used with SLIDE_RETRACT_SPEED: raise it to save time
const float SLIDE_RETRACT_DECEL = 4000.0f; // mm/s^2  used with SLIDE_RETRACT_SPEED, slowing down

// ---- VIBRATION --------------------------------------------------------------
const float VIBRATE_DEG = 7.5f;            // +/- deg (V1 ~ +/-50 us)
const int   VIBRATE_HALF_PERIOD_MS = 50;   // 10 Hz, like V1

// ---- SEQUENCE ---------------------------------------------------------------
// ARM / SCOOP: angle name, a number, or HOLD, then its speed in deg/s (0 = ARM_SPEED / SCOOP_SPEED).
// SLIDE: mm, a name, or HOLD, then a named slide speed. VIBRATE: +/- deg (0 = off).
// NEXT WHEN: WAIT_MS(ms) | WAIT_ARRIVED(ms) (every axis stopped) | WAIT_ARM_ARRIVED(ms) | WAIT_SCOOP_ARRIVED(ms)
//            | WAIT_SLIDE_ARRIVED(ms) | WAIT_ARM(%) | WAIT_SCOOP(%) | WAIT_SLIDE(%)
//            "arrived" + ms waits after arriving; a negative ms goes that long before arriving.
// OR AFTER: ms after which the step ends anyway (something stuck). 0 = no limit.
// A.ACC% A.DEC% / S.ACC% S.DEC%: this row's arm / scoop move speeds up / slows down at that % of
//            ARM_ / SCOOP_ ACCEL and DECEL (100 = those; 1-255). Only that one move.
// FLAG (testing): STEP_RUN normal | STEP_SKIP this row doesn't run | STEP_STOP the run ends after this row
//            and the robot goes back to the start pose. Leave every row on STEP_RUN for the competition.
// HOLD = that axis keeps doing what it was doing: a move still running when its step ends keeps going.
const Step SEQUENCE[] PROGMEM = {
//  STEP           ARM          deg/s      SCOOP      deg/s       SLIDE     SLIDE mm/s           VIBRATE      NEXT WHEN              OR AFTER A.ACC% A.DEC% S.ACC% S.DEC% FLAG
  { "IDLE",        A_START,     ARM_FAST,  S_START,   SCOOP_FAST, HOLD,     0,                   0,           WAIT_ARRIVED(0),       0,       100,   100,   100,   100,   STEP_RUN  },
  { "SCOOP_DOWN",  HOLD,        0,         S_DOWN,    430,        HOLD,     0,                   0,           WAIT_MS(300),          0,       100,   100,   251,   100,   STEP_RUN  },  // V2: scoop first
  { "ARM_DOWN",    A_DOWN,      358.3,     HOLD,      0,          HOLD,     0,                   0,           WAIT_ARM_ARRIVED(0),   0,       251,   70,    100,   100,   STEP_RUN  },  // V2
  { "SWEEP_UP",    A_PEAK,      185.6,     S_SCOOP,   160.3,      HOLD,     0,                   0,           WAIT_ARM_ARRIVED(0),   0,       100,   100,   100,   100,   STEP_RUN  },  // V2 scoopSweep, first half: arm lifts, scoop turns for 2 s
  { "SWEEP_DOWN",  A_SCOOP,     84.6,      HOLD,      0,          HOLD,     0,                   0,           WAIT_ARRIVED(0),       0,       100,   100,   100,   100,   STEP_RUN  },  // second half: arm drops, the scoop's move carries on
  { "WAIT",        HOLD,        0,         HOLD,      0,          HOLD,     0,                   0,           WAIT_MS(0),            0,       100,   100,   100,   100,   STEP_RUN  },
  { "PICKUP",      HOLD,        0,         S_PICKUP,  SCOOP_FAST, HOLD,     0,                   0,           WAIT_SCOOP(75),        0,       100,   100,   85,    200,   STEP_RUN  },  // V2 (V1 used 100 ms)
  { "LIFT",        A_TRANSFER,  358.3,     HOLD,      0,          HOLD,     0,                   0,           WAIT_ARM(10),          0,       100,   100,   100,   100,   STEP_RUN  },  // V1: 1 s lift, scoop still at pickup
  { "CARRY",       HOLD,        0,         S_CARRY,   95.2,       HOLD,     0,                   0,           WAIT_ARM_ARRIVED(0),   0,       100,   100,   100,   100,   STEP_RUN  },  // V1: carry angle until nearly up
  { "POUR",        HOLD,        0,         S_POUR,    430,        HOLD,     0,                   0,           WAIT_ARRIVED(0),       0,       100,   100,   100,   100,   STEP_RUN  },  // V1: tip into the bucket
  { "VIBRATE",     HOLD,        0,         HOLD,      0,          HOLD,     0,                   VIBRATE_DEG, WAIT_MS(750),          0,       100,   100,   100,   100,   STEP_RUN  },  // V1: shake the rest out
  { "EXTEND",      A_MOVE,      0,         S_DOWN,    429.1,      X_CREEP,  SLIDE_FAST,          0,           WAIT_ARRIVED(0),       0,       100,   100,   100,   100,   STEP_RUN  },  // arm clears while slides go
  { "CREEP",       HOLD,        0,         HOLD,      0,          X_TIP,    SLIDE_CREEP,         0,           WAIT_ARRIVED(0),       4000,    100,   100,   100,   100,   STEP_RUN  },  // blends from fast, no stop
  { "TIP_HOLD",    HOLD,        0,         HOLD,      0,          HOLD,     0,                   0,           WAIT_MS(1500),         0,       100,   100,   100,   100,   STEP_RUN  },  // let the rocks fall
  { "RETRACT",     HOLD,        0,         HOLD,      0,          200,      SLIDE_RETRACT_SPEED, 0,           WAIT_ARRIVED(0),       0,       100,   100,   100,   100,   STEP_RUN  },  // back in, fast
};
