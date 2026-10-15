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

// ---- ARM ANGLES (deg) -------------------------------------------------------
const float A_START    =  -47.4f;  // power-on pose (V2 940)
const float A_DOWN     =  100.6f;  // V2 200
const float A_PEAK     =   70.6f;  // sweep lifts to here halfway (V2 350)
const float A_SCOOP    =  110.6f;  // sweep ends here: scoop flat on the floor (V2 150)
const float A_TRANSFER =  -43.4f;  // over the bucket (V2 920)
const float A_MOVE     =   40.6f;  // clear of the slides (V2 500)

// ---- SCOOP ANGLES (deg) -----------------------------------------------------
const float S_START    =   -9.4f;  // power-on pose (V2 2300 us)
const float S_DOWN     = -249.4f;  // V2 700 us
const float S_SCOOP    =  -69.4f;  // end of sweep: blade flat on the floor (V2 1900 us)
const float S_PICKUP   =  -39.4f;  // lip up (V2 2100 us)
const float S_CARRY    =  -84.4f;  // during the lift: keeps the rocks in (V2 transfer pose, 1800 us)
const float S_POUR     = -104.4f;  // at the top: tip further to pour (V1 tipped ~17 deg more here)

// ---- SLIDE POSITIONS (mm) ---------------------------------------------------
const float X_CREEP    =  450.0f;  // slow down here, just before the bucket touches the pole
const float X_TIP      =  530.0f;  // bucket fully tipped   (SAFE PLACEHOLDERS: CAD says ~1650 / ~1740)

// ---- SPEEDS -----------------------------------------------------------------
const float ARM_SPEED   =  300.0f; // deg/s     (PROFILED moves)
const float ARM_ACCEL   = 1600.0f; // deg/s^2
const float SCOOP_SPEED =  450.0f; // deg/s
const float SCOOP_ACCEL = 2250.0f; // deg/s^2
const float SLIDE_FAST  = 1000.0f; // mm/s
const float SLIDE_CREEP =  100.0f; // mm/s      bucket tips slowly at this speed
const float SLIDE_MAX   = 1100.0f; // mm/s      (free speed ~1480, keep under ~1170)
const float SLIDE_ACCEL = 3000.0f; // mm/s^2

// ---- VIBRATION / TIMED MOVES ------------------------------------------------
const float VIBRATE_DEG = 7.5f;            // +/- deg (V1 ~ +/-50 us)
const int   VIBRATE_HALF_PERIOD_MS = 50;   // 10 Hz, like V1
const float TIMED_MOVE_EASE = 0.0f;        // 0 = constant speed (V2 sweep), 0.25 = smooth start/stop

// ---- SEQUENCE ---------------------------------------------------------------
// ARM / SCOOP: angle name, a number, or HOLD.   SLIDE: mm, a name, or HOLD.
// MOVE: INSTANT (V2 jump), PROFILED (smooth), or ms (timed: arm + scoop finish together).
// NEXT WHEN: WAIT_MS(ms) | WAIT_ARRIVED(ms) | WAIT_ARM(%) | WAIT_SLIDE(%)
// A move still running when its step ends keeps going (that's how steps overlap).
const Step SEQUENCE[] PROGMEM = {
//  STEP           ARM          SCOOP      SLIDE     SLIDE mm/s   MOVE      ARM VIA  VIBRATE      NEXT WHEN
  { "SCOOP_DOWN",  HOLD,        S_DOWN,    HOLD,     0,           INSTANT,  NONE,    0,           WAIT_MS(500)     },  // V2: scoop first
  { "ARM_DOWN",    A_DOWN,      HOLD,      HOLD,     0,           INSTANT,  NONE,    0,           WAIT_MS(500)     },  // V2
  { "SWEEP",       A_SCOOP,     S_SCOOP,   HOLD,     0,           2000,     A_PEAK,  0,           WAIT_ARRIVED(0)  },  // V2 scoopSweep
  { "PICKUP",      HOLD,        S_PICKUP,  HOLD,     0,           INSTANT,  NONE,    0,           WAIT_MS(1000)    },  // V2 (V1 used 100 ms)
  { "LIFT",        A_TRANSFER,  HOLD,      HOLD,     0,           1000,     NONE,    0,           WAIT_MS(500)     },  // V1: 1 s lift, scoop still at pickup
  { "CARRY",       HOLD,        S_CARRY,   HOLD,     0,           PROFILED, NONE,    0,           WAIT_ARM(90)     },  // V1: carry angle until nearly up
  { "POUR",        HOLD,        S_POUR,    HOLD,     0,           PROFILED, NONE,    0,           WAIT_ARRIVED(0)  },  // V1: tip into the bucket
  { "VIBRATE",     HOLD,        HOLD,      HOLD,     0,           PROFILED, NONE,    VIBRATE_DEG, WAIT_MS(1000)    },  // V1: shake the rest out
  { "EXTEND",      A_MOVE,      HOLD,      X_CREEP,  SLIDE_FAST,  PROFILED, NONE,    0,           WAIT_ARRIVED(0)  },  // arm clears while slides go
  { "CREEP",       HOLD,        HOLD,      X_TIP,    SLIDE_CREEP, PROFILED, NONE,    0,           WAIT_ARRIVED(0)  },  // blends from fast, no stop
  { "TIP_HOLD",    HOLD,        HOLD,      HOLD,     0,           PROFILED, NONE,    0,           WAIT_MS(300)     },  // let the rocks fall
  { "RETRACT",     HOLD,        HOLD,      0,        SLIDE_MAX,   PROFILED, NONE,    0,           WAIT_ARRIVED(0)  },  // back in at max speed
};
