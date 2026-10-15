// =============================================================================
//  ROBOT CONFIG  -  what the robot IS: pins, gear ratios, servo calibration,
//  slide gains and safety limits.  What the robot DOES is in Sequence.h.
//
//  Every number here must match ROBOT_SPECS.md. Change the spec first.
// =============================================================================
#pragma once
#include <Arduino.h>

// ---- PINS ---------------------------------------------------------------------
const int OUTTAKE_MA = 8;          // motor driver IN1   (confirmed 14 Oct)
const int OUTTAKE_MB = 9;          // motor driver IN2
const int OUTTAKE_PWM = 11;        // motor driver PWM
const int OUTTAKE_ENC_A = 2;       // encoder: must stay on D2/D3 (interrupt pins)
const int OUTTAKE_ENC_B = 3;
const int END_LED_PIN = LED_BUILTIN;   // R25 end-of-run light (wire a bright external LED here)

// PCA9685 outputs, 1-based like the board's silkscreen (code subtracts 1)
const int ARM_R_OUTPUT = 1;
const int ARM_L_OUTPUT = 2;
const int SCOOP_OUTPUT = 3;

// ---- SLIDE DRIVETRAIN  (gear-ratio change = edit SLIDE_STEP_UP only) -----------
const float SLIDE_COUNTS_PER_MOTOR_REV = 384.5f;   // goBILDA 5203 435 rpm, encoder before the step-up
const float SLIDE_STEP_UP = 1.8f;                  // spool turns this many times per motor turn
const float SLIDE_SPOOL_DIA_MM = 36.15f;           // measured
const float SLIDE_STRINGING = 1.0f;                // mm of slide per mm of string (1:1)
const float SLIDE_MOTOR_FREE_RPM = 435.0f;         // at 12 V
const float SLIDE_MM_PER_COUNT =
    3.14159265f * SLIDE_SPOOL_DIA_MM * SLIDE_STRINGING / (SLIDE_COUNTS_PER_MOTOR_REV / SLIDE_STEP_UP);   // 0.5317
const float SLIDE_FREE_SPEED_MM_S =
    SLIDE_MOTOR_FREE_RPM / 60.0f * SLIDE_STEP_UP * 3.14159265f * SLIDE_SPOOL_DIA_MM * SLIDE_STRINGING; // ~1480

const int SLIDE_ENCODER_DIR = -1;  // raw count goes negative on extend (tested 14 Oct)
const int SLIDE_MOTOR_DIR = 1;     // + PWM must EXTEND   (UNCONFIRMED: check with 'j 100 300')

// Slide gains are in ENCODER COUNTS (motor side), so they stay valid when the
// gear ratio changes (only KG / KA need a recheck). Tune with calibration/Slide_PID_Tune.
const float SLIDE_KP = 0.4f;       // PWM per count of error
const float SLIDE_KI = 0.0f;
const float SLIDE_KD = 0.0f;
const float SLIDE_KS = 12.0f;      // PWM to break static friction
const float SLIDE_KV = 0.09f;      // PWM per count/s
const float SLIDE_KA = 0.0f;       // PWM per count/s^2
const float SLIDE_KG = 0.0f;       // constant PWM against gravity on the incline (+ = extend)
const int   SLIDE_MAX_PWM = 250;
const float SLIDE_TOLERANCE_MM = 1.5f;        // PID deadband
const float SLIDE_ARRIVE_TOL_MM = 12.0f;      // "close enough" to finish a step
const float SLIDE_SETTLED_TOL_MM = 80.0f;     // stopped this close also counts (logs SETTLED_SHORT)

// ---- SERVO CALIBRATION  (set by calibration/Servo_Offset_Cal) ------------------
// Reference pose = scoop laid FLAT on the intake floor (a hard stop).
// The CAD says that pose is ARM 110.6 deg, SCOOP -69.4 deg.
const float CAL_ARM_DEG = 110.6f;
const float CAL_SCOOP_DEG = -69.4f;
// Pulse widths that put each servo at the reference pose. Set 15 Oct from Tinzen:
// the V2 SCOOP pose (arm 150 = R 940 / L 2060 us, scoop 1900 us) lies flat on the
// floor. Refine with calibration/Servo_Offset_Cal.
const float ARM_R_CAL_US = 940.0f;
const float ARM_L_CAL_US = 2060.0f;
const float SCOOP_CAL_US = 1900.0f;
// Scale: goBILDA 300 deg servos (2000 us / 300 deg). Arm goes through 40T:48T gears.
const float ARM_US_PER_DEG = 8.0f;
const float SCOOP_US_PER_DEG = 6.667f;
const int ARM_R_DIR = -1;          // +arm degrees = shorter pulse on the right servo
const int ARM_L_DIR = 1;
const int SCOOP_DIR = 1;

// Angle conventions:
//   ARM   degrees forward of vertical (+ = out toward the intake floor)
//   SCOOP degrees relative to the arm, 0 = the CAD pose (blade flat on top, arm vertical)
const float ARM_MIN_DEG = -55.0f;
const float ARM_MAX_DEG = 125.0f;
const float SCOOP_MIN_DEG = -260.0f;
const float SCOOP_MAX_DEG = 20.0f;
const int PULSE_ABS_MIN = 500;     // hard clamp on any pulse (us)
const int PULSE_ABS_MAX = 2500;
const float SERVO_NATIVE_DEG_S = 430.0f;   // unloaded servo speed (estimate) for INSTANT moves
const float ARM_GEAR_RATIO = 48.0f / 40.0f;// arm is slower than its servos by this much

// ---- SAFETY -----------------------------------------------------------------------
const int   STALL_MIN_PWM = 90;            // pushing this hard ...
const float STALL_MAX_VEL = 50.0f;         // ... but slower than this (counts/s) ...
const unsigned long STALL_TIME_MS = 200;   // ... for this long = stall
const unsigned long STALL_IGNORE_START_MS = 300;
const int   DIRCHECK_MIN_PWM = 80;         // wiring check: driving one way, running away the other
const float DIRCHECK_MIN_VEL = 600.0f;     // counts/s
const float DIRCHECK_MIN_ERR = 200.0f;     // counts
const unsigned long DIRCHECK_TIME_MS = 150;
const unsigned long SLIDE_TIMEOUT_MARGIN_MS = 1500;   // step fails after profile time + this
const unsigned long RUN_WATCHDOG_MS = 60000;          // force DONE well before 120 s
