// =============================================================================
// METR2800 GROUP 1 - ROBOT CONTROLLER V2.1   (robot v2: servo intake)
// =============================================================================
// Same state machine as V2:
//   DOWN -> SCOOP -> PICKUP -> TRANSFER -> BEFORE_EXTEND
//        -> OUTTAKE_EXTEND -> OUTTAKE_RETRACT -> DONE
//
// What V2.1 changes:
//   * Arm (2 mirrored servos) and scoop servo follow MotionProfile (ServoAxis)
//   * SCOOP sweep is one profiled path (same shape as v2, smooth start/stop)
//   * Outtake slides are closed-loop: motion profile + PID + feedforward
//   * Scoop vibration is back in TRANSFER (v1 behaviour, helped transfer)
//   * No delay() anywhere -> loop runs ~1 kHz, nothing blocks
//   * CSV telemetry over USB serial -> tools/capture_log.py -> tools/run_viewer.html
//   * End-of-run LED (rule R25), 'x' abort, stall / wiring safety, watchdog
//
// Every state keeps v2's timing as a MINIMUM time, and also waits for its
// motion profiles to finish. Lower the *_MIN_MS values to go faster.
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotorController.h"
#include "ServoAxis.h"

// =============================================================================
// RUN MODE  (change this for testing)
// =============================================================================
//   RUN_FULL         competition: full sequence, starts on power-up
//   RUN_INTAKE_ONLY  DOWN .. BEFORE_EXTEND, then DONE (outtake never moves)
//   RUN_OUTTAKE_ONLY arm to A_MOVE, then extend / dump / retract
//   RUN_CONSOLE      no automatic sequence: serial commands to jog + tune
//                    (type ? in tools/capture_log.py for the command list)
enum RunMode { RUN_FULL = 0, RUN_INTAKE_ONLY = 1, RUN_OUTTAKE_ONLY = 2, RUN_CONSOLE = 3 };
const RunMode RUN_MODE = RUN_FULL;

// Bench testing over USB: the Nano restarts (and would start the run) every
// time you upload, plug in USB, or open the serial port. true = after boot it
// holds the start pose and waits for 'g' + Enter (typed into
// tools/capture_log.py). The end LED blinks while it waits.
// MUST be false at competition: the power switch is the start action.
const bool WAIT_FOR_GO = false;

// =============================================================================
// FSM  (identical states to V2)
// =============================================================================
enum RobotState {
  IDLE,

  DOWN,
  SCOOP,
  PICKUP,
  TRANSFER,
  BEFORE_EXTEND,
  OUTTAKE_EXTEND,
  OUTTAKE_RETRACT,

  DONE,
  NUM_STATES
};

// =============================================================================
// SERIAL / TELEMETRY
// =============================================================================
const unsigned long SERIAL_BAUD = 115200;
const bool TELEMETRY_ENABLED = true;        // harmless with no USB attached
const unsigned long LOG_INTERVAL_MS = 20;   // 50 Hz telemetry rows
const char FW_NAME[] = "Robot_Controller_V2_1";
const int LOG_SCHEMA = 1;

// =============================================================================
// PINS
// =============================================================================
// Outtake motor driver (unchanged from V2). Encoder MUST stay on D2/D3
// (the Nano's only hardware interrupt pins).
const int OUTTAKE_MA = 6;
const int OUTTAKE_MB = 7;
const int OUTTAKE_PWM = 11;
const int OUTTAKE_ENC_A = 2;
const int OUTTAKE_ENC_B = 3;

// R25: an LED must show when the program has ended. LED_BUILTIN (D13) works,
// but the camera needs to see it -> wire a bright external LED to this pin.
const int END_LED_PIN = LED_BUILTIN;

// =============================================================================
// OUTTAKE (goBILDA 5203 13.7:1, 435 RPM, 384.5 counts/gearbox rev, encoder
//          on the motor before the 1.8:1 step-up -> ~2790 counts/s free speed,
//          213.6 counts per spool rev. See AGENTS.md.)
// =============================================================================
const int OUTTAKE_ENCODER_DIRECTION = 1;    // + PWM must make ticks go UP
const int OUTTAKE_MOTOR_DIRECTION = 1;      // + PWM must EXTEND the slides

const float OUTTAKE_START_TICKS = 0.0f;     // slides fully retracted at boot

// !!! CALIBRATE: ticks at the point where the bucket has fully tipped. !!!
// Use RUN_CONSOLE: 'j 120 300' jogs out, watch oP in the log, 'z' re-zeros.
// Deliberately short default so an uncalibrated run can't slam the end stop.
const float OUTTAKE_EXTEND_TICKS = 1000.0f;
const float OUTTAKE_RETRACT_TICKS = 0.0f;
const float OUTTAKE_SOFT_MAX_TICKS = 12000.0f;  // console moves are clamped
const float OUTTAKE_MM_PER_TICK = 0.0f;         // 0 = not measured yet (viewer auto-scales).
                                                // Measure with Slide_Encoder_Check "c 300" (MM_PER_COUNT).

// Motion profile (ticks/s, ticks/s^2). Free speed ~2790 t/s -> cruise at ~70%.
const float OUTTAKE_EXTEND_MAX_VEL = 2000.0f;
const float OUTTAKE_EXTEND_MAX_ACC = 6000.0f;
const float OUTTAKE_RETRACT_MAX_VEL = 2000.0f;
const float OUTTAKE_RETRACT_MAX_ACC = 6000.0f;

const int   OUTTAKE_MAX_PWM = 250;              // during profiled moves
const int   OUTTAKE_HOLD_MAX_PWM = 63;          // while dumping (v1 used 63)
const float OUTTAKE_TOLERANCE_TICKS = 3.0f;     // PID deadband for kS
const float OUTTAKE_ARRIVE_TOL_TICKS = 25.0f;   // "close enough" to move on

// PID (PWM per tick). v1 values.
const float OUTTAKE_KP = 0.4f;
const float OUTTAKE_KI = 0.0f;
const float OUTTAKE_KD = 0.0f;

// Feedforward. This is what lets it TRACK the profile instead of lagging.
//   kS  PWM to overcome static friction (sign follows direction of travel)
//   kV  PWM per tick/s  (~255 / free speed at your battery voltage)
//   kA  PWM per tick/s^2 (usually tiny, leave 0 to start)
//   kG  constant PWM to hold against gravity on the incline (+ = extend)
// The viewer's "FF fit" panel estimates kS/kV from a logged run.
const float OUTTAKE_KS = 12.0f;
const float OUTTAKE_KV = 0.09f;
const float OUTTAKE_KA = 0.0f;
const float OUTTAKE_KG = 0.0f;

const unsigned long OUTTAKE_DUMP_MS = 1500;           // hold at top while rocks fall (v1: 3350 @ 63 PWM)
const unsigned long OUTTAKE_TIMEOUT_MARGIN_MS = 1500; // give up after profile time + this

// "Arrived" = profile done and within OUTTAKE_ARRIVE_TOL_TICKS, OR stopped
// moving within OUTTAKE_SETTLED_TOL_TICKS. Gravity/friction can leave the PID a
// few dozen ticks short; without this it would sit there until the timeout.
// A SETTLED_SHORT event means kG / kS (or KI) need tuning.
const float OUTTAKE_SETTLED_VEL = 30.0f;          // ticks/s
const unsigned long OUTTAKE_SETTLED_MS = 100;
const float OUTTAKE_SETTLED_TOL_TICKS = 150.0f;

// Stall = high PWM but not moving (e.g. bucket jammed on the pole / end stop)
const int   STALL_MIN_PWM = 90;
const float STALL_MAX_VEL = 50.0f;          // ticks/s
const unsigned long STALL_TIME_MS = 200;
const unsigned long STALL_IGNORE_START_MS = 300;  // motor spin-up isn't a stall
const float NO_MOTION_TICKS = 100.0f;   // stalled having moved less than this = fault

// Wiring check: pushing toward the setpoint but running away from it fast
// means the motor or encoder direction is flipped. Cuts the motor.
const int   DIRCHECK_MIN_PWM = 80;
const float DIRCHECK_MIN_VEL = 600.0f;      // ticks/s
const float DIRCHECK_MIN_ERR = 200.0f;      // ticks
const unsigned long DIRCHECK_TIME_MS = 150;

MotorController outtake(OUTTAKE_MA, OUTTAKE_MB, OUTTAKE_PWM, OUTTAKE_ENC_A, OUTTAKE_ENC_B);

// =============================================================================
// SERVOS (PCA9685)
// =============================================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_FREQ = 50;
const long I2C_CLOCK_HZ = 100000;     // 100 kHz: robust next to a brushed motor
const unsigned long I2C_TIMEOUT_US = 3000;    // a stuck bus resets instead of freezing loop()
const unsigned long SERVO_WRITE_INTERVAL_MS = 20;  // servos read a new pulse every 20 ms
const int SERVO_MIN = 700;
const int SERVO_MAX = 2300;
const int PULSE_ABS_MIN = 500;        // hard safety clamp on any pulse
const int PULSE_ABS_MAX = 2500;

// User-facing PCA9685 outputs (1-based, as in ServoTesting.ino)
const int ARM_R = 1;
const int ARM_L = 2;
const int SCOOP_SERVO = 3;

// Per-side trim (us). Two servos on one shaft fight unless they agree.
// Find with ServoTesting: command the same arm position and adjust one side
// until neither servo hums / gets warm at rest.
const int ARM_R_TRIM_US = 0;
const int ARM_L_TRIM_US = 0;

// ---- Servo angles in degrees ------------------------------------------------
// goBILDA 2000-series servos, standard mode: 300 deg over 500-2500 us.
//   Scoop (direct drive): 2000 / 300 = 6.667 us per degree.
//   Arm: 40T servo gear -> 48T arm gear, so 8.0 us per ARM degree, which is
//        exactly 5 arm position units per degree (1000 units = 1600 us).
// Angles: arm   = degrees forward of vertical (+ = toward the dump zone)
//         scoop = degrees from SCOOP_ZERO_US (+ = larger pulse)
// Zeros were estimated from the CAD (the scoop just touches the floor at A_DOWN
// with these values). CALIBRATE in RUN_CONSOLE: "A 0" should stand the arm
// posts vertical (check with a phone level); adjust ARM_VERTICAL_POS until it
// does. "S 0" puts the scoop in its start pose (as drawn in the CAD).
const float ARM_UNITS_PER_DEG = 5.0f;
const float ARM_VERTICAL_POS = 650.0f;     // arm position with the posts vertical
const float SCOOP_US_PER_DEG = 6.667f;
const float SCOOP_ZERO_US = 2300.0f;       // scoop pulse at the start/CAD pose
#define ARM_POS_FROM_DEG(d)   ((int)(ARM_VERTICAL_POS - (d) * ARM_UNITS_PER_DEG + 0.5f))
#define SCOOP_US_FROM_DEG(d)  ((int)(SCOOP_ZERO_US + (d) * SCOOP_US_PER_DEG + 0.5f))
// Positions below can be written either way, e.g. A_DOWN = ARM_POS_FROM_DEG(90).
// With the estimated zeros the current values are:
//   A_START -58  A_TRANSFER -54  A_MOVE 30  A_PEAK 60  A_DOWN 90  A_SCOOP 100 (deg)
//   S_PICKUP -30  S_SCOOP -60  S_TRANSFER -75  S_PEAK -180  S_DOWN -240 (deg)

// Arm Positions (0-1000, V2 values)
const int A_START = 940;
const int A_DOWN = 200;
const int A_PEAK = 350;
const int A_SCOOP = 150;
const int A_TRANSFER = 920;
const int A_MOVE = 500;

// Scoop Positions (us, V2 values)
const int S_START = 2300;
const int S_DOWN = 700;
const int S_PEAK = 1100;   // (unused in V2 sequence, kept for reference)
const int S_SCOOP = 1900;
const int S_PICKUP = 2100;
const int S_TRANSFER = 1800;

// Servo motion profiles. No feedback on servos, so keep these BELOW what the
// servo can actually do under load or it will lag the profile.
//   arm units/s (1000 units = 1600 us), scoop us/s
const float ARM_MAX_VEL = 1500.0f;
const float ARM_MAX_ACC = 8000.0f;
const float ARM_LOADED_MAX_VEL = 1200.0f;   // TRANSFER move, scoop full of rocks
const float ARM_LOADED_MAX_ACC = 5000.0f;
const float SCOOP_MAX_VEL = 3000.0f;
const float SCOOP_MAX_ACC = 15000.0f;

// Coordinated scoop sweep (V2 scoopSweep: scoop S_DOWN->S_SCOOP while arm
// goes A_DOWN->A_PEAK->A_SCOOP). V2 took 101 x 20 ms = ~2.0 s at constant
// speed; here the same path follows a trapezoid profile in the same time.
const unsigned long SCOOP_SWEEP_MS = 2000;
const float SCOOP_SWEEP_ACCEL_FRACTION = 0.25f;   // 25% accel, 50% cruise, 25% decel

// Vibration in TRANSFER (v1: +/-4 deg on the old servo ~= +/-50 us, toggle 50 ms)
const bool VIBRATE_ENABLED = true;
const int VIBRATE_AMPLITUDE_US = 50;
const unsigned int VIBRATE_HALF_PERIOD_MS = 50;
const unsigned long VIBRATE_SETTLE_MS = 50;       // after arriving at transfer pose
const unsigned long VIBRATE_MIN_MS = 1000;        // v1 vibrated for 1000 ms

// =============================================================================
// SEQUENCE TIMING (minimum time in each state, V2 values)
// =============================================================================
const unsigned long DOWN_ARM_DELAY_MS = 500;      // V2: scoop down, wait 500, arm down
const bool DOWN_ARM_WAITS_FOR_SCOOP = true;       // arm also waits for the scoop profile to finish
const unsigned long DOWN_MIN_MS = 1000;
const unsigned long PICKUP_MIN_MS = 1000;
const unsigned long TRANSFER_MIN_MS = 2000;
const unsigned long BEFORE_EXTEND_MIN_MS = 1000;

const unsigned long RUN_WATCHDOG_MS = 60000;      // force DONE well before 120 s

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void writeArm(int position);
void writeScoop(int pulseUs);
void writeServoUs(int channel, int pulseUs);
int microsecondsToCounts(int pulseUs);
const __FlashStringHelper* stateName(RobotState s);
void changeState(RobotState newState);
bool stateDelay(unsigned long ms);
unsigned long stateTime();
void runStateMachine();
void applySweep(float progress);
void startSweep();
void updateOuttakeMonitors();
bool outtakeNear(float targetTicks);
bool outtakeArrived(float targetTicks);
void logTelemetry();
void logEvent(const __FlashStringHelper* code, long value);
void printHeader();
void printConfig();
void printRunSummary();
void printHelp();
void handleSerial();
void processCommand(char* cmd);
void stopEverything();
bool runActive();
void startRun(RobotState firstState);

// =============================================================================
// GLOBALS
// =============================================================================
ServoAxis arm(writeArm, 0, 1000);
ServoAxis scoop(writeScoop, SERVO_MIN, SERVO_MAX);
MotionProfile sweepProfile;

RobotState state = IDLE;
bool entering = false;               // true on the first FSM pass of a state
uint8_t phase = 0;                   // sub-step inside a state (logged)
unsigned long stateStartTime = 0;
unsigned long phaseStartTime = 0;
unsigned long runStartMs = 0;
unsigned long stateDurations[NUM_STATES];

// outtake monitors
bool outtakeStalled = false;
bool stallCondition = false;
unsigned long stallSinceMs = 0;
bool dirBad = false;
unsigned long dirBadSinceMs = 0;
unsigned long jogEndMs = 0;
float extendStartTicks = 0.0f;
bool outtakeSettled = false;
bool settleCondition = false;
unsigned long settledSinceMs = 0;
unsigned long outtakeTimeoutMs = 0;
bool servoDriverOk = true;

// telemetry
unsigned long lastLogMs = 0;
unsigned long loopMaxUs = 0;

bool waitingForGo = false;

// serial console
char cmdBuf[24];
uint8_t cmdLen = 0;

// =============================================================================
// SERVO OUTPUT
// =============================================================================
int microsecondsToCounts(int pulseUs) {
  // At 50 Hz, the PWM period is 20,000 microseconds. (Same as V2 so the
  // calibrated positions from ServoTesting.ino stay valid.)
  return (int)(pulseUs * 4096.0 / 20000.0 + 0.5);
}

void writeServoUs(int channel, int pulseUs) {
  pulseUs = constrain(pulseUs, PULSE_ABS_MIN, PULSE_ABS_MAX);
  pwm.setPWM(channel - 1, 0, microsecondsToCounts(pulseUs));
}

// Coordinated arm: 0-1000 mirrored onto both servos (V2 mapping + trims)
void writeArm(int position) {
  position = constrain(position, 0, 1000);
  const long span = (long)position * (SERVO_MAX - SERVO_MIN) / 1000;
  writeServoUs(ARM_R, SERVO_MIN + span + ARM_R_TRIM_US);
  writeServoUs(ARM_L, SERVO_MAX - span + ARM_L_TRIM_US);
}

void writeScoop(int pulseUs) {
  writeServoUs(SCOOP_SERVO, pulseUs);
}

// =============================================================================
// FSM HELPERS
// =============================================================================
const __FlashStringHelper* stateName(RobotState s) {
  switch (s) {
    case IDLE:            return F("IDLE");
    case DOWN:            return F("DOWN");
    case SCOOP:           return F("SCOOP");
    case PICKUP:          return F("PICKUP");
    case TRANSFER:        return F("TRANSFER");
    case BEFORE_EXTEND:   return F("BEFORE_EXTEND");
    case OUTTAKE_EXTEND:  return F("OUTTAKE_EXTEND");
    case OUTTAKE_RETRACT: return F("OUTTAKE_RETRACT");
    case DONE:            return F("DONE");
    default:              return F("UNKNOWN");
  }
}

bool stateDelay(unsigned long ms) {
  return millis() - stateStartTime >= ms;
}

unsigned long stateTime() {
  return millis() - stateStartTime;
}

void changeState(RobotState newState) {
  const unsigned long now = millis();
  if (state < NUM_STATES) stateDurations[state] += now - stateStartTime;

  state = newState;
  stateStartTime = now;
  phaseStartTime = now;
  phase = 0;
  entering = true;

  // S,<ms>,<STATE>,<index>
  Serial.print(F("S,"));
  Serial.print(now);
  Serial.print(',');
  Serial.print(stateName(state));
  Serial.print(',');
  Serial.println((int)state);
}

bool runActive() {
  return state != IDLE && state != DONE;
}

void startRun(RobotState firstState) {
  for (int i = 0; i < NUM_STATES; i++) stateDurations[i] = 0;
  digitalWrite(END_LED_PIN, LOW);
  jogEndMs = 0;
  runStartMs = millis();
  logEvent(F("RUN_START"), (long)RUN_MODE);
  changeState(firstState);
}

bool outtakeNear(float targetTicks) {
  return fabs(outtake.getPosition() - targetTicks) <= OUTTAKE_ARRIVE_TOL_TICKS;
}

bool outtakeArrived(float targetTicks) {
  if (!outtake.isProfileFinished()) return false;
  const float err = fabs(outtake.getPosition() - targetTicks);
  return err <= OUTTAKE_ARRIVE_TOL_TICKS || (outtakeSettled && err <= OUTTAKE_SETTLED_TOL_TICKS);
}

static void logArrival(float targetTicks) {
  const long pos = (long)outtake.getPosition();
  if (outtakeNear(targetTicks)) logEvent(F("ARRIVED"), pos);
  else logEvent(F("SETTLED_SHORT"), pos);
}

// Profile duration + margin, so long moves are never cut off by the timeout.
static unsigned long profileTimeoutMs() {
  return (unsigned long)(outtake.getProfileTotalTime() * 1000.0f) + OUTTAKE_TIMEOUT_MARGIN_MS;
}

// Coordinated sweep path. progress 0..1000, same formulas as V2 scoopSweep().
void applySweep(float progress) {
  long p = (long)(progress + 0.5f);
  if (p < 0) p = 0;
  if (p > 1000) p = 1000;

  const int scoopPulse = S_DOWN + (long)(S_SCOOP - S_DOWN) * p / 1000;
  int armPosition;
  if (p < 500) {
    armPosition = A_DOWN + (long)(A_PEAK - A_DOWN) * p / 500;
  } else {
    armPosition = A_PEAK - (long)(A_PEAK - A_SCOOP) * (p - 500) / 500;
  }

  scoop.jumpTo(scoopPulse);
  arm.jumpTo(armPosition);
}

// Pick profile limits so the 0..1000 path takes SCOOP_SWEEP_MS.
void startSweep() {
  float f = SCOOP_SWEEP_ACCEL_FRACTION;
  if (f < 0.05f) f = 0.05f;
  if (f > 0.5f) f = 0.5f;
  const float T = SCOOP_SWEEP_MS / 1000.0f;
  const float ta = f * T;
  const float vPeak = 1000.0f / (T - ta);
  sweepProfile.setLimits(vPeak, vPeak / ta);
  sweepProfile.moveTo(0.0f, 1000.0f);
}

// =============================================================================
// STATE MACHINE
// =============================================================================
void runStateMachine() {
  const bool enteredState = entering;
  entering = false;

  switch (state) {
    case IDLE:
      break;

    // ---- V2: scoop down, wait 500 ms, arm down, leave after 1000 ms -------
    case DOWN:
      if (enteredState) {
        scoop.setLimits(SCOOP_MAX_VEL, SCOOP_MAX_ACC);
        scoop.moveTo(S_DOWN);
      }
      if (phase == 0 && stateDelay(DOWN_ARM_DELAY_MS)
          && (!DOWN_ARM_WAITS_FOR_SCOOP || scoop.isFinished())) {
        arm.setLimits(ARM_MAX_VEL, ARM_MAX_ACC);
        arm.moveTo(A_DOWN);
        phase = 1;
        phaseStartTime = millis();
      }
      if (phase == 1 && arm.isFinished() && scoop.isFinished() && stateDelay(DOWN_MIN_MS)) {
        changeState(SCOOP);
      }
      break;

    // ---- V2: coordinated sweep, then straight to PICKUP --------------------
    case SCOOP:
      if (enteredState) {
        startSweep();
      }
      applySweep(sweepProfile.update());
      if (sweepProfile.isFinished()) {
        changeState(PICKUP);
      }
      break;

    // ---- V2: scoop to pickup, 1000 ms --------------------------------------
    case PICKUP:
      if (enteredState) {
        scoop.setLimits(SCOOP_MAX_VEL, SCOOP_MAX_ACC);
        scoop.moveTo(S_PICKUP);
      }
      if (scoop.isFinished() && stateDelay(PICKUP_MIN_MS)) {
        changeState(TRANSFER);
      }
      break;

    // ---- V2: arm + scoop to transfer, 2000 ms. V2.1: + v1 vibration --------
    // phase 0 = moving, 1 = settling, 2 = vibrating
    case TRANSFER:
      if (enteredState) {
        arm.setLimits(ARM_LOADED_MAX_VEL, ARM_LOADED_MAX_ACC);
        arm.moveTo(A_TRANSFER);
        scoop.moveTo(S_TRANSFER);
      }
      if (phase == 0 && arm.isFinished() && scoop.isFinished()) {
        phase = 1;
        phaseStartTime = millis();
      }
      if (phase == 1 && (millis() - phaseStartTime) >= VIBRATE_SETTLE_MS) {
        if (VIBRATE_ENABLED) {
          scoop.startVibration(VIBRATE_AMPLITUDE_US, VIBRATE_HALF_PERIOD_MS);
          logEvent(F("VIB_ON"), VIBRATE_AMPLITUDE_US);
        }
        phase = 2;
        phaseStartTime = millis();
      }
      if (phase == 2 && stateDelay(TRANSFER_MIN_MS)
          && (!VIBRATE_ENABLED || (millis() - phaseStartTime) >= VIBRATE_MIN_MS)) {
        if (scoop.isVibrating()) {
          scoop.stopVibration();
          logEvent(F("VIB_OFF"), (long)(millis() - phaseStartTime));
        }
        changeState(BEFORE_EXTEND);
      }
      break;

    // ---- V2: arm to A_MOVE (clear of slides), 1000 ms ----------------------
    case BEFORE_EXTEND:
      if (enteredState) {
        arm.setLimits(ARM_MAX_VEL, ARM_MAX_ACC);
        arm.moveTo(A_MOVE);
      }
      if (arm.isFinished() && stateDelay(BEFORE_EXTEND_MIN_MS)) {
        changeState(RUN_MODE == RUN_INTAKE_ONLY ? DONE : OUTTAKE_EXTEND);
      }
      break;

    // ---- Profiled extend, then hold while the bucket tips and rocks fall ---
    // phase 0 = moving, 1 = dump hold
    case OUTTAKE_EXTEND:
      if (enteredState) {
        if (OUTTAKE_EXTEND_TICKS <= 0.0f) {
          logEvent(F("NOT_CALIBRATED"), 0);
          changeState(DONE);
          break;
        }
        extendStartTicks = outtake.getPosition();
        outtake.setProfileLimits(OUTTAKE_EXTEND_MAX_VEL, OUTTAKE_EXTEND_MAX_ACC);
        outtake.setTarget(OUTTAKE_EXTEND_TICKS, OUTTAKE_MAX_PWM);
        outtakeTimeoutMs = profileTimeoutMs();
      }
      if (phase == 0) {
        bool arrived = false;
        if (outtakeArrived(OUTTAKE_EXTEND_TICKS)) {
          logArrival(OUTTAKE_EXTEND_TICKS);
          arrived = true;
        } else if (outtakeStalled && stateDelay(STALL_IGNORE_START_MS)) {
          if (fabs(outtake.getPosition() - extendStartTicks) < NO_MOTION_TICKS) {
            // Pushing hard but the encoder never moved: motor reversed into the
            // end stop, no motor power, or encoder unplugged. Stop, don't dump.
            stopEverything();
            logEvent(F("NO_MOTION"), (long)outtake.getPosition());
            changeState(DONE);
            break;
          }
          logEvent(F("STALL"), (long)outtake.getPosition());
          arrived = true;
        } else if (stateDelay(outtakeTimeoutMs)) {
          logEvent(F("TIMEOUT"), (long)outtake.getPosition());
          arrived = true;
        }
        if (arrived) {
          outtake.setMaxPwm(OUTTAKE_HOLD_MAX_PWM);   // gentle hold, like v1's 63
          phase = 1;
          phaseStartTime = millis();
        }
      }
      if (phase == 1 && (millis() - phaseStartTime) >= OUTTAKE_DUMP_MS) {
        changeState(OUTTAKE_RETRACT);
      }
      break;

    // ---- Profiled retract to home ------------------------------------------
    case OUTTAKE_RETRACT:
      if (enteredState) {
        outtake.setProfileLimits(OUTTAKE_RETRACT_MAX_VEL, OUTTAKE_RETRACT_MAX_ACC);
        outtake.setTarget(OUTTAKE_RETRACT_TICKS, OUTTAKE_MAX_PWM);
        outtakeTimeoutMs = profileTimeoutMs();
      }
      if (outtakeArrived(OUTTAKE_RETRACT_TICKS)) {
        logArrival(OUTTAKE_RETRACT_TICKS);
        changeState(DONE);
      } else if (outtakeStalled && stateDelay(STALL_IGNORE_START_MS)) {
        logEvent(F("STALL"), (long)outtake.getPosition());
        changeState(DONE);
      } else if (stateDelay(outtakeTimeoutMs)) {
        logEvent(F("TIMEOUT"), (long)outtake.getPosition());
        changeState(DONE);
      }
      break;

    // ---- All functions cease (R25/R26). Servos hold, motor off, LED on -----
    case DONE:
      if (enteredState) {
        outtake.setMode(MotorController::COAST);
        scoop.stopVibration();
        arm.stopVibration();
        arm.jumpTo(arm.getCommand());       // freeze any move cut short by an abort
        scoop.jumpTo(scoop.getCommand());
        digitalWrite(END_LED_PIN, HIGH);
        printRunSummary();
        printConfig();
      }
      break;

    default:
      break;
  }
}

// =============================================================================
// OUTTAKE SAFETY MONITORS
// =============================================================================
void updateOuttakeMonitors() {
  const unsigned long now = millis();
  const MotorController::MotorMode mode = outtake.getMode();
  const bool driving = (mode == MotorController::ACTIVE || mode == MotorController::OPEN_LOOP);
  const int cmd = outtake.getCommandPower();
  const float vel = outtake.getVelocity();

  // Stall: pushing hard, not moving.
  const bool cond = driving && abs(cmd) >= STALL_MIN_PWM && fabs(vel) <= STALL_MAX_VEL;
  if (cond) {
    if (!stallCondition) {
      stallCondition = true;
      stallSinceMs = now;
    }
  } else {
    stallCondition = false;
  }
  outtakeStalled = stallCondition && (now - stallSinceMs) >= STALL_TIME_MS;

  // Outside a run (console jogs/moves) nothing else acts on a stall: cut it.
  if (outtakeStalled && !runActive()) {
    outtake.setMode(MotorController::COAST);
    jogEndMs = 0;
    stallCondition = false;
    outtakeStalled = false;
    logEvent(F("STALL_CUT"), (long)outtake.getPosition());
  }

  // Settled: barely moving (used to accept a slightly-short arrival).
  if (fabs(vel) <= OUTTAKE_SETTLED_VEL) {
    if (!settleCondition) {
      settleCondition = true;
      settledSinceMs = now;
    }
  } else {
    settleCondition = false;
  }
  outtakeSettled = settleCondition && (now - settledSinceMs) >= OUTTAKE_SETTLED_MS;

  // Wiring check: commanded one way, running away fast the other way.
  bool bad = false;
  if (mode == MotorController::OPEN_LOOP) {
    bad = abs(cmd) >= DIRCHECK_MIN_PWM &&
          ((cmd > 0 && vel < -DIRCHECK_MIN_VEL) || (cmd < 0 && vel > DIRCHECK_MIN_VEL));
  } else if (mode == MotorController::ACTIVE) {
    const float err = outtake.getMovingTarget() - outtake.getPosition();
    bad = fabs(err) > DIRCHECK_MIN_ERR && abs(cmd) >= DIRCHECK_MIN_PWM &&
          ((err > 0 && cmd > 0 && vel < -DIRCHECK_MIN_VEL) ||
           (err < 0 && cmd < 0 && vel >  DIRCHECK_MIN_VEL));
  }

  if (bad) {
    if (!dirBad) {
      dirBad = true;
      dirBadSinceMs = now;
    } else if ((now - dirBadSinceMs) >= DIRCHECK_TIME_MS) {
      dirBad = false;
      outtake.setMode(MotorController::COAST);
      logEvent(F("DIR_MISMATCH"), (long)vel);
      if (runActive()) changeState(DONE);
    }
  } else {
    dirBad = false;
  }
}

// =============================================================================
// TELEMETRY
// =============================================================================
//   D,t,st,ph,aT,aC,sT,sC,sO,oT,oS,oP,oV,oPWM,oFF,fl,lm
//     t    ms since boot          st/ph  state index / sub-phase
//     aT   arm target (0-1000)    aC     arm profiled command
//     sT   scoop target (us)      sC     scoop profiled command
//     sO   scoop output (us, includes vibration)
//     oT   outtake target (ticks) oS     outtake profile setpoint
//     oP   outtake position       oV     outtake velocity (ticks/s)
//     oPWM outtake command        oFF    feedforward part of oPWM
//     fl   flags (bit0 arm moving, 1 scoop moving, 2 outtake profiling,
//                 3 vibrating, 4 stalled, 5 sweep active, 6 coast, 7 open loop)
//     lm   worst loop time since last row (us)
//   S,t,STATE,index      state change
//   E,t,CODE,value       event (VIB_ON, ARRIVED, STALL, TIMEOUT, ABORT ...)
//   R,t,run_ms           run finished
//   #...                 header / config / comments
static inline void field(long v) {
  Serial.print(',');
  Serial.print(v);
}

void logTelemetry() {
  if (!TELEMETRY_ENABLED) return;
  const unsigned long now = millis();
  if ((now - lastLogMs) < LOG_INTERVAL_MS) return;
  lastLogMs = now;

  const MotorController::MotorMode mode = outtake.getMode();
  uint8_t flags = 0;
  if (!arm.isFinished())               flags |= 1;
  if (!scoop.isFinished())             flags |= 2;
  if (!outtake.isProfileFinished())    flags |= 4;
  if (scoop.isVibrating())             flags |= 8;
  if (outtakeStalled)                  flags |= 16;
  if (state == SCOOP && !sweepProfile.isFinished()) flags |= 32;
  if (mode == MotorController::COAST)     flags |= 64;
  if (mode == MotorController::OPEN_LOOP) flags |= 128;

  Serial.print('D');
  field((long)now);
  field((long)state);
  field((long)phase);
  field(arm.getTarget());
  field(arm.getCommand());
  field(scoop.getTarget());
  field(scoop.getCommand());
  field(scoop.getOutput());
  field((long)outtake.getTarget());
  field((long)outtake.getMovingTarget());
  field((long)outtake.getPosition());
  field((long)outtake.getVelocity());
  field(outtake.getCommandPower());
  field(outtake.getFeedforward());
  field((long)flags);
  field((long)(loopMaxUs > 65535UL ? 65535UL : loopMaxUs));
  Serial.println();

  loopMaxUs = 0;
}

void logEvent(const __FlashStringHelper* code, long value) {
  Serial.print(F("E,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(code);
  Serial.print(',');
  Serial.println(value);
}

// Short header at boot (keeps boot fast); full config is printed at DONE.
void printHeader() {
  Serial.print(F("#FW,"));
  Serial.print(FW_NAME);
  Serial.print(F(",schema="));
  Serial.print(LOG_SCHEMA);
  Serial.print(F(",mode="));
  Serial.print((int)RUN_MODE);
  Serial.print(F(",wait_for_go="));
  Serial.println(WAIT_FOR_GO ? 1 : 0);
  Serial.println(F("#COLS,t,st,ph,aT,aC,sT,sC,sO,oT,oS,oP,oV,oPWM,oFF,fl,lm"));
}

static void cfgL(const __FlashStringHelper* key, long v) {
  Serial.print(F("#CFG,"));
  Serial.print(key);
  Serial.print(',');
  Serial.println(v);
}

static void cfgF(const __FlashStringHelper* key, float v) {
  Serial.print(F("#CFG,"));
  Serial.print(key);
  Serial.print(',');
  Serial.println(v, 4);
}

void printConfig() {
  Serial.print(F("#STATES"));
  for (int i = 0; i < NUM_STATES; i++) {
    Serial.print(',');
    Serial.print(stateName((RobotState)i));
  }
  Serial.println();
  cfgL(F("RUN_MODE"), RUN_MODE);
  cfgL(F("WAIT_FOR_GO"), WAIT_FOR_GO ? 1 : 0);
  cfgL(F("LOG_INTERVAL_MS"), LOG_INTERVAL_MS);
  cfgL(F("A_START"), A_START);
  cfgL(F("A_DOWN"), A_DOWN);
  cfgL(F("A_PEAK"), A_PEAK);
  cfgL(F("A_SCOOP"), A_SCOOP);
  cfgL(F("A_TRANSFER"), A_TRANSFER);
  cfgL(F("A_MOVE"), A_MOVE);
  cfgL(F("S_START"), S_START);
  cfgL(F("S_DOWN"), S_DOWN);
  cfgL(F("S_SCOOP"), S_SCOOP);
  cfgL(F("S_PICKUP"), S_PICKUP);
  cfgL(F("S_TRANSFER"), S_TRANSFER);
  cfgL(F("SERVO_MIN"), SERVO_MIN);
  cfgL(F("SERVO_MAX"), SERVO_MAX);
  cfgL(F("ARM_R_TRIM_US"), ARM_R_TRIM_US);
  cfgF(F("ARM_VERTICAL_POS"), ARM_VERTICAL_POS);
  cfgF(F("ARM_UNITS_PER_DEG"), ARM_UNITS_PER_DEG);
  cfgF(F("SCOOP_ZERO_US"), SCOOP_ZERO_US);
  cfgF(F("SCOOP_US_PER_DEG"), SCOOP_US_PER_DEG);
  cfgF(F("A_DOWN_DEG"), (ARM_VERTICAL_POS - A_DOWN) / ARM_UNITS_PER_DEG);
  cfgF(F("A_TRANSFER_DEG"), (ARM_VERTICAL_POS - A_TRANSFER) / ARM_UNITS_PER_DEG);
  cfgF(F("S_TRANSFER_DEG"), (S_TRANSFER - SCOOP_ZERO_US) / SCOOP_US_PER_DEG);
  cfgL(F("ARM_L_TRIM_US"), ARM_L_TRIM_US);
  cfgF(F("ARM_MAX_VEL"), ARM_MAX_VEL);
  cfgF(F("ARM_MAX_ACC"), ARM_MAX_ACC);
  cfgF(F("ARM_LOADED_MAX_VEL"), ARM_LOADED_MAX_VEL);
  cfgF(F("ARM_LOADED_MAX_ACC"), ARM_LOADED_MAX_ACC);
  cfgF(F("SCOOP_MAX_VEL"), SCOOP_MAX_VEL);
  cfgF(F("SCOOP_MAX_ACC"), SCOOP_MAX_ACC);
  cfgL(F("SCOOP_SWEEP_MS"), SCOOP_SWEEP_MS);
  cfgL(F("VIBRATE_ENABLED"), VIBRATE_ENABLED ? 1 : 0);
  cfgL(F("VIBRATE_AMPLITUDE_US"), VIBRATE_AMPLITUDE_US);
  cfgL(F("VIBRATE_HALF_PERIOD_MS"), VIBRATE_HALF_PERIOD_MS);
  cfgL(F("DOWN_MIN_MS"), DOWN_MIN_MS);
  cfgL(F("PICKUP_MIN_MS"), PICKUP_MIN_MS);
  cfgL(F("TRANSFER_MIN_MS"), TRANSFER_MIN_MS);
  cfgL(F("BEFORE_EXTEND_MIN_MS"), BEFORE_EXTEND_MIN_MS);
  cfgF(F("OUTTAKE_EXTEND_TICKS"), OUTTAKE_EXTEND_TICKS);
  cfgF(F("OUTTAKE_RETRACT_TICKS"), OUTTAKE_RETRACT_TICKS);
  cfgF(F("OUTTAKE_MM_PER_TICK"), OUTTAKE_MM_PER_TICK);
  cfgF(F("OUTTAKE_EXTEND_MAX_VEL"), OUTTAKE_EXTEND_MAX_VEL);
  cfgF(F("OUTTAKE_EXTEND_MAX_ACC"), OUTTAKE_EXTEND_MAX_ACC);
  cfgF(F("OUTTAKE_RETRACT_MAX_VEL"), OUTTAKE_RETRACT_MAX_VEL);
  cfgF(F("OUTTAKE_RETRACT_MAX_ACC"), OUTTAKE_RETRACT_MAX_ACC);
  cfgL(F("OUTTAKE_MAX_PWM"), OUTTAKE_MAX_PWM);
  cfgL(F("OUTTAKE_HOLD_MAX_PWM"), OUTTAKE_HOLD_MAX_PWM);
  cfgL(F("OUTTAKE_DUMP_MS"), OUTTAKE_DUMP_MS);
  cfgF(F("OUTTAKE_KP"), OUTTAKE_KP);
  cfgF(F("OUTTAKE_KI"), OUTTAKE_KI);
  cfgF(F("OUTTAKE_KD"), OUTTAKE_KD);
  cfgF(F("OUTTAKE_KS"), OUTTAKE_KS);
  cfgF(F("OUTTAKE_KV"), OUTTAKE_KV);
  cfgF(F("OUTTAKE_KA"), OUTTAKE_KA);
  cfgF(F("OUTTAKE_KG"), OUTTAKE_KG);
  Serial.println(F("#END_CFG"));
}

void printRunSummary() {
  const unsigned long now = millis();
  const unsigned long runMs = now - runStartMs;
  Serial.print(F("R,"));
  Serial.print(now);
  Serial.print(',');
  Serial.println(runMs);

  Serial.print(F("# run "));
  Serial.print(runMs);
  Serial.println(F(" ms (from boot to DONE, add bootloader + reaction time)"));
  for (int i = 1; i < DONE; i++) {
    Serial.print(F("#   "));
    Serial.print(stateName((RobotState)i));
    Serial.print(F(" "));
    Serial.println(stateDurations[i]);
  }
}

// =============================================================================
// SERIAL CONSOLE
// =============================================================================
//   x            abort / stop everything (works in every mode)
//   g            start the run when WAIT_FOR_GO = true
// RUN_CONSOLE only:
//   g            run full sequence from DOWN
//   e            run outtake only (arm to A_MOVE, extend, dump, retract)
//   o <ticks> [ticks/s]  profiled outtake move (optional cruise speed)
//   j <pwm> <ms> open-loop jog
//   c            coast outtake                 z             zero encoder here
//   a <0-1000>   profiled arm move             s <us>        profiled scoop move
//   A <deg>      arm, degrees forward of vertical   S <deg>  scoop, degrees from zero
//   v            toggle scoop vibration        p             print config
//   ?            help
void printHelp() {
  Serial.println(F("# x abort | g full run | e outtake run | o <ticks> [t/s] | j <pwm> <ms>"));
  Serial.println(F("# c coast | z zero enc | a <0-1000> | s <us> | A <deg> | S <deg> | v vibrate | p config"));
}

void stopEverything() {
  outtake.setMode(MotorController::COAST);
  jogEndMs = 0;
  scoop.stopVibration();
  arm.stopVibration();
}

void handleSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        processCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

void processCommand(char* cmd) {
  while (*cmd == ' ') cmd++;
  const char op = cmd[0];
  char* rest = cmd + 1;
  char* next = rest;
  const long a1 = strtol(rest, &next, 10);
  const bool hasA1 = (next != rest);
  const long a2 = strtol(next, &next, 10);

  if (op == 'x') {
    stopEverything();
    logEvent(F("ABORT"), 0);
    if (runActive()) changeState(DONE);
    return;
  }

  if (waitingForGo && op == 'g') {        // WAIT_FOR_GO bench start
    waitingForGo = false;
    startRun(RUN_MODE == RUN_OUTTAKE_ONLY ? BEFORE_EXTEND : DOWN);
    return;
  }

  if (RUN_MODE != RUN_CONSOLE) return;   // competition modes: only 'x'

  const bool busy = runActive();
  if (busy) {                            // during a run only 'x' is accepted
    Serial.println(F("# busy: x to abort"));
    return;
  }
  if ((op == 'o' || op == 'a' || op == 's' || op == 'j' || op == 'A' || op == 'S') && !hasA1) {
    Serial.println(F("# needs a number, ? for help"));
    return;
  }
  switch (op) {
    case '?': printHelp(); break;
    case 'p': printConfig(); break;
    case 'g': startRun(DOWN); break;
    case 'e': startRun(BEFORE_EXTEND); break;   // arm clears the slides first
    case 'o': {
      float t = (float)a1;
      if (t < 0) t = 0;
      if (t > OUTTAKE_SOFT_MAX_TICKS) t = OUTTAKE_SOFT_MAX_TICKS;
      jogEndMs = 0;
      // optional 2nd arg = cruise speed (ticks/s), e.g. "o 3000 800".
      // Moves at 2-3 different speeds give the viewer's FF fit clean kS/kV.
      const float vel = (a2 > 0) ? (float)a2 : OUTTAKE_EXTEND_MAX_VEL;
      outtake.setProfileLimits(vel, OUTTAKE_EXTEND_MAX_ACC);
      outtake.setTarget(t, OUTTAKE_MAX_PWM);
      logEvent(F("CMD_O"), (long)t);
      break;
    }
    case 'j': {
      const long ms = (a2 > 0 && a2 <= 2000) ? a2 : 200;
      outtake.setOpenLoop((int)constrain(a1, -255, 255));
      jogEndMs = millis() + ms;
      logEvent(F("CMD_J"), a1);
      break;
    }
    case 'c': stopEverything(); logEvent(F("CMD_C"), 0); break;
    case 'z':
      outtake.begin(0.0f);
      outtake.setMode(MotorController::COAST);
      logEvent(F("CMD_Z"), 0);
      break;
    case 'a':
      arm.setLimits(ARM_MAX_VEL, ARM_MAX_ACC);
      arm.moveTo((int)a1);
      logEvent(F("CMD_A"), a1);
      break;
    case 's':
      scoop.setLimits(SCOOP_MAX_VEL, SCOOP_MAX_ACC);
      scoop.moveTo((int)a1);
      logEvent(F("CMD_S"), a1);
      break;
    case 'A':                                   // arm in degrees forward of vertical
      arm.setLimits(ARM_MAX_VEL, ARM_MAX_ACC);
      arm.moveTo(ARM_POS_FROM_DEG(a1));
      logEvent(F("CMD_A_DEG"), a1);
      break;
    case 'S':                                   // scoop in degrees from SCOOP_ZERO_US
      scoop.setLimits(SCOOP_MAX_VEL, SCOOP_MAX_ACC);
      scoop.moveTo(SCOOP_US_FROM_DEG(a1));
      logEvent(F("CMD_S_DEG"), a1);
      break;
    case 'v':
      if (scoop.isVibrating()) scoop.stopVibration();
      else scoop.startVibration(VIBRATE_AMPLITUDE_US, VIBRATE_HALF_PERIOD_MS);
      logEvent(F("CMD_V"), scoop.isVibrating() ? 1 : 0);
      break;
    default:
      Serial.println(F("# ? for help"));
      break;
  }
}

// =============================================================================
// SETUP / LOOP
// =============================================================================
void configureOuttake() {
  outtake.setUnitsTicks();
  outtake.setEncoderDirection(OUTTAKE_ENCODER_DIRECTION);
  outtake.setMotorDirection(OUTTAKE_MOTOR_DIRECTION);
  outtake.setPID(OUTTAKE_KP, OUTTAKE_KI, OUTTAKE_KD);
  outtake.setFeedforward(OUTTAKE_KS, OUTTAKE_KV, OUTTAKE_KA, OUTTAKE_KG);
  outtake.setProfileLimits(OUTTAKE_EXTEND_MAX_VEL, OUTTAKE_EXTEND_MAX_ACC);
  outtake.setMaxPwm(OUTTAKE_MAX_PWM);
  outtake.setTolerance(OUTTAKE_TOLERANCE_TICKS);
  outtake.begin(OUTTAKE_START_TICKS);
  outtake.setMode(MotorController::COAST);   // limp until first command
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  printHeader();

  pinMode(END_LED_PIN, OUTPUT);
  digitalWrite(END_LED_PIN, LOW);

  servoDriverOk = pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  if (!servoDriverOk) logEvent(F("NO_SERVO_DRIVER"), 0);
  arm.setWriteInterval(SERVO_WRITE_INTERVAL_MS);
  scoop.setWriteInterval(SERVO_WRITE_INTERVAL_MS);

  // V2 initServos(): jump straight to start pose (no profile, robot is set up there)
  arm.setLimits(ARM_MAX_VEL, ARM_MAX_ACC);
  scoop.setLimits(SCOOP_MAX_VEL, SCOOP_MAX_ACC);
  arm.begin(A_START);
  scoop.begin(S_START);

  configureOuttake();

  if (!servoDriverOk && RUN_MODE != RUN_CONSOLE) {
    changeState(DONE);   // no servo board: don't swing the slide into an unknown arm pose
    return;
  }

  if (WAIT_FOR_GO && RUN_MODE != RUN_CONSOLE) {
    waitingForGo = true;
    Serial.println(F("# WAIT_FOR_GO: holding start pose. Type g + Enter to start the run."));
    return;
  }

  switch (RUN_MODE) {
    case RUN_FULL:
    case RUN_INTAKE_ONLY:  startRun(DOWN); break;
    case RUN_OUTTAKE_ONLY: startRun(BEFORE_EXTEND); break;
    case RUN_CONSOLE:
    default:
      printConfig();
      printHelp();
      break;
  }
}

void loop() {
  const unsigned long loopStartUs = micros();

  handleSerial();
  const unsigned long now = millis();

  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    logEvent(F("I2C_TIMEOUT"), 0);
  }

  outtake.update();
  updateOuttakeMonitors();

  if (jogEndMs != 0 && (long)(now - jogEndMs) >= 0) {
    jogEndMs = 0;
    outtake.setMode(MotorController::COAST);
  }

  if (runActive() && (millis() - runStartMs) > RUN_WATCHDOG_MS) {
    stopEverything();
    logEvent(F("WATCHDOG"), (long)(millis() - runStartMs));
    changeState(DONE);
  }

  if (waitingForGo) digitalWrite(END_LED_PIN, ((now / 250) % 2) ? HIGH : LOW);

  runStateMachine();

  arm.update();
  scoop.update();

  logTelemetry();

  const unsigned long loopUs = micros() - loopStartUs;
  if (loopUs > loopMaxUs) loopMaxUs = loopUs;
}
