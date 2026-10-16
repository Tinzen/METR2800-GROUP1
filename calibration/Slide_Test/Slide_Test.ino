// =============================================================================
// SLIDE TEST - outtake slides only: motion profile + encoder + 50 Hz logging
// =============================================================================
// Nothing moves until you type a command. Use with:
//     python tools/capture_log.py --keep-open --label slidetest
// then type commands + Enter. Every move is logged in the same format as
// Robot_Controller_V2_1, so tools/run_viewer.html plots it (position vs
// setpoint, velocity, PWM, feedforward fit).
//
// Gains and speeds can be changed LIVE (no re-upload). When happy,
// type 'p' and paste the printed lines into Robot_Controller_V2_1.ino.
//
// UNITS: everything you type is in mm (positions), mm/s (vel), mm/s^2 (acc),
// measured from where you last typed 'z'. Internally the code uses encoder
// ticks: 1 tick = MM_PER_TICK mm (0.2954 mm, 36.15 mm spool, 1:1 gearing).
//
// Commands (type ? for this list):
//   m <mm> [vel] [acc]   smooth move TO a position, e.g. m 150  or  m 150 500 2000
//   mr <mm> [vel] [acc]  smooth move BY a distance, e.g. mr -20
//   j <pwm> <ms>     open-loop jog (+ = extend), e.g. j 100 300
//   z                zero here (do this fully retracted)
//   e / r            extend to 'ext' / retract to 0
//   h                hold current position     c   coast (motor off)
//   t <n> [dwell_ms] cycle: extend/retract n times
//   f                feedforward test (tuning)
//   kp ki kd ks kv ka kg <value>   set a gain      e.g. kv 0.095
//   vel <mm/s>  acc <mm/s^2>       profile limits  e.g. vel 400
//   ext <mm>         extend target (used by e and t)
//   d                toggle output: raw data (for Python) / readable mm
//   s                status            p            print paste-ready config
//   x                STOP everything (always works)
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotorController.h"
#include "ServoAxis.h"

// Types used in function arguments must be declared before the first function:
// the Arduino IDE inserts auto-generated function prototypes at that point.
enum TestState { T_IDLE, T_MOVE, T_HOLD, T_JOG, T_NUM };
enum Seq { SEQ_NONE, SEQ_CYCLE, SEQ_FF };

// ---------------- pins (same as Robot_Controller_V2_1) ----------------
const int OUTTAKE_MA = 8;
const int OUTTAKE_MB = 9;
const int OUTTAKE_PWM = 11;
const int OUTTAKE_ENC_A = 2;     // must be D2/D3 (interrupt pins)
const int OUTTAKE_ENC_B = 3;
const int OUTTAKE_ENCODER_DIRECTION = -1;  // -1: raw count went DOWN when extending (tested 14 Oct)
const int OUTTAKE_MOTOR_DIRECTION = 1;     // + PWM must EXTEND

// ---------------- units ----------------
// goBILDA 435 rpm: 384.5 ticks per gearbox rev. 1:1 to the spool (16 Oct; was 1.8:1) -> 384.5 ticks
// per spool rev. Spool 36.15 mm -> 113.57 mm per spool rev.
// Assumes slide travel = cable pulled off the spool. Check with a ruler.
const float MM_PER_TICK = 3.14159265f * 36.15f / 384.5f;   // 0.2954
float toMm(float ticks) { return ticks * MM_PER_TICK; }
float toTicks(float mm) { return mm / MM_PER_TICK; }

// ---------------- starting values ----------------
// SPEED / ACCELERATION: edit the mm numbers here to change the start-up
// defaults, or change them live with 'vel' / 'acc' (lost on restart).
const float DEFAULT_VEL_MM_S  = 167.0f;    // mm/s   (16 Oct: was 300 at the old scale, same motor speed)
const float DEFAULT_ACC_MM_S2 = 556.0f;    // mm/s^2 (was 1000)
// No travel limit: moves go wherever you ask (only targets below 0 are
// clamped to 0). The stall cut-out is the only protection at the far end.
float kP = 0.4f, kI = 0.0f, kD = 0.0f;
float kS = 12.0f, kV = 0.09f, kA = 0.0f, kG = 0.0f;
float maxVel = toTicks(DEFAULT_VEL_MM_S);    // stored in ticks/s
float maxAcc = toTicks(DEFAULT_ACC_MM_S2);   // stored in ticks/s^2
float extendTicks = toTicks(139.0f); // 'ext' PLACEHOLDER (was 250) - set to where the bucket tips
bool rawData = true;                 // 'd' toggles: true = D lines for Python

const int MAX_PWM = 250;               // during moves
const int HOLD_MAX_PWM = 80;           // holding after a move
const unsigned long HOLD_TIMEOUT_MS = 10000;   // then coast so nothing cooks
const float ARRIVE_TOL = 25.0f;
const float SETTLED_TOL = 150.0f;
const float SETTLED_VEL = 30.0f;
const unsigned long SETTLED_MS = 100;
const unsigned long MOVE_TIMEOUT_MARGIN_MS = 1500;

const int   STALL_MIN_PWM = 90;
const float STALL_MAX_VEL = 50.0f;
const unsigned long STALL_TIME_MS = 200;
const unsigned long STALL_IGNORE_START_MS = 300;
const float NO_MOTION_TICKS = 100.0f;
const int   DIRCHECK_MIN_PWM = 80;
const float DIRCHECK_MIN_VEL = 600.0f;
const float DIRCHECK_MIN_ERR = 200.0f;
const unsigned long DIRCHECK_TIME_MS = 150;

// ---------------- arm park (keeps the intake out of the slide's path) -------
// On boot the arm moves from A_START to A_MOVE (like V2's BEFORE_EXTEND) and
// holds there. Set false if the servos are unplugged / unpowered.
const bool PARK_ARM = true;
const int SERVO_MIN = 700, SERVO_MAX = 2300;
const int ARM_R = 1, ARM_L = 2, SCOOP_SERVO = 3;   // PCA9685 outputs, 1-based
const int ARM_R_TRIM_US = 0, ARM_L_TRIM_US = 0;
const int A_START = 940, A_MOVE = 500;
const int S_START = 2300, S_TRANSFER = 1800;

const unsigned long LOG_INTERVAL_MS = 20;

// ---------------- state ----------------

MotorController outtake(OUTTAKE_MA, OUTTAKE_MB, OUTTAKE_PWM, OUTTAKE_ENC_A, OUTTAKE_ENC_B);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
bool servoDriverOk = false;

void writeServoUs(int channel, int us);
void writeArm(int position);
void writeScoop(int us);
ServoAxis arm(writeArm, 0, 1000);
ServoAxis scoop(writeScoop, SERVO_MIN, SERVO_MAX);

TestState state = T_IDLE;
unsigned long stateStartMs = 0;
unsigned long jogEndMs = 0;

// current move bookkeeping
float mvStart = 0, mvTarget = 0, mvVel = 0;
unsigned long mvStartMs = 0, mvTimeoutMs = 0, mvProfileMs = 0;
float mvPeakErr = 0, mvOvershoot = 0;
unsigned long mvProfileDoneMs = 0;

// sequencer
Seq seq = SEQ_NONE;
int seqStep = 0, seqTotal = 0;
unsigned long seqDwellMs = 500;

// monitors
bool stalled = false, stallCond = false;  unsigned long stallSince = 0;
bool settled = false, settleCond = false; unsigned long settleSince = 0;
bool dirBad = false;                      unsigned long dirBadSince = 0;

unsigned long lastLogMs = 0, loopMaxUs = 0;
char cmdBuf[32];
uint8_t cmdLen = 0;

// ---------------- helpers ----------------
const __FlashStringHelper* stateName(TestState s) {
  switch (s) {
    case T_IDLE: return F("IDLE");
    case T_MOVE: return F("MOVE");
    case T_HOLD: return F("HOLD");
    case T_JOG:  return F("JOG");
    default:     return F("UNKNOWN");
  }
}

void logEventL(const __FlashStringHelper* code, long v) {
  Serial.print(F("E,")); Serial.print(millis()); Serial.print(',');
  Serial.print(code); Serial.print(','); Serial.println(v);
}

void setState(TestState s) {
  state = s;
  stateStartMs = millis();
  Serial.print(F("S,")); Serial.print(stateStartMs); Serial.print(',');
  Serial.print(stateName(s)); Serial.print(','); Serial.println((int)s);
}

int microsecondsToCounts(int us) { return (int)(us * 4096.0 / 20000.0 + 0.5); }
void writeServoUs(int channel, int us) {
  if (!servoDriverOk) return;
  us = constrain(us, 500, 2500);
  pwm.setPWM(channel - 1, 0, microsecondsToCounts(us));
}
void writeArm(int position) {
  position = constrain(position, 0, 1000);
  const long span = (long)position * (SERVO_MAX - SERVO_MIN) / 1000;
  writeServoUs(ARM_R, SERVO_MIN + span + ARM_R_TRIM_US);
  writeServoUs(ARM_L, SERVO_MAX - span + ARM_L_TRIM_US);
}
void writeScoop(int us) { writeServoUs(SCOOP_SERVO, us); }

void applyGains() {
  outtake.setPID(kP, kI, kD);
  outtake.setFeedforward(kS, kV, kA, kG);
}

void cfgF(const __FlashStringHelper* k, float v) {
  Serial.print(F("#CFG,")); Serial.print(k); Serial.print(','); Serial.println(v, 4);
}
void printCfg() {
  cfgF(F("OUTTAKE_KP"), kP); cfgF(F("OUTTAKE_KI"), kI); cfgF(F("OUTTAKE_KD"), kD);
  cfgF(F("OUTTAKE_KS"), kS); cfgF(F("OUTTAKE_KV"), kV); cfgF(F("OUTTAKE_KA"), kA); cfgF(F("OUTTAKE_KG"), kG);
  cfgF(F("OUTTAKE_EXTEND_MAX_VEL"), maxVel); cfgF(F("OUTTAKE_EXTEND_MAX_ACC"), maxAcc);
  cfgF(F("OUTTAKE_EXTEND_TICKS"), extendTicks);
  cfgF(F("OUTTAKE_MM_PER_TICK"), MM_PER_TICK);   // for the 3D twin
  cfgF(F("OUTTAKE_MAX_PWM"), MAX_PWM); cfgF(F("A_START"), A_START); cfgF(F("S_START"), S_START);
}

void printPaste() {
  Serial.println(F("# ---- paste into Robot_Controller_V2_1.ino ----"));
  Serial.print(F("# const float OUTTAKE_KP = ")); Serial.print(kP, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KI = ")); Serial.print(kI, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KD = ")); Serial.print(kD, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KS = ")); Serial.print(kS, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KV = ")); Serial.print(kV, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KA = ")); Serial.print(kA, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KG = ")); Serial.print(kG, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_EXTEND_TICKS = ")); Serial.print(extendTicks, 1); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_EXTEND_MAX_VEL = ")); Serial.print(maxVel, 1); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_EXTEND_MAX_ACC = ")); Serial.print(maxAcc, 1); Serial.println(F("f;"));
  Serial.println(F("# ----------------------------------------------"));
  printCfg();
}

void printStatus() {
  Serial.print(F("# state ")); Serial.print(stateName(state));
  Serial.print(F(" | pos ")); Serial.print(toMm(outtake.getPosition()), 1);
  Serial.print(F(" mm | target ")); Serial.print(toMm(outtake.getTarget()), 1);
  Serial.print(F(" | ext ")); Serial.print(toMm(extendTicks), 1);
  Serial.print(F(" | vel ")); Serial.print(toMm(maxVel), 0);
  Serial.print(F(" mm/s | acc ")); Serial.print(toMm(maxAcc), 0); Serial.println(F(" mm/s^2"));
  Serial.print(F("# kP ")); Serial.print(kP, 3); Serial.print(F(" kI ")); Serial.print(kI, 4);
  Serial.print(F(" kD ")); Serial.print(kD, 4); Serial.print(F(" kS ")); Serial.print(kS, 2);
  Serial.print(F(" kV ")); Serial.print(kV, 4); Serial.print(F(" kA ")); Serial.print(kA, 5);
  Serial.print(F(" kG ")); Serial.println(kG, 2);
  if (PARK_ARM && !servoDriverOk) Serial.println(F("# WARNING: PCA9685 not found, arm not parked"));
}

void printHelp() {
  Serial.println(F("# m <mm> go to | mr <mm> move by | z zero | j <pwm> <ms> jog | e extend | r retract"));
  Serial.println(F("# h hold | c coast | t <n> [dwell] cycle | f FF test | s status | p paste cfg"));
  Serial.println(F("# kp ki kd ks kv ka kg <v> | vel <mm/s> | acc <mm/s2> | ext <mm>"));
  Serial.println(F("# d raw/readable output | x STOP"));
}

void stopAll(const __FlashStringHelper* why) {
  outtake.setMode(MotorController::COAST);
  jogEndMs = 0;
  seq = SEQ_NONE;
  if (state != T_IDLE) setState(T_IDLE);
  logEventL(why, (long)outtake.getPosition());
}

bool servosBusy() { return PARK_ARM && servoDriverOk && !arm.isFinished(); }

void startMove(float target, float vel, float acc) {
  if (servosBusy()) { Serial.println(F("# arm still parking, try again")); return; }
  if (target < 0) target = 0;
  if (vel <= 0) vel = maxVel;
  if (acc <= 0) acc = maxAcc;
  jogEndMs = 0;
  mvStart = outtake.getPosition();
  mvTarget = target;
  mvVel = vel;
  outtake.setProfileLimits(vel, acc);
  outtake.setTarget(target, MAX_PWM);
  mvProfileMs = (unsigned long)(outtake.getProfileTotalTime() * 1000.0f);
  mvTimeoutMs = mvProfileMs + MOVE_TIMEOUT_MARGIN_MS;
  mvStartMs = millis();
  mvPeakErr = 0;
  mvOvershoot = 0;
  mvProfileDoneMs = 0;
  setState(T_MOVE);
  logEventL(F("MOVE"), (long)target);
}

// Next step of a cycle / FF test, or nothing.
void nextSeqMove() {
  if (seq == SEQ_NONE) return;
  if (seqStep >= seqTotal) {
    Serial.println(F("# sequence done"));
    logEventL(F("SEQ_DONE"), seqTotal);
    seq = SEQ_NONE;
    return;
  }
  const int i = seqStep++;
  if (seq == SEQ_CYCLE) {
    startMove((i % 2 == 0) ? extendTicks : 0.0f, maxVel, maxAcc);
  } else {   // SEQ_FF: lo first, then hi/lo at 600, 1200, 1800 t/s
    const float travel = extendTicks;
    const float lo = 0.1f * travel, hi = 0.9f * travel;
    static const float speeds[3] = {600.0f, 1200.0f, 1800.0f};
    if (i == 0) startMove(lo, 600.0f, maxAcc);
    else startMove(((i - 1) % 2 == 0) ? hi : lo, speeds[(i - 1) / 2], maxAcc);
  }
}

void finishMove(const __FlashStringHelper* how) {
  const unsigned long now = millis();
  const float pos = outtake.getPosition();
  Serial.print(F("# move ")); Serial.print(toMm(mvStart), 1); Serial.print(F(" -> "));
  Serial.print(toMm(mvTarget), 1); Serial.print(F(" mm @")); Serial.print(toMm(mvVel), 0);
  Serial.print(F(" mm/s: planned ")); Serial.print(mvProfileMs / 1000.0f, 2);
  Serial.print(F(" s, took ")); Serial.print((now - mvStartMs) / 1000.0f, 2);
  Serial.print(F(" s (")); Serial.print(how); Serial.print(F("), now at "));
  Serial.print(toMm(pos), 1); Serial.print(F(" mm, peak lag "));
  Serial.print(toMm(mvPeakErr), 1); Serial.print(F(" mm, overshoot ")); Serial.print(toMm(mvOvershoot), 1);
  Serial.print(F(" mm, final err ")); Serial.print(toMm(mvTarget - pos), 1); Serial.println(F(" mm"));
  logEventL(how, (long)pos);
  logEventL(F("PEAK_ERR"), (long)mvPeakErr);
  outtake.setMaxPwm(HOLD_MAX_PWM);    // keep holding the target, gently
  setState(T_HOLD);
}

// ---------------- monitors ----------------
void updateMonitors() {
  const unsigned long now = millis();
  const MotorController::MotorMode mode = outtake.getMode();
  const bool driving = (mode == MotorController::ACTIVE || mode == MotorController::OPEN_LOOP);
  const int cmd = outtake.getCommandPower();
  const float vel = outtake.getVelocity();

  const bool sc = driving && abs(cmd) >= STALL_MIN_PWM && fabs(vel) <= STALL_MAX_VEL;
  if (sc) { if (!stallCond) { stallCond = true; stallSince = now; } } else stallCond = false;
  stalled = stallCond && (now - stallSince) >= STALL_TIME_MS;

  if (fabs(vel) <= SETTLED_VEL) { if (!settleCond) { settleCond = true; settleSince = now; } } else settleCond = false;
  settled = settleCond && (now - settleSince) >= SETTLED_MS;

  bool bad = false;
  if (mode == MotorController::OPEN_LOOP) {
    bad = abs(cmd) >= DIRCHECK_MIN_PWM && ((cmd > 0 && vel < -DIRCHECK_MIN_VEL) || (cmd < 0 && vel > DIRCHECK_MIN_VEL));
  } else if (mode == MotorController::ACTIVE) {
    const float err = outtake.getMovingTarget() - outtake.getPosition();
    bad = fabs(err) > DIRCHECK_MIN_ERR && abs(cmd) >= DIRCHECK_MIN_PWM &&
          ((err > 0 && cmd > 0 && vel < -DIRCHECK_MIN_VEL) || (err < 0 && cmd < 0 && vel > DIRCHECK_MIN_VEL));
  }
  if (bad) {
    if (!dirBad) { dirBad = true; dirBadSince = now; }
    else if ((now - dirBadSince) >= DIRCHECK_TIME_MS) {
      dirBad = false;
      Serial.println(F("# DIR_MISMATCH: flip OUTTAKE_MOTOR_DIRECTION or OUTTAKE_ENCODER_DIRECTION"));
      stopAll(F("DIR_MISMATCH"));
    }
  } else dirBad = false;
}

// ---------------- test state machine ----------------
void runTest() {
  const unsigned long now = millis();
  const unsigned long inState = now - stateStartMs;

  switch (state) {
    case T_IDLE:
      break;

    case T_JOG:
      if (stalled) { stopAll(F("STALL_CUT")); break; }
      if (jogEndMs != 0 && (long)(now - jogEndMs) >= 0) {
        jogEndMs = 0;
        outtake.setMode(MotorController::COAST);
        setState(T_IDLE);
        Serial.print(F("# jog done, pos ")); Serial.print(toMm(outtake.getPosition()), 1); Serial.println(F(" mm"));
      }
      break;

    case T_MOVE: {
      const float pos = outtake.getPosition();
      const float err = fabs(outtake.getMovingTarget() - pos);
      if (err > mvPeakErr) mvPeakErr = err;
      const float dir = (mvTarget >= mvStart) ? 1.0f : -1.0f;
      const float over = (pos - mvTarget) * dir;
      if (over > mvOvershoot) mvOvershoot = over;
      if (outtake.isProfileFinished() && mvProfileDoneMs == 0) mvProfileDoneMs = now;

      const float finalErr = fabs(pos - mvTarget);
      if (outtake.isProfileFinished() && finalErr <= ARRIVE_TOL) {
        finishMove(F("ARRIVED"));
      } else if (outtake.isProfileFinished() && settled && finalErr <= SETTLED_TOL) {
        finishMove(F("SETTLED_SHORT"));
      } else if (stalled && inState >= STALL_IGNORE_START_MS) {
        if (fabs(pos - mvStart) < NO_MOTION_TICKS) {
          Serial.println(F("# NO_MOTION: motor unpowered/reversed or encoder unplugged"));
          stopAll(F("NO_MOTION"));
        } else {
          finishMove(F("STALL"));
        }
      } else if (inState >= mvTimeoutMs) {
        finishMove(F("TIMEOUT"));
      }
      break;
    }

    case T_HOLD:
      if (seq != SEQ_NONE && inState >= seqDwellMs) {
        nextSeqMove();
      } else if (inState >= HOLD_TIMEOUT_MS) {
        outtake.setMode(MotorController::COAST);
        setState(T_IDLE);
        logEventL(F("HOLD_TIMEOUT"), (long)outtake.getPosition());
      }
      break;

    default:
      break;
  }
}

// ---------------- serial commands ----------------
void processCommand(char* line) {
  char* tok[4] = {0, 0, 0, 0};
  int n = 0;
  for (char* p = strtok(line, " ,"); p && n < 4; p = strtok(0, " ,")) tok[n++] = p;
  if (n == 0) return;
  const char* c = tok[0];
  const float a1 = n > 1 ? atof(tok[1]) : 0.0f;
  const float a2 = n > 2 ? atof(tok[2]) : 0.0f;
  const float a3 = n > 3 ? atof(tok[3]) : 0.0f;

  if (!strcmp(c, "x")) { stopAll(F("STOP")); return; }
  if (!strcmp(c, "?")) { printHelp(); return; }
  if (!strcmp(c, "s")) { printStatus(); return; }
  if (!strcmp(c, "p")) { printPaste(); return; }
  if (!strcmp(c, "c")) { stopAll(F("COAST")); return; }

  // live parameters
  float* gain = 0;
  if      (!strcmp(c, "kp")) gain = &kP;
  else if (!strcmp(c, "ki")) gain = &kI;
  else if (!strcmp(c, "kd")) gain = &kD;
  else if (!strcmp(c, "ks")) gain = &kS;
  else if (!strcmp(c, "kv")) gain = &kV;
  else if (!strcmp(c, "ka")) gain = &kA;
  else if (!strcmp(c, "kg")) gain = &kG;
  if (gain) {
    if (n < 2) { Serial.println(F("# needs a value")); return; }
    *gain = a1;
    applyGains();
    Serial.print(F("# ")); Serial.print(c); Serial.print(F(" = ")); Serial.println(a1, 4);
    printCfg();
    return;
  }
  if (!strcmp(c, "vel") || !strcmp(c, "acc") || !strcmp(c, "ext")) {
    if (n < 2 || a1 <= 0) { Serial.println(F("# needs a positive value")); return; }
    if (!strcmp(c, "vel")) maxVel = toTicks(a1);
    else if (!strcmp(c, "acc")) maxAcc = toTicks(a1);
    else extendTicks = toTicks(a1);
    printCfg();
    printStatus();
    return;
  }
  if (!strcmp(c, "d")) {
    rawData = !rawData;
    Serial.println(rawData ? F("# output: raw data (for Python logger)")
                           : F("# output: readable mm (for Serial Monitor)"));
    return;
  }

  // motion (refused while a move/jog/sequence is running: x first)
  const bool busy = (state == T_MOVE || state == T_JOG || seq != SEQ_NONE);
  if (busy) { Serial.println(F("# busy: x to stop first")); return; }

  if (!strcmp(c, "z")) {
    outtake.begin(0.0f);
    outtake.setMode(MotorController::COAST);
    setState(T_IDLE);
    logEventL(F("ZERO"), 0);
    Serial.println(F("# encoder zeroed"));
  } else if (!strcmp(c, "j")) {
    if (n < 2) { Serial.println(F("# j <pwm> <ms>")); return; }
    const long ms = (n > 2 && a2 > 0 && a2 <= 3000) ? (long)a2 : 200;
    outtake.setOpenLoop((int)constrain(a1, -255.0f, 255.0f));
    jogEndMs = millis() + ms;
    setState(T_JOG);
    logEventL(F("JOG"), (long)a1);
  } else if (!strcmp(c, "m")) {
    if (n < 2) { Serial.println(F("# m <mm> [vel mm/s] [acc mm/s2]")); return; }
    startMove(toTicks(a1), n > 2 ? toTicks(a2) : 0, n > 3 ? toTicks(a3) : 0);
  } else if (!strcmp(c, "mr")) {
    if (n < 2) { Serial.println(F("# mr <mm> [vel mm/s] [acc mm/s2]")); return; }
    startMove(outtake.getPosition() + toTicks(a1), n > 2 ? toTicks(a2) : 0, n > 3 ? toTicks(a3) : 0);
  } else if (!strcmp(c, "o")) {
    if (n < 2) { Serial.println(F("# o <ticks> [vel] [acc]")); return; }
    startMove(a1, a2, a3);
  } else if (!strcmp(c, "e")) {
    startMove(extendTicks, 0, 0);
  } else if (!strcmp(c, "r")) {
    startMove(0.0f, 0, 0);
  } else if (!strcmp(c, "h")) {
    outtake.setMode(MotorController::HOLD);
    outtake.setMaxPwm(HOLD_MAX_PWM);
    setState(T_HOLD);
  } else if (!strcmp(c, "t")) {
    if (n < 2 || a1 < 1) { Serial.println(F("# t <n> [dwell_ms]")); return; }
    seq = SEQ_CYCLE; seqStep = 0; seqTotal = 2 * (int)a1;
    seqDwellMs = (n > 2 && a2 >= 0) ? (unsigned long)a2 : 500;
    nextSeqMove();
  } else if (!strcmp(c, "f")) {
    seq = SEQ_FF; seqStep = 0; seqTotal = 7; seqDwellMs = 400;
    Serial.println(F("# FF test: 7 moves between 10% and 90% of ext"));
    nextSeqMove();
  } else {
    Serial.println(F("# unknown, ? for help"));
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; processCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

// ---------------- telemetry (same schema as Robot_Controller_V2_1) ---------
static inline void field(long v) { Serial.print(','); Serial.print(v); }

void logTelemetry() {
  const unsigned long now = millis();
  if (!rawData) {                 // readable mode: 4 lines a second
    if ((now - lastLogMs) < 250) return;
    lastLogMs = now;
    Serial.print(F("pos ")); Serial.print(toMm(outtake.getPosition()), 1);
    Serial.print(F(" mm   target ")); Serial.print(toMm(outtake.getMovingTarget()), 1);
    Serial.print(F(" mm   pwm ")); Serial.print(outtake.getCommandPower());
    Serial.print(F("   ")); Serial.println(stateName(state));
    return;
  }
  if ((now - lastLogMs) < LOG_INTERVAL_MS) return;
  lastLogMs = now;
  const MotorController::MotorMode mode = outtake.getMode();
  uint8_t fl = 0;
  if (!arm.isFinished()) fl |= 1;
  if (!scoop.isFinished()) fl |= 2;
  if (!outtake.isProfileFinished()) fl |= 4;
  if (stalled) fl |= 16;
  if (mode == MotorController::COAST) fl |= 64;
  if (mode == MotorController::OPEN_LOOP) fl |= 128;
  Serial.print('D');
  field((long)now); field((long)state); field((long)(seq != SEQ_NONE ? seqStep : 0));
  field(arm.getTarget()); field(arm.getCommand());
  field(scoop.getTarget()); field(scoop.getCommand()); field(scoop.getOutput());
  field((long)outtake.getTarget()); field((long)outtake.getMovingTarget());
  field((long)outtake.getPosition()); field((long)outtake.getVelocity());
  field(outtake.getCommandPower()); field(outtake.getFeedforward());
  field((long)fl); field((long)(loopMaxUs > 65535UL ? 65535UL : loopMaxUs));
  Serial.println();
  loopMaxUs = 0;
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("#FW,Slide_Test,schema=1,mode=9"));
  Serial.println(F("#COLS,t,st,ph,aT,aC,sT,sC,sO,oT,oS,oP,oV,oPWM,oFF,fl,lm"));
  Serial.println(F("#STATES,IDLE,MOVE,HOLD,JOG"));

  servoDriverOk = pwm.begin();
  if (servoDriverOk) {
    pwm.setPWMFreq(50);
    Wire.setClock(100000);
    Wire.setWireTimeout(3000, true);
  }
  arm.setWriteInterval(20);
  scoop.setWriteInterval(20);
  arm.setLimits(800.0f, 4000.0f);       // gentle park move
  scoop.setLimits(2000.0f, 10000.0f);
  if (PARK_ARM && servoDriverOk) {
    arm.begin(A_START);
    scoop.begin(S_START);
    arm.moveTo(A_MOVE);
    scoop.moveTo(S_TRANSFER);
    Serial.println(F("# parking arm at A_MOVE (clear of slides)"));
  } else if (PARK_ARM) {
    Serial.println(F("# WARNING: PCA9685 not found, arm NOT parked. Check the slide path is clear."));
  }

  outtake.setUnitsTicks();
  outtake.setEncoderDirection(OUTTAKE_ENCODER_DIRECTION);
  outtake.setMotorDirection(OUTTAKE_MOTOR_DIRECTION);
  applyGains();
  outtake.setProfileLimits(maxVel, maxAcc);
  outtake.setMaxPwm(MAX_PWM);
  outtake.setTolerance(3.0f);
  outtake.begin(0.0f);
  outtake.setMode(MotorController::COAST);

  setState(T_IDLE);
  printCfg();
  printHelp();
  Serial.println(F("# start: retract fully, 'z', then 'j 100 300' to check direction"));
  Serial.println(F("# Using Serial Monitor? type d for readable mm output"));
}

void loop() {
  const unsigned long t0 = micros();
  handleSerial();
  outtake.update();
  updateMonitors();
  runTest();
  if (PARK_ARM && servoDriverOk) { arm.update(); scoop.update(); }
  logTelemetry();
  const unsigned long dt = micros() - t0;
  if (dt > loopMaxUs) loopMaxUs = dt;
}
