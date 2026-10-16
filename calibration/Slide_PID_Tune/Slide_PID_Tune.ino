// =============================================================================
// SLIDE PID TUNE - tune the outtake slide controller, one test at a time
// =============================================================================
// Two tests, each prints a one-line score so you can compare settings:
//
//   b <a> <b> [n]   BOUNCE: smooth (profiled) moves a -> b -> a ... n round trips.
//                   Tunes the whole controller as the robot uses it.
//                   e.g.  b 50 400 3
//   step <mm>       STEP: jump the target instantly (no profile), pure PID.
//                   Shows raw P/D behaviour: rise time, overshoot, settling.
//                   e.g.  step 200
//
// Change a gain, run the same test again, compare the numbers. Example:
//   b 50 400 3      -> note avg lag / overshoot / settle
//   kp 0.6
//   b 50 400 3      -> better or worse?
//
// Other commands:
//   z  zero here (fully retracted)   m <mm>  move to   r  back to 0   x  STOP
//   kp ki kd ks kv ka kg <value>     change a gain
//   vel <mm/s>  acc <mm/s^2>         profile speed / acceleration
//   g  show gains (+ lines to paste into Robot_Controller_V2_1)
//   d  toggle output: raw data (Python logger) / readable mm (Serial Monitor)
//
// Units: positions in mm. Gains are per ENCODER COUNT (same as V2_1), e.g.
// kp 0.4 = 0.4 PWM per count of error = 0.75 PWM per mm.
// Battery must be ON (encoder drifts on USB power alone).
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotorController.h"
#include "ServoAxis.h"

// Declared before any function (the Arduino IDE adds prototypes there).
enum TestState { IDLE, MOVE, HOLD, STEP };
enum TestKind  { TEST_NONE, TEST_BOUNCE, TEST_STEP };

// ---------------- pins (confirmed 14 Oct 2026) ----------------
const int OUTTAKE_MA = 8, OUTTAKE_MB = 9, OUTTAKE_PWM = 11;
const int OUTTAKE_ENC_A = 2, OUTTAKE_ENC_B = 3;
const int OUTTAKE_ENCODER_DIRECTION = -1;
const int OUTTAKE_MOTOR_DIRECTION = 1;

// ---------------- units ----------------
// 435 rpm motor, 384.5 counts/rev, 1:1 to the spool (16 Oct; was 1.8:1), 36.15 mm spool, 1:1 string.
const float MM_PER_TICK = 3.14159265f * 36.15f / 384.5f;   // 0.2954
float toMm(float ticks) { return ticks * MM_PER_TICK; }
float toTicks(float mm) { return mm / MM_PER_TICK; }

// ---------------- starting gains (= V2_1 / Slide_Test) ----------------
float kP = 0.4f, kI = 0.0f, kD = 0.0f;
float kS = 12.0f, kV = 0.09f, kA = 0.0f, kG = 0.0f;
float maxVel = toTicks(167.0f);      // ticks/s   (vel command is in mm/s; was 300 at the old scale)
float maxAcc = toTicks(556.0f);      // ticks/s^2 (acc command is in mm/s^2; was 1000)

// ---------------- test settings ----------------
const int   MAX_PWM = 250;           // during tests
const int   STEP_MAX_PWM = 150;      // step test is harsher: cap it lower
const int   HOLD_PWM = 80;           // holding between / after tests
const float SETTLE_MM = 1.7f;        // "settled" = within this of target...
const unsigned long SETTLE_MS = 150; // ...for this long
const unsigned long TIMEOUT_MARGIN_MS = 2000;
const unsigned long DWELL_MS = 400;  // pause between bounce moves
const unsigned long HOLD_TIMEOUT_MS = 10000;
// stall cut-out: pushing hard but not moving
const int   STALL_PWM = 90;
const float STALL_VEL = 50.0f;       // ticks/s
const unsigned long STALL_MS = 300;

// ---------------- arm park (keeps the arm out of the slide's path) --------
const bool PARK_ARM = true;          // false if the servos are unplugged
const int SERVO_MIN = 700, SERVO_MAX = 2300;
const int ARM_R = 1, ARM_L = 2, SCOOP_SERVO = 3;
const int A_START = 940, A_MOVE = 500, S_START = 2300, S_TRANSFER = 1800;

// ---------------- objects ----------------
MotorController outtake(OUTTAKE_MA, OUTTAKE_MB, OUTTAKE_PWM, OUTTAKE_ENC_A, OUTTAKE_ENC_B);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
bool servoOk = false;
void writeServoUs(int ch, int us) {
  if (servoOk) pwm.setPWM(ch - 1, 0, (int)(constrain(us, 500, 2500) * 4096.0 / 20000.0 + 0.5));
}
void writeArm(int p) {
  const long span = (long)constrain(p, 0, 1000) * (SERVO_MAX - SERVO_MIN) / 1000;
  writeServoUs(ARM_R, SERVO_MIN + span);
  writeServoUs(ARM_L, SERVO_MAX - span);
}
void writeScoop(int us) { writeServoUs(SCOOP_SERVO, us); }
ServoAxis arm(writeArm, 0, 1000);
ServoAxis scoop(writeScoop, SERVO_MIN, SERVO_MAX);

// ---------------- state ----------------
TestState state = IDLE;
unsigned long stateMs = 0;
TestKind test = TEST_NONE;
float bounceA = 0, bounceB = 0;
int movesLeft = 0, movesDone = 0;

// current move measurements
float mvStart, mvTarget;
unsigned long mvStartMs, mvProfileMs, mvTimeoutMs, settleSince;
float peakLag, overshoot;
unsigned long rise10Ms, rise90Ms;
// totals across a test
float sumLag, sumOver, sumSettle, sumErr, worstOver;

bool rawData = true;
unsigned long lastLogMs = 0, stallSince = 0;
char line[40];
uint8_t lineLen = 0;

// ---------------- helpers ----------------
void setState(TestState s) {
  static const char* const names[] = {"IDLE", "MOVE", "HOLD", "STEP"};
  state = s;
  stateMs = millis();
  Serial.print(F("S,")); Serial.print(stateMs); Serial.print(',');
  Serial.print(names[s]); Serial.print(','); Serial.println((int)s);
}

void event(const __FlashStringHelper* code, long v) {
  Serial.print(F("E,")); Serial.print(millis()); Serial.print(',');
  Serial.print(code); Serial.print(','); Serial.println(v);
}

void applyGains() {
  outtake.setPID(kP, kI, kD);
  outtake.setFeedforward(kS, kV, kA, kG);
}

void showGains() {
  Serial.print(F("# kp ")); Serial.print(kP, 3); Serial.print(F("  ki ")); Serial.print(kI, 4);
  Serial.print(F("  kd ")); Serial.print(kD, 4); Serial.print(F("  ks ")); Serial.print(kS, 1);
  Serial.print(F("  kv ")); Serial.print(kV, 4); Serial.print(F("  ka ")); Serial.print(kA, 5);
  Serial.print(F("  kg ")); Serial.print(kG, 1);
  Serial.print(F("  | vel ")); Serial.print(toMm(maxVel), 0); Serial.print(F(" mm/s  acc "));
  Serial.print(toMm(maxAcc), 0); Serial.print(F(" mm/s^2  | pos "));
  Serial.print(toMm(outtake.getPosition()), 1); Serial.println(F(" mm"));
}

void printPaste() {
  Serial.println(F("# ---- paste into Robot_Controller_V2_1.ino ----"));
  Serial.print(F("# const float OUTTAKE_KP = ")); Serial.print(kP, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KI = ")); Serial.print(kI, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KD = ")); Serial.print(kD, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KS = ")); Serial.print(kS, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KV = ")); Serial.print(kV, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KA = ")); Serial.print(kA, 5); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_KG = ")); Serial.print(kG, 4); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_EXTEND_MAX_VEL = ")); Serial.print(maxVel, 1); Serial.println(F("f;"));
  Serial.print(F("# const float OUTTAKE_EXTEND_MAX_ACC = ")); Serial.print(maxAcc, 1); Serial.println(F("f;"));
}

void stopAll(const __FlashStringHelper* why) {
  outtake.setMode(MotorController::COAST);
  test = TEST_NONE;
  movesLeft = 0;
  if (state != IDLE) setState(IDLE);
  event(why, (long)outtake.getPosition());
  Serial.print(F("# stopped: ")); Serial.println(why);
}

// ---------------- moves ----------------
void beginMove(float targetMm, bool profiled) {
  mvStart = outtake.getPosition();
  mvTarget = toTicks(targetMm < 0 ? 0 : targetMm);
  mvStartMs = millis();
  peakLag = overshoot = 0;
  settleSince = 0;
  rise10Ms = rise90Ms = 0;
  stallSince = 0;
  if (profiled) {
    outtake.setProfileLimits(maxVel, maxAcc);
    outtake.setTarget(mvTarget, MAX_PWM);
    mvProfileMs = (unsigned long)(outtake.getProfileTotalTime() * 1000.0f);
    setState(MOVE);
  } else {
    outtake.setTargetPID(mvTarget, STEP_MAX_PWM);
    mvProfileMs = 0;
    setState(STEP);
  }
  mvTimeoutMs = mvProfileMs + TIMEOUT_MARGIN_MS;
  event(F("MOVE"), (long)mvTarget);
}

void nextBounceMove() {
  if (movesLeft <= 0) {                       // test finished: summary
    const float n = movesDone > 0 ? movesDone : 1;
    Serial.print(F("# == BOUNCE RESULT (")); Serial.print(movesDone);
    Serial.print(F(" moves): avg lag ")); Serial.print(sumLag / n, 1);
    Serial.print(F(" mm | avg overshoot ")); Serial.print(sumOver / n, 1);
    Serial.print(F(" mm (worst ")); Serial.print(worstOver, 1);
    Serial.print(F(") | avg settle ")); Serial.print(sumSettle / n, 2);
    Serial.print(F(" s | avg final err ")); Serial.print(sumErr / n, 1);
    Serial.println(F(" mm =="));
    showGains();
    test = TEST_NONE;
    return;
  }
  const float to = (movesDone % 2 == 0) ? bounceB : bounceA;
  movesLeft--;
  beginMove(to, true);
}

// Called when a move has settled (or timed out).
void finishMove(bool settledOk) {
  const unsigned long now = millis();
  const float pos = outtake.getPosition();
  const float settleS = settledOk ? (settleSince - mvStartMs) / 1000.0f - mvProfileMs / 1000.0f : -1;
  const float err = toMm(mvTarget - pos);

  Serial.print(F("# ")); Serial.print(state == STEP ? F("step ") : F("move "));
  Serial.print(toMm(mvStart), 0); Serial.print(F(" -> ")); Serial.print(toMm(mvTarget), 0);
  Serial.print(F(" mm: "));
  if (state == STEP) {
    Serial.print(F("rise ")); Serial.print(rise90Ms > rise10Ms ? (rise90Ms - rise10Ms) / 1000.0f : 0, 2);
    Serial.print(F(" s, "));
  } else {
    Serial.print(F("peak lag ")); Serial.print(toMm(peakLag), 1); Serial.print(F(" mm, "));
  }
  Serial.print(F("overshoot ")); Serial.print(toMm(overshoot), 1);
  Serial.print(F(" mm, settle "));
  if (settledOk) { Serial.print(settleS, 2); Serial.print(F(" s")); } else Serial.print(F("NEVER (timeout)"));
  Serial.print(F(", final err ")); Serial.print(err, 1); Serial.println(F(" mm"));
  event(settledOk ? F("SETTLED") : F("TIMEOUT"), (long)pos);

  // add to test totals (a bounce test's first move, getting to 'a', isn't scored)
  const bool scored = (movesDone >= 0);
  movesDone++;
  if (scored) {
    sumLag += toMm(peakLag);
    sumOver += toMm(overshoot);
    if (toMm(overshoot) > worstOver) worstOver = toMm(overshoot);
    sumSettle += settledOk ? settleS : (now - mvStartMs) / 1000.0f;
    sumErr += fabs(err);
  }

  outtake.setMaxPwm(HOLD_PWM);       // hold the target gently
  setState(HOLD);
}

void resetTotals() { movesDone = 0; sumLag = sumOver = sumSettle = sumErr = worstOver = 0; }

// ---------------- runs every loop ----------------
void runTest() {
  const unsigned long now = millis();
  const float pos = outtake.getPosition();

  if (state == MOVE || state == STEP) {
    // measure
    const float lag = fabs(outtake.getMovingTarget() - pos);
    if (state == MOVE && lag > peakLag) peakLag = lag;
    const float dir = (mvTarget >= mvStart) ? 1.0f : -1.0f;
    const float over = (pos - mvTarget) * dir;
    if (over > overshoot) overshoot = over;
    const float frac = (pos - mvStart) / (mvTarget - mvStart + 0.001f);
    if (!rise10Ms && frac >= 0.1f) rise10Ms = now;
    if (!rise90Ms && frac >= 0.9f) rise90Ms = now;

    // settled? (only counts after the profile has finished)
    const bool profileDone = (state == STEP) || outtake.isProfileFinished();
    if (profileDone && fabs(pos - mvTarget) <= toTicks(SETTLE_MM)) {
      if (!settleSince) settleSince = now;
      if (now - settleSince >= SETTLE_MS) { finishMove(true); return; }
    } else settleSince = 0;

    // safety
    const bool stuck = abs(outtake.getCommandPower()) >= STALL_PWM && fabs(outtake.getVelocity()) < STALL_VEL;
    if (stuck) { if (!stallSince) stallSince = now; } else stallSince = 0;
    if (stallSince && now - stallSince >= STALL_MS && now - mvStartMs > 300) {
      if (fabs(pos - mvStart) < toTicks(11)) {
        Serial.println(F("# NO_MOTION: battery off, motor/encoder unplugged, or direction wrong"));
        stopAll(F("NO_MOTION"));
      } else stopAll(F("STALL"));
      return;
    }
    if (now - mvStartMs >= mvTimeoutMs) finishMove(false);
  }
  else if (state == HOLD) {
    if (test == TEST_BOUNCE && now - stateMs >= DWELL_MS) nextBounceMove();
    else if (test == TEST_STEP) test = TEST_NONE;
    else if (test == TEST_NONE && now - stateMs >= HOLD_TIMEOUT_MS) {
      outtake.setMode(MotorController::COAST);
      setState(IDLE);
    }
  }
}

// ---------------- commands ----------------
void command(char* s) {
  char* tok[4] = {0, 0, 0, 0};
  int n = 0;
  for (char* p = strtok(s, " ,"); p && n < 4; p = strtok(0, " ,")) tok[n++] = p;
  if (!n) return;
  const char* c = tok[0];
  const float a1 = n > 1 ? atof(tok[1]) : 0, a2 = n > 2 ? atof(tok[2]) : 0, a3 = n > 3 ? atof(tok[3]) : 0;

  if (!strcmp(c, "x")) { stopAll(F("STOP")); return; }
  if (!strcmp(c, "g")) { showGains(); printPaste(); return; }
  if (!strcmp(c, "d")) {
    rawData = !rawData;
    Serial.println(rawData ? F("# output: raw data (Python)") : F("# output: readable mm"));
    return;
  }

  float* gain = 0;
  if      (!strcmp(c, "kp")) gain = &kP;
  else if (!strcmp(c, "ki")) gain = &kI;
  else if (!strcmp(c, "kd")) gain = &kD;
  else if (!strcmp(c, "ks")) gain = &kS;
  else if (!strcmp(c, "kv")) gain = &kV;
  else if (!strcmp(c, "ka")) gain = &kA;
  else if (!strcmp(c, "kg")) gain = &kG;
  if (gain) {
    if (n < 2) { Serial.println(F("# needs a value, e.g. kp 0.6")); return; }
    *gain = a1; applyGains(); showGains(); return;
  }
  if (!strcmp(c, "vel") || !strcmp(c, "acc")) {
    if (n < 2 || a1 <= 0) { Serial.println(F("# needs a positive value")); return; }
    if (c[0] == 'v') maxVel = toTicks(a1); else maxAcc = toTicks(a1);
    showGains(); return;
  }

  if (state == MOVE || state == STEP || test != TEST_NONE) { Serial.println(F("# busy: x to stop first")); return; }
  if (!strcmp(c, "z")) {
    outtake.begin(0.0f);
    outtake.setMode(MotorController::COAST);
    setState(IDLE);
    Serial.println(F("# zeroed"));
    return;
  }
  if (PARK_ARM && servoOk && !arm.isFinished()) { Serial.println(F("# arm still parking, try again")); return; }

  if (!strcmp(c, "m")) {
    if (n < 2) { Serial.println(F("# m <mm>")); return; }
    resetTotals(); test = TEST_NONE; beginMove(a1, true);
  } else if (!strcmp(c, "r")) {
    resetTotals(); test = TEST_NONE; beginMove(0, true);
  } else if (!strcmp(c, "b")) {
    if (n < 3) { Serial.println(F("# b <a mm> <b mm> [round trips], e.g. b 50 400 3")); return; }
    bounceA = a1; bounceB = a2;
    const int trips = (n > 3 && a3 >= 1) ? (int)a3 : 3;
    resetTotals();
    test = TEST_BOUNCE;
    movesLeft = 2 * trips + 1;                // first move gets us to a
    movesDone = -1;                           // ...and isn't scored
    beginMove(bounceA, true);
    movesLeft--;
  } else if (!strcmp(c, "step")) {
    if (n < 2) { Serial.println(F("# step <mm>, e.g. step 200")); return; }
    resetTotals(); test = TEST_STEP; beginMove(a1, false);
  } else {
    Serial.println(F("# unknown. b, step, m, r, z, x, kp..kg, vel, acc, g, d"));
  }
}

void readSerial() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLen) { line[lineLen] = 0; command(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) line[lineLen++] = ch;
  }
}

// ---------------- output (same D-line format as V2_1 for the Python logger) --
void logOut() {
  const unsigned long now = millis();
  if (!rawData) {
    if (now - lastLogMs < 250) return;
    lastLogMs = now;
    Serial.print(F("pos ")); Serial.print(toMm(outtake.getPosition()), 1);
    Serial.print(F(" mm   target ")); Serial.print(toMm(outtake.getMovingTarget()), 1);
    Serial.print(F(" mm   pwm ")); Serial.println(outtake.getCommandPower());
    return;
  }
  if (now - lastLogMs < 20) return;
  lastLogMs = now;
  const MotorController::MotorMode mode = outtake.getMode();
  uint8_t fl = 0;
  if (!outtake.isProfileFinished()) fl |= 4;
  if (stallSince) fl |= 16;
  if (mode == MotorController::COAST) fl |= 64;
  long v[] = {(long)now, (long)state, (long)movesDone, arm.getTarget(), arm.getCommand(),
              scoop.getTarget(), scoop.getCommand(), scoop.getOutput(),
              (long)outtake.getTarget(), (long)outtake.getMovingTarget(),
              (long)outtake.getPosition(), (long)outtake.getVelocity(),
              outtake.getCommandPower(), outtake.getFeedforward(), (long)fl, 0};
  Serial.print('D');
  for (uint8_t i = 0; i < 16; i++) { Serial.print(','); Serial.print(v[i]); }
  Serial.println();
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("#FW,Slide_PID_Tune,schema=1,mode=9"));
  Serial.println(F("#COLS,t,st,ph,aT,aC,sT,sC,sO,oT,oS,oP,oV,oPWM,oFF,fl,lm"));
  Serial.println(F("#STATES,IDLE,MOVE,HOLD,STEP"));
  Serial.print(F("#CFG,OUTTAKE_MM_PER_TICK,")); Serial.println(MM_PER_TICK, 4);

  servoOk = PARK_ARM && pwm.begin();
  if (servoOk) {
    pwm.setPWMFreq(50);
    Wire.setClock(100000);
    Wire.setWireTimeout(3000, true);
    arm.setWriteInterval(20); scoop.setWriteInterval(20);
    arm.setLimits(800.0f, 4000.0f); scoop.setLimits(2000.0f, 10000.0f);
    arm.begin(A_START); scoop.begin(S_START);
    arm.moveTo(A_MOVE); scoop.moveTo(S_TRANSFER);
  }

  outtake.setUnitsTicks();
  outtake.setEncoderDirection(OUTTAKE_ENCODER_DIRECTION);
  outtake.setMotorDirection(OUTTAKE_MOTOR_DIRECTION);
  applyGains();
  outtake.setMaxPwm(MAX_PWM);
  outtake.setTolerance(3.0f);
  outtake.begin(0.0f);
  outtake.setMode(MotorController::COAST);
  setState(IDLE);

  Serial.println(F("# Slide PID tune. Retract fully, z, then e.g.  b 50 400 3  or  step 200"));
  Serial.println(F("# Serial Monitor? type d for readable output"));
  showGains();
}

void loop() {
  readSerial();
  outtake.update();
  runTest();
  if (servoOk) { arm.update(); scoop.update(); }
  logOut();
}
