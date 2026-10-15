// =============================================================================
// METR2800 GROUP 1 - ROBOT CONTROLLER V2.1   (robot v2: servo intake)
// =============================================================================
// The sequence is a table in Sequence.h (one row per step). This file just
// runs that table, one step at a time:
//
//   enter step  -> start the moves in that row (arm, scoop, slides, vibration)
//   every loop  -> update the moves, check the row's "NEXT WHEN" condition
//   condition   -> next row ... last row -> DONE (motor off, LED on)
//
// Hardware numbers (pins, ratios, servo calibration, gains) are in RobotConfig.h.
// Tune the sequence in tools/sequence_editor.html; calibrate with calibration/.
//
// Default sequence (V2 intake, V1 transfer, profiled outtake):
//   SCOOP_DOWN ARM_DOWN SWEEP PICKUP | LIFT CARRY POUR VIBRATE |
//   EXTEND CREEP TIP_HOLD RETRACT | DONE
// =============================================================================

#include <Wire.h>
#include "MotorController.h"
#include "ServoAxis.h"
#include "RobotConfig.h"
#include "Sequence.h"

// =============================================================================
// TEST SETTINGS  (change these for bench work)
// =============================================================================
//   RUN_FULL     competition: runs the sequence on power-up
//   RUN_CONSOLE  waits for serial commands (jog, single steps, live link). Type ?
enum RunMode { RUN_FULL = 0, RUN_CONSOLE = 1 };
const RunMode RUN_MODE = RUN_FULL;

// true = after boot, hold the start pose and wait for 'g' (USB resets the Nano
// every time the port opens). MUST be false at competition.
const bool WAIT_FOR_GO = false;

const uint8_t FIRST_STEP = 0;        // run from this step (0 = first row)
const uint8_t LAST_STEP = 255;       // ... to this step (255 = last row)
const bool STEP_MODE_AT_BOOT = false;// true = pause after every step, 'n' = next
const uint8_t SPEED_PCT_AT_BOOT = 100;  // slow-motion first runs: e.g. 25

// =============================================================================
// SERIAL / TELEMETRY
// =============================================================================
const unsigned long SERIAL_BAUD = 115200;
const bool TELEMETRY_ENABLED = true;
const unsigned long LOG_INTERVAL_MS = 20;    // 50 Hz
const char FW_NAME[] = "Robot_Controller_V2_1";
const int LOG_SCHEMA = 2;
// The log keeps the V2 column units so tools/run_viewer.html still works:
// arm as 0-1000 "position" (650 = vertical, 5 per deg), scoop as us (2300 = 0 deg).
const float LOG_ARM_ZERO = 650.0f, LOG_ARM_PER_DEG = 5.0f;
const float LOG_SCOOP_ZERO = 2300.0f, LOG_SCOOP_PER_DEG = 6.667f;

const long I2C_CLOCK_HZ = 100000;
const unsigned long I2C_TIMEOUT_US = 3000;
const unsigned long SERVO_WRITE_INTERVAL_MS = 20;

// =============================================================================
// OBJECTS
// =============================================================================
const uint8_t NUM_STEPS = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

void writeArm(float deg);
void writeScoop(float deg);

MotorController slide(OUTTAKE_MA, OUTTAKE_MB, OUTTAKE_PWM, OUTTAKE_ENC_A, OUTTAKE_ENC_B);
ServoAxis arm(writeArm, ARM_MIN_DEG, ARM_MAX_DEG);
ServoAxis scoop(writeScoop, SCOOP_MIN_DEG, SCOOP_MAX_DEG);

// =============================================================================
// RUNNER STATE
// =============================================================================
enum RunState { R_IDLE, R_RUNNING, R_PAUSED, R_DONE };
RunState runState = R_IDLE;
uint8_t stepIndex = 0;
uint8_t lastStep = 0;
Step cur;                         // current row (copied out of flash)
unsigned long stepStartMs = 0;
unsigned long runStartMs = 0;
unsigned long arrivedAtMs = 0;    // WAIT_ARRIVED: when everything arrived (0 = not yet)
bool stepMode = STEP_MODE_AT_BOOT;
float speedK = SPEED_PCT_AT_BOOT / 100.0f;
bool armMovedThisStep = false, scoopMovedThisStep = false, slideMovedThisStep = false;
bool fallbackUsed = false;
bool waitingForGo = false;

struct TimedMove {               // MOVE = ms: the axis is interpolated by time
  bool active;                   // (one per axis, so a later step that only moves
  unsigned long t0, dur;         //  the scoop doesn't stop the arm's move)
  float from, to, via;
  bool viaOn;
  float p;
};
TimedMove tmArm = {}, tmScoop = {};

// slides (mm in the sequence, counts in the controller)
float slideTargetMm = 0.0f;
float slideVEndMm = 0.0f;         // >0: this segment hands its speed to the next step
unsigned long slideTimeoutMs = 0;
bool slideStalled = false, stallCond = false, dirBad = false;
bool settleCond = false, slideSettled = false;
unsigned long stallSinceMs = 0, dirBadSinceMs = 0, settledSinceMs = 0, jogEndMs = 0;
bool servoDriverOk = true;

unsigned long lastLogMs = 0, loopMaxUs = 0;
char cmdBuf[28];
uint8_t cmdLen = 0;

// =============================================================================
// UNITS
// =============================================================================
inline float mmToCounts(float mm) { return mm / SLIDE_MM_PER_COUNT; }
inline float countsToMm(float c) { return c * SLIDE_MM_PER_COUNT; }
inline float slideMm() { return countsToMm(slide.getPosition()); }
inline bool isHold(float v) { return v > 9000.0f; }

// =============================================================================
// PCA9685 SERVO BOARD  (minimal driver: same registers and 50 Hz prescale as
// the Adafruit library V2 used, without its ~2 KB of flash)
// =============================================================================
const uint8_t PCA_ADDR = 0x40;

bool pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool pcaBegin() {
  Wire.begin();
  if (!pcaWrite(0x00, 0x80)) return false;   // MODE1 restart (= Adafruit reset())
  delay(10);
  pcaWrite(0x00, 0x10);                      // sleep so the prescale can change
  pcaWrite(0xFE, 121);                       // 50 Hz at the 25 MHz nominal clock (Adafruit gives 121 too)
  pcaWrite(0x00, 0x20);                      // wake, register auto-increment
  delay(5);
  pcaWrite(0x00, 0xA0);                      // restart + auto-increment
  return true;
}

void pcaSetPulse(uint8_t channel, uint16_t offCount) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x06 + 4 * channel);            // LEDn_ON_L
  Wire.write(0);
  Wire.write(0);
  Wire.write(offCount & 0xFF);
  Wire.write(offCount >> 8);
  Wire.endTransmission();
}

// =============================================================================
// SERVO OUTPUT  (degrees -> pulse width, using the calibration in RobotConfig.h)
// =============================================================================
uint16_t microsecondsToCounts(float us) { return (uint16_t)(us * 4096.0f / 20000.0f + 0.5f); }

void writeServoUs(int output, float us) {
  if (us < PULSE_ABS_MIN) us = PULSE_ABS_MIN;
  if (us > PULSE_ABS_MAX) us = PULSE_ABS_MAX;
  pcaSetPulse(output - 1, microsecondsToCounts(us));
}

float armRUs(float deg)  { return ARM_R_CAL_US + ARM_R_DIR * ARM_US_PER_DEG * (deg - CAL_ARM_DEG); }
float armLUs(float deg)  { return ARM_L_CAL_US + ARM_L_DIR * ARM_US_PER_DEG * (deg - CAL_ARM_DEG); }
float scoopUs(float deg) { return SCOOP_CAL_US + SCOOP_DIR * SCOOP_US_PER_DEG * (deg - CAL_SCOOP_DEG); }

void writeArm(float deg) {
  writeServoUs(ARM_R_OUTPUT, armRUs(deg));
  writeServoUs(ARM_L_OUTPUT, armLUs(deg));
}
void writeScoop(float deg) { writeServoUs(SCOOP_OUTPUT, scoopUs(deg)); }

// =============================================================================
// LOGGING HELPERS
// =============================================================================
void logEvent(const __FlashStringHelper* code, long value) {
  Serial.print(F("E,")); Serial.print(millis()); Serial.print(',');
  Serial.print(code); Serial.print(','); Serial.println(value);
}

void printStepName(uint8_t i) {
  char nm[13];
  memcpy_P(nm, SEQUENCE[i].name, sizeof(nm));
  nm[12] = '\0';
  Serial.print(nm);
}

uint8_t stateCode() {            // 0 IDLE, 1..N steps, N+1 DONE  (matches #STATES)
  if (runState == R_DONE) return NUM_STEPS + 1;
  if (runState == R_IDLE) return 0;
  return stepIndex + 1;
}

// =============================================================================
// TIMED MOVES
// =============================================================================
float easeProgress(float u) {
  if (u <= 0.0f) return 0.0f;
  if (u >= 1.0f) return 1.0f;
  float e = TIMED_MOVE_EASE;
  if (e <= 0.001f) return u;
  if (e > 0.5f) e = 0.5f;
  const float vp = 1.0f / (1.0f - e);
  if (u < e) return 0.5f * vp * u * u / e;
  if (u < 1.0f - e) return 0.5f * vp * e + vp * (u - e);
  const float r = 1.0f - u;
  return 1.0f - 0.5f * vp * r * r / e;
}

void updateTimedMove(TimedMove& m, ServoAxis& ax) {
  if (!m.active) return;
  float u = (float)(millis() - m.t0) / (float)m.dur;
  if (u > 1.0f) u = 1.0f;
  const float p = easeProgress(u);
  m.p = p;
  float v;
  if (m.viaOn) v = (p < 0.5f) ? m.from + (m.via - m.from) * (p / 0.5f)
                              : m.via + (m.to - m.via) * ((p - 0.5f) / 0.5f);
  else v = m.from + (m.to - m.from) * p;
  ax.jumpTo(v);
  if (u >= 1.0f) m.active = false;
}

void startTimedMove(TimedMove& m, ServoAxis& ax, float to, float via, unsigned long dur) {
  m.active = true;
  m.t0 = stepStartMs;
  m.dur = dur < 1 ? 1 : dur;
  m.from = ax.getCommand();
  m.to = to;
  m.viaOn = !isHold(via);
  m.via = via;
  m.p = 0.0f;
}

// =============================================================================
// STEP RUNNER
// =============================================================================
void loadStep(uint8_t i, Step& s) { memcpy_P(&s, &SEQUENCE[i], sizeof(Step)); }

float slideSpeedFor(const Step& s) {
  float v = (s.slideSpeed > 0.0f) ? s.slideSpeed : SLIDE_MAX;
  if (v > SLIDE_MAX) v = SLIDE_MAX;
  return v * speedK;
}

void commandSlide(const Step& s, uint8_t i) {
  const float pos = slideMm();
  const float speed = slideSpeedFor(s);
  const float accel = SLIDE_ACCEL * speedK * speedK;

  // entry speed: handed over by the previous step (fast -> creep blend).
  // Step mode pauses between steps, so it never blends.
  const float vStart = stepMode ? 0.0f : slideVEndMm;

  // exit speed: blend into the next row if it keeps the slides going the same way
  float vEnd = 0.0f;
  if (!stepMode && s.waitType == W_ARRIVED && i < lastStep) {
    Step nx;
    loadStep(i + 1, nx);
    if (!isHold(nx.slide)) {
      const float d1 = s.slide - pos, d2 = nx.slide - s.slide;
      if (d1 * d2 > 0.0f) {
        const float nSpeed = slideSpeedFor(nx);
        vEnd = (nSpeed < speed) ? nSpeed : speed;
      }
    }
  }

  slide.setProfileLimits(mmToCounts(speed), mmToCounts(accel));
  slide.setTargetBlend(mmToCounts(s.slide), SLIDE_MAX_PWM, mmToCounts(vStart), mmToCounts(vEnd));
  slideTargetMm = s.slide;
  slideVEndMm = vEnd;
  slideMovedThisStep = true;
  slideTimeoutMs = (unsigned long)(slide.getProfileTotalTime() * 1000.0f) + SLIDE_TIMEOUT_MARGIN_MS;
  stallCond = false;
  slideStalled = false;
}

void enterStep(uint8_t i) {
  stepIndex = i;
  loadStep(i, cur);
  stepStartMs = millis();
  arrivedAtMs = 0;
  armMovedThisStep = !isHold(cur.arm);
  scoopMovedThisStep = !isHold(cur.scoop);
  slideMovedThisStep = false;

  Serial.print(F("S,")); Serial.print(stepStartMs); Serial.print(',');
  printStepName(i); Serial.print(','); Serial.println(stateCode());

  // ---- servos -------------------------------------------------------------
  // An axis this row moves drops whatever it was doing; HOLD axes carry on.
  if (armMovedThisStep) tmArm.active = false;
  if (scoopMovedThisStep) tmScoop.active = false;
  if (cur.move > 0) {
    const unsigned long dur = (unsigned long)(cur.move / speedK);
    if (armMovedThisStep) startTimedMove(tmArm, arm, cur.arm, cur.armVia, dur);
    if (scoopMovedThisStep) startTimedMove(tmScoop, scoop, cur.scoop, NONE, dur);
  } else if (cur.move == INSTANT) {
    if (armMovedThisStep) arm.instantTo(cur.arm);
    if (scoopMovedThisStep) scoop.instantTo(cur.scoop);
  } else {
    arm.setLimits(ARM_SPEED * speedK, ARM_ACCEL * speedK * speedK);
    scoop.setLimits(SCOOP_SPEED * speedK, SCOOP_ACCEL * speedK * speedK);
    if (armMovedThisStep) arm.moveTo(cur.arm);
    if (scoopMovedThisStep) scoop.moveTo(cur.scoop);
  }

  if (cur.vibrate > 0.0f) {
    scoop.startVibration(cur.vibrate, VIBRATE_HALF_PERIOD_MS);
    logEvent(F("VIB_ON"), (long)(cur.vibrate * 10));
  } else if (scoop.isVibrating()) {
    scoop.stopVibration();
    logEvent(F("VIB_OFF"), 0);
  }

  // ---- slides -------------------------------------------------------------
  if (!isHold(cur.slide)) commandSlide(cur, i);
  else slideVEndMm = 0.0f;
}

bool slideArrived() {
  if (!slide.isProfileFinished()) return false;
  if (slideVEndMm > 0.0f) return true;        // handing over to the next segment
  const float err = fabs(slideTargetMm - slideMm());
  if (err <= SLIDE_ARRIVE_TOL_MM) return true;
  if (slideSettled && err <= SLIDE_SETTLED_TOL_MM) {
    logEvent(F("SETTLED_SHORT"), (long)(slideTargetMm - slideMm()));
    return true;
  }
  return false;
}

bool armDone()   { return tmArm.active ? false : arm.isFinished(); }
bool scoopDone() { return tmScoop.active ? false : scoop.isFinished(); }
float armProgress() { return tmArm.active ? tmArm.p : arm.getProgress(); }

bool stepConditionMet() {
  const unsigned long now = millis();
  const unsigned long el = now - stepStartMs;
  const unsigned long wv = (unsigned long)(cur.waitValue / speedK);
  switch (cur.waitType) {
    case W_MS:
      return el >= wv;
    case W_ARRIVED: {
      const bool all = (!armMovedThisStep || armDone()) &&
                       (!scoopMovedThisStep || scoopDone()) &&
                       (!slideMovedThisStep || slideArrived());
      if (!all) { arrivedAtMs = 0; return false; }
      if (arrivedAtMs == 0) arrivedAtMs = now | 1UL;
      return (now - arrivedAtMs) >= wv;
    }
    case W_ARM:
      return armProgress() * 100.0f >= cur.waitValue;
    case W_SLIDE:
      if (!slideMovedThisStep) return true;
      return slide.getProfileProgress() * 100.0f >= cur.waitValue;
  }
  return true;
}

int8_t findStep(const char* name) {
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    char nm[13];
    memcpy_P(nm, SEQUENCE[i].name, sizeof(nm));
    if (strncmp(nm, name, 12) == 0) return (int8_t)i;
  }
  return -1;
}

void finishRun();

// slides failed (stall / timeout): try the RETRACT step once, otherwise stop
void slideFault(const __FlashStringHelper* code) {
  logEvent(code, (long)slideMm());
  const int8_t r = findStep("RETRACT");
  if (!fallbackUsed && r > (int8_t)stepIndex) {
    fallbackUsed = true;
    logEvent(F("FALLBACK"), r);
    slideVEndMm = 0.0f;
    enterStep((uint8_t)r);
  } else {
    finishRun();
  }
}

void updateRunner() {
  if (runState != R_RUNNING) return;
  updateTimedMove(tmArm, arm);
  updateTimedMove(tmScoop, scoop);

  if (slideMovedThisStep) {
    const unsigned long el = millis() - stepStartMs;
    if (slideStalled && el >= STALL_IGNORE_START_MS) { slideFault(F("STALL")); return; }
    if (el >= slideTimeoutMs && !slideArrived()) { slideFault(F("TIMEOUT")); return; }
  }

  if (!stepConditionMet()) return;

  if (stepIndex >= lastStep) { finishRun(); return; }
  if (stepMode) {
    runState = R_PAUSED;
    Serial.print(F("# paused after ")); printStepName(stepIndex); Serial.println(F(" - n = next, x = stop"));
    return;
  }
  enterStep(stepIndex + 1);
}

void startRun(uint8_t first) {
  if (first >= NUM_STEPS) first = 0;
  lastStep = (LAST_STEP >= NUM_STEPS) ? NUM_STEPS - 1 : LAST_STEP;
  if (first > lastStep) lastStep = NUM_STEPS - 1;
  runStartMs = millis();
  fallbackUsed = false;
  slideVEndMm = 0.0f;
  digitalWrite(END_LED_PIN, LOW);
  runState = R_RUNNING;
  logEvent(F("RUN_START"), first);
  enterStep(first);
}

void printConfig();

void finishRun() {
  runState = R_DONE;
  tmArm.active = tmScoop.active = false;
  slide.setMode(MotorController::COAST);
  scoop.stopVibration();
  arm.jumpTo(arm.getCommand());          // freeze anything cut short
  scoop.jumpTo(scoop.getCommand());
  digitalWrite(END_LED_PIN, HIGH);
  const unsigned long now = millis();
  Serial.print(F("S,")); Serial.print(now); Serial.print(F(",DONE,")); Serial.println(stateCode());
  Serial.print(F("R,")); Serial.print(now); Serial.print(','); Serial.println(now - runStartMs);
  Serial.print(F("# run ")); Serial.print(now - runStartMs);
  Serial.println(F(" ms (boot-to-DONE; add bootloader + reaction time)"));
  printConfig();
}

void stopEverything() {
  slide.setMode(MotorController::COAST);
  jogEndMs = 0;
  scoop.stopVibration();
  tmArm.active = tmScoop.active = false;
}

// =============================================================================
// SLIDE SAFETY MONITORS
// =============================================================================
void updateSlideMonitors() {
  const unsigned long now = millis();
  const MotorController::MotorMode mode = slide.getMode();
  const bool driving = (mode == MotorController::ACTIVE || mode == MotorController::OPEN_LOOP);
  const int cmd = slide.getCommandPower();
  const float vel = slide.getVelocity();

  // stall: pushing hard, not moving
  const bool c = driving && abs(cmd) >= STALL_MIN_PWM && fabs(vel) <= STALL_MAX_VEL;
  if (c) { if (!stallCond) { stallCond = true; stallSinceMs = now; } }
  else stallCond = false;
  slideStalled = stallCond && (now - stallSinceMs) >= STALL_TIME_MS;

  if (slideStalled && runState != R_RUNNING) {          // console moves: just cut it
    slide.setMode(MotorController::COAST);
    jogEndMs = 0;
    stallCond = false;
    slideStalled = false;
    logEvent(F("STALL_CUT"), (long)slideMm());
  }

  // settled: barely moving (accept a slightly short arrival)
  if (fabs(vel) <= 30.0f) { if (!settleCond) { settleCond = true; settledSinceMs = now; } }
  else settleCond = false;
  slideSettled = settleCond && (now - settledSinceMs) >= 100;

  // wiring check: driving one way, running away fast the other way
  bool bad = false;
  if (mode == MotorController::OPEN_LOOP) {
    bad = abs(cmd) >= DIRCHECK_MIN_PWM &&
          ((cmd > 0 && vel < -DIRCHECK_MIN_VEL) || (cmd < 0 && vel > DIRCHECK_MIN_VEL));
  } else if (mode == MotorController::ACTIVE) {
    const float err = slide.getMovingTarget() - slide.getPosition();
    bad = fabs(err) > DIRCHECK_MIN_ERR && abs(cmd) >= DIRCHECK_MIN_PWM &&
          ((err > 0 && cmd > 0 && vel < -DIRCHECK_MIN_VEL) ||
           (err < 0 && cmd < 0 && vel >  DIRCHECK_MIN_VEL));
  }
  if (bad) {
    if (!dirBad) { dirBad = true; dirBadSinceMs = now; }
    else if ((now - dirBadSinceMs) >= DIRCHECK_TIME_MS) {
      dirBad = false;
      slide.setMode(MotorController::COAST);
      logEvent(F("DIR_MISMATCH"), (long)vel);
      if (runState == R_RUNNING || runState == R_PAUSED) finishRun();
    }
  } else dirBad = false;
}

// =============================================================================
// TELEMETRY
// =============================================================================
//   D,t,st,ph,aT,aC,sT,sC,sO,oT,oS,oP,oV,oPWM,oFF,fl,lm   (same columns as before)
//     st   0 IDLE, 1..N = step (see #STATES), N+1 DONE    ph  1 = timed move active
//     aT/aC arm target / position as 0-1000 (650 = vertical, 5 per deg)
//     sT/sC/sO scoop target / position / output, us-equivalent (2300 = 0 deg, 6.667 per deg)
//     oT/oS/oP/oV slide target / setpoint / position (counts) / velocity (counts/s)
//     oPWM, oFF  motor command and its feedforward part
//     fl   bit0 arm moving, 1 scoop moving, 2 slide profiling, 3 vibrating,
//          4 stalled, 5 timed move, 6 coast, 7 open loop
//   S,t,STEP,code   step change     E,t,CODE,value   event     R,t,run_ms   finished
static inline void field(long v) { Serial.print(','); Serial.print(v); }
inline long logArm(float d) { return lround(LOG_ARM_ZERO - d * LOG_ARM_PER_DEG); }
inline long logScoop(float d) { return lround(LOG_SCOOP_ZERO + d * LOG_SCOOP_PER_DEG); }

void logTelemetry() {
  if (!TELEMETRY_ENABLED) return;
  const unsigned long now = millis();
  if ((now - lastLogMs) < LOG_INTERVAL_MS) return;
  lastLogMs = now;

  const MotorController::MotorMode mode = slide.getMode();
  uint8_t fl = 0;
  if (!arm.isFinished() || tmArm.active)     fl |= 1;
  if (!scoop.isFinished() || tmScoop.active) fl |= 2;
  if (!slide.isProfileFinished())   fl |= 4;
  if (scoop.isVibrating())          fl |= 8;
  if (slideStalled)                 fl |= 16;
  if (tmArm.active || tmScoop.active) fl |= 32;
  if (mode == MotorController::COAST)     fl |= 64;
  if (mode == MotorController::OPEN_LOOP) fl |= 128;

  Serial.print('D');
  field((long)now);
  field(stateCode());
  field((tmArm.active || tmScoop.active) ? 1 : 0);
  field(logArm(arm.getTarget()));
  field(logArm(arm.getCommand()));
  field(logScoop(scoop.getTarget()));
  field(logScoop(scoop.getCommand()));
  field(logScoop(scoop.getOutput()));
  field((long)slide.getTarget());
  field((long)slide.getMovingTarget());
  field((long)slide.getPosition());
  field((long)slide.getVelocity());
  field(slide.getCommandPower());
  field(slide.getFeedforward());
  field((long)fl);
  field((long)(loopMaxUs > 65535UL ? 65535UL : loopMaxUs));
  Serial.println();
  loopMaxUs = 0;
}

void printHeader() {
  Serial.print(F("#FW,")); Serial.print(FW_NAME);
  Serial.print(F(",schema=")); Serial.print(LOG_SCHEMA);
  Serial.print(F(",mode=")); Serial.print((int)RUN_MODE);
  Serial.print(F(",wait_for_go=")); Serial.println(WAIT_FOR_GO ? 1 : 0);
  Serial.println(F("#COLS,t,st,ph,aT,aC,sT,sC,sO,oT,oS,oP,oV,oPWM,oFF,fl,lm"));
  Serial.print(F("#STATES,IDLE"));
  for (uint8_t i = 0; i < NUM_STEPS; i++) { Serial.print(','); printStepName(i); }
  Serial.println(F(",DONE"));
}

// prints v with dp decimals using integer maths (avoids the float printer)
void printFixed(float v, uint8_t dp) {
  long m = 1;
  for (uint8_t i = 0; i < dp; i++) m *= 10;
  long n = lround(v * m);
  if (n < 0) { Serial.print('-'); n = -n; }
  Serial.print(n / m);
  if (dp) {
    Serial.print('.');
    long f = n % m;
    for (long d = m / 10; d > 1 && f < d; d /= 10) Serial.print('0');
    Serial.print(f);
  }
}

static void cfgF(const __FlashStringHelper* k, float v) {
  Serial.print(F("#CFG,")); Serial.print(k); Serial.print(','); printFixed(v, 4); Serial.println();
}

void printConfig() {
  cfgF(F("RUN_MODE"), RUN_MODE);
  cfgF(F("SPEED_PCT"), speedK * 100.0f);
  // keys the run viewer uses to turn the log columns into degrees / mm
  cfgF(F("ARM_VERTICAL_POS"), LOG_ARM_ZERO);
  cfgF(F("ARM_UNITS_PER_DEG"), LOG_ARM_PER_DEG);
  cfgF(F("SCOOP_ZERO_US"), LOG_SCOOP_ZERO);
  cfgF(F("SCOOP_US_PER_DEG"), LOG_SCOOP_PER_DEG);
  cfgF(F("OUTTAKE_MM_PER_TICK"), SLIDE_MM_PER_COUNT);
  cfgF(F("OUTTAKE_EXTEND_TICKS"), mmToCounts(X_TIP));
  cfgF(F("OUTTAKE_MAX_PWM"), SLIDE_MAX_PWM);
  // servo calibration
  cfgF(F("CAL_ARM_DEG"), CAL_ARM_DEG);
  cfgF(F("CAL_SCOOP_DEG"), CAL_SCOOP_DEG);
  cfgF(F("ARM_R_CAL_US"), ARM_R_CAL_US);
  cfgF(F("ARM_L_CAL_US"), ARM_L_CAL_US);
  cfgF(F("SCOOP_CAL_US"), SCOOP_CAL_US);
  cfgF(F("ARM_US_PER_DEG_REAL"), ARM_US_PER_DEG);
  cfgF(F("SCOOP_US_PER_DEG_REAL"), SCOOP_US_PER_DEG);
  cfgF(F("X_CREEP"), X_CREEP);
  cfgF(F("X_TIP"), X_TIP);
  // slide gains (counts)
  cfgF(F("OUTTAKE_KP"), SLIDE_KP);
  cfgF(F("OUTTAKE_KI"), SLIDE_KI);
  cfgF(F("OUTTAKE_KD"), SLIDE_KD);
  cfgF(F("OUTTAKE_KS"), SLIDE_KS);
  cfgF(F("OUTTAKE_KV"), SLIDE_KV);
  cfgF(F("OUTTAKE_KA"), SLIDE_KA);
  cfgF(F("OUTTAKE_KG"), SLIDE_KG);
  Serial.println(F("#END_CFG"));
}

// =============================================================================
// SERIAL CONSOLE
// =============================================================================
//   x            stop everything (always works)
//   g            run the sequence (from FIRST_STEP)       r <n>  run from step n
//   t            toggle step mode (pause after each step)  n      next step
//   k <pct>      speed % for the next run (25 = slow motion)
//   l            list steps                                p      print config
// Jog / live link (console mode, or while WAIT_FOR_GO is waiting):
//   A <deg>      arm to angle (profiled)     S <deg>   scoop to angle (profiled)
//   Q            print pose: #POSE,arm,scoop,slide_mm
//   o <mm> [mm/s] slide to mm (profiled)     j <pwm> <ms>  open-loop slide jog
//   z            zero slide encoder here     c         coast slide     v  toggle vibration
void printHelp() {
  Serial.println(F("# x stop | g run | r <n> run from | t step mode | n next | k <pct> speed | l list | p cfg"));
  Serial.println(F("# A <deg> arm | S <deg> scoop | Q pose | o <mm> [mm/s] | j <pwm> <ms> | z zero | c coast | v vib"));
}

void listSteps() {
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    Serial.print(F("#STEP,")); Serial.print(i); Serial.print(','); printStepName(i); Serial.println();
  }
}

void printPose() {
  Serial.print(F("#POSE,")); printFixed(arm.getCommand(), 1);
  Serial.print(','); printFixed(scoop.getCommand(), 1);
  Serial.print(','); printFixed(slideMm(), 1);
  Serial.println();
}

// small number parser ("-12.5"); strtod costs ~700 bytes of flash
float parseNum(char*& p, bool& ok) {
  while (*p == ' ') p++;
  bool neg = false;
  if (*p == '-') { neg = true; p++; } else if (*p == '+') p++;
  float v = 0.0f, scale = 0.0f;
  ok = false;
  for (; *p; p++) {
    if (*p >= '0' && *p <= '9') {
      ok = true;
      if (scale == 0.0f) v = v * 10.0f + (*p - '0');
      else { v += (*p - '0') * scale; scale *= 0.1f; }
    } else if (*p == '.' && scale == 0.0f) scale = 0.1f;
    else break;
  }
  return neg ? -v : v;
}

void processCommand(char* cmd) {
  while (*cmd == ' ') cmd++;
  const char op = cmd[0];
  char* p = cmd + 1;
  bool hasA1, hasA2;
  const float a1 = parseNum(p, hasA1);
  const float a2 = parseNum(p, hasA2);

  if (op == 'x') {
    stopEverything();
    logEvent(F("ABORT"), 0);
    if (runState == R_RUNNING || runState == R_PAUSED) finishRun();
    return;
  }
  if (op == 'n' && runState == R_PAUSED) {
    runState = R_RUNNING;
    if (stepIndex >= lastStep) finishRun(); else enterStep(stepIndex + 1);
    return;
  }
  if (op == 't') { stepMode = !stepMode; logEvent(F("STEP_MODE"), stepMode); return; }

  const bool idle = (runState == R_IDLE || runState == R_DONE);
  const bool consoleOk = (RUN_MODE == RUN_CONSOLE) || waitingForGo;
  if (op == 'g' && idle && consoleOk) {
    waitingForGo = false;
    startRun(FIRST_STEP);
    return;
  }
  if (!idle) { Serial.println(F("# busy: x to stop")); return; }
  if (!consoleOk) return;              // competition run finished: ignore
  if ((op == 'A' || op == 'S' || op == 'o' || op == 'j' || op == 'k' || op == 'r') && !hasA1) {
    Serial.println(F("# needs a number, ? for help"));
    return;
  }
  switch (op) {
    case '?': printHelp(); break;
    case 'p': printConfig(); break;
    case 'l': listSteps(); break;
    case 'r': waitingForGo = false; startRun((uint8_t)a1); break;
    case 'k': {
      float k = a1 / 100.0f;
      if (k < 0.05f) k = 0.05f;
      if (k > 1.0f) k = 1.0f;
      speedK = k;
      logEvent(F("SPEED_PCT"), (long)(k * 100));
      break;
    }
    case 'A':
      arm.setLimits(ARM_SPEED * 0.5f, ARM_ACCEL * 0.25f);      // jogs at half speed
      arm.moveTo(a1);
      logEvent(F("CMD_A"), (long)(a1 * 10));
      break;
    case 'S':
      scoop.setLimits(SCOOP_SPEED * 0.5f, SCOOP_ACCEL * 0.25f);
      scoop.moveTo(a1);
      logEvent(F("CMD_S"), (long)(a1 * 10));
      break;
    case 'Q': printPose(); break;
    case 'o': {
      const float v = (a2 > 0.0f) ? a2 : SLIDE_FAST;
      slide.setProfileLimits(mmToCounts(v), mmToCounts(SLIDE_ACCEL));
      slide.setTarget(mmToCounts(a1 < 0.0f ? 0.0f : a1), SLIDE_MAX_PWM);
      jogEndMs = 0;
      logEvent(F("CMD_O"), (long)a1);
      break;
    }
    case 'j': {
      const long ms = (a2 > 0 && a2 <= 2000) ? (long)a2 : 200;
      slide.setOpenLoop((int)constrain((long)a1, -255L, 255L));
      jogEndMs = millis() + ms;
      logEvent(F("CMD_J"), (long)a1);
      break;
    }
    case 'z': slide.begin(0.0f); slide.setMode(MotorController::COAST); logEvent(F("CMD_Z"), 0); break;
    case 'c': stopEverything(); logEvent(F("CMD_C"), 0); break;
    case 'v':
      if (scoop.isVibrating()) scoop.stopVibration();
      else scoop.startVibration(VIBRATE_DEG, VIBRATE_HALF_PERIOD_MS);
      logEvent(F("CMD_V"), scoop.isVibrating());
      break;
    default: Serial.println(F("# ? for help")); break;
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

// =============================================================================
// SETUP / LOOP
// =============================================================================
void configureSlide() {
  slide.setUnitsTicks();
  slide.setEncoderDirection(SLIDE_ENCODER_DIR);
  slide.setMotorDirection(SLIDE_MOTOR_DIR);
  slide.setPID(SLIDE_KP, SLIDE_KI, SLIDE_KD);
  slide.setFeedforward(SLIDE_KS, SLIDE_KV, SLIDE_KA, SLIDE_KG);
  slide.setProfileLimits(mmToCounts(SLIDE_FAST), mmToCounts(SLIDE_ACCEL));
  slide.setMaxPwm(SLIDE_MAX_PWM);
  slide.setTolerance(mmToCounts(SLIDE_TOLERANCE_MM));
  slide.begin(0.0f);                          // slides are fully in at power-up
  slide.setMode(MotorController::COAST);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  printHeader();
  pinMode(END_LED_PIN, OUTPUT);
  digitalWrite(END_LED_PIN, LOW);

  servoDriverOk = pcaBegin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  if (!servoDriverOk) logEvent(F("NO_SERVO_DRIVER"), 0);

  arm.setWriteInterval(SERVO_WRITE_INTERVAL_MS);
  scoop.setWriteInterval(SERVO_WRITE_INTERVAL_MS);
  arm.setNativeSpeed(SERVO_NATIVE_DEG_S / ARM_GEAR_RATIO);
  scoop.setNativeSpeed(SERVO_NATIVE_DEG_S);
  arm.begin(A_START);                         // power-on pose (robot is set up here)
  scoop.begin(S_START);
  configureSlide();

  if (!servoDriverOk && RUN_MODE == RUN_FULL) { finishRun(); return; }

  if (RUN_MODE == RUN_CONSOLE) {
    printConfig();
    listSteps();
    printHelp();
    return;
  }
  if (WAIT_FOR_GO) {
    waitingForGo = true;
    Serial.println(F("# WAIT_FOR_GO: holding start pose. g = run, ? = jog commands."));
    return;
  }
  startRun(FIRST_STEP);
}

void loop() {
  const unsigned long loopStartUs = micros();
  handleSerial();
  const unsigned long now = millis();

  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    logEvent(F("I2C_TIMEOUT"), 0);
  }

  slide.update();
  updateSlideMonitors();

  if (jogEndMs != 0 && (long)(now - jogEndMs) >= 0) {
    jogEndMs = 0;
    slide.setMode(MotorController::COAST);
  }

  if ((runState == R_RUNNING || runState == R_PAUSED) && (now - runStartMs) > RUN_WATCHDOG_MS) {
    stopEverything();
    logEvent(F("WATCHDOG"), (long)(now - runStartMs));
    finishRun();
  }

  if (waitingForGo) digitalWrite(END_LED_PIN, ((now / 250) % 2) ? HIGH : LOW);

  updateRunner();
  arm.update();
  scoop.update();
  logTelemetry();

  const unsigned long loopUs = micros() - loopStartUs;
  if (loopUs > loopMaxUs) loopMaxUs = loopUs;
}
