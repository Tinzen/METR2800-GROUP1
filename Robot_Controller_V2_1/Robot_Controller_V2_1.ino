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
//   IDLE SCOOP_DOWN ARM_DOWN SWEEP_UP SWEEP_DOWN WAIT PICKUP | LIFT CARRY POUR VIBRATE |
//   EXTEND CREEP TIP_HOLD RETRACT | DONE
// Every arm, scoop and slide move is a trapezoid motion profile. Servos are open loop:
// "arrived" means their profile finished.
// =============================================================================

#include <Wire.h>
#include "MotorController.h"
#include "ServoAxis.h"
#include "RobotConfig.h"
#include "Sequence.h"

// =============================================================================
// TEST SETTINGS  (change these for bench work)
// =============================================================================
//   RUN_FULL     competition: runs the sequence on power-up. Leave it like this:
//                the sequence editor can still hold the robot still. At every
//                power-up the robot waits BENCH_WINDOW_MS for a computer to send
//                'b' (bench). The editor does that when you click Connect; with
//                no computer attached (competition) it runs as normal.
//   RUN_CONSOLE  always waits for serial commands (jog, single steps). Type ?
enum RunMode { RUN_FULL = 0, RUN_CONSOLE = 1 };
const RunMode RUN_MODE = RUN_FULL;
const unsigned long BENCH_WINDOW_MS = 150;   // added to every power-up

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
bool slideUsed = false;           // a step of this run has commanded the slides (WAIT_ARRIVED waits for them)
bool slideDone = true;            // the slides' last move has arrived (latched until the next slide command)
bool shortLogged = false;         // SETTLED_SHORT logged for this move
bool fallbackUsed = false;
bool resetting = false;           // a STEP_STOP row ended the run: going back to the start pose
unsigned long resetMs = 0;
bool waitingForGo = false;
bool consoleMode = (RUN_MODE == RUN_CONSOLE);   // true = bench: wait for commands
bool manualServos = false;                       // calibration: raw pulses, not degrees

// slides (mm in the sequence, counts in the controller)
float slideTargetMm = 0.0f;
float slideVEndMm = 0.0f;         // >0: this segment hands its speed to the next step
unsigned long slideTimeoutMs = 0, slideCmdMs = 0;
bool slideStalled = false, stallCond = false, dirBad = false;
bool settleCond = false, slideSettled = false;
unsigned long stallSinceMs = 0, dirBadSinceMs = 0, settledSinceMs = 0, jogEndMs = 0;
bool servoDriverOk = true;

unsigned long lastLogMs = 0, loopMaxUs = 0;
char cmdBuf[40];                  // longest command: P <arm> <scoop> <mm> <mm/s> <vib>
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
  if (manualServos) return;
  writeServoUs(ARM_R_OUTPUT, armRUs(deg));
  writeServoUs(ARM_L_OUTPUT, armLUs(deg));
}
void writeScoop(float deg) { if (!manualServos) writeServoUs(SCOOP_OUTPUT, scoopUs(deg)); }

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
// STEP RUNNER
// =============================================================================
void loadStep(uint8_t i, Step& s) { memcpy_P(&s, &SEQUENCE[i], sizeof(Step)); }

// the next row that runs after row i (STEP_SKIP rows don't), 255 = none (end of the run)
uint8_t __attribute__((noinline)) nextRunStep(int16_t i) {
  for (int16_t k = i + 1; k <= (int16_t)lastStep; k++)
    if (!(pgm_read_byte(&SEQUENCE[k].flags) & STEP_SKIP)) return (uint8_t)k;
  return 255;
}

float slideSpeedFor(const Step& s) {
  return ((s.slideSpeed > 0.0f) ? s.slideSpeed : SLIDE_FAST) * speedK;
}

void commandSlide(const Step& s, uint8_t i) {
  const float pos = slideMm();
  const float speed = slideSpeedFor(s);
  const bool retract = (s.slideSpeed == SLIDE_RETRACT_SPEED);
  const float accel = (retract ? SLIDE_RETRACT_ACCEL : SLIDE_ACCEL) * speedK * speedK;
  const float decel = (retract ? SLIDE_RETRACT_DECEL : SLIDE_DECEL) * speedK * speedK;

  // entry speed: handed over by the previous step (fast -> creep blend), or, if the previous
  // move is still running (an "arrived - ms" guard ended its step early), its speed now when it
  // goes the same way. Step mode pauses between steps, so it never blends.
  float vStart = stepMode ? 0.0f : slideVEndMm;
  if (!stepMode && slideUsed && !slideDone && slide.getMode() == MotorController::ACTIVE && !slide.isProfileFinished()) {
    const float pv = countsToMm(slide.getProfileVelocity()) * ((s.slide >= pos) ? 1.0f : -1.0f);
    vStart = (pv > 0.0f) ? pv : 0.0f;
  }

  // exit speed: blend into the next row if it keeps the slides going the same way
  float vEnd = 0.0f;
  const uint8_t ni = (s.flags & STEP_STOP) ? 255 : nextRunStep(i);
  if (!stepMode && (s.waitType == W_ARRIVED || s.waitType == W_SLIDE_DONE) && ni != 255) {
    Step nx;
    loadStep(ni, nx);
    if (!isHold(nx.slide)) {
      const float d1 = s.slide - pos, d2 = nx.slide - s.slide;
      if (d1 * d2 > 0.0f) {
        const float nSpeed = slideSpeedFor(nx);
        vEnd = (nSpeed < speed) ? nSpeed : speed;
      }
    }
  }

  slide.setProfileLimits(mmToCounts(speed), mmToCounts(accel), mmToCounts(decel));
  slide.setTargetBlend(mmToCounts(s.slide), SLIDE_MAX_PWM, mmToCounts(vStart), mmToCounts(vEnd));
  slideTargetMm = s.slide;
  slideUsed = true;
  slideVEndMm = vEnd;
  slideMovedThisStep = true;
  slideDone = false;
  shortLogged = false;
  slideCmdMs = millis();
  slideTimeoutMs = (unsigned long)(slide.getProfileTotalTime() * 1000.0f) + SLIDE_TIMEOUT_MARGIN_MS;
  stallCond = false;
  slideStalled = false;
}

// a row's ACC% / DEC%: 0 (left out) = 100 %
float __attribute__((noinline)) pctK(uint8_t p) { return p ? p * 0.01f : 1.0f; }

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
  // An axis this row moves gets a new profile, starting from where it is and how fast it is
  // going (a move still running is taken over smoothly). HOLD axes carry on.
  const float k2 = speedK * speedK;
  if (armMovedThisStep) {
    arm.setLimits(((cur.armSpeed > 0.0f) ? cur.armSpeed : ARM_SPEED) * speedK, ARM_ACCEL * k2 * pctK(cur.armAccPct), ARM_DECEL * k2 * pctK(cur.armDecPct));
    arm.moveTo(cur.arm);
  }
  if (scoopMovedThisStep) {
    scoop.setLimits(((cur.scoopSpeed > 0.0f) ? cur.scoopSpeed : SCOOP_SPEED) * speedK, SCOOP_ACCEL * k2 * pctK(cur.scoopAccPct), SCOOP_DECEL * k2 * pctK(cur.scoopDecPct));
    scoop.moveTo(cur.scoop);
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
  if (slideDone) return true;                 // latched: it got there (later steps don't re-judge it)
  if (!slide.isProfileFinished()) return false;
  if (slideVEndMm > 0.0f) return true;        // handing over to the next segment
  const float err = fabs(slideTargetMm - slideMm());
  if (err <= SLIDE_ARRIVE_TOL_MM) { slideDone = true; return true; }
  if (slideSettled && err <= SLIDE_SETTLED_TOL_MM) {
    if (!shortLogged) { shortLogged = true; logEvent(F("SETTLED_SHORT"), (long)(slideTargetMm - slideMm())); }
    slideDone = true;
    return true;
  }
  return false;
}

bool armDone()   { return arm.isFinished(); }
bool scoopDone() { return scoop.isFinished(); }
float armProgress() { return arm.getProgress(); }

// seconds until the slides' move is planned to finish (0 = done, or not used in this run)
float slideLeft() { return (slideUsed && !slideDone) ? slide.getProfileTimeLeft() : 0.0f; }

// seconds until the guard's axis (W_ARRIVED: the last of all three) is planned to arrive
float guardLeft(uint8_t w) {
  if (w == W_ARM_DONE) return arm.getTimeLeft();
  if (w == W_SCOOP_DONE) return scoop.getTimeLeft();
  const float s = slideLeft();
  if (w == W_SLIDE_DONE) return s;
  float left = arm.getTimeLeft();
  const float sc = scoop.getTimeLeft();
  if (sc > left) left = sc;
  if (s > left) left = s;
  return left;
}

bool guardArrived(uint8_t w) {
  if (w == W_ARM_DONE) return armDone();
  if (w == W_SCOOP_DONE) return scoopDone();
  if (w == W_SLIDE_DONE) return slideArrived();
  return armDone() && scoopDone() && (!slideUsed || slideArrived());   // every axis has stopped
}

bool stepConditionMet() {
  const unsigned long now = millis();
  const float el = (float)(now - stepStartMs);
  const float wv = cur.waitValue / speedK;   // ms (arrived guards: negative = before arriving)
  switch (cur.waitType) {
    case W_MS:
      return el >= wv;
    case W_ARRIVED:
    case W_ARM_DONE:
    case W_SCOOP_DONE:
    case W_SLIDE_DONE:
      if (wv < 0.0f) return guardLeft(cur.waitType) * 1000.0f <= -wv;   // that long BEFORE arriving
      if (!guardArrived(cur.waitType)) { arrivedAtMs = 0; return false; }
      if (arrivedAtMs == 0) arrivedAtMs = now ? now : 1UL;   // (not now | 1: one ms in the future, so the wait wrapped)
      return (float)(now - arrivedAtMs) >= wv;
    case W_ARM:
      return armProgress() * 100.0f >= cur.waitValue;
    case W_SCOOP:
      return scoop.getProgress() * 100.0f >= cur.waitValue;
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

void endManualServos(float armDeg, float scoopDeg);
// console jogs and the STEP_STOP reset share these (flash): servos at half speed / quarter accel, slides at v mm/s
void __attribute__((noinline)) jogServo(ServoAxis& ax, float speed, float acc, float dec, float deg) {
  endManualServos(CAL_ARM_DEG, CAL_SCOOP_DEG);
  ax.setLimits(speed * 0.5f, acc * 0.25f, dec * 0.25f);
  ax.moveTo(deg);
}
void __attribute__((noinline)) slideGo(float mm, float v) {
  slide.setProfileLimits(mmToCounts(v), mmToCounts(SLIDE_ACCEL), mmToCounts(SLIDE_DECEL));
  slide.setTarget(mmToCounts(mm < 0.0f ? 0.0f : mm), SLIDE_MAX_PWM);
  jogEndMs = 0;
}

// STEP_STOP: back to the start pose like the editor's Reset (servos at jog speed, slides to 0 at 300 mm/s), then DONE
void __attribute__((noinline)) startReset() {
  resetting = true;
  resetMs = millis();
  logEvent(F("STOP_HERE"), stepIndex);
  scoop.stopVibration();
  jogServo(arm, ARM_SPEED, ARM_ACCEL, ARM_DECEL, A_START);
  jogServo(scoop, SCOOP_SPEED, SCOOP_ACCEL, SCOOP_DECEL, S_START);
  slideGo(0.0f, 300.0f);
}

void updateRunner() {
  if (runState != R_RUNNING) return;

  if (resetting) {   // done when the moves are (8 s at most), or the slides stall (zero not quite at the stop)
    if ((armDone() && scoopDone() && slide.isProfileFinished()) || slideStalled || millis() - resetMs > 8000UL) finishRun();
    return;
  }

  if (slideUsed && !slideDone) {      // a slide move still on its way (it may have started in an earlier step)
    const unsigned long el = millis() - slideCmdMs;
    if (slideStalled && el >= STALL_IGNORE_START_MS) { slideFault(F("STALL")); return; }
    if (el >= slideTimeoutMs && !slideArrived()) { slideFault(F("TIMEOUT")); return; }
  }

  if (!stepConditionMet()) {
    // OR AFTER: something is stuck (or slower than planned): carry on after the step's time limit
    if (cur.limitMs == 0 || (millis() - stepStartMs) < (unsigned long)(cur.limitMs / speedK)) return;
    logEvent(F("STEP_LIMIT"), stepIndex);
  }

  if (cur.flags & STEP_STOP) { startReset(); return; }
  const uint8_t nx = nextRunStep(stepIndex);
  if (nx == 255) { finishRun(); return; }
  if (stepMode) {
    runState = R_PAUSED;
    Serial.print(F("# paused after ")); printStepName(stepIndex); Serial.println(F(" - n = next, x = stop"));
    return;
  }
  enterStep(nx);
}

void startRun(uint8_t first) {
  if (first >= NUM_STEPS) first = 0;
  lastStep = (LAST_STEP >= NUM_STEPS) ? NUM_STEPS - 1 : LAST_STEP;
  if (first > lastStep) lastStep = NUM_STEPS - 1;
  runStartMs = millis();
  fallbackUsed = false;
  slideVEndMm = 0.0f;
  slideUsed = false;
  slideDone = true;                                // nothing commanded yet this run
  slideTargetMm = slideMm();                       // wherever a jog left them
  digitalWrite(END_LED_PIN, LOW);
  runState = R_RUNNING;
  resetting = false;
  logEvent(F("RUN_START"), first);
  const uint8_t f = nextRunStep((int16_t)first - 1);   // a skipped first row: start at the next one
  if (f == 255) { finishRun(); return; }
  enterStep(f);
}

void printConfig();

void finishRun() {
  runState = R_DONE;
  resetting = false;
  slide.setMode(MotorController::COAST);
  scoop.stopVibration();
  arm.jumpTo(arm.getCommand());          // freeze anything cut short
  scoop.jumpTo(scoop.getCommand());
  digitalWrite(END_LED_PIN, HIGH);
  const unsigned long now = millis();
  Serial.print(F("S,")); Serial.print(now); Serial.print(F(",DONE,")); Serial.println(stateCode());
  Serial.print(F("R,")); Serial.print(now); Serial.print(','); Serial.println(now - runStartMs);
  printConfig();
}

void stopEverything() {
  slide.setMode(MotorController::COAST);
  jogEndMs = 0;
  scoop.stopVibration();
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
//     st   0 IDLE, 1..N = step (see #STATES), N+1 DONE    ph  always 0 (was: timed move active)
//     aT/aC arm target / position as 0-1000 (650 = vertical, 5 per deg)
//     sT/sC/sO scoop target / position / output, us-equivalent (2300 = 0 deg, 6.667 per deg)
//     oT/oS/oP/oV slide target / setpoint / position (counts) / velocity (counts/s)
//     oPWM, oFF  motor command and its feedforward part
//     fl   bit0 arm moving, 1 scoop moving, 2 slide profiling, 3 vibrating,
//          4 stalled, 5 unused, 6 coast, 7 open loop
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
  if (!arm.isFinished())            fl |= 1;
  if (!scoop.isFinished())          fl |= 2;
  if (!slide.isProfileFinished())   fl |= 4;
  if (scoop.isVibrating())          fl |= 8;
  if (slideStalled)                 fl |= 16;
  if (mode == MotorController::COAST)     fl |= 64;
  if (mode == MotorController::OPEN_LOOP) fl |= 128;

  Serial.print('D');
  field((long)now);
  field(stateCode());
  field(0);
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
  cfgF(F("RUN_MODE"), consoleMode);
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
//   b            bench: stop, then wait for commands (also the power-up handshake)
// Servo calibration (sequence editor > Calibrate servos):
//   U <out> <us> raw pulse on PCA output 1-3  L <out>   limp (0 = all)
//   M [arm] [scoop]  back to degree control, holding that pose (default: the reference pose)
// Follow the editor's timeline (sequence editor > Robot follows timeline):
//   P <arm> <scoop> <mm> <mm/s> <vib>  servos straight to these angles (own speed),
//                slides re-planned toward mm, arriving at mm/s (mm < 0 = slides untouched);
//                vib > 0 = shake +/- deg, 0 = stop shaking.
//                Sent ~25 times a second while you scrub or play. No reply (keeps the link quiet).
void printHelp() {
  Serial.println(F("# x g r<n> t n k<%> l p b | A S Q o j z c v | U L M P: see README.md"));   // (short: flash)
}

void enterBench() {
  waitingForGo = false;
  consoleMode = true;
  Serial.println(F("#MODE,1"));
}

// Back to degree control after calibration. The servos are near the reference pose
// then, so start the axes there (no jump back to wherever they were before).
void endManualServos(float armDeg, float scoopDeg) {
  if (!manualServos) return;
  manualServos = false;
  arm.begin(armDeg);
  scoop.begin(scoopDeg);
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
  bool hasA1, hasA2, ok;
  const float a1 = parseNum(p, hasA1);
  const float a2 = parseNum(p, hasA2);
  const float a3m = parseNum(p, ok);     // P only (missing mm = -1: slides untouched; others 0)
  const float a3 = ok ? a3m : -1.0f;
  const float a4 = parseNum(p, ok);
  const float a5 = parseNum(p, ok);

  if (op == 'x') {
    stopEverything();
    logEvent(F("ABORT"), 0);
    if (runState == R_RUNNING || runState == R_PAUSED) finishRun();
    return;
  }
  if (op == 'n' && runState == R_PAUSED) {
    runState = R_RUNNING;
    const uint8_t nx = nextRunStep(stepIndex);
    if (nx == 255) finishRun(); else enterStep(nx);
    return;
  }
  if (op == 't') { stepMode = !stepMode; logEvent(F("STEP_MODE"), stepMode); return; }
  if (op == 'b') {                       // bench: stop whatever is happening, then wait
    stopEverything();
    if (runState == R_RUNNING || runState == R_PAUSED) finishRun();
    enterBench();
    return;
  }

  const bool idle = (runState == R_IDLE || runState == R_DONE);
  const bool consoleOk = consoleMode || waitingForGo || runState == R_DONE;   // after a run: jog OK
  if (op == 'g' && idle && consoleOk) {
    waitingForGo = false;
    startRun(FIRST_STEP);
    return;
  }
  if (!idle) { Serial.println(F("# busy: x to stop")); return; }
  if (!consoleOk) return;              // competition run finished: ignore
  if ((op == 'A' || op == 'S' || op == 'o' || op == 'j' || op == 'k' || op == 'r' || op == 'U' || op == 'L' || op == 'P') && !hasA1) {
    Serial.println(F("# needs a number, ? for help"));
    return;
  }
  switch (op) {
    case '?': printHelp(); break;
    case 'p': printConfig(); break;
    case 'l': listSteps(); break;
    case 'r': waitingForGo = false; endManualServos(CAL_ARM_DEG, CAL_SCOOP_DEG); startRun((uint8_t)a1); break;
    case 'k': {
      float k = a1 / 100.0f;
      if (k < 0.05f) k = 0.05f;
      if (k > 1.0f) k = 1.0f;
      speedK = k;
      logEvent(F("SPEED_PCT"), (long)(k * 100));
      break;
    }
    case 'A':
      jogServo(arm, ARM_SPEED, ARM_ACCEL, ARM_DECEL, a1);      // jogs at half speed
      logEvent(F("CMD_A"), (long)(a1 * 10));
      break;
    case 'S':
      jogServo(scoop, SCOOP_SPEED, SCOOP_ACCEL, SCOOP_DECEL, a1);
      logEvent(F("CMD_S"), (long)(a1 * 10));
      break;
    case 'Q': printPose(); break;
    case 'o': {
      slideGo(a1, (a2 > 0.0f) ? a2 : SLIDE_FAST);
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
    case 'U':                            // raw pulse: U <output 1-3> <us>
      if (a1 < 1 || a1 > 3 || !hasA2) { Serial.println(F("# U <1-3> <us>")); break; }
      manualServos = true;
      writeServoUs((int)a1, a2);
      logEvent(F("CMD_U"), (long)a1 * 10000L + (long)a2);
      break;
    case 'L':                            // limp: L <output 1-3>, 0 = all
      manualServos = true;
      for (uint8_t o = 1; o <= 3; o++) if (a1 == 0 || a1 == o) pcaSetPulse(o - 1, 4096);   // full-off bit
      logEvent(F("CMD_L"), (long)a1);
      break;
    case 'M': endManualServos(hasA1 ? a1 : CAL_ARM_DEG, hasA2 ? a2 : CAL_SCOOP_DEG); logEvent(F("CMD_M"), 0); break;
    case 'P': {                          // follow the editor's timeline (see the list above)
      endManualServos(a1, a2);
      arm.jumpTo(a1);                    // the servos move at their own speed (the editor allows for it)
      scoop.jumpTo(a2);
      if (a3 >= 0.0f) {                  // mm < 0 = leave the slides alone
        const float tgt = mmToCounts(a3);
        if (fabs(tgt - slide.getTarget()) > 1.0f) {        // re-plan only when the target moved
          const float dir = tgt - slide.getPosition();
          float pv = (slide.getMode() == MotorController::ACTIVE) ? slide.getProfileVelocity() * dir : 0.0f;
          float ev = mmToCounts(a4) * dir;                // > 0 = same way as the move
          slide.setProfileLimits(mmToCounts(SLIDE_RETRACT_SPEED), mmToCounts(SLIDE_RETRACT_ACCEL), mmToCounts(SLIDE_RETRACT_DECEL));
          slide.setTargetBlend(tgt, SLIDE_MAX_PWM, pv > 0.0f ? pv / fabs(dir) : 0.0f, ev > 0.0f ? ev / fabs(dir) : 0.0f);
          jogEndMs = 0;
        }
      }
      if (a5 > 0.0f) { if (!scoop.isVibrating()) scoop.startVibration(a5, VIBRATE_HALF_PERIOD_MS); }
      else scoop.stopVibration();
      break;
    }
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
  slide.setProfileLimits(mmToCounts(SLIDE_FAST), mmToCounts(SLIDE_ACCEL), mmToCounts(SLIDE_DECEL));
  slide.setMaxPwm(SLIDE_MAX_PWM);
  slide.setTolerance(mmToCounts(SLIDE_TOLERANCE_MM));
  slide.begin(0.0f);                          // slides are fully in at power-up
  slide.setMode(MotorController::COAST);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  printHeader();
  if (!consoleMode) {                         // power-up handshake: a computer may ask for bench
    Serial.flush();
    const unsigned long t0 = millis();
    while (millis() - t0 < BENCH_WINDOW_MS) {
      if (Serial.available() > 0 && Serial.read() == 'b') { consoleMode = true; break; }
    }
  }
  if (consoleMode) Serial.println(F("#MODE,1"));
  pinMode(END_LED_PIN, OUTPUT);
  digitalWrite(END_LED_PIN, LOW);

  servoDriverOk = pcaBegin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  if (!servoDriverOk) logEvent(F("NO_SERVO_DRIVER"), 0);

  arm.setWriteInterval(SERVO_WRITE_INTERVAL_MS);
  scoop.setWriteInterval(SERVO_WRITE_INTERVAL_MS);
  arm.begin(A_START);                         // power-on pose (robot is set up here)
  scoop.begin(S_START);
  configureSlide();

  if (!servoDriverOk && !consoleMode) { finishRun(); return; }

  if (consoleMode) {
    printConfig();
    listSteps();
    printHelp();
    return;
  }
  if (WAIT_FOR_GO) {
    waitingForGo = true;
    Serial.println(F("# waiting: g = run"));
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
