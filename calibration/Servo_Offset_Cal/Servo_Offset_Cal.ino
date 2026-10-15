// =============================================================================
// SERVO OFFSET CAL - find the 3 servo pulse widths at the reference pose
// =============================================================================
// Reference pose = scoop laid FLAT on the intake floor (a hard stop).
// The CAD says that is ARM 110.6 deg, SCOOP -69.4 deg. The pulses you find
// here go into Robot_Controller_V2_1/RobotConfig.h (or the sequence editor's
// Calibration tab), so the twin's angles match the real robot.
//
// Serial Monitor: 115200 baud, "Newline". Battery ON. Full steps in
// calibration/README.md. Servos start LIMP (no pulse) until you type cal.
//
//   cal          all 3 servos to the reference pose (current numbers)
//   level        arm posts vertical + scoop blade level (scale check)
//   r <us>  l <us>  s <us>    set right arm / left arm / scoop pulse
//   r+ r-  l+ l-  s+ s-       nudge by the step size
//   a+ a-        nudge BOTH arm servos together (arm up/down, no fighting)
//   step <us>    nudge size (default 5)
//   off r|l|s|all   make that servo limp      on r|l|s|all   drive it again
//   p            print the numbers to paste
//   ?            help
// =============================================================================
#include <Wire.h>

// ---- copy of the servo section of RobotConfig.h (keep in sync) --------------
const int ARM_R_OUTPUT = 1, ARM_L_OUTPUT = 2, SCOOP_OUTPUT = 3;   // PCA9685 outputs
const float CAL_ARM_DEG = 110.6f;
const float CAL_SCOOP_DEG = -69.4f;
float armRUs = 940.0f;              // starting guesses = current RobotConfig.h values
float armLUs = 2060.0f;
float scoopUs = 1900.0f;
const float ARM_US_PER_DEG = 8.0f;
const float SCOOP_US_PER_DEG = 6.667f;
const int ARM_R_DIR = -1, ARM_L_DIR = 1, SCOOP_DIR = 1;
const int PULSE_MIN = 500, PULSE_MAX = 2500;

int stepUs = 5;
bool onR = false, onL = false, onS = false;   // start limp

// ---- minimal PCA9685 driver (same as V2.1) ----------------------------------
const uint8_t PCA_ADDR = 0x40;
bool pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool pcaBegin() {
  Wire.begin();
  if (!pcaWrite(0x00, 0x80)) return false;
  delay(10);
  pcaWrite(0x00, 0x10);
  pcaWrite(0xFE, 121);            // 50 Hz
  pcaWrite(0x00, 0x20);
  delay(5);
  pcaWrite(0x00, 0xA0);
  return true;
}
void pcaRaw(uint8_t ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x06 + 4 * ch);
  Wire.write(on & 0xFF); Wire.write(on >> 8);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  Wire.endTransmission();
}
void servoOut(int output, bool on, float us) {
  if (!on) { pcaRaw(output - 1, 0, 4096); return; }      // full-off bit: no pulse, servo limp
  if (us < PULSE_MIN) us = PULSE_MIN;
  if (us > PULSE_MAX) us = PULSE_MAX;
  pcaRaw(output - 1, 0, (uint16_t)(us * 4096.0f / 20000.0f + 0.5f));
}
void writeAll() {
  servoOut(ARM_R_OUTPUT, onR, armRUs);
  servoOut(ARM_L_OUTPUT, onL, armLUs);
  servoOut(SCOOP_OUTPUT, onS, scoopUs);
}


void show() {
  Serial.print(F("R ")); Serial.print(armRUs, 0); Serial.print(onR ? F("") : F(" (limp)"));
  Serial.print(F("   L ")); Serial.print(armLUs, 0); Serial.print(onL ? F("") : F(" (limp)"));
  Serial.print(F("   S ")); Serial.print(scoopUs, 0); Serial.print(onS ? F("") : F(" (limp)"));
  Serial.println(F(" us"));
}

void printPaste() {
  Serial.println(F("---- paste into Robot_Controller_V2_1/RobotConfig.h (or the editor's Calibration tab) ----"));
  Serial.print(F("const float ARM_R_CAL_US = ")); Serial.print(armRUs, 1); Serial.println(F("f;"));
  Serial.print(F("const float ARM_L_CAL_US = ")); Serial.print(armLUs, 1); Serial.println(F("f;"));
  Serial.print(F("const float SCOOP_CAL_US = ")); Serial.print(scoopUs, 1); Serial.println(F("f;"));
  Serial.println(F("-------------------------------------------------------------------------------------------"));
}

void help() {
  Serial.println(F("# cal | level | r/l/s <us> | r+ r- l+ l- s+ s- | a+ a- (both arm) | step <us> | off/on r|l|s|all | p | ?"));
}

// "level" pose: arm vertical (0 deg), scoop 0 deg (blade level, like the CAD)
float refR, refL, refS;      // reference pulses (kept so 'cal' returns there)
bool atLevel = false;

void goLevel() {
  refR = armRUs; refL = armLUs; refS = scoopUs;
  armRUs = refR + ARM_R_DIR * ARM_US_PER_DEG * (0.0f - CAL_ARM_DEG);
  armLUs = refL + ARM_L_DIR * ARM_US_PER_DEG * (0.0f - CAL_ARM_DEG);
  scoopUs = refS + SCOOP_DIR * SCOOP_US_PER_DEG * (0.0f - CAL_SCOOP_DEG);
  atLevel = true;
  onR = onL = onS = true;
  writeAll();
  Serial.println(F("# LEVEL pose: posts should be vertical, blade level (phone level). Nudge r/l/s until they are, then p."));
  Serial.println(F("# Enter these pulses in the editor's Calibration tab > Scale check. 'cal' goes back."));
  show();
}

void handle(char* c) {
  while (*c == ' ') c++;
  char* arg = c;
  while (*arg && *arg != ' ') arg++;
  while (*arg == ' ') arg++;
  const float v = atof(arg);

  if (!strncmp(c, "cal", 3)) {
    if (atLevel) { armRUs = refR; armLUs = refL; scoopUs = refS; atLevel = false; }
    onR = onL = onS = true;
    writeAll();
    Serial.println(F("# REFERENCE pose: scoop should lie flat on the floor. See README for the order to tune."));
    show();
  } else if (!strncmp(c, "level", 5)) goLevel();
  else if (!strncmp(c, "step", 4)) { if (v > 0) stepUs = (int)v; Serial.print(F("# step ")); Serial.println(stepUs); }
  else if (!strncmp(c, "off", 3) || !strncmp(c, "on", 2)) {
    const bool on = (c[1] == 'n');
    if (strstr(arg, "all")) onR = onL = onS = on;
    else if (arg[0] == 'r') onR = on;
    else if (arg[0] == 'l') onL = on;
    else if (arg[0] == 's') onS = on;
    writeAll(); show();
  } else if (c[0] == 'p') printPaste();
  else if (c[0] == '?') help();
  else if (c[0] == 'a' && (c[1] == '+' || c[1] == '-')) {
    // both arm servos together: + = arm further out/down (more degrees)
    const float d = (c[1] == '+') ? stepUs : -stepUs;
    armRUs += ARM_R_DIR * d; armLUs += ARM_L_DIR * d;
    writeAll(); show();
  } else if (c[0] == 'r' || c[0] == 'l' || c[0] == 's') {
    float* t = (c[0] == 'r') ? &armRUs : (c[0] == 'l') ? &armLUs : &scoopUs;
    if (c[1] == '+') *t += stepUs;
    else if (c[1] == '-') *t -= stepUs;
    else if (v >= PULSE_MIN && v <= PULSE_MAX) *t = v;
    else { Serial.println(F("# pulse must be 500-2500")); return; }
    writeAll(); show();
  } else help();
}

char buf[24];
uint8_t n = 0;

void setup() {
  Serial.begin(115200);
  Serial.println(F("# SERVO OFFSET CAL. Servos are limp. Push the scoop near the floor by hand, then type cal."));
  if (!pcaBegin()) Serial.println(F("# !! no PCA9685 on I2C (check wiring / battery)"));
  writeAll();
  help();
}

void loop() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (n) { buf[n] = 0; handle(buf); n = 0; } }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
}
