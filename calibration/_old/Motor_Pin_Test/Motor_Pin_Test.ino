// =============================================================================
// MOTOR PIN TEST - holds the motor driver signals steady so you can measure
// them with a multimeter. No encoder, no safety cut-outs.
// =============================================================================
// The motor driver (IRF3205 module) takes TWO signals per motor:
//   DIR1  direction: 5 V = one way, 0 V = the other way
//   PWM1  speed: switched on/off very fast. More "on" time = faster.
//
// SET THESE TWO PINS to wherever the DIR1 / PWM1 wires ACTUALLY go on the Nano.
// The schematic says D2 / D3, but the encoder is really on D2 / D3, so the
// schematic doesn't match the robot. Trace the wires and fill these in.
// PWM_PIN must be a PWM pin on the Nano: 3, 5, 6, 9, 10 or 11.
const int DIR_PIN = 6;
const int PWM_PIN = 11;

// Serial Monitor 115200, type then Enter:
//   1   forward  (DIR = 5 V, PWM = 50 %)
//   2   reverse  (DIR = 0 V, PWM = 50 %)
//   0   off
// Auto-off after 10 s so nothing cooks.
//
// Unplug the motor for the voltage checks. A stalled goBILDA motor draws a lot of current.
// =============================================================================

const int PWM_TEST = 128;          // 50 %: a multimeter on the PWM pin reads about 2.5 V
const unsigned long AUTO_OFF_MS = 10000;

unsigned long onSince = 0;
bool on = false;

void drive(int dir) {
  digitalWrite(DIR_PIN, dir > 0);
  analogWrite(PWM_PIN, dir == 0 ? 0 : PWM_TEST);
  on = (dir != 0);
  onSince = millis();
  if (dir > 0)  Serial.println("FORWARD: DIR=5V  PWM~2.5V  -> motor terminals ~half battery V");
  if (dir < 0)  Serial.println("REVERSE: DIR=0V  PWM~2.5V  -> motor terminals flip polarity");
  if (dir == 0) Serial.println("OFF: DIR=0V  PWM=0V");
}

void setup() {
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  drive(0);
  Serial.print("Motor pin test. DIR on D"); Serial.print(DIR_PIN);
  Serial.print(", PWM on D"); Serial.println(PWM_PIN);
  Serial.println("1 = forward, 2 = reverse, 0 = off");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') drive(+1);
    if (c == '2') drive(-1);
    if (c == '0') drive(0);
  }
  if (on && millis() - onSince > AUTO_OFF_MS) { Serial.println("auto-off"); drive(0); }
}
