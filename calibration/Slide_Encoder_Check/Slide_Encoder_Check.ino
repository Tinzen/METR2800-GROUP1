// =============================================================================
// SLIDE ENCODER CHECK - shows how far the slide has moved, in mm
// =============================================================================
// Serial Monitor: 115200 baud. Type a command in the box at the top, press Enter.
//
//   z          zero here (do this with the slide fully retracted)
//   +          short motor pulse OUT
//   -          short motor pulse IN
//   c 300      calibrate: zero, move the slide exactly 300 mm by hand
//              (measure with a ruler), then type "c 300". Prints the
//              MM_PER_COUNT to paste into the code below.
//
// Output:  pos  152.3 mm   (count 4210)
//
// Checks:
//   Push slide OUT by hand  -> mm goes UP     (if not: swap encoder A/B wires,
//                                              or flip OUTTAKE_ENCODER_DIRECTION)
//   Type +                  -> slide goes OUT (if not: swap motor wires,
//                                              or flip OUTTAKE_MOTOR_DIRECTION)
//   Slide not moving        -> count must NOT change. If it drifts, the encoder
//                              wiring is bad (ground / 5 V / noise).
//
// WARNING: this does not park the arm. Check the slide path is clear first.
// =============================================================================

#include <Encoder.h>

const int MOTOR_A = 8, MOTOR_B = 9, MOTOR_PWM = 11;   // same pins as V2_1
Encoder enc(2, 3);                                     // encoder on D2 / D3

const int PULSE_PWM = 100;   // 0-255, keep low for testing
const int PULSE_MS  = 200;

// mm of slide travel per encoder count. Set this with the "c" command.
// pi * 36.15 mm spool / 384.5 counts per spool rev (1:1 gearing since 16 Oct) = 0.2954 mm/count.
// Wrong if the slide is cascade-rigged; use "c" to check with a ruler.
float MM_PER_COUNT = 0.2954;   // 36.15 mm spool (measured), 1:1. Extending reads NEGATIVE here (raw count).

void motor(int pwm) {
  digitalWrite(MOTOR_A, pwm > 0);
  digitalWrite(MOTOR_B, pwm < 0);
  analogWrite(MOTOR_PWM, abs(pwm));
}

void pulse(int pwm) {
  motor(pwm);
  delay(PULSE_MS);
  motor(0);
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_A, OUTPUT); pinMode(MOTOR_B, OUTPUT); pinMode(MOTOR_PWM, OUTPUT);
  motor(0);
  Serial.println("Commands: z = zero, + = out, - = in, c <mm> = calibrate");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "z") { enc.write(0); Serial.println("zeroed"); }
    else if (cmd == "+") pulse(+PULSE_PWM);
    else if (cmd == "-") pulse(-PULSE_PWM);
    else if (cmd.startsWith("c")) {
      float mm = cmd.substring(1).toFloat();
      long count = enc.read();
      if (mm > 0 && count != 0) {
        MM_PER_COUNT = mm / count;
        Serial.print("Calibrated. Paste into code:  float MM_PER_COUNT = ");
        Serial.print(MM_PER_COUNT, 5); Serial.println(";");
      } else Serial.println("zero first, move the slide, then e.g. c 300");
    }
  }

  // print 5x per second
  static unsigned long last = 0;
  if (millis() - last >= 200) {
    last = millis();
    long count = enc.read();
    Serial.print("pos "); Serial.print(count * MM_PER_COUNT, 1);
    Serial.print(" mm   (count "); Serial.print(count);
    Serial.print(")   A="); Serial.print(digitalRead(2));     // raw encoder pins:
    Serial.print(" B="); Serial.println(digitalRead(3));      // flickering = noise
  }
}
