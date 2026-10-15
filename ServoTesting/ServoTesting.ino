#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "RobotConfig.h"
#include "ServoAxis.h"

// ==================================================
// PCA9685 SETUP
// ==================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_FREQ = 50;

// User-facing outputs:
// Output 1 = Right arm
// Output 2 = Left arm
// Output 3 = Intake scoop
//
// RobotConfig.h must define:
// ARM_R_OUTPUT, ARM_L_OUTPUT, SCOOP_OUTPUT

const int ARM_R = 1;
const int ARM_L = 2;
const int SCOOP = 3;

// ==================================================
// TESTING LIMITS
// ==================================================

// These are software limits in DEGREES.
//
// Adjust these to the safe operating range of your
// mechanism. The pulse-width conversion also enforces
// PULSE_ABS_MIN and PULSE_ABS_MAX from RobotConfig.h.

const float TEST_MIN_DEG = 0.0f;
const float TEST_MAX_DEG = 180.0f;

// ==================================================
// ADJUSTMENT SETTINGS
// ==================================================

int stepSize = 1;

int selectedChannel = ARM_R;

float currentTargetDeg = 90.0f;

bool positionSet = false;

// Coordinated arm position:
// -1 means no coordinated position has been commanded.
int armPosition = -1;

// ==================================================
// FIXED-SIZE SERIAL INPUT BUFFER
// ==================================================

const byte COMMAND_BUFFER_SIZE = 32;

char commandBuffer[COMMAND_BUFFER_SIZE];

byte commandLength = 0;

bool commandOverflow = false;

// ==================================================
// PCA9685 OUTPUT
// ==================================================

// Convert pulse width in microseconds to PCA9685 counts.
//
// At 50 Hz, the PWM period is 20,000 microseconds.

uint16_t microsecondsToCounts(float us) {
  return (uint16_t)(
      us * 4096.0f / 20000.0f + 0.5f
  );
}

// Set a PCA9685 channel directly.
//
// channel is zero-based: 0-15.

void pcaSetPulse(int channel, uint16_t counts) {
  if (channel < 0 || channel > 15) {
    return;
  }

  if (counts > 4095) {
    counts = 4095;
  }

  pwm.setPWM(channel, 0, counts);
}

// Write a pulse width to a user-facing servo output.
//
// output is one-based: 1-16.

void writeServoUs(int output, float us) {
  if (us < PULSE_ABS_MIN) {
    us = PULSE_ABS_MIN;
  }

  if (us > PULSE_ABS_MAX) {
    us = PULSE_ABS_MAX;
  }

  pcaSetPulse(
      output - 1,
      microsecondsToCounts(us)
  );
}

// ==================================================
// CALIBRATION CONVERSIONS
// ==================================================

// Convert an arm angle to its calibrated pulse width.
//
// These functions use the calibration constants from
// RobotConfig.h.

float armRUs(float deg) {
  return ARM_R_CAL_US
      + ARM_R_DIR
      * ARM_US_PER_DEG
      * (deg - CAL_ARM_DEG);
}

float armLUs(float deg) {
  return ARM_L_CAL_US
      + ARM_L_DIR
      * ARM_US_PER_DEG
      * (deg - CAL_ARM_DEG);
}

float scoopUs(float deg) {
  return SCOOP_CAL_US
      + SCOOP_DIR
      * SCOOP_US_PER_DEG
      * (deg - CAL_SCOOP_DEG);
}

// ==================================================
// SERVO WRITER FUNCTIONS
// ==================================================

// These functions receive ANGLES in degrees from
// ServoAxis and convert them into calibrated pulses.

// Right arm only.

void writeRightArm(float deg) {
  writeServoUs(
      ARM_R_OUTPUT,
      armRUs(deg)
  );
}

// Left arm only.

void writeLeftArm(float deg) {
  writeServoUs(
      ARM_L_OUTPUT,
      armLUs(deg)
  );
}

// Scoop only.

void writeScoop(float deg) {
  writeServoUs(
      SCOOP_OUTPUT,
      scoopUs(deg)
  );
}

// Optional helper to write both arms at the same
// calibrated logical angle.

void writeArm(float deg) {
  writeRightArm(deg);
  writeLeftArm(deg);
}

// ==================================================
// SERVO AXIS OBJECTS
// ==================================================

// ServoAxis operates in degrees.
//
// The calibration functions above convert degrees
// into pulse widths before writing to the PCA9685.

ServoAxis rightArmAxis(
    writeRightArm,
    TEST_MIN_DEG,
    TEST_MAX_DEG
);

ServoAxis leftArmAxis(
    writeLeftArm,
    TEST_MIN_DEG,
    TEST_MAX_DEG
);

ServoAxis scoopAxis(
    writeScoop,
    TEST_MIN_DEG,
    TEST_MAX_DEG
);

// Track whether each axis has been initialised.
//
// No ServoAxis.begin() calls are made automatically
// during setup, so no new servo positions are
// commanded when the board starts.

bool axisInitialized[3] = {
  false,
  false,
  false
};

// ==================================================
// GET SERVO AXIS
// ==================================================

ServoAxis& getServoAxis(int channel) {
  switch (channel) {
    case ARM_R:
      return rightArmAxis;

    case ARM_L:
      return leftArmAxis;

    default:
      return scoopAxis;
  }
}

// ==================================================
// GET SERVO NAME
// ==================================================

const __FlashStringHelper* getServoName(int channel) {
  switch (channel) {
    case ARM_R:
      return F("Right Arm");

    case ARM_L:
      return F("Left Arm");

    case SCOOP:
      return F("Intake Scoop");

    default:
      return F("Unknown");
  }
}

// ==================================================
// INITIALISE OR MOVE AN AXIS
// ==================================================

void commandServo(float targetDeg) {
  // Enforce software angle limits.

  if (targetDeg < TEST_MIN_DEG ||
      targetDeg > TEST_MAX_DEG) {

    Serial.println();
    Serial.println(F("ERROR: Angle outside software limits!"));

    Serial.print(F("Allowed range: "));
    Serial.print(TEST_MIN_DEG);
    Serial.print(F(" - "));
    Serial.print(TEST_MAX_DEG);
    Serial.println(F(" degrees"));

    Serial.println(F("Servo target unchanged."));
    Serial.println();

    return;
  }

  ServoAxis& axis = getServoAxis(selectedChannel);

  // The first command establishes the initial position.
  // Later commands use the motion profile.

  if (!axisInitialized[selectedChannel - 1]) {
    axis.begin(targetDeg);

    axisInitialized[selectedChannel - 1] = true;
  } else {
    axis.moveTo(targetDeg);
  }

  currentTargetDeg = targetDeg;
  positionSet = true;

  Serial.print(F("Servo: "));
  Serial.print(getServoName(selectedChannel));

  Serial.print(F(" | Output: "));
  Serial.print(selectedChannel);

  Serial.print(F(" | Target: "));
  Serial.print(targetDeg, 1);

  Serial.println(F(" degrees"));
}

// ==================================================
// COORDINATED ARM CONTROL
// ==================================================

// Position 0:
//   Both arms receive TEST_MIN_DEG.
//
// Position 1000:
//   Both arms receive TEST_MAX_DEG.
//
// The calibration functions independently convert
// the common logical angle for each physical servo.
//
// ARM_R_DIR and ARM_L_DIR determine their respective
// pulse-width directions.

bool setArms(int position) {
  position = constrain(position, 0, 1000);

  float targetDeg =
      TEST_MIN_DEG
      + (float)position
      * (TEST_MAX_DEG - TEST_MIN_DEG)
      / 1000.0f;

  // Validate before commanding either arm.

  if (targetDeg < TEST_MIN_DEG ||
      targetDeg > TEST_MAX_DEG) {

    Serial.println(F("ERROR: Invalid arm target."));
    return false;
  }

  // Initialise or move the right arm.

  if (!axisInitialized[ARM_R - 1]) {
    rightArmAxis.begin(targetDeg);

    axisInitialized[ARM_R - 1] = true;
  } else {
    rightArmAxis.moveTo(targetDeg);
  }

  // Initialise or move the left arm.

  if (!axisInitialized[ARM_L - 1]) {
    leftArmAxis.begin(targetDeg);

    axisInitialized[ARM_L - 1] = true;
  } else {
    leftArmAxis.moveTo(targetDeg);
  }

  armPosition = position;

  Serial.println();
  Serial.println(F("========== COORDINATED ARMS =========="));

  Serial.print(F("Position: "));
  Serial.print(position);
  Serial.println(F(" / 1000"));

  Serial.print(F("Target angle: "));
  Serial.print(targetDeg, 1);
  Serial.println(F(" degrees"));

  Serial.println(F("======================================"));
  Serial.println();

  return true;
}

// ==================================================
// PARSE A NON-NEGATIVE INTEGER
// ==================================================

bool parseInteger(const char* text, long& result) {
  if (*text == '\0') {
    return false;
  }

  // Only allow decimal digits.

  for (const char* p = text; *p != '\0'; p++) {
    if (!isdigit((unsigned char)*p)) {
      return false;
    }
  }

  char* endPointer;

  result = strtol(text, &endPointer, 10);

  if (*endPointer != '\0') {
    return false;
  }

  return true;
}

// ==================================================
// PRINT HELP
// ==================================================

void printHelp() {
  Serial.println();

  Serial.println(F("========== SERVO TESTING =========="));

  Serial.println();
  Serial.println(F("--- Select Servo ---"));

  Serial.println(F("s 1       Select right arm"));
  Serial.println(F("s 2       Select left arm"));
  Serial.println(F("s 3       Select intake scoop"));

  Serial.println();
  Serial.println(F("--- Set Target Angle ---"));

  Serial.println(F("p 90      Set target to 90 degrees"));
  Serial.println(F("+         Increase target angle"));
  Serial.println(F("-         Decrease target angle"));

  Serial.println();
  Serial.println(F("--- Coordinated Arm Control ---"));

  Serial.println(F("i 0       Move arms to minimum angle"));
  Serial.println(F("i 250     Move arms to 25% of range"));
  Serial.println(F("i 500     Move arms to 50% of range"));
  Serial.println(F("i 750     Move arms to 75% of range"));
  Serial.println(F("i 1000    Move arms to maximum angle"));

  Serial.println(F("i+        Increase coordinated position"));
  Serial.println(F("i-        Decrease coordinated position"));

  Serial.println();
  Serial.println(F("--- Adjustment ---"));

  Serial.println(F("step 1    Set step to 1 degree"));
  Serial.println(F("step 5    Set step to 5 degrees"));

  Serial.println();
  Serial.println(F("--- Information ---"));

  Serial.println(F("status    Show selected servo status"));
  Serial.println(F("?         Show help"));

  Serial.println();
  Serial.println(F("==================================="));
  Serial.println();
}

// ==================================================
// PROCESS SERIAL COMMANDS
// ==================================================

void processCommand(char* command) {
  // Remove leading whitespace.

  while (isspace((unsigned char)*command)) {
    command++;
  }

  // Remove trailing whitespace.

  size_t length = strlen(command);

  while (length > 0 &&
         isspace((unsigned char)command[length - 1])) {

    command[--length] = '\0';
  }

  if (length == 0) {
    return;
  }

  // ----------------------------------------------
  // HELP
  // ----------------------------------------------

  if (strcmp(command, "?") == 0) {
    printHelp();
    return;
  }

  // ----------------------------------------------
  // SELECT SERVO
  // ----------------------------------------------

  if (strncmp(command, "s ", 2) == 0) {
    long channel;

    if (!parseInteger(command + 2, channel) ||
        channel < 1 ||
        channel > 3) {

      Serial.println(F("Choose output 1, 2 or 3."));
      return;
    }

    selectedChannel = (int)channel;

    positionSet =
        axisInitialized[selectedChannel - 1];

    Serial.println();

    Serial.print(F("Selected servo: "));
    Serial.println(getServoName(selectedChannel));

    Serial.print(F("Output: "));
    Serial.println(selectedChannel);

    Serial.print(F("Software angle range: "));
    Serial.print(TEST_MIN_DEG, 1);
    Serial.print(F(" - "));
    Serial.print(TEST_MAX_DEG, 1);
    Serial.println(F(" degrees"));

    if (positionSet) {
      Serial.print(F("Current target: "));
      Serial.print(currentTargetDeg, 1);
      Serial.println(F(" degrees"));
    } else {
      Serial.println(
          F("No initial position commanded for this servo.")
      );
    }

    Serial.println();

    return;
  }

  // ----------------------------------------------
  // COORDINATED ARM POSITION
  // ----------------------------------------------

  if (strncmp(command, "i ", 2) == 0) {
    long position;

    if (!parseInteger(command + 2, position) ||
        position > 1000) {

      Serial.println(
          F("Enter an arm position from 0 to 1000.")
      );

      return;
    }

    setArms((int)position);
    return;
  }

  // ----------------------------------------------
  // INCREASE COORDINATED ARM POSITION
  // ----------------------------------------------

  if (strcmp(command, "i+") == 0) {
    if (armPosition < 0) {
      Serial.println(
          F("Set an initial position first, e.g. i 500.")
      );

      return;
    }

    int newPosition = armPosition + stepSize;

    if (newPosition > 1000) {
      newPosition = 1000;
    }

    setArms(newPosition);
    return;
  }

  // ----------------------------------------------
  // DECREASE COORDINATED ARM POSITION
  // ----------------------------------------------

  if (strcmp(command, "i-") == 0) {
    if (armPosition < 0) {
      Serial.println(
          F("Set an initial position first, e.g. i 500.")
      );

      return;
    }

    int newPosition = armPosition - stepSize;

    if (newPosition < 0) {
      newPosition = 0;
    }

    setArms(newPosition);
    return;
  }

  // ----------------------------------------------
  // SET INDIVIDUAL TARGET ANGLE
  // ----------------------------------------------

  if (strncmp(command, "p ", 2) == 0) {
    long angleDeg;

    if (!parseInteger(command + 2, angleDeg)) {
      Serial.println(
          F("Enter a whole-number angle in degrees.")
      );

      return;
    }

    commandServo((float)angleDeg);
    return;
  }

  // ----------------------------------------------
  // INCREASE INDIVIDUAL TARGET ANGLE
  // ----------------------------------------------

  if (strcmp(command, "+") == 0) {
    if (!positionSet) {
      Serial.println(
          F("Set an initial position first, e.g. p 90.")
      );

      return;
    }

    float newTarget = currentTargetDeg + stepSize;

    if (newTarget > TEST_MAX_DEG) {
      Serial.println(F("Maximum angle reached."));
      return;
    }

    commandServo(newTarget);
    return;
  }

  // ----------------------------------------------
  // DECREASE INDIVIDUAL TARGET ANGLE
  // ----------------------------------------------

  if (strcmp(command, "-") == 0) {
    if (!positionSet) {
      Serial.println(
          F("Set an initial position first, e.g. p 90.")
      );

      return;
    }

    float newTarget = currentTargetDeg - stepSize;

    if (newTarget < TEST_MIN_DEG) {
      Serial.println(F("Minimum angle reached."));
      return;
    }

    commandServo(newTarget);
    return;
  }

  // ----------------------------------------------
  // CHANGE ADJUSTMENT STEP
  // ----------------------------------------------

  if (strncmp(command, "step ", 5) == 0) {
    long newStep;

    if (!parseInteger(command + 5, newStep) ||
        newStep < 1 ||
        newStep > 90) {

      Serial.println(
          F("Step must be between 1 and 90 degrees.")
      );

      return;
    }

    stepSize = (int)newStep;

    Serial.print(F("Adjustment step: "));
    Serial.print(stepSize);
    Serial.println(F(" degrees"));

    return;
  }

  // ----------------------------------------------
  // DISPLAY STATUS
  // ----------------------------------------------

  if (strcmp(command, "status") == 0) {
    Serial.println();

    Serial.println(F("========== SERVO STATUS =========="));

    Serial.print(F("Servo: "));
    Serial.println(getServoName(selectedChannel));

    Serial.print(F("Output: "));
    Serial.println(selectedChannel);

    Serial.print(F("Software angle range: "));
    Serial.print(TEST_MIN_DEG, 1);
    Serial.print(F(" - "));
    Serial.print(TEST_MAX_DEG, 1);
    Serial.println(F(" degrees"));

    Serial.print(F("Adjustment step: "));
    Serial.print(stepSize);
    Serial.println(F(" degrees"));

    if (axisInitialized[selectedChannel - 1]) {
      ServoAxis& axis = getServoAxis(selectedChannel);

      Serial.print(F("Target: "));
      Serial.print(axis.getTarget(), 1);
      Serial.println(F(" degrees"));

      Serial.print(F("Commanded position estimate: "));
      Serial.print(axis.getCommand(), 1);
      Serial.println(F(" degrees"));

      Serial.print(F("Last written angle: "));
      Serial.print(axis.getOutput(), 1);
      Serial.println(F(" degrees"));

      Serial.print(F("Motion complete: "));
      Serial.println(axis.isFinished() ? F("Yes") : F("No"));

    } else {
      Serial.println(
          F("No initial position commanded.")
      );
    }

    if (armPosition >= 0) {
      Serial.print(F("Coordinated arm position: "));
      Serial.print(armPosition);
      Serial.println(F(" / 1000"));
    } else {
      Serial.println(
          F("No coordinated arm position commanded.")
      );
    }

    Serial.println(F("=================================="));
    Serial.println();

    return;
  }

  // ----------------------------------------------
  // UNKNOWN COMMAND
  // ----------------------------------------------

  Serial.println(
      F("Unknown command. Enter ? for help.")
  );
}

// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);

  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(500);

  // Motion-profile settings.
  //
  // Units:
  //   velocity     = degrees per second
  //   acceleration = degrees per second squared
  //
  // Adjust these after testing.

  rightArmAxis.setLimits(60.0f, 120.0f);
  leftArmAxis.setLimits(60.0f, 120.0f);
  scoopAxis.setLimits(60.0f, 120.0f);

  Serial.println();

  Serial.println(F("Servo testing ready."));
  Serial.println(F("PCA9685 frequency: 50 Hz"));

  Serial.println(
      F("Servo positions will not be commanded automatically.")
  );

  Serial.println(
      F("ServoAxis motion profiling is enabled.")
  );

  Serial.println(
      F("Angles are converted using RobotConfig.h calibration.")
  );

  Serial.println(
      F("Pulse widths are clamped to the absolute configured limits.")
  );

  printHelp();
}

// ==================================================
// LOOP
// ==================================================

void loop() {
  // Update all initialised axes.
  //
  // These calls are non-blocking, allowing serial
  // commands to be processed while servos move.

  if (axisInitialized[ARM_R - 1]) {
    rightArmAxis.update();
  }

  if (axisInitialized[ARM_L - 1]) {
    leftArmAxis.update();
  }

  if (axisInitialized[SCOOP - 1]) {
    scoopAxis.update();
  }

  // Read serial input without blocking.

  while (Serial.available() > 0) {
    char incoming = Serial.read();

    if (incoming == '\n' || incoming == '\r') {
      if (commandOverflow) {
        Serial.println(
            F("Command too long. Enter a shorter command.")
        );

      } else if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';

        processCommand(commandBuffer);
      }

      // Reset the input buffer.

      commandLength = 0;
      commandOverflow = false;

    } else {
      if (!commandOverflow) {
        if (commandLength < COMMAND_BUFFER_SIZE - 1) {
          commandBuffer[commandLength++] = incoming;
        } else {
          commandOverflow = true;
        }
      }
    }
  }
}