#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ==================================================
// PCA9685 SETUP
// ==================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_FREQ = 50;

// User-facing output numbers:
// Output 1 = Right intake
// Output 2 = Left intake
// Output 3 = Intake scoop
//
// The PCA9685 library uses channel numbers 0-15,
// so we subtract 1 when sending commands.

const int INTAKE_R = 1;
const int INTAKE_L = 2;
const int SCOOP    = 3;

// ==================================================
// INDIVIDUAL SERVO CALIBRATION LIMITS
// ==================================================

// IMPORTANT:
// These are example values, not verified limits.
// Replace them with the safe limits you measure.

const int INTAKE_R_MIN = 700;
const int INTAKE_R_MAX = 2300;

const int INTAKE_L_MIN = 700;
const int INTAKE_L_MAX = 2300;

const int SCOOP_MIN = 700;
const int SCOOP_MAX = 2300;

// ==================================================
// COORDINATED INTAKE CONTROL
// ==================================================

// These values define the coordinated intake range.
//
// At position 0:
//   Right intake = 700 us
//   Left intake  = 2300 us
//
// At position 1000:
//   Right intake = 2300 us
//   Left intake  = 700 us
//
// These are example endpoints. Verify that both
// servos can safely reach these pulse widths.

const int INTAKE_CLOSED_PULSE = 700;
const int INTAKE_OPEN_PULSE   = 2300;

// ==================================================
// ADJUSTMENT SETTINGS
// ==================================================

int stepSize = 10;

int selectedChannel = INTAKE_R;
int currentPulseUs = 1500;

bool positionSet = false;

// Current coordinated intake position.
// -1 means no coordinated position has been commanded.
int intakePosition = -1;

// ==================================================
// FIXED-SIZE SERIAL INPUT BUFFER
// ==================================================

const byte COMMAND_BUFFER_SIZE = 24;

char commandBuffer[COMMAND_BUFFER_SIZE];
byte commandLength = 0;
bool commandOverflow = false;

// ==================================================
// GET SERVO NAME
// ==================================================

const __FlashStringHelper* getServoName(int channel) {
  switch (channel) {
    case INTAKE_R:
      return F("Intake Right");

    case INTAKE_L:
      return F("Intake Left");

    case SCOOP:
      return F("Intake Scoop");

    default:
      return F("Unknown");
  }
}

// ==================================================
// GET SERVO MINIMUM
// ==================================================

int getServoMin(int channel) {
  switch (channel) {
    case INTAKE_R:
      return INTAKE_R_MIN;

    case INTAKE_L:
      return INTAKE_L_MIN;

    case SCOOP:
      return SCOOP_MIN;

    default:
      return 0;
  }
}

// ==================================================
// GET SERVO MAXIMUM
// ==================================================

int getServoMax(int channel) {
  switch (channel) {
    case INTAKE_R:
      return INTAKE_R_MAX;

    case INTAKE_L:
      return INTAKE_L_MAX;

    case SCOOP:
      return SCOOP_MAX;

    default:
      return 0;
  }
}

// ==================================================
// CONVERT MICROSECONDS TO PCA9685 COUNTS
// ==================================================

int microsecondsToCounts(int pulseUs) {
  // At 50 Hz, the PWM period is 20,000 microseconds.
  return (int)(pulseUs * 4096.0 / 20000.0 + 0.5);
}

// ==================================================
// MOVE ONE SERVO
// ==================================================

void setServo(int pulseUs) {
  int minPulse = getServoMin(selectedChannel);
  int maxPulse = getServoMax(selectedChannel);

  // Enforce individual servo limits.
  if (pulseUs < minPulse || pulseUs > maxPulse) {
    Serial.println();
    Serial.println(F("ERROR: Pulse width outside servo limits!"));

    Serial.print(F("Servo: "));
    Serial.println(getServoName(selectedChannel));

    Serial.print(F("Allowed range: "));
    Serial.print(minPulse);
    Serial.print(F(" - "));
    Serial.print(maxPulse);
    Serial.println(F(" us"));

    Serial.println(F("Servo position unchanged."));
    Serial.println();

    return;
  }

  int counts = microsecondsToCounts(pulseUs);

  // Convert user-facing output number to PCA9685 channel.
  pwm.setPWM(selectedChannel - 1, 0, counts);

  currentPulseUs = pulseUs;
  positionSet = true;

  Serial.print(F("Servo: "));
  Serial.print(getServoName(selectedChannel));

  Serial.print(F(" | Output: "));
  Serial.print(selectedChannel);

  Serial.print(F(" | Pulse: "));
  Serial.print(currentPulseUs);

  Serial.print(F(" us | Counts: "));
  Serial.println(counts);
}

// ==================================================
// COORDINATED INTAKE CONTROL
// ==================================================

// position = 0:
//   Right intake = INTAKE_CLOSED_PULSE
//   Left intake  = INTAKE_OPEN_PULSE
//
// position = 1000:
//   Right intake = INTAKE_OPEN_PULSE
//   Left intake  = INTAKE_CLOSED_PULSE
//
// The two servos receive mirrored pulse widths.
//
// Both pulse widths are checked against their individual
// calibration limits before either servo is commanded.

bool setIntakes(int position) {
  // Keep the requested position within 0-1000.
  position = constrain(position, 0, 1000);

  // Calculate the pulse-width range.
  const int pulseRange =
      INTAKE_OPEN_PULSE - INTAKE_CLOSED_PULSE;

  // Calculate the right intake pulse.
  int rightPulse =
      INTAKE_CLOSED_PULSE
      + (long)position * pulseRange / 1000;

  // Calculate the mirrored left intake pulse.
  int leftPulse =
      INTAKE_OPEN_PULSE
      - (long)position * pulseRange / 1000;

  // Check the right intake's individual limits.
  if (rightPulse < INTAKE_R_MIN ||
      rightPulse > INTAKE_R_MAX) {

    Serial.println(
      F("ERROR: Right intake pulse is outside its limits.")
    );

    Serial.println(F("Neither intake was commanded."));

    return false;
  }

  // Check the left intake's individual limits.
  if (leftPulse < INTAKE_L_MIN ||
      leftPulse > INTAKE_L_MAX) {

    Serial.println(
      F("ERROR: Left intake pulse is outside its limits.")
    );

    Serial.println(F("Neither intake was commanded."));

    return false;
  }

  // Convert pulse widths to PCA9685 counts.
  int rightCounts = microsecondsToCounts(rightPulse);
  int leftCounts = microsecondsToCounts(leftPulse);

  // Command the right intake on output 1.
  pwm.setPWM(INTAKE_R - 1, 0, rightCounts);

  // Command the left intake on output 2.
  pwm.setPWM(INTAKE_L - 1, 0, leftCounts);

  // Store the coordinated position.
  intakePosition = position;

  Serial.println();
  Serial.println(F("========== COORDINATED INTAKES =========="));

  Serial.print(F("Position: "));
  Serial.print(position);
  Serial.println(F(" / 1000"));

  Serial.print(F("Right intake: "));
  Serial.print(rightPulse);
  Serial.println(F(" us"));

  Serial.print(F("Left intake: "));
  Serial.print(leftPulse);
  Serial.println(F(" us"));

  Serial.println(F("=========================================="));
  Serial.println();

  return true;
}

// ==================================================
// PRINT HELP
// ==================================================

void printHelp() {
  Serial.println();
  Serial.println(F("========== SERVO CALIBRATION =========="));

  Serial.println();
  Serial.println(F("--- Select Individual Servo ---"));
  Serial.println(F("s 1       Select intake right"));
  Serial.println(F("s 2       Select intake left"));
  Serial.println(F("s 3       Select intake scoop"));

  Serial.println();
  Serial.println(F("--- Individual Position Control ---"));
  Serial.println(F("p 1500    Set pulse width in microseconds"));
  Serial.println(F("+         Increase pulse width"));
  Serial.println(F("-         Decrease pulse width"));

  Serial.println();
  Serial.println(F("--- Coordinated Intake Control ---"));
  Serial.println(F("i 0       Move intakes to position 0"));
  Serial.println(F("i 250     Move intakes to position 250"));
  Serial.println(F("i 500     Move intakes to position 500"));
  Serial.println(F("i 750     Move intakes to position 750"));
  Serial.println(F("i 1000    Move intakes to position 1000"));
  Serial.println(F("i+        Increase coordinated position"));
  Serial.println(F("i-        Decrease coordinated position"));

  Serial.println();
  Serial.println(F("--- Adjustment ---"));
  Serial.println(F("step 1    Set adjustment step to 1 us"));
  Serial.println(F("step 10   Set adjustment step to 10 us"));

  Serial.println();
  Serial.println(F("--- Information ---"));
  Serial.println(F("status    Show selected servo status"));
  Serial.println(F("?         Show help"));

  Serial.println();
  Serial.println(F("======================================="));
  Serial.println();
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
        channel < 1 || channel > 3) {

      Serial.println(F("Choose output 1, 2 or 3."));
      return;
    }

    selectedChannel = (int)channel;

    // Require a new initial position for this servo.
    positionSet = false;

    Serial.println();

    Serial.print(F("Selected servo: "));
    Serial.println(getServoName(selectedChannel));

    Serial.print(F("Output: "));
    Serial.println(selectedChannel);

    Serial.print(F("Minimum pulse: "));
    Serial.print(getServoMin(selectedChannel));
    Serial.println(F(" us"));

    Serial.print(F("Maximum pulse: "));
    Serial.print(getServoMax(selectedChannel));
    Serial.println(F(" us"));

    Serial.println();
    Serial.println(F("Set an initial position, e.g. p 1500."));

    return;
  }

  // ----------------------------------------------
  // COORDINATED INTAKE POSITION
  // ----------------------------------------------

  if (strncmp(command, "i ", 2) == 0) {
    long position;

    if (!parseInteger(command + 2, position) ||
        position > 1000) {

      Serial.println(
        F("Enter an intake position from 0 to 1000.")
      );

      return;
    }

    setIntakes((int)position);
    return;
  }

  // ----------------------------------------------
  // INCREASE COORDINATED INTAKE POSITION
  // ----------------------------------------------

  if (strcmp(command, "i+") == 0) {
    if (intakePosition < 0) {
      Serial.println(
        F("Set an initial intake position first, e.g. i 500.")
      );

      return;
    }

    int newPosition = intakePosition + stepSize;

    if (newPosition > 1000) {
      newPosition = 1000;
    }

    setIntakes(newPosition);
    return;
  }

  // ----------------------------------------------
  // DECREASE COORDINATED INTAKE POSITION
  // ----------------------------------------------

  if (strcmp(command, "i-") == 0) {
    if (intakePosition < 0) {
      Serial.println(
        F("Set an initial intake position first, e.g. i 500.")
      );

      return;
    }

    int newPosition = intakePosition - stepSize;

    if (newPosition < 0) {
      newPosition = 0;
    }

    setIntakes(newPosition);
    return;
  }

  // ----------------------------------------------
  // SET INDIVIDUAL PULSE WIDTH
  // ----------------------------------------------

  if (strncmp(command, "p ", 2) == 0) {
    long pulseUs;

    if (!parseInteger(command + 2, pulseUs) ||
        pulseUs > 32767) {

      Serial.println(
        F("Enter a valid positive pulse width in microseconds.")
      );

      return;
    }

    setServo((int)pulseUs);
    return;
  }

  // ----------------------------------------------
  // INCREASE INDIVIDUAL PULSE WIDTH
  // ----------------------------------------------

  if (strcmp(command, "+") == 0) {
    if (!positionSet) {
      Serial.println(
        F("Set an initial position first, e.g. p 1500.")
      );

      return;
    }

    long newPulse = (long)currentPulseUs + stepSize;

    if (newPulse > getServoMax(selectedChannel)) {
      Serial.println(
        F("Maximum pulse width reached. Position unchanged.")
      );

      return;
    }

    setServo((int)newPulse);
    return;
  }

  // ----------------------------------------------
  // DECREASE INDIVIDUAL PULSE WIDTH
  // ----------------------------------------------

  if (strcmp(command, "-") == 0) {
    if (!positionSet) {
      Serial.println(
        F("Set an initial position first, e.g. p 1500.")
      );

      return;
    }

    long newPulse = (long)currentPulseUs - stepSize;

    if (newPulse < getServoMin(selectedChannel)) {
      Serial.println(
        F("Minimum pulse width reached. Position unchanged.")
      );

      return;
    }

    setServo((int)newPulse);
    return;
  }

  // ----------------------------------------------
  // CHANGE ADJUSTMENT STEP
  // ----------------------------------------------

  if (strncmp(command, "step ", 5) == 0) {
    long newStep;

    if (!parseInteger(command + 5, newStep) ||
        newStep < 1 || newStep > 500) {

      Serial.println(
        F("Step must be an integer between 1 and 500 us.")
      );

      return;
    }

    stepSize = (int)newStep;

    Serial.print(F("Adjustment step: "));
    Serial.print(stepSize);
    Serial.println(F(" us"));

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

    Serial.print(F("Minimum pulse: "));
    Serial.print(getServoMin(selectedChannel));
    Serial.println(F(" us"));

    Serial.print(F("Maximum pulse: "));
    Serial.print(getServoMax(selectedChannel));
    Serial.println(F(" us"));

    Serial.print(F("Adjustment step: "));
    Serial.print(stepSize);
    Serial.println(F(" us"));

    if (positionSet) {
      Serial.print(F("Last commanded pulse: "));
      Serial.print(currentPulseUs);
      Serial.println(F(" us"));
    } else {
      Serial.println(
        F("No individual position commanded for this servo yet.")
      );
    }

    if (intakePosition >= 0) {
      Serial.print(F("Last coordinated intake position: "));
      Serial.print(intakePosition);
      Serial.println(F(" / 1000"));
    } else {
      Serial.println(
        F("No coordinated intake position commanded yet.")
      );
    }

    Serial.println(F("=================================="));
    Serial.println();

    return;
  }

  // ----------------------------------------------
  // UNKNOWN COMMAND
  // ----------------------------------------------

  Serial.println(F("Unknown command. Enter ? for help."));
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

  Serial.println();
  Serial.println(F("Servo calibration ready."));
  Serial.println(F("No servo movement commanded automatically."));
  Serial.println(F("Individual pulse limits are enabled."));
  Serial.println(F("Coordinated intake control is enabled."));
  Serial.println(
    F("IMPORTANT: Replace example limits with your measured values.")
  );

  printHelp();
}

// ==================================================
// LOOP
// ==================================================

void loop() {
  // Read serial data one character at a time.

  while (Serial.available() > 0) {
    char incoming = Serial.read();

    // Process a command when the user presses Enter.
    if (incoming == '\n' || incoming == '\r') {
      if (commandOverflow) {
        Serial.println(
          F("Command too long. Please enter a shorter command.")
        );
      } else if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
      }

      // Reset the input buffer for the next command.
      commandLength = 0;
      commandOverflow = false;
    } else {
      // Ignore characters after a command becomes too long.
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