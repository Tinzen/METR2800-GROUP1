#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================================================
// PCA9685 SETUP
// ==================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_FREQ = 50;

// Servo channels
const int INTAKE_R = 4;
const int INTAKE_L = 5;
const int SCOOP    = 3;


// ==================================================
// INDIVIDUAL SERVO CALIBRATION LIMITS
// ==================================================
//
// IMPORTANT:
// These are EXAMPLE values, not verified limits for
// your specific goBILDA servos.
//
// Replace them with the safe minimum and maximum
// pulse widths you measure during calibration.
//
// Do not assume that every servo has the same range.
//

// Intake right — Channel 4
const int INTAKE_R_MIN = 700;
const int INTAKE_R_MAX = 2300;

// Intake left — Channel 5
const int INTAKE_L_MIN = 700;
const int INTAKE_L_MAX = 2300;

// Intake scoop — Channel 3
const int SCOOP_MIN = 700;
const int SCOOP_MAX = 2300;


// ==================================================
// ADJUSTMENT SETTINGS
// ==================================================

// Initial adjustment step in microseconds
int stepSize = 10;

// Currently selected servo
int selectedChannel = INTAKE_R;

// Last commanded pulse width
int currentPulseUs = 1500;

// Whether a position has been commanded for the
// currently selected servo
bool positionSet = false;


// ==================================================
// GET SERVO NAME
// ==================================================

const char* getServoName(int channel) {

  switch (channel) {

    case INTAKE_R:
      return "Intake Right";

    case INTAKE_L:
      return "Intake Left";

    case SCOOP:
      return "Intake Scoop";

    default:
      return "Unknown";
  }
}


// ==================================================
// GET SELECTED SERVO MINIMUM
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
// GET SELECTED SERVO MAXIMUM
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
// MOVE SERVO
// ==================================================

void setServo(int pulseUs) {

  // Get the limits for the currently selected servo
  int minPulse = getServoMin(selectedChannel);
  int maxPulse = getServoMax(selectedChannel);

  // Check the selected servo's individual limits
  if (pulseUs < minPulse || pulseUs > maxPulse) {

    Serial.println();
    Serial.println("ERROR: Pulse width outside servo limits!");

    Serial.print("Servo: ");
    Serial.println(getServoName(selectedChannel));

    Serial.print("Allowed range: ");
    Serial.print(minPulse);
    Serial.print(" - ");
    Serial.print(maxPulse);
    Serial.println(" us");

    Serial.println("Servo position unchanged.");
    Serial.println();

    return;
  }

  // Convert pulse width to PCA9685 counts
  int counts = microsecondsToCounts(pulseUs);

  // Command the selected servo
  pwm.setPWM(selectedChannel, 0, counts);

  // Update the last commanded position
  currentPulseUs = pulseUs;
  positionSet = true;

  // Print the result
  Serial.print("Servo: ");
  Serial.print(getServoName(selectedChannel));

  Serial.print(" | Channel: ");
  Serial.print(selectedChannel);

  Serial.print(" | Pulse: ");
  Serial.print(currentPulseUs);

  Serial.print(" us | Counts: ");
  Serial.println(counts);
}


// ==================================================
// PRINT HELP
// ==================================================

void printHelp() {

  Serial.println();
  Serial.println("========== SERVO CALIBRATION ==========");

  Serial.println();
  Serial.println("--- Select Servo ---");
  Serial.println("s 3       Select intake scoop");
  Serial.println("s 4       Select intake right");
  Serial.println("s 5       Select intake left");

  Serial.println();
  Serial.println("--- Position Control ---");
  Serial.println("p 1500    Set pulse width in microseconds");
  Serial.println("+         Increase pulse width");
  Serial.println("-         Decrease pulse width");

  Serial.println();
  Serial.println("--- Adjustment ---");
  Serial.println("step 1    Set adjustment step to 1 us");
  Serial.println("step 10   Set adjustment step to 10 us");

  Serial.println();
  Serial.println("--- Information ---");
  Serial.println("status    Show position and servo limits");
  Serial.println("?         Show help");

  Serial.println();
  Serial.println("=======================================");
  Serial.println();
}


// ==================================================
// PROCESS SERIAL COMMANDS
// ==================================================

void processCommand(String command) {

  command.trim();

  if (command.length() == 0) {
    return;
  }


  // ----------------------------------------------
  // HELP
  // ----------------------------------------------

  if (command == "?") {

    printHelp();
    return;
  }


  // ----------------------------------------------
  // SELECT SERVO
  // ----------------------------------------------

  if (command.startsWith("s ")) {

    String value = command.substring(2);
    value.trim();

    // Validate channel input
    if (value != "3" &&
        value != "4" &&
        value != "5") {

      Serial.println("Choose channel 3, 4 or 5.");
      return;
    }

    int channel = value.toInt();

    selectedChannel = channel;

    // Require a new initial position for the
    // newly selected servo
    positionSet = false;

    Serial.println();

    Serial.print("Selected servo: ");
    Serial.println(getServoName(selectedChannel));

    Serial.print("Channel: ");
    Serial.println(selectedChannel);

    Serial.print("Minimum pulse: ");
    Serial.print(getServoMin(selectedChannel));
    Serial.println(" us");

    Serial.print("Maximum pulse: ");
    Serial.print(getServoMax(selectedChannel));
    Serial.println(" us");

    Serial.println();
    Serial.println("Set an initial position, e.g. p 1500.");

    return;
  }


  // ----------------------------------------------
  // SET PULSE WIDTH DIRECTLY
  // ----------------------------------------------

  if (command.startsWith("p ")) {

    String value = command.substring(2);
    value.trim();

    if (value.length() == 0) {

      Serial.println(
        "Enter a pulse width in microseconds."
      );

      return;
    }

    // Accept positive integers only
    for (unsigned int i = 0; i < value.length(); i++) {

      if (!isDigit(value.charAt(i))) {

        Serial.println(
          "Enter a positive integer."
        );

        return;
      }
    }

    int pulseUs = value.toInt();

    setServo(pulseUs);

    return;
  }


  // ----------------------------------------------
  // INCREASE PULSE WIDTH
  // ----------------------------------------------

  if (command == "+") {

    if (!positionSet) {

      Serial.println(
        "Set an initial position first, e.g. p 1500."
      );

      return;
    }

    // Use a wider integer for the calculation
    long newPulse = (long)currentPulseUs + stepSize;

    if (newPulse > getServoMax(selectedChannel)) {

      Serial.println(
        "Maximum pulse width reached. Position unchanged."
      );

      return;
    }

    setServo((int)newPulse);

    return;
  }


  // ----------------------------------------------
  // DECREASE PULSE WIDTH
  // ----------------------------------------------

  if (command == "-") {

    if (!positionSet) {

      Serial.println(
        "Set an initial position first, e.g. p 1500."
      );

      return;
    }

    long newPulse = (long)currentPulseUs - stepSize;

    if (newPulse < getServoMin(selectedChannel)) {

      Serial.println(
        "Minimum pulse width reached. Position unchanged."
      );

      return;
    }

    setServo((int)newPulse);

    return;
  }


  // ----------------------------------------------
  // CHANGE ADJUSTMENT STEP
  // ----------------------------------------------

  if (command.startsWith("step ")) {

    String value = command.substring(5);
    value.trim();

    if (value.length() == 0) {

      Serial.println(
        "Enter a step size between 1 and 500 us."
      );

      return;
    }

    // Validate input
    for (unsigned int i = 0; i < value.length(); i++) {

      if (!isDigit(value.charAt(i))) {

        Serial.println(
          "Enter a positive integer."
        );

        return;
      }
    }

    int newStep = value.toInt();

    if (newStep < 1 || newStep > 500) {

      Serial.println(
        "Step must be between 1 and 500 us."
      );

      return;
    }

    stepSize = newStep;

    Serial.print("Adjustment step: ");
    Serial.print(stepSize);
    Serial.println(" us");

    return;
  }


  // ----------------------------------------------
  // DISPLAY STATUS
  // ----------------------------------------------

  if (command == "status") {

    Serial.println();
    Serial.println("========== SERVO STATUS ==========");

    Serial.print("Servo: ");
    Serial.println(getServoName(selectedChannel));

    Serial.print("Channel: ");
    Serial.println(selectedChannel);

    Serial.print("Minimum pulse: ");
    Serial.print(getServoMin(selectedChannel));
    Serial.println(" us");

    Serial.print("Maximum pulse: ");
    Serial.print(getServoMax(selectedChannel));
    Serial.println(" us");

    Serial.print("Adjustment step: ");
    Serial.print(stepSize);
    Serial.println(" us");

    if (positionSet) {

      Serial.print("Last commanded pulse: ");
      Serial.print(currentPulseUs);
      Serial.println(" us");

    } else {

      Serial.println(
        "No position commanded for this servo yet."
      );
    }

    Serial.println("==================================");
    Serial.println();

    return;
  }


  // ----------------------------------------------
  // UNKNOWN COMMAND
  // ----------------------------------------------

  Serial.println(
    "Unknown command. Enter ? for help."
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

  Serial.println();
  Serial.println("Servo calibration ready.");

  Serial.println(
    "No servo movement commanded automatically."
  );

  Serial.println(
    "Individual pulse limits are enabled."
  );

  Serial.println(
    "IMPORTANT: Replace example limits with your measured values."
  );

  printHelp();
}


// ==================================================
// LOOP
// ==================================================

void loop() {

  if (Serial.available()) {

    String command = Serial.readStringUntil('\n');

    processCommand(command);
  }
}