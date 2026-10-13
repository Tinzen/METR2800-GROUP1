#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotorController.h"

// =============================================================================
// EDIT THESE VALUES FIRST
// =============================================================================

// FSM
enum RobotState {
  IDLE, 

  DOWN,
  SCOOP,
  PICKUP,
  TRANSFER,
  BEFORE_EXTEND,
  OUTTAKE_EXTEND,
  OUTTAKE_RETRACT, 

  DONE
};

const char* stateName(RobotState s) {
  switch (s) {
    case IDLE:             return "IDLE";
    case DOWN:             return "DOWN";
    case SCOOP:            return "SCOOP";
    case PICKUP:           return "PICKUP";
    case TRANSFER:         return "TRANSFER";
    case BEFORE_EXTEND:    return "BEFORE_EXTEND";
    case OUTTAKE_EXTEND:   return "OUTTAKE_EXTEND";
    case OUTTAKE_RETRACT:  return "OUTTAKE_RETRACT";
    case DONE:             return "DONE";
    default:               return "UNKNOWN";
  }
}

// Serial debug
const unsigned long DEBUG_BAUD = 9600;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 100;

// =============================================================================
// MOTOR SETTINGS
// =============================================================================

// Motor driver pins
const int OUTTAKE_MA = 6;
const int OUTTAKE_MB = 7;
const int OUTTAKE_PWM = 11;
const int OUTTAKE_ENC_A = 2;
const int OUTTAKE_ENC_B = 3;

// DIRECTIONS
const int INTAKE_ENCODER_DIRECTION = 1;
const int INTAKE_MOTOR_DIRECTION = 1;
const int OUTTAKE_ENCODER_DIRECTION = 1;
const int OUTTAKE_MOTOR_DIRECTION = 1;

// Outtake limits are ticks/sec and ticks/sec^2.
// Encoder counts for one full 360 degree rotation of the intake arm.
// If the encoder is before a gearbox, use counts per full ARM revolution,
// not just counts per motor shaft revolution.
// Outtake slides stay in encoder ticks because they are linear, not rotational.
const float OUTTAKE_START_TICKS = 0.0f;
const int OUTTAKE_MAX_PWM = 200;
const float OUTTAKE_MAX_TICKS_PER_SEC = 1000.0f;
const float OUTTAKE_MAX_TICKS_PER_SEC2 = 2000.0f;
const float OUTTAKE_TOLERANCE_TICKS = 3.0f;

const float OUTTAKE_KP = 0.4f;
const float OUTTAKE_KI = 0.0f;
const float OUTTAKE_KD = 0.0f;

MotorController outtake(OUTTAKE_MA, OUTTAKE_MB, OUTTAKE_PWM, OUTTAKE_ENC_A, OUTTAKE_ENC_B);

// =============================================================================
// SERVO SETTINGS
// =============================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_FREQ = 50;
const int SERVO_MIN = 700;
const int SERVO_MAX = 2300;

const int ARM_R = 1;
const int ARM_L = 2;
const int SCOOP_SERVO = 3;

// Arm Positions
const int A_START = 940;
const int A_DOWN = 200;
const int A_PEAK = 350;
const int A_SCOOP = 150;
const int A_TRANSFER = 920;
const int A_MOVE = 500;

// Scoop Positions
const int S_START = 2300;
const int S_DOWN = 700;
const int S_PEAK = 1100;
const int S_SCOOP = 1900;
const int S_PICKUP = 2100;
const int S_TRANSFER = 1800;

// =============================================================================
// FSM
// =============================================================================
unsigned long stateStartTime = 0;
unsigned long servoJiggleTime = 0;
RobotState state = DOWN;
// Helper delay
bool stateDelay(unsigned long ms) {
  return millis() - stateStartTime >= ms;
}

void changeState(RobotState newState) {
  state = newState;
  stateStartTime = millis();
}

void resetTimer() {
  stateStartTime = millis();
}

void runStateMachine() {
  static RobotState lastState = IDLE;
  bool enteredState = (state != lastState);
  lastState = state;

  switch (state) {
    break;
    case DOWN:
      if (enteredState) {
        setScoop(S_DOWN);
        delay(500);
        setArm(A_DOWN);        
      }
      
      if (stateDelay(1000)) {
        changeState(SCOOP);
      }
      break;

    case SCOOP:
      if (enteredState) {
        scoopSweep();
      }
      
      changeState(PICKUP);
      break; 

    case PICKUP:
      if (enteredState) {
        setScoop(S_PICKUP);
      }
        
      if (stateDelay(1000)) {
        changeState(TRANSFER);
      }
      break;

    case TRANSFER:
      if (enteredState) {
        setArm(A_TRANSFER);
        setScoop(S_TRANSFER);
      }

      if (stateDelay(2000)) {
        changeState(BEFORE_EXTEND);
      }
      break;

     case BEFORE_EXTEND:
      if (enteredState) {
        setArm(A_MOVE);
      }
              
      if (stateDelay(1000)) {
        changeState(DONE);
      }
      break;

    case EXTEND:
      if (enteredState) {
        intake.stopMotor();
        setServo(SERVO_DOWN);
      }

      //outtake.setTarget(500,200);
      
      if (stateDelay(3150)) {
        if (stateDelay(6500)) {
            changeState(OUTTAKE_DELAY);
            //outtake.setMotor(0,0);
        } else {
          outtake.setMotor(63,200);
        }
      } else {
        outtake.setMotor(250,250);
      }
  

      break;

    case RETRACT:
      if (enteredState) {
      }

      if (stateDelay(2950)) {
       outtake.stopMotor();
      } else {
        outtake.setMotor(-250,250);
        }
      break;

    case DONE:
      if (enteredState) {
        
      }
       
      break;
      
  }
}

// ==================================================
// FSM LOGIC
// ==================================================

int microsecondsToCounts(int pulseUs) {
  // At 50 Hz, the PWM period is 20,000 microseconds.
  return (int)(pulseUs * 4096.0 / 20000.0 + 0.5);
}

void setServo(int channel, int pulseUs) {
  int counts = microsecondsToCounts(pulseUs);

  // Convert user-facing output number to PCA9685 channel.
  pwm.setPWM(channel - 1, 0, counts);
}

// Coordinated Intake Control
bool setArm(int position) {
  position = constrain(position, 0, 1000);

  int rightPulse = SERVO_MIN
                 + (long)position * (SERVO_MAX - SERVO_MIN) / 1000;

  int leftPulse = SERVO_MAX
                - (long)position * (SERVO_MAX - SERVO_MIN) / 1000;

  setServo(ARM_R, rightPulse);
  setServo(ARM_L, leftPulse);

  return true;
}

void setScoop(int pulseUs) {
  setServo(SCOOP_SERVO, pulseUs);
}

void scoopSweep() {
  const int steps = 100;
  const int stepDelayMs = 20;

  for (int i = 0; i <= steps; i++) {
    // Progress goes from 0 to 1000 across the sweep.
    long progress = (long)i * 1000 / steps;

    // Scoop moves from S_DOWN to S_SCOOP.
    int scoopPulse = S_DOWN + (S_SCOOP - S_DOWN) * progress / 1000;
    setScoop(scoopPulse);

    // Arm rises to A_PEAK at the midpoint, then lowers to A_SCOOP.
    int armPosition;

    if (progress < 500) {
      armPosition = A_DOWN + (A_PEAK - A_DOWN) * progress / 500;
    } else {
      armPosition = A_PEAK - (A_PEAK - A_SCOOP) * (progress - 500) / 500;
    }

    setArm(armPosition);
    delay(stepDelayMs);
  }
}

void initServos() {
  setArm(A_START);
  setScoop(S_START);
}

void configureControllers() {
  // Outtake slides: tick-based controller.
  outtake.setUnitsTicks();
  outtake.setEncoderDirection(OUTTAKE_ENCODER_DIRECTION);
  outtake.setMotorDirection(OUTTAKE_MOTOR_DIRECTION);
  outtake.setPID(OUTTAKE_KP, OUTTAKE_KI, OUTTAKE_KD);
  outtake.setProfileLimits(OUTTAKE_MAX_TICKS_PER_SEC, OUTTAKE_MAX_TICKS_PER_SEC2);
  outtake.setMaxPwm(OUTTAKE_MAX_PWM);
  outtake.setTolerance(OUTTAKE_TOLERANCE_TICKS);
  outtake.begin(OUTTAKE_START_TICKS);
}

void printStatus() {
  static unsigned long lastPrint = 0;
  const unsigned long now = millis();

  if ((now - lastPrint) < DEBUG_PRINT_INTERVAL_MS) return;
  lastPrint = now;

  Serial.print("State: ");
  Serial.print(stateName(state));

  Serial.print("    Outtake_Pos");
  Serial.println(outtake.getPosition());

  // Serial.print("I_deg:");
  // Serial.print(intake.getPosition(), 1);

  // Serial.print(" I_moveDeg:");
  // Serial.print(intake.getMovingTarget(), 1);

  // Serial.print(" I_targetDeg:");
  // Serial.print(intake.getTarget(), 1);

  // Serial.print(" I_power:");
  // Serial.print(intake.getMotorPower());

  // Serial.print(" I_done:");
  // Serial.print(intake.isProfileFinished() ? 1 : 0);

  // Serial.print(" O_ticks:");
  // Serial.print(outtake.getPosition(), 0);

  // Serial.print(" O_moveTicks:");
  // Serial.print(outtake.getMovingTarget(), 0);

  // Serial.print(" O_targetTicks:");
  // Serial.print(outtake.getTarget(), 0);

  // Serial.print(" O_power:");
  // Serial.println(outtake.getMotorPower());
}

// ==================================================
// Setup and Loop
// ==================================================

void setup() {
  Serial.begin(DEBUG_BAUD);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  initServos();
  configureControllers();

  Serial.println("---------- Robot start ----------");

  changeState(DOWN);
  //changeState(OUTTAKE_EXTEND);
}

void loop() {
  outtake.update();
  printStatus();
  
  runStateMachine();
  // Example commands:
  //intake.setTarget(90.0f, INTAKE_MAX_PWM);       // arm moves to 90 degrees
  // intake.moveRelative(-15.0f, INTAKE_MAX_PWM);   // arm moves 15 degrees down
  // outtake.setTarget(500.0f, OUTTAKE_MAX_PWM);    // slides move to 500 ticks
}
