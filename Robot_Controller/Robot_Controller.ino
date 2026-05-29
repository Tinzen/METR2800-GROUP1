#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotorController.h"

// =============================================================================
// EDIT THESE VALUES FIRST
// =============================================================================

// FSM
enum RobotState {
  IDLE, 

  SERVO_FIRST_MOVE,
  INTAKE_DOWN,
  INTAKE_SCOOP,
  INTAKE_TRANSFER_1,
  INTAKE_TRANSFER_2,
  INTAKE_TRANSFER_3,
  OUTTAKE_EXTEND,
  OUTTAKE_DELAY,
  OUTTAKE_RETRACT, 

  DONE
};

const char* stateName(RobotState s) {
  switch (s) {
    case IDLE:             return "IDLE";
    case SERVO_FIRST_MOVE: return "SERVO_FIRST_MOVE";
    case INTAKE_DOWN:      return "INTAKE_DOWN";
    case INTAKE_SCOOP:     return "INTAKE_SCOOP";
    case INTAKE_TRANSFER_1:return "INTAKE_TRANSFER_1";
    case INTAKE_TRANSFER_2:return "INTAKE_TRANSFER_2";
    case INTAKE_TRANSFER_3: return "INTAKE_TRANSFER_3";
    case OUTTAKE_EXTEND:   return "OUTTAKE_EXTEND";
    case OUTTAKE_DELAY:   return "OUTTAKE_DELAY";
    case OUTTAKE_RETRACT:  return "OUTTAKE_RETRACT";
    case DONE:             return "DONE";
    default:               return "UNKNOWN";
  }
}

// Serial debug
const unsigned long DEBUG_BAUD = 9600;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 100;

// Motor driver pins
const int INTAKE_MA = 6;
const int INTAKE_MB = 7;
const int INTAKE_PWM = 10;
const int INTAKE_ENC_A = 2;
const int INTAKE_ENC_B = 3;

const int OUTTAKE_MA = 8;
const int OUTTAKE_MB = 9;
const int OUTTAKE_PWM = 11;
const int OUTTAKE_ENC_A = 4;
const int OUTTAKE_ENC_B = 5;

const int SERVO_CHANNEL = 4;


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

//INTAKE ARM STUFF
const float INTAKE_START_DEGREES = 98.0f;

const float INTAKE_ENCODER_TICKS_PER_ARM_REV = 580.0f;
const float INTAKE_TOLERANCE_DEGREES = 2.0f;
const int INTAKE_MAX_PWM = 150;
const float INTAKE_MAX_DEG_PER_SEC = 90.0f;
const float INTAKE_MAX_DEG_PER_SEC2 = 180.0f;

const float INTAKE_KP = 3.0f;
const float INTAKE_KI = 0.005f;
const float INTAKE_KD = 0.2f;

// =============================================================================
// SERVO SETTINGS
// =============================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int TORQUE_SERVO = 5;
const int SERVO_MIN = 110;
const int SERVO_MAX = 580;

const int SERVO_DOWN = 31;
const int SERVO_SCOOP = 127;
const int SERVO_PICKUP = 140;
const int SERVO_TRANSFER_1 = 120;
const int SERVO_TRANSFER = 137;
const int SERVO_START = 170;

// =============================================================================
// CONTROLLERS
// =============================================================================

MotorController intake(INTAKE_MA, INTAKE_MB, INTAKE_PWM, INTAKE_ENC_A, INTAKE_ENC_B);
MotorController outtake(OUTTAKE_MA, OUTTAKE_MB, OUTTAKE_PWM, OUTTAKE_ENC_A, OUTTAKE_ENC_B);


void vibrateServo(int angleA, int angleB, unsigned long intervalMs) {
  static unsigned long lastMoveTime = 0;
  static bool toggle = false;

  unsigned long now = millis();

  if (now - lastMoveTime >= intervalMs) {
    lastMoveTime = now;
    toggle = !toggle;

    if (toggle) {
      setServo(angleA);
    } else {
      setServo(angleB);
    }
  }
}

void vibrateOuttake(int motorPower, unsigned long intervalMs) {
  static unsigned long lastMoveTime = 0;
  static bool toggle = false;

  unsigned long now = millis();

  if (now - lastMoveTime >= intervalMs) {
    lastMoveTime = now;
    toggle = !toggle;

    if (toggle) {
      outtake.setMotor(-motorPower, -motorPower);
    } else {
      outtake.setMotor(motorPower, motorPower);
    }
  }
}

// THIS IS STATE MACHINE TIMER LOGIC

unsigned long stateStartTime = 0;
unsigned long servoJiggleTime = 0;
RobotState state = SERVO_FIRST_MOVE;
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
    case SERVO_FIRST_MOVE:
      if (enteredState) {
          setServo(SERVO_DOWN);  
        }
      if (stateDelay(300)) {
        changeState(INTAKE_DOWN);
      }
    break;

    case INTAKE_DOWN:
      if (enteredState) {
        intake.setTarget(17.0f, 150);   
      }
      
      if (intake.isAtTarget()) {
        changeState(INTAKE_SCOOP);
      }
      break;

    case INTAKE_SCOOP:
      if (enteredState) {
        setServo(SERVO_SCOOP);
      }
      
      intake.setMotor(-10, 50);

      if (stateDelay(1100)) {
        changeState(INTAKE_TRANSFER_1);
      }
      break; 

    case INTAKE_TRANSFER_1:
      if (enteredState) {
        setServo(SERVO_PICKUP);
      }

      intake.setMotor(-8, 50);
        
      if (stateDelay(100)) {
        changeState(INTAKE_TRANSFER_2);
      }
      break;

    case INTAKE_TRANSFER_2:
      if (enteredState) {
        intake.setTarget(105,150);
      }

      if (stateDelay(500)) {
        setServo(SERVO_TRANSFER_1);
      }
      if (intake.getPosition() > 90.0f) {
        changeState(INTAKE_TRANSFER_3);
      }
      break;

     case INTAKE_TRANSFER_3:
      if (enteredState) {
        setServo(SERVO_TRANSFER);
      }
      
      vibrateServo(SERVO_TRANSFER - 4, SERVO_TRANSFER + 4, 50);
        
      if (stateDelay(1000)) {
        changeState(OUTTAKE_EXTEND);
      }
      break;

    case OUTTAKE_EXTEND:
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

    case OUTTAKE_DELAY:
      if (enteredState) {
        intake.stopMotor();
        
      }


      //vibrateOuttake(250,500);

      if (stateDelay(0)) {
          changeState(OUTTAKE_RETRACT);
      }

    break;


    case OUTTAKE_RETRACT:
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
//THIS IS STATE MACHINE TIMER LOGIC

void setServo(int angle) {
  angle = constrain(angle, 0, 180);
  const int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(SERVO_CHANNEL, 0, pulse);
}

void configureControllers() {
  // Intake arm: degree-based controller.
  intake.setUnitsDegrees(INTAKE_ENCODER_TICKS_PER_ARM_REV);
  intake.setEncoderDirection(INTAKE_ENCODER_DIRECTION);
  intake.setMotorDirection(INTAKE_MOTOR_DIRECTION);
  intake.setPID(INTAKE_KP, INTAKE_KI, INTAKE_KD);
  intake.setProfileLimits(INTAKE_MAX_DEG_PER_SEC, INTAKE_MAX_DEG_PER_SEC2);
  intake.setMaxPwm(INTAKE_MAX_PWM);
  intake.setTolerance(INTAKE_TOLERANCE_DEGREES);
  intake.begin(INTAKE_START_DEGREES);

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

  Serial.print("    Intake_Pos");
  Serial.print(intake.getPosition());

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

void setup() {
  Serial.begin(DEBUG_BAUD);

  pwm.begin();
  pwm.setPWMFreq(50);
  setServo(SERVO_START);

  configureControllers();

  Serial.println("---------- Robot start ----------");
  Serial.println("Intake units: degrees. Outtake units: encoder ticks.");

  changeState(SERVO_FIRST_MOVE);
  //changeState(OUTTAKE_EXTEND);
}

void loop() {
  intake.update();
  outtake.update();
  printStatus();
  
  runStateMachine();
  // Example commands:
  //intake.setTarget(90.0f, INTAKE_MAX_PWM);       // arm moves to 90 degrees
  // intake.moveRelative(-15.0f, INTAKE_MAX_PWM);   // arm moves 15 degrees down
  // outtake.setTarget(500.0f, OUTTAKE_MAX_PWM);    // slides move to 500 ticks
}
