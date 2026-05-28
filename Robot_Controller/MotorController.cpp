#include "MotorController.h"
#include <Arduino.h>
#include <math.h>

MotorController::MotorController(int ma, int mb, int mp, int encA, int encB)
: MA(ma),
  MB(mb),
  MP(mp),
  encoder(encA, encB),
  pid(0.0f, 0.0f, 0.0f),
  profile(),
  unitMode(DEGREES),
  ticksPerUnit(1.0f),       // 360 ticks/rev default: 1 tick = 1 degree
  encoderDirection(1),
  motorDirection(1),
  target(0.0f),
  movingTarget(0.0f),
  maxCommandPwm(255),
  motorPower(0),
  lastTime(0),
  interval(10),
  mode(ACTIVE),
  tolerance(1.0f),
  useMotionProfile(false)
{
}

void MotorController::setUnitsDegrees(float encoderTicksPerRevolution) {
  unitMode = DEGREES;

  if (encoderTicksPerRevolution <= 0.0f) {
    encoderTicksPerRevolution = 360.0f;
  }

  ticksPerUnit = encoderTicksPerRevolution / 360.0f;
  if (ticksPerUnit <= 0.0f) ticksPerUnit = 1.0f;
}

void MotorController::setUnitsTicks() {
  unitMode = TICKS;
  ticksPerUnit = 1.0f;
}

void MotorController::setEncoderDirection(int direction) {
  encoderDirection = (direction < 0) ? -1 : 1;
}

void MotorController::setMotorDirection(int direction) {
  motorDirection = (direction < 0) ? -1 : 1;
}

void MotorController::begin(float knownStartPosition) {
  pinMode(MA, OUTPUT);
  pinMode(MB, OUTPUT);
  pinMode(MP, OUTPUT);

  stopMotor();
  calibratePosition(knownStartPosition);

  target = knownStartPosition;
  movingTarget = knownStartPosition;
  profile.moveTo(knownStartPosition, knownStartPosition);
  pid.reset();
  lastTime = millis();
}

void MotorController::calibratePosition(float knownPosition) {
  encoder.write(positionToRawTicks(knownPosition));
}

void MotorController::setPID(float kp, float ki, float kd) {
  pid.setTunings(kp, ki, kd);
}

void MotorController::setProfileLimits(float maxVelocity, float maxAcceleration) {
  profile.setLimits(maxVelocity, maxAcceleration);
}

void MotorController::setLimits(float maxVelocity, float maxAcceleration) {
  setProfileLimits(maxVelocity, maxAcceleration);
}

void MotorController::setMaxPwm(int maxPwm) {
  maxCommandPwm = constrain(abs(maxPwm), 0, 255);
  pid.setOutputLimits(-maxCommandPwm, maxCommandPwm);
}

void MotorController::setTolerance(float newTolerance) {
  tolerance = fabs(newTolerance);
}

void MotorController::setUpdateInterval(unsigned long intervalMs) {
  interval = intervalMs;
}

void MotorController::setTarget(float targetPosition) {
  setTarget(targetPosition, maxCommandPwm);
}

void MotorController::setTarget(float targetPosition, int maxPwm) {
  const int requestedPwm = constrain(abs(maxPwm), 0, 255);

  // Do not restart the profile if loop() accidentally repeats the same command.
  if (fabs(target - targetPosition) <= 0.001f && maxCommandPwm == requestedPwm) {
    return;
  }

  target = targetPosition;
  maxCommandPwm = requestedPwm;
  pid.setOutputLimits(-maxCommandPwm, maxCommandPwm);

  mode = ACTIVE;
  pid.reset();

  movingTarget = getPosition();
  useMotionProfile = true;
  profile.moveTo(movingTarget, target);
}


void MotorController::setTargetPID(float targetPosition, int maxPwm) {
  const int requestedPwm = constrain(abs(maxPwm), 0, 255);

  // Do not reset PID if loop() accidentally repeats the same command.
  if (fabs(target - targetPosition) <= 0.001f && maxCommandPwm == requestedPwm && !useMotionProfile) {
    return;
  }

  target = targetPosition;
  maxCommandPwm = requestedPwm;
  pid.setOutputLimits(-maxCommandPwm, maxCommandPwm);

  mode = ACTIVE;
  pid.reset();

  movingTarget = target;
  useMotionProfile = false;
}

void MotorController::moveRelative(float deltaPosition) {
  moveRelative(deltaPosition, maxCommandPwm);
}

void MotorController::moveRelative(float deltaPosition, int maxPwm) {
  setTarget(getPosition() + deltaPosition, maxPwm);
}

float MotorController::getPosition() {
  return rawTicksToPosition(encoder.read());
}

float MotorController::getTarget() const {
  return target;
}

float MotorController::getMovingTarget() const {
  return movingTarget;
}

long MotorController::getRawEncoderTicks() {
  return encoder.read();
}

int MotorController::getMotorPower() const {
  return motorPower;
}

bool MotorController::isAtTarget() {
  return fabs(getPosition() - target) <= tolerance;
}


bool MotorController::isProfileFinished() const {
  return profile.isFinished();
}

MotorController::UnitMode MotorController::getUnitMode() const {
  return unitMode;
}

void MotorController::setMode(MotorMode newMode) {
  mode = newMode;

  if (mode == HOLD) {
    target = getPosition();
    movingTarget = target;
    profile.moveTo(target, target);
    pid.reset();
  }

  if (mode == COAST) {
    stopMotor();
  }
}

void MotorController::setMotor(int pwmCommand, int maxPwm) {
  maxPwm = constrain(abs(maxPwm), 0, 255);

  const int limitedCommand = constrain(pwmCommand, -maxPwm, maxPwm);
  const int driverCommand = limitedCommand * motorDirection;
  motorPower = driverCommand;

  if (driverCommand > 0) {
    digitalWrite(MA, HIGH);
    digitalWrite(MB, LOW);
    analogWrite(MP, driverCommand);
  } else if (driverCommand < 0) {
    digitalWrite(MA, LOW);
    digitalWrite(MB, HIGH);
    analogWrite(MP, -driverCommand);
  } else {
    analogWrite(MP, 0);
    digitalWrite(MA, LOW);
    digitalWrite(MB, LOW);
  }
}

void MotorController::stopMotor() {
  setMotor(0, 255);
}

void MotorController::update() {
  if (mode == COAST) {
    stopMotor();
    lastTime = millis();
    return;
  }

  const unsigned long now = millis();
  if ((now - lastTime) < interval) return;

  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  if (dt <= 0.0f) return;

  // If a delay() or blocking action pauses the robot, do not let the D term
  // or integral term receive a giant timestep.
  if (dt > 0.05f) dt = 0.05f;

  const float position = getPosition();
  
  if (useMotionProfile) {
  movingTarget = profile.update();
  } else {
    movingTarget = target;
  }

  const float pidOutput = pid.compute(movingTarget, position, dt);
  setMotor((int)roundToLong(pidOutput), maxCommandPwm);
}

float MotorController::rawTicksToPosition(long rawTicks) const {
  return ((float)rawTicks * (float)encoderDirection) / ticksPerUnit;
}

long MotorController::positionToRawTicks(float position) const {
  return roundToLong(position * ticksPerUnit * (float)encoderDirection);
}

long MotorController::roundToLong(float value) const {
  if (value >= 0.0f) {
    return (long)(value + 0.5f);
  }
  return (long)(value - 0.5f);
}
