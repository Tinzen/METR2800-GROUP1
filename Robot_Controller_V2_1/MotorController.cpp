#include "MotorController.h"
#include <Arduino.h>
#include <math.h>

// Low-pass factor for the velocity estimate (0..1). Higher = less smoothing.
static const float VELOCITY_FILTER_ALPHA = 0.35f;

// Instances are globals (zero-filled at boot), so only non-zero members are
// set here. Keeps flash small on the Nano.
MotorController::MotorController(int ma, int mb, int mp, int encA, int encB)
: MA(ma),
  MB(mb),
  MP(mp),
  encoder(encA, encB),
  pid(0.0f, 0.0f, 0.0f),
  unitMode(DEGREES),
  ticksPerUnit(1.0f),       // 360 ticks/rev default: 1 tick = 1 degree
  encoderDirection(1),
  motorDirection(1),
  maxCommandPwm(255),
  interval(10),
  mode(ACTIVE),
  tolerance(1.0f)
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
  lastPosition = knownStartPosition;
  velocity = 0.0f;
  profile.moveTo(knownStartPosition, knownStartPosition);
  pid.reset();
  lastTime = millis();
}

void MotorController::calibratePosition(float knownPosition) {
  encoder.write(positionToRawTicks(knownPosition));
  lastPosition = knownPosition;
}

void MotorController::setPID(float kp, float ki, float kd) {
  pid.setTunings(kp, ki, kd);
}

void MotorController::setFeedforward(float kS, float kV, float kA, float kG) {
  ffS = kS;
  ffV = kV;
  ffA = kA;
  ffG = kG;
}

void MotorController::setProfileLimits(float maxVelocity, float maxAcceleration, float maxDeceleration) {
  profile.setLimits(maxVelocity, maxAcceleration, maxDeceleration);
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
  // V2.1: only skip if we are actually still in closed-loop profile mode.
  if (fabs(target - targetPosition) <= 0.001f && maxCommandPwm == requestedPwm
      && mode == ACTIVE && useMotionProfile) {
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


void MotorController::setTargetBlend(float targetPosition, int maxPwm, float startVel, float endVel) {
  target = targetPosition;
  maxCommandPwm = constrain(abs(maxPwm), 0, 255);
  pid.setOutputLimits(-maxCommandPwm, maxCommandPwm);
  mode = ACTIVE;
  // keep the integrator when blending out of a moving segment
  if (startVel <= 0.0f) pid.reset();
  movingTarget = getPosition();
  useMotionProfile = true;
  profile.moveToVel(movingTarget, target, startVel, endVel);
}

void MotorController::setTargetPID(float targetPosition, int maxPwm) {
  const int requestedPwm = constrain(abs(maxPwm), 0, 255);

  // Do not reset PID if loop() accidentally repeats the same command.
  if (fabs(target - targetPosition) <= 0.001f && maxCommandPwm == requestedPwm
      && !useMotionProfile && mode == ACTIVE) {
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

void MotorController::setOpenLoop(int pwmCommand) {
  openLoopPwm = constrain(pwmCommand, -255, 255);
  mode = OPEN_LOOP;
  lastFeedforward = 0;
  setMotor(openLoopPwm, 255);
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

int MotorController::getCommandPower() const {
  return commandPower;
}

int MotorController::getFeedforward() const {
  return lastFeedforward;
}

float MotorController::getVelocity() const {
  return velocity;
}

float MotorController::getProfileVelocity() const {
  return useMotionProfile ? profile.getVelocity() : 0.0f;
}

bool MotorController::isAtTarget() {
  return fabs(getPosition() - target) <= tolerance;
}


bool MotorController::isProfileFinished() const {
  return profile.isFinished();
}

float MotorController::getProfileTotalTime() const {
  return profile.getTotalTime();
}

float MotorController::getProfileTimeLeft() const {
  return useMotionProfile ? profile.getTimeLeft() : 0.0f;
}

float MotorController::getProfileProgress() const {
  return profile.getProgress();
}

MotorController::UnitMode MotorController::getUnitMode() const {
  return unitMode;
}

MotorController::MotorMode MotorController::getMode() const {
  return mode;
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
    lastFeedforward = 0;
    stopMotor();
  }
}

void MotorController::setMotor(int pwmCommand, int maxPwm) {
  maxPwm = constrain(abs(maxPwm), 0, 255);

  const int limitedCommand = constrain(pwmCommand, -maxPwm, maxPwm);
  const int driverCommand = limitedCommand * motorDirection;
  commandPower = limitedCommand;
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
  const unsigned long now = millis();
  if ((now - lastTime) < interval) return;

  const float rawDt = (now - lastTime) / 1000.0f;
  lastTime = now;

  if (rawDt <= 0.0f) return;

  const float position = getPosition();

  // V2.1: velocity estimate runs in every mode so telemetry always has it.
  const float rawVelocity = (position - lastPosition) / rawDt;
  lastPosition = position;
  velocity += VELOCITY_FILTER_ALPHA * (rawVelocity - velocity);

  if (mode == COAST) {
    stopMotor();
    return;
  }

  if (mode == OPEN_LOOP) {
    setMotor(openLoopPwm, 255);
    return;
  }

  // If a delay() or blocking action pauses the robot, do not let the D term
  // or integral term receive a giant timestep.
  float dt = rawDt;
  if (dt > 0.05f) dt = 0.05f;

  float profileVel = 0.0f;
  float profileAcc = 0.0f;

  if (useMotionProfile) {
    movingTarget = profile.update();
    profileVel = profile.getVelocity();
    profileAcc = profile.getAcceleration();
  } else {
    movingTarget = target;
  }

  const float pidOutput = pid.compute(movingTarget, position, dt);

  // V2.1 feedforward. All gains default to 0 (pure v1 PID).
  float ff = ffG + (ffV * profileVel) + (ffA * profileAcc);
  const float error = movingTarget - position;
  if (profileVel > 0.5f)            ff += ffS;
  else if (profileVel < -0.5f)      ff -= ffS;
  else if (error > tolerance)       ff += ffS;
  else if (error < -tolerance)      ff -= ffS;
  lastFeedforward = (int)roundToLong(ff);

  float output = pidOutput + ff;
  if (output >  maxCommandPwm) output =  maxCommandPwm;
  if (output < -maxCommandPwm) output = -maxCommandPwm;

  setMotor((int)roundToLong(output), maxCommandPwm);
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
