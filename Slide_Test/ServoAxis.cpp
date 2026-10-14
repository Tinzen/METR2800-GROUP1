#include "ServoAxis.h"

ServoAxis::ServoAxis(ServoWriteFn writer, int minValue, int maxValue)
: writeFn(writer),
  profile(),
  minVal(minValue),
  maxVal(maxValue),
  target(minValue),
  command((float)minValue),
  profiling(false),
  lastWritten(minValue),
  lastWriteMs(0),
  writeIntervalMs(10),
  hasWritten(false),
  vibrating(false),
  vibAmplitude(0),
  vibHalfPeriodMs(50),
  vibLastToggleMs(0),
  vibPositive(false)
{
}

void ServoAxis::setLimits(float maxVelocity, float maxAcceleration) {
  profile.setLimits(maxVelocity, maxAcceleration);
}

void ServoAxis::setWriteInterval(unsigned long ms) {
  writeIntervalMs = ms;
}

int ServoAxis::clampValue(long value) const {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return (int)value;
}

void ServoAxis::writeNow(int value) {
  writeFn(value);
  lastWritten = value;
  lastWriteMs = millis();
  hasWritten = true;
}

void ServoAxis::begin(int startValue) {
  target = clampValue(startValue);
  command = (float)target;
  profiling = false;
  profile.moveTo(command, command);
  writeNow(target);
}

void ServoAxis::moveTo(int targetValue) {
  targetValue = clampValue(targetValue);

  // Ignore repeated commands so the profile is not restarted every loop.
  if (targetValue == target && profiling) return;

  target = targetValue;
  profile.moveTo(command, (float)target);
  profiling = !profile.isFinished();
  if (!profiling) command = (float)target;
}

void ServoAxis::jumpTo(int value) {
  target = clampValue(value);
  command = (float)target;
  profiling = false;
}

void ServoAxis::startVibration(int amplitude, unsigned int halfPeriodMs) {
  vibAmplitude = abs(amplitude);
  vibHalfPeriodMs = (halfPeriodMs < 1) ? 1 : halfPeriodMs;
  vibLastToggleMs = millis();
  vibPositive = true;
  vibrating = true;
}

void ServoAxis::stopVibration() {
  vibrating = false;
}

void ServoAxis::update() {
  const unsigned long now = millis();

  if (profiling) {
    command = profile.update();
    if (profile.isFinished()) {
      profiling = false;
      command = (float)target;
    }
  }

  long out = (long)(command + (command >= 0.0f ? 0.5f : -0.5f));

  if (vibrating) {
    if ((now - vibLastToggleMs) >= vibHalfPeriodMs) {
      vibLastToggleMs = now;
      vibPositive = !vibPositive;
    }
    out += vibPositive ? vibAmplitude : -vibAmplitude;
  }

  const int value = clampValue(out);

  // Only touch the I2C bus when the value changed, and no faster than the
  // write interval (servos only sample every 20 ms anyway).
  if (!hasWritten || (value != lastWritten && (now - lastWriteMs) >= writeIntervalMs)) {
    writeNow(value);
  }
}

bool ServoAxis::isFinished() const {
  return !profiling;
}

bool ServoAxis::isVibrating() const {
  return vibrating;
}

int ServoAxis::getTarget() const {
  return target;
}

int ServoAxis::getCommand() const {
  return (int)(command + (command >= 0.0f ? 0.5f : -0.5f));
}

int ServoAxis::getOutput() const {
  return lastWritten;
}

float ServoAxis::getVelocity() const {
  return profiling ? profile.getVelocity() : 0.0f;
}
