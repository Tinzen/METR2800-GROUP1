#include "ServoAxis.h"
#include <math.h>

// Objects of this class are globals (zero-filled at boot), so the constructor
// only sets the members that must start non-zero. Keeps flash small on the Nano.
ServoAxis::ServoAxis(ServoWriteFn writer, float minDeg, float maxDeg)
: writeFn(writer), minVal(minDeg), maxVal(maxDeg),
  mode(IDLE), writeIntervalMs(20), vibHalfPeriodMs(50), vibPositive(true)
{}

void ServoAxis::setLimits(float maxVelocity, float maxAcceleration, float maxDeceleration) {
  profile.setLimits(maxVelocity, maxAcceleration, maxDeceleration);
}

void ServoAxis::setWriteInterval(unsigned long ms) { writeIntervalMs = ms; }

float ServoAxis::clampValue(float v) const {
  if (v < minVal) return minVal;
  if (v > maxVal) return maxVal;
  return v;
}

void ServoAxis::writeNow(float v) {
  writeFn(v);
  lastWritten = v;
  lastWriteMs = millis();
  hasWritten = true;
}

void ServoAxis::begin(float startDeg) {
  target = clampValue(startDeg);
  command = target;
  mode = IDLE;
  profile.moveTo(target, target);
  writeNow(target);
}

void ServoAxis::moveTo(float targetDeg) {
  target = clampValue(targetDeg);
  // take over a running move at its current speed (smooth), or start from rest
  profile.moveFrom(command, target, (mode == PROFILE) ? profile.getVelocity() : 0.0f);
  mode = PROFILE;
}

void ServoAxis::jumpTo(float deg) {
  command = clampValue(deg);
  target = command;
  mode = IDLE;
}

void ServoAxis::startVibration(float amplitudeDeg, unsigned int halfPeriodMs) {
  vibrating = amplitudeDeg > 0.0f;
  vibAmplitude = amplitudeDeg;
  vibHalfPeriodMs = halfPeriodMs;
  vibLastToggleMs = millis();
  vibPositive = true;
}

void ServoAxis::stopVibration() { vibrating = false; }

void ServoAxis::update() {
  const unsigned long now = millis();
  float out;

  if (mode == PROFILE) {
    command = profile.update();
    if (profile.isFinished()) mode = IDLE;
    out = command;
  } else {
    out = command;
  }

  if (vibrating) {
    if ((now - vibLastToggleMs) >= vibHalfPeriodMs) {
      vibLastToggleMs = now;
      vibPositive = !vibPositive;
    }
    out += vibPositive ? vibAmplitude : -vibAmplitude;
  }
  out = clampValue(out);

  // Servos only read a new pulse every ~20 ms; writing more often just loads I2C.
  if (!hasWritten || (fabs(out - lastWritten) > 0.05f && (now - lastWriteMs) >= writeIntervalMs)) {
    writeNow(out);
  }
}

bool ServoAxis::isFinished() const { return mode == IDLE; }

float ServoAxis::getProgress() const {
  return (mode == PROFILE) ? profile.getProgress() : 1.0f;
}

float ServoAxis::getTimeLeft() const {
  return (mode == PROFILE) ? profile.getTimeLeft() : 0.0f;
}

bool ServoAxis::isVibrating() const { return vibrating; }
float ServoAxis::getTarget() const { return target; }
float ServoAxis::getCommand() const { return command; }
float ServoAxis::getOutput() const { return lastWritten; }
