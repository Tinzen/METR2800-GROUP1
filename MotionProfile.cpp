#include "MotionProfile.h"
#include <math.h>

MotionProfile::MotionProfile()
: maxVel(100.0f),
  maxAcc(200.0f),
  startPos(0.0f),
  targetPos(0.0f),
  currentPos(0.0f),
  direction(1.0f),
  startTimeMs(0),
  totalDist(0.0f),
  accelDist(0.0f),
  cruiseDist(0.0f),
  vPeak(0.0f),
  accelTime(0.0f),
  cruiseTime(0.0f),
  totalTime(0.0f),
  done(true)
{}

void MotionProfile::setLimits(float maxVelocity, float maxAcceleration) {
  maxVel = fabs(maxVelocity);
  maxAcc = fabs(maxAcceleration);

  // Guard against divide-by-zero profiles.
  if (maxVel < 0.001f) maxVel = 0.001f;
  if (maxAcc < 0.001f) maxAcc = 0.001f;
}

float MotionProfile::getTarget() const {
  return targetPos;
}

float MotionProfile::getCurrent() const {
  return currentPos;
}

void MotionProfile::moveTo(float startPosition, float targetPosition) {
  startPos = startPosition;
  targetPos = targetPosition;
  currentPos = startPosition;
  startTimeMs = millis();

  const float signedDistance = targetPos - startPos;
  direction = (signedDistance >= 0.0f) ? 1.0f : -1.0f;
  totalDist = fabs(signedDistance);

  if (totalDist < 0.001f) {
    accelDist = 0.0f;
    cruiseDist = 0.0f;
    vPeak = 0.0f;
    accelTime = 0.0f;
    cruiseTime = 0.0f;
    totalTime = 0.0f;
    currentPos = targetPos;
    done = true;
    return;
  }

  // Symmetric accel/decel. If max velocity cannot be reached, the profile
  // becomes triangular. Otherwise it becomes trapezoidal.
  const float triangularPeakVel = sqrt(totalDist * maxAcc);
  vPeak = (triangularPeakVel < maxVel) ? triangularPeakVel : maxVel;

  accelTime = vPeak / maxAcc;
  accelDist = 0.5f * maxAcc * accelTime * accelTime;

  if ((2.0f * accelDist) >= totalDist) {
    accelDist = totalDist * 0.5f;
    cruiseDist = 0.0f;
    vPeak = sqrt(2.0f * maxAcc * accelDist);
    accelTime = vPeak / maxAcc;
    cruiseTime = 0.0f;
  } else {
    cruiseDist = totalDist - (2.0f * accelDist);
    cruiseTime = cruiseDist / vPeak;
  }

  totalTime = (2.0f * accelTime) + cruiseTime;
  done = false;
}

float MotionProfile::computeDistance(unsigned long nowMs) const {
  if (done) return totalDist;

  float elapsed = (nowMs - startTimeMs) / 1000.0f;

  if (elapsed <= 0.0f) return 0.0f;
  if (elapsed >= totalTime) return totalDist;

  if (elapsed < accelTime) {
    return 0.5f * maxAcc * elapsed * elapsed;
  }

  elapsed -= accelTime;

  if (elapsed < cruiseTime) {
    return accelDist + (vPeak * elapsed);
  }

  elapsed -= cruiseTime;

  float position = accelDist + cruiseDist + (vPeak * elapsed)
                 - (0.5f * maxAcc * elapsed * elapsed);

  if (position < 0.0f) position = 0.0f;
  if (position > totalDist) position = totalDist;
  return position;
}

float MotionProfile::update() {
  if (done) return targetPos;

  const unsigned long now = millis();
  const float profiledDistance = computeDistance(now);

  currentPos = startPos + (direction * profiledDistance);

  const float elapsed = (now - startTimeMs) / 1000.0f;
  if (elapsed >= totalTime || profiledDistance >= totalDist) {
    currentPos = targetPos;
    done = true;
  }

  return currentPos;
}

bool MotionProfile::isFinished() const {
  return done;
}
