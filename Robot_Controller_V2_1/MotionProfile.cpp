#include "MotionProfile.h"
#include <math.h>

// Used as a member of global objects (zero-filled at boot): only the members
// that must start non-zero are set here, which keeps flash small on the Nano.
MotionProfile::MotionProfile()
: maxVel(100.0f), maxAcc(200.0f), direction(1.0f), done(true), progress(1.0f)
{}

void MotionProfile::setLimits(float maxVelocity, float maxAcceleration) {
  maxVel = fabs(maxVelocity);
  maxAcc = fabs(maxAcceleration);
  if (maxVel < 0.001f) maxVel = 0.001f;   // guard against divide-by-zero
  if (maxAcc < 0.001f) maxAcc = 0.001f;
}

float MotionProfile::getTarget() const { return targetPos; }
float MotionProfile::getCurrent() const { return currentPos; }

void MotionProfile::moveTo(float startPosition, float targetPosition) {
  moveToVel(startPosition, targetPosition, 0.0f, 0.0f);
}

void MotionProfile::moveToVel(float startPosition, float targetPosition, float startVel, float endVel) {
  startPos = startPosition;
  targetPos = targetPosition;
  currentPos = startPosition;
  startTimeMs = millis();
  currentVel = 0.0f;
  currentAcc = 0.0f;
  progress = 0.0f;

  const float signedDistance = targetPos - startPos;
  direction = (signedDistance >= 0.0f) ? 1.0f : -1.0f;
  totalDist = fabs(signedDistance);
  v0 = fabs(startVel);
  v1 = fabs(endVel);
  if (v1 > maxVel) v1 = maxVel;

  if (totalDist < 0.001f) {
    vPeak = 0.0f; acc1 = 0.0f; dec3 = maxAcc;
    d1 = d2 = 0.0f; t1 = t2 = t3 = 0.0f; totalTime = 0.0f;
    currentPos = targetPos;
    progress = 1.0f;
    done = true;
    return;
  }
  plan();
  done = false;
}

// Phase 1: v0 -> vPeak (accel or decel), phase 2: cruise, phase 3: vPeak -> v1.
// With v0 = v1 = 0 this is the v1 class's symmetric trapezoid/triangle.
void MotionProfile::plan() {
  const float a = maxAcc;
  const float d = totalDist;
  float vp = maxVel;
  if (v0 > vp) vp = v0;            // arriving faster than this move's cruise: slow down first

  float da = fabs(vp * vp - v0 * v0) / (2.0f * a);
  float dd = (vp * vp - v1 * v1) / (2.0f * a);

  dec3 = a;
  if (da + dd > d) {
    // Not enough room to reach cruise speed: triangular peak.
    float vt2 = a * d + 0.5f * (v0 * v0 + v1 * v1);
    float vt = sqrt(vt2);
    if (vt >= v0 && vt >= v1) {
      vp = vt;
    } else if (v1 > v0) {
      // can't even reach v1: accelerate the whole way
      vp = sqrt(v0 * v0 + 2.0f * a * d);
      v1 = vp;
    } else {
      // can't slow to v1 in time: decelerate the whole way, harder than maxAcc if needed
      vp = v0;
      dec3 = (v0 * v0 - v1 * v1) / (2.0f * d);
      if (dec3 < 0.001f) dec3 = 0.001f;
    }
    da = fabs(vp * vp - v0 * v0) / (2.0f * a);
    dd = (vp * vp - v1 * v1) / (2.0f * dec3);
    if (da + dd > d) {                 // float rounding
      const float s = d / (da + dd);
      da *= s; dd *= s;
    }
  }

  vPeak = vp;
  acc1 = (vp >= v0) ? a : -a;
  d1 = da;
  d2 = d - da - dd;
  if (d2 < 0.0f) d2 = 0.0f;
  t1 = fabs(vp - v0) / a;
  t2 = (vp > 0.0001f) ? d2 / vp : 0.0f;
  t3 = (vp - v1) / dec3;
  if (t3 < 0.0f) t3 = 0.0f;
  totalTime = t1 + t2 + t3;
}

void MotionProfile::sample(float e, float& dist, float& vel, float& acc) const {
  if (e <= 0.0f) { dist = 0.0f; vel = v0; acc = acc1; return; }
  if (e < t1) {
    dist = v0 * e + 0.5f * acc1 * e * e;
    vel = v0 + acc1 * e;
    acc = acc1;
    return;
  }
  e -= t1;
  if (e < t2) {
    dist = d1 + vPeak * e;
    vel = vPeak;
    acc = 0.0f;
    return;
  }
  e -= t2;
  if (e < t3) {
    dist = d1 + d2 + vPeak * e - 0.5f * dec3 * e * e;
    vel = vPeak - dec3 * e;
    acc = -dec3;
    return;
  }
  dist = totalDist;
  vel = v1;
  acc = 0.0f;
}

float MotionProfile::update() {
  if (done) {
    currentVel = 0.0f;
    currentAcc = 0.0f;
    return targetPos;
  }

  const unsigned long now = millis();
  const float elapsed = (now - startTimeMs) / 1000.0f;
  float dist, vel, acc;
  sample(elapsed, dist, vel, acc);
  if (dist < 0.0f) dist = 0.0f;
  if (dist > totalDist) dist = totalDist;

  currentPos = startPos + direction * dist;
  currentVel = direction * vel;
  currentAcc = direction * acc;
  progress = (totalDist > 0.0f) ? dist / totalDist : 1.0f;

  if (elapsed >= totalTime) {
    currentPos = targetPos;
    currentVel = 0.0f;
    currentAcc = 0.0f;
    progress = 1.0f;
    done = true;
  }
  return currentPos;
}

bool MotionProfile::isFinished() const { return done; }
float MotionProfile::getVelocity() const { return currentVel; }
float MotionProfile::getAcceleration() const { return currentAcc; }
float MotionProfile::getTotalTime() const { return totalTime; }
float MotionProfile::getProgress() const { return progress; }
float MotionProfile::getMaxVelocity() const { return maxVel; }
float MotionProfile::getMaxAcceleration() const { return maxAcc; }
