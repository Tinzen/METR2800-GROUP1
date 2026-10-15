#ifndef MOTIONPROFILE_H
#define MOTIONPROFILE_H

#include <Arduino.h>

// Trapezoidal / triangular motion profile.
//
// moveTo() behaves exactly like the v1 class (starts and ends at rest).
// moveToVel() can also start and/or end moving, which lets two moves blend
// without stopping in between (slides: fast cruise -> slow creep).
class MotionProfile {
  public:
    MotionProfile();

    // Units are whatever the owner uses (degrees, encoder ticks, ...).
    void setLimits(float maxVelocity, float maxAcceleration);

    float getTarget() const;
    float getCurrent() const;

    void moveTo(float startPosition, float targetPosition);
    // startVel / endVel are speeds (>= 0) in the direction of travel.
    void moveToVel(float startPosition, float targetPosition, float startVel, float endVel);
    float update();
    bool isFinished() const;

    float getVelocity() const;      // signed units/s at last update()
    float getAcceleration() const;  // signed units/s^2 at last update()
    float getTotalTime() const;     // seconds for the current move
    float getProgress() const;      // 0..1 of distance at last update()
    float getMaxVelocity() const;
    float getMaxAcceleration() const;

  private:
    float maxVel;
    float maxAcc;

    float startPos;
    float targetPos;
    float currentPos;
    float direction;
    unsigned long startTimeMs;

    float totalDist;
    float v0;          // entry speed
    float v1;          // exit speed
    float vPeak;
    float acc1;        // signed accel of phase 1 (+ speeds up, - slows down)
    float dec3;        // decel magnitude of phase 3
    float d1, d2;      // distance of phase 1 and 2 (cruise)
    float t1, t2, t3;
    float totalTime;

    bool done;

    float currentVel;
    float currentAcc;
    float progress;

    void plan();
    void sample(float elapsed, float& dist, float& vel, float& acc) const;
};

#endif
