#ifndef MOTIONPROFILE_H
#define MOTIONPROFILE_H

#include <Arduino.h>

// Trapezoidal / triangular motion profile.
//
// Identical behaviour to the v1 class. V2.1 only adds read-only getters for
// the profile's velocity and acceleration at the last update(). These are used
// for motor feedforward (kV, kA) and for telemetry.
class MotionProfile {
  public:
    MotionProfile();

    // Units are whatever the owner uses: servo units, microseconds,
    // encoder ticks, etc.
    void setLimits(float maxVelocity, float maxAcceleration);

    float getTarget() const;
    float getCurrent() const;

    void moveTo(float startPosition, float targetPosition);
    float update();
    bool isFinished() const;

    // ---- V2.1 additions (read-only, do not change v1 behaviour) ----
    float getVelocity() const;      // signed units/s at last update()
    float getAcceleration() const;  // signed units/s^2 at last update()
    float getTotalTime() const;     // seconds for the current move
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
    float accelDist;
    float cruiseDist;
    float vPeak;
    float accelTime;
    float cruiseTime;
    float totalTime;

    bool done;

    float currentVel;   // V2.1
    float currentAcc;   // V2.1

    float computeDistance(unsigned long nowMs) const;
    void computeRates(float elapsedSec);  // V2.1
};

#endif
