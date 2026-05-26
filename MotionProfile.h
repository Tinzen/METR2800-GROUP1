#ifndef MOTIONPROFILE_H
#define MOTIONPROFILE_H

#include <Arduino.h>

class MotionProfile {
  public:
    MotionProfile();

    // Units are whatever the owner uses: degrees for the intake arm,
    // ticks/mm/etc. for linear mechanisms.
    void setLimits(float maxVelocity, float maxAcceleration);
    

    float getTarget() const;
    float getCurrent() const;

    void moveTo(float startPosition, float targetPosition);
    float update();
    bool isFinished() const;

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

    float computeDistance(unsigned long nowMs) const;
};

#endif
