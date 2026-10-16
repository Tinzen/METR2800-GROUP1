#ifndef SERVOAXIS_H
#define SERVOAXIS_H

#include <Arduino.h>
#include "MotionProfile.h"

// One servo "axis" (the arm pair, or the scoop), in DEGREES.
//
// Two ways to move:
//   moveTo(t)     profiled (smooth start/stop) at setLimits() speed/accel. Called while a
//                 move is still running, it carries on from the current speed (or brakes
//                 first if the new target is behind), so there is never a jerk.
//   jumpTo(v)     set the command directly (the editor's follow-the-timeline stream)
// A hobby servo can't report back: "finished" means the profile finished.
//
// The writer callback converts degrees to pulse widths (calibration lives
// in the .ino), so this class knows nothing about microseconds.
typedef void (*ServoWriteFn)(float degrees);

class ServoAxis {
  public:
    ServoAxis(ServoWriteFn writer, float minDeg, float maxDeg);

    void setLimits(float maxVelocity, float maxAcceleration);  // deg/s, deg/s^2
    void setWriteInterval(unsigned long ms);

    void begin(float startDeg);       // write now, no motion
    void moveTo(float targetDeg);
    void jumpTo(float deg);

    void startVibration(float amplitudeDeg, unsigned int halfPeriodMs);
    void stopVibration();

    void update();                    // call every loop()

    bool isFinished() const;          // profile complete
    float getProgress() const;        // 0..1 of the current move
    bool isVibrating() const;
    float getTarget() const;
    float getCommand() const;         // best estimate of where the servo is (no vibration)
    float getOutput() const;          // last value written (incl. vibration)

  private:
    enum Mode { IDLE, PROFILE };
    ServoWriteFn writeFn;
    MotionProfile profile;
    float minVal, maxVal;

    Mode mode;
    float target;
    float command;

    float lastWritten;
    unsigned long lastWriteMs;
    unsigned long writeIntervalMs;
    bool hasWritten;

    bool vibrating;
    float vibAmplitude;
    unsigned int vibHalfPeriodMs;
    unsigned long vibLastToggleMs;
    bool vibPositive;

    float clampValue(float v) const;
    void writeNow(float v);
};

#endif
