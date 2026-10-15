#ifndef SERVOAXIS_H
#define SERVOAXIS_H

#include <Arduino.h>
#include "MotionProfile.h"

// One servo "axis" (the arm pair, or the scoop), in DEGREES.
//
// Three ways to move:
//   moveTo(t)     profiled (smooth start/stop) at setLimits() speed/accel
//   instantTo(t)  jump: send the target at once, the servo moves at its own
//                 speed (V2 behaviour). Arrival is estimated from
//                 setNativeSpeed() because a hobby servo can't report back.
//   jumpTo(v)     set the command directly (the step runner uses this for
//                 timed / coordinated moves it computes itself)
//
// The writer callback converts degrees to pulse widths (calibration lives
// in the .ino), so this class knows nothing about microseconds.
typedef void (*ServoWriteFn)(float degrees);

class ServoAxis {
  public:
    ServoAxis(ServoWriteFn writer, float minDeg, float maxDeg);

    void setLimits(float maxVelocity, float maxAcceleration);  // deg/s, deg/s^2
    void setNativeSpeed(float degPerSec);                        // for instantTo()
    void setWriteInterval(unsigned long ms);

    void begin(float startDeg);       // write now, no motion
    void moveTo(float targetDeg);
    void instantTo(float targetDeg);
    void jumpTo(float deg);

    void startVibration(float amplitudeDeg, unsigned int halfPeriodMs);
    void stopVibration();

    void update();                    // call every loop()

    bool isFinished() const;          // move complete (estimated for instantTo)
    float getProgress() const;        // 0..1 of the current move
    bool isVibrating() const;
    float getTarget() const;
    float getCommand() const;         // best estimate of where the servo is (no vibration)
    float getOutput() const;          // last value written (incl. vibration)

  private:
    enum Mode { IDLE, PROFILE, INSTANT };
    ServoWriteFn writeFn;
    MotionProfile profile;
    float minVal, maxVal;
    float nativeSpeed;

    Mode mode;
    float target;
    float command;
    float instantFrom;
    unsigned long instantStartMs;
    unsigned long instantDurMs;

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
