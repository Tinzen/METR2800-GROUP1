#ifndef SERVOAXIS_H
#define SERVOAXIS_H

#include <Arduino.h>
#include "MotionProfile.h"

// One profiled "axis" driven by hobby servo(s), using the same MotionProfile
// class as the v1 intake motor.
//
// There is no position feedback, so the profile is the commanded trajectory.
// If the limits are faster than the servo can physically move, the servo lags
// behind the profile and you lose the smooth start and stop. Tune the limits
// below the servo's loaded speed.
//
// The axis works in whatever units the writer function expects:
//   arm   -> 0..1000 position (mirrored onto both arm servos)
//   scoop -> pulse width in microseconds
//
// Optional vibration adds +/-amplitude on top of the command, toggling every
// halfPeriodMs (same behaviour as v1 vibrateServo()).
typedef void (*ServoWriteFn)(int value);

class ServoAxis {
  public:
    ServoAxis(ServoWriteFn writer, int minValue, int maxValue);

    void setLimits(float maxVelocity, float maxAcceleration); // units/s, units/s^2
    void setWriteInterval(unsigned long ms);   // min ms between servo writes

    void begin(int startValue);       // write immediately, no profile
    void moveTo(int targetValue);     // profiled move from the current command
    void jumpTo(int value);           // set command directly (coordinated paths)

    void startVibration(int amplitude, unsigned int halfPeriodMs);
    void stopVibration();

    void update();                    // call every loop()

    bool isFinished() const;          // profile complete
    bool isVibrating() const;
    int getTarget() const;
    int getCommand() const;           // profiled command, no vibration
    int getOutput() const;            // last value written (incl. vibration)
    float getVelocity() const;        // profile velocity, units/s

  private:
    ServoWriteFn writeFn;
    MotionProfile profile;
    int minVal;
    int maxVal;

    int target;
    float command;
    bool profiling;

    int lastWritten;
    unsigned long lastWriteMs;
    unsigned long writeIntervalMs;
    bool hasWritten;

    bool vibrating;
    int vibAmplitude;
    unsigned int vibHalfPeriodMs;
    unsigned long vibLastToggleMs;
    bool vibPositive;

    int clampValue(long value) const;
    void writeNow(int value);
};

#endif
