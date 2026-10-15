#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Encoder.h>
#include "PIDController.h"
#include "MotionProfile.h"

// DC motor + quadrature encoder position controller (motion profile + PID).
//
// V2.1 changes vs v1 (all additive, defaults reproduce v1 behaviour):
//  - Optional feedforward: kS (static friction), kV (per unit/s of profile
//    velocity), kA (per unit/s^2), kG (constant, e.g. gravity on an incline).
//    All default to 0, so behaviour is identical to v1 until you set them.
//  - OPEN_LOOP mode: setOpenLoop(pwm) holds a fixed PWM and stops the PID.
//    In v1, calling setMotor() from the state machine fought with update(),
//    which kept running PID on every tick.
//  - Filtered velocity estimate (getVelocity) for telemetry and stall checks.
//  - setTarget() no longer ignores a repeated target when the motor was in
//    COAST/OPEN_LOOP, so it always re-engages closed loop.
class MotorController {
  public:
    enum MotorMode { ACTIVE, HOLD, COAST, OPEN_LOOP };
    enum UnitMode { DEGREES, TICKS };

    MotorController(int ma, int mb, int mp, int encA, int encB);

    // Unit setup. The controller defaults to DEGREES with 360 ticks/rev
    // unless you override it in setup().
    void setUnitsDegrees(float encoderTicksPerRevolution);
    void setUnitsTicks();

    // Call these before begin() if the mechanism is wired opposite to the
    // software-positive direction.
    void setEncoderDirection(int direction);
    void setMotorDirection(int direction);

    // knownStartPosition is in the selected units.
    void begin(float knownStartPosition = 0.0f);
    void calibratePosition(float knownPosition);

    void setPID(float kp, float ki, float kd);
    void setFeedforward(float kS, float kV, float kA, float kG);   // V2.1
    void setProfileLimits(float maxVelocity, float maxAcceleration);
    void setLimits(float maxVelocity, float maxAcceleration); // alias
    void setMaxPwm(int maxPwm);
    void setTolerance(float tolerance);
    void setUpdateInterval(unsigned long intervalMs);

    void setTarget(float targetPosition);
    void setTarget(float targetPosition, int maxPwm);
    void setTargetPID(float targetPosition, int maxPwm);
    // Profiled move that may start and/or end moving (speeds >= 0, units/s),
    // so consecutive moves blend without stopping.
    void setTargetBlend(float targetPosition, int maxPwm, float startVel, float endVel);

    void moveRelative(float deltaPosition);
    void moveRelative(float deltaPosition, int maxPwm);

    void setOpenLoop(int pwmCommand);                           // V2.1

    float getPosition();
    float getTarget() const;
    float getMovingTarget() const;
    long getRawEncoderTicks();
    int getMotorPower() const;       // driver command (after motor direction)
    int getCommandPower() const;     // V2.1: command in software-positive sense
    int getFeedforward() const;      // V2.1: FF part of last command (PWM)
    float getVelocity() const;       // V2.1: filtered, units/s
    float getProfileVelocity() const;// V2.1
    bool isAtTarget();
    bool isProfileFinished() const;
    float getProfileTotalTime() const;   // V2.1: seconds for the current move
    float getProfileProgress() const;    // 0..1 of the current move
    UnitMode getUnitMode() const;
    MotorMode getMode() const;       // V2.1

    void setMode(MotorMode mode);
    void setMotor(int pwmCommand, int maxPwm);
    void stopMotor();
    void update();

  private:
    int MA;
    int MB;
    int MP;

    Encoder encoder;
    PIDController pid;
    MotionProfile profile;

    UnitMode unitMode;
    float ticksPerUnit;
    int encoderDirection;
    int motorDirection;

    float target;
    float movingTarget;
    int maxCommandPwm;
    int motorPower;
    int commandPower;      // V2.1

    unsigned long lastTime;
    unsigned long interval;
    MotorMode mode;
    float tolerance;

    // V2.1 feedforward, velocity, open loop
    float ffS;
    float ffV;
    float ffA;
    float ffG;
    int lastFeedforward;
    float lastPosition;
    float velocity;
    int openLoopPwm;

    float rawTicksToPosition(long rawTicks) const;
    long positionToRawTicks(float position) const;
    long roundToLong(float value) const;

    bool useMotionProfile;
};

#endif
