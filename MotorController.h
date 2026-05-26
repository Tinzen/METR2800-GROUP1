#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Encoder.h>
#include "PIDController.h"
#include "MotionProfile.h"

class MotorController {
  public:
    enum MotorMode { ACTIVE, HOLD, COAST };
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

    // knownStartPosition is in the selected units:
    // degrees for the intake arm, ticks for the slides.
    void begin(float knownStartPosition = 0.0f);
    void calibratePosition(float knownPosition);

    void setPID(float kp, float ki, float kd);
    void setProfileLimits(float maxVelocity, float maxAcceleration);
    void setLimits(float maxVelocity, float maxAcceleration); // alias
    void setMaxPwm(int maxPwm);
    void setTolerance(float tolerance);
    void setUpdateInterval(unsigned long intervalMs);

    void setTarget(float targetPosition);
    void setTarget(float targetPosition, int maxPwm);
    void setTargetPID(float targetPosition, int maxPwm);
    
    void moveRelative(float deltaPosition);
    void moveRelative(float deltaPosition, int maxPwm);

    float getPosition();
    float getTarget() const;
    float getMovingTarget() const;
    long getRawEncoderTicks();
    int getMotorPower() const;
    bool isAtTarget();
    bool isProfileFinished() const;
    UnitMode getUnitMode() const;

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

    unsigned long lastTime;
    unsigned long interval;
    MotorMode mode;
    float tolerance;

    float rawTicksToPosition(long rawTicks) const;
    long positionToRawTicks(float position) const;
    long roundToLong(float value) const;

    bool useMotionProfile;
};

#endif
