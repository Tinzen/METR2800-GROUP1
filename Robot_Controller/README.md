# Intake degree refactor

This version removes FeedForward entirely and makes the intake arm use degrees as its default command unit.

## Files to keep in the Arduino sketch folder

- Robot_Controller.ino
- MotorController.h
- MotorController.cpp
- PIDController.h
- PIDController.cpp
- MotionProfile.h
- MotionProfile.cpp

FeedForward.h and FeedForward.cpp are no longer needed.

## Main values to edit

In Robot_Controller.ino, edit the constants at the top:

- INTAKE_START_DEGREES: physical arm angle at boot, in degrees. Use 0.0 if the arm starts at zero degrees.
- INTAKE_ENCODER_TICKS_PER_ARM_REV: encoder counts for one full 360 degree rotation of the arm.
- INTAKE_KP, INTAKE_KI, INTAKE_KD: PID gains in PWM per degree.
- INTAKE_MAX_DEG_PER_SEC and INTAKE_MAX_DEG_PER_SEC2: motion profile limits in degrees.
- MOVE_INTAKE_ON_BOOT and INTAKE_BOOT_TARGET_DEGREES: simple startup test move.

For linear slides, the outtake remains in encoder ticks.

## Example

If INTAKE_START_DEGREES is 0.0 and you call:

```cpp
intake.setTarget(90.0f, INTAKE_MAX_PWM);
```

then the intake motion profile moves from 0 degrees to 90 degrees.
