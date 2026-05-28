#include "PIDController.h"
#include <Arduino.h>
#include <math.h>

PIDController::PIDController(float p, float i, float d)
: kp(p),
  ki(i),
  kd(d),
  integral(0.0f),
  previousError(0.0f),
  hasPreviousError(false),
  minOut(-255.0f),
  maxOut(255.0f),
  integralLimit(1000.0f)
{}

void PIDController::setTunings(float p, float i, float d) {
  kp = p;
  ki = i;
  kd = d;
}

void PIDController::setOutputLimits(float minOutput, float maxOutput) {
  if (minOutput > maxOutput) {
    const float temp = minOutput;
    minOutput = maxOutput;
    maxOutput = temp;
  }

  minOut = minOutput;
  maxOut = maxOutput;
}

void PIDController::setIntegralLimit(float maxIntegralMagnitude) {
  integralLimit = fabs(maxIntegralMagnitude);
}

void PIDController::reset() {
  integral = 0.0f;
  previousError = 0.0f;
  hasPreviousError = false;
}

float PIDController::compute(float target, float position, float dtSeconds) {
  if (dtSeconds <= 0.0f) return 0.0f;

  const float error = target - position;

  integral += error * dtSeconds;
  integral = constrain(integral, -integralLimit, integralLimit);

  float derivative = 0.0f;
  if (hasPreviousError) {
    derivative = (error - previousError) / dtSeconds;
  }

  previousError = error;
  hasPreviousError = true;

  float output = (kp * error) + (ki * integral) + (kd * derivative);
  return constrain(output, minOut, maxOut);
}
