#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
  public:
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float minOutput, float maxOutput);
    void setIntegralLimit(float maxIntegralMagnitude);
    void reset();

    float compute(float target, float position, float dtSeconds);

  private:
    float kp;
    float ki;
    float kd;

    float integral;
    float previousError;
    bool hasPreviousError;

    float minOut;
    float maxOut;
    float integralLimit;
};

#endif
