#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
  PID(float Kp, float Ki, float Kd);

  
  float compute(float error);

  
  void set(float Kp, float Ki, float Kd);
  void setOutputLimits(float minOutput, float maxOutput);
  void reset();

private:
  float _Kp, _Ki, _Kd;
  float _integral;
  float _previousError;
  float _minOutput, _maxOutput;
  unsigned long _lastTime;
  float _filteredDerivative;
};

#endif
