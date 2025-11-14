#include "PID.h"

PID::PID(float Kp, float Ki, float Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _integral = 0;
    _previousError = 0;
    _filteredDerivative = 0; 
    _minOutput = -600;
    _maxOutput = 600;
    _lastTime = millis();
}

void PID::set(float Kp, float Ki, float Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void PID::setOutputLimits(float minOutput, float maxOutput) {
    _minOutput = minOutput;
    _maxOutput = maxOutput;
}

void PID::reset() {
    _integral = 0;
    _previousError = 0;
    _filteredDerivative = 0;
    _lastTime = millis();
}

float PID::compute(float error) {
    float p = error;
    _integral += p;
    float d = p - _previousError;

    _previousError = p;

    float correction = _Kp * p + _Ki * _integral + _Kd * d;

   
    if (correction > _maxOutput) correction = _maxOutput;
    else if (correction < _minOutput) correction = _minOutput;

    return correction;
}
