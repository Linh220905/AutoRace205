
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
    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001; 

    
    float pTerm = _Kp * error;

 
    float derivative = (error - _previousError) / dt;
    
    // Áp dụng bộ lọc thông thấp (Exponential Moving Average)
    // Alpha nhỏ -> lọc mạnh hơn, D-term mượt hơn nhưng trễ hơn
    // Alpha lớn -> lọc yếu hơn, D-term nhạy hơn nhưng giật hơn
    _filteredDerivative = (0.2 * derivative) + (0.8 * _filteredDerivative); 
    
    float dTerm = _Kd * _filteredDerivative;


    float iTermPotential = _Ki * (_integral + error * dt);

   
    float output = pTerm + iTermPotential + dTerm;

  
    bool saturated = false;
    if (output > _maxOutput) {
        output = _maxOutput;
        saturated = true;
    } else if (output < _minOutput) {
        output = _minOutput;
        saturated = true;
    }

   
    if (_Ki != 0 && !saturated) {
        _integral += error * dt;
    }

 
    _previousError = error;
    _lastTime = now;

    return output;
}