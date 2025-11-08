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
    _filteredDerivative = (0.2 * derivative) + (0.8 * _filteredDerivative);
    float dTerm = _Kd * _filteredDerivative;

    // ✅ Chỉ tích lỗi khi error nhỏ (ví dụ ±2) -> tránh kẹt
    if (abs(error) <= 2) {
        _integral += error * dt;
    } else {
        // Nếu lệch lớn, giảm tích phân dần
        _integral *= 0.9;
    }

    // Giới hạn giá trị tích phân để tránh wind-up
    _integral = constrain(_integral, -1000, 1000);

    float iTerm = _Ki * _integral;
    float output = pTerm + iTerm + dTerm;

    // Giới hạn đầu ra
    output = constrain(output, _minOutput, _maxOutput);

    // ✅ Khi error = 0, “xả” dần tích phân để hồi nhanh
    if (error == 0) {
        _integral *= 0.8;
        _filteredDerivative = 0;
    }

    _previousError = error;
    _lastTime = now;

    return output;
}
