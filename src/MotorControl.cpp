#include "MotorControl.h"

void motor_setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  motor_stop();
}

// Hàm điều khiển động cơ với tốc độ và hướng
void motor_move(int speedLeft, int speedRight) {
  speedLeft  = constrain(speedLeft,  -MAX_SPEED, MAX_SPEED);
  speedRight = constrain(speedRight, -MAX_SPEED, MAX_SPEED);

  // --- Động cơ trái ---
  if (speedLeft > 0) {           // Tiến
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speedLeft);
  } else if (speedLeft < 0) {    // Lùi
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -speedLeft);
  } else {                       // Dừng
    motor_stopLeft();
  }

  // --- Động cơ phải ---
  if (speedRight > 0) {         
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speedRight);
  } else if (speedRight < 0) {   
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -speedRight);
  } else {                     
    motor_stopRight();
  }
}

void motor_stopLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void motor_stopRight() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void motor_stop() {
  motor_stopLeft();
  motor_stopRight();
}
