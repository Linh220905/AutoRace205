#include "MotorControl.h"

void motor_setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  motor_stop();
}

void motor_move(int speedLeft, int speedRight) {
  
  speedLeft = constrain(speedLeft, MIN_SPEED, MAX_SPEED);
  speedRight = constrain(speedRight, MIN_SPEED, MAX_SPEED);


  if (speedLeft > 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, speedLeft);
  } else if (speedLeft < 0) {
    analogWrite(IN1,-speedLeft );
    analogWrite(IN2, 0);
  } else {
    motor_stopLeft();
  }

  if (speedRight > 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, speedRight);
  } else if (speedRight < 0) {
   
    analogWrite(IN4, -speedRight);
    analogWrite(IN3, 0);
  } else {
    motor_stopRight();
  }
}

void motor_stopLeft() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}

void motor_stopRight() {
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void motor_stop() {
  motor_stopLeft();
  motor_stopRight();
}