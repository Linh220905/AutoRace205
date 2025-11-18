#include "MotorControl.h"

void motor_setup() {
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  motor_stop();
}

void motor_move(int speedLeft, int speedRight) {
  // Accept -255..255, where sign is direction
  speedLeft = constrain(speedLeft, MIN_SPEED, MAX_SPEED);
  speedRight = constrain(speedRight, MIN_SPEED, MAX_SPEED);

  // LEFT motor
  if (speedLeft > 0) {
    digitalWrite(M2_DIR, LOW);           
    analogWrite(M2_PWM, speedLeft);
  } else if (speedLeft < 0) {
    digitalWrite(M2_DIR, HIGH);          
    analogWrite(M2_PWM, -speedLeft);
  } else {
    motor_stopLeft();
  }

  
  if (speedRight > 0) {
    digitalWrite(M1_DIR, LOW);           
    analogWrite(M1_PWM, speedRight);
  } else if (speedRight < 0) {
    digitalWrite(M1_DIR, HIGH);         
    analogWrite(M1_PWM, -speedRight);
  } else {
    motor_stopRight();
  }
}

void motor_stopLeft() {
  analogWrite(M1_PWM, 0);

  digitalWrite(M1_DIR, LOW);
}

void motor_stopRight() {
  analogWrite(M2_PWM, 0);
  
  digitalWrite(M2_DIR, LOW);
}

void motor_stop() {
  motor_stopLeft();
  motor_stopRight();
}
