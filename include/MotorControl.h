#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

// Motor 1 (Left)
#define M1_PWM 6
#define M1_DIR 7

// Motor 2 (Right)
#define M2_PWM 5
#define M2_DIR 4


#define MAX_SPEED 255
#define MIN_SPEED -255


void motor_setup();

void motor_move(int speedLeft, int speedRight);

void motor_stop();
void motor_stopLeft();
void motor_stopRight();

#endif
