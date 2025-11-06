#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>


#define IN1 9  
#define IN2 10
#define IN3 12
#define IN4 11



#define MAX_SPEED 255
#define MIN_SPEED -255


void motor_setup();

void motor_move(int speedLeft, int speedRight);

void motor_stop();
void motor_stopLeft();
void motor_stopRight();

#endif
