#include <Arduino.h>
#include "MotorControl.h"
#include "PID.h"
#include "LineSensor.h"

#define START_BUTTON_PIN 14
#define STOP_BUTTON_PIN  15
#define ULTRA_PIN A5

LineSensor line(3, 8, 9, 10, 11);
 

float correction = 0;
float lastCorrection = 0;
int error = 0;
int lastError = 0;

bool isRunning = false;
unsigned long start;

PID pid(0, 0, 0);


unsigned long holdUntilTime = 0; 
const unsigned long HOLD_INTERSECTION_DURATION = 420;
const unsigned long HOLD_MAX_ERROR_DURATION = 150;   

unsigned long lastLoopTime = 0;
unsigned long LOOP_INTERVAL = 20;


int TRIM_LEFT = 0;
int TRIM_RIGHT = 0;

void checkStartButton();
void checkStopButton();
void resetAllStates();

void setup() {
  Serial.begin(115200);
  motor_setup();
  line.setDebug(true);
  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(STOP_BUTTON_PIN, INPUT);
  Serial.println("San sang - nhan START de chay, STOP de dung!");
}

void loop() {
  checkStartButton();
  checkStopButton();

  if (!isRunning) {
    motor_stop();
    return;
  }

  unsigned long now = millis(); 
  unsigned long loopTime = now - start;
   

 
  float kp, ki, kd;
  int baseSpeed;

  if (loopTime < 3800) {
    
    baseSpeed = 255;
    kp = 0.53;
    ki = 0.05;
    kd = 0.3;
    LOOP_INTERVAL = 20;
  }
  else if (loopTime < 9000) {
   
    TRIM_LEFT = 0;
    baseSpeed = 200;
    kp = 0.87;
    ki = 0.003;
    kd = 0.482;
    LOOP_INTERVAL = 19.8; 
  }
  else if (loopTime < 500000) {
    
    TRIM_LEFT = 0;
    TRIM_RIGHT = 0;
    baseSpeed = 160;
    kp = 200;
    ki = 0;
    kd = 100;
    LOOP_INTERVAL = 0;
  }
  else {
   
    baseSpeed = 250;
    kp = 120;
    ki = 90;
    kd = 110;
    LOOP_INTERVAL = 20;
  }

  pid.set(kp,ki,kd);
  

 
  if (LOOP_INTERVAL > 0 && (now - lastLoopTime < LOOP_INTERVAL)) {
    return;
  }
  lastLoopTime = now;


  error = line.readError(); 

  
  bool isHolding = (now < holdUntilTime);

  if (!isHolding) { 
    if (loopTime >= 9000 && loopTime < 500000) {
      
      if (abs(lastError) > 2 && error == 0) {
        holdUntilTime = now + HOLD_INTERSECTION_DURATION;
        Serial.println(">>> GIỮ (Ngã rẽ)");
      }
    } 
  }

 
  isHolding = (now < holdUntilTime);

 

  if (isHolding) {
    correction = lastCorrection; 
  } else {
    correction = pid.compute(error); 
  }


  int leftSpeed  = baseSpeed + correction + TRIM_LEFT;
  int rightSpeed = baseSpeed - correction + TRIM_RIGHT;
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (loopTime >= 9000 && loopTime < 500000)
  {
    leftSpeed = max(-50,leftSpeed);
    rightSpeed = max(-50, rightSpeed);
  }

 

  motor_move(leftSpeed, rightSpeed);

  
  Serial.print("Err: "); Serial.print(error);
  Serial.print(" Corr: "); Serial.print(correction);
  Serial.print(" Hold: "); Serial.print(isHolding);
  Serial.print(" L: "); Serial.print(leftSpeed);
  Serial.print(" R: "); Serial.println(rightSpeed);

 
  lastError = error;
  if (!isHolding) {
   
    lastCorrection = correction;
  }

 
  int adcValue = analogRead(ULTRA_PIN);
  float voltage = adcValue * (5.0 / 1023.0);
  float distance_cm = voltage * 10.0;
  Serial.println(distance_cm);

  if (distance_cm < 0) {
    Serial.println("Vat can gan! Dung xe lai!");
    motor_stop();
    isRunning = false;
    delay(300);
  }
}

void checkStopButton() {
  int value = analogRead(STOP_BUTTON_PIN);
  if (value < 100 && isRunning) {
    Serial.println("Dung xe!");
    isRunning = false;
    motor_stop();
    delay(300);
  }
}

void checkStartButton() {
  int value = analogRead(START_BUTTON_PIN);
  if (value < 100 && !isRunning) {
    Serial.println("Bat dau chay!");
    start = millis();
    resetAllStates();
    isRunning = true;
    lastLoopTime = millis();
    delay(300);
  }
}

void resetAllStates() {
  error = 0;
  lastError = 0;
  correction = 0;
  lastCorrection = 0;
  holdUntilTime = 0; 
  Serial.println("Da reset toan bo bien!");
}