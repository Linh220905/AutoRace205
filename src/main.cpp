#include <Arduino.h>
#include "MotorControl.h"
#include "PID.h"
#include "LineSensor.h"

#define START_BUTTON_PIN 14
#define STOP_BUTTON_PIN  15

LineSensor line(2, 3, 4, 5, 6);
float correction = 0;
float lastCorrection = 0;
int error = 0;
int lastError = 0;

bool isRunning = false;
unsigned long start;

bool holdingCorrection = false;    
unsigned long holdStartTime = 0;    

void checkStartButton();
void checkStopButton();

void setup() {
  Serial.begin(115200);

  motor_setup();
  line.setDebug(true);

  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(STOP_BUTTON_PIN, INPUT);

  Serial.println("Sẵn sàng - nhấn START để chạy, STOP để dừng!");
}

void loop() {
  checkStartButton();
  checkStopButton();

  if (!isRunning) {
    motor_stop();
    return;
  }

  int kp, ki, kd;
  int baseSpeed;

 
  if (millis() - start < 10000) {
    baseSpeed = 255;
    kp = 170;
    ki = 10;
    kd = 80;
  }
  else if (millis() - start > 10000 && millis() - start < 500000) {
    
    baseSpeed = 160;
    kp = 200;
    ki = 0;
    kd = 90;

    int newError = line.readError();
   
    if (abs(lastError) > 2 && newError == 0) {
      holdingCorrection = true;
      holdStartTime = millis();
      Serial.println(">>> Giữ correction");
    }

   
    if (holdingCorrection && millis() - holdStartTime > 200) {
      holdingCorrection = false;
      Serial.println("Hết thời gian giữ correction.");
    }

    
    error = newError;
  }
  else {
    baseSpeed = 250;
    kp = 120;
    ki = 90;
    kd = 110;
  }


  PID pid(kp, ki, kd);
  if (holdingCorrection) {
    correction = lastCorrection; 
  } else {
    error = line.readError();
    correction = pid.compute(error);
  }

 
  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  motor_move(leftSpeed, rightSpeed);

  
  Serial.print("Err: "); Serial.print(error);
  Serial.print(" Corr: "); Serial.print(correction);
  Serial.print(" Hold: "); Serial.print(holdingCorrection);
  Serial.print(" L: "); Serial.print(leftSpeed);
  Serial.print(" R: "); Serial.println(rightSpeed);


  lastError = error;
  lastCorrection = correction;
}

void checkStopButton() {
  int value = analogRead(STOP_BUTTON_PIN);
  if (value < 100 && isRunning) {
    Serial.println(" Dừng xe!");
    isRunning = false;
    motor_stop();
    delay(300);
  }
}

void checkStartButton() {
  int value = analogRead(START_BUTTON_PIN);
  if (value < 100 && !isRunning) {
    Serial.println(" Bắt đầu chạy!");
    start = millis();
    isRunning = true;
    delay(300);
  }
}
