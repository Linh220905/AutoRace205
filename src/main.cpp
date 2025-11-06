#include <Arduino.h>
#include "MotorControl.h"
#include "PID.h"
#include "LineSensor.h"

#define START_BUTTON_PIN 14
#define STOP_BUTTON_PIN  15

LineSensor line(2, 3, 4, 5, 7);
float correction = 0;
float lastCorrection = 0;
int error = 0;
int lastError = 0;

// Run control loop at a fixed 50ms period
const unsigned long LOOP_PERIOD_MS = 50;

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
  unsigned long frameStart = millis();
  checkStartButton();
  checkStopButton();
 

  if (!isRunning) {
    motor_stop();
    return;
  }

  float kp, ki, kd;
  int baseSpeed;

 
  if (millis() - start < 1100000) {
    baseSpeed = 140;
    kp = 0.2;
    ki = 0.1;
    kd = 0.3;
  }
  // else if (millis() - start > 11000 && millis() - start < 16000) {
  //   Serial.println("Pass 1");
    
  //   baseSpeed = 170;
  //   kp = 300;
  //   ki = 0;
  //   kd = 50;

  //   int newError = line.readError();
   
  //   if (abs(lastError) > 2 && newError == 0) {
  //     holdingCorrection = true;
  //     holdStartTime = millis();
  //     Serial.println(">>> Giữ correction");
  //   }

   
  //   if (holdingCorrection && millis() - holdStartTime > 350) {
  //     holdingCorrection = false;
  //     Serial.println("Hết thời gian giữ correction.");
  //   }

    
  //   error = newError;
  // }
  // else if (millis() - start > 16000 && millis() - start < 20000) {
  //   Serial.println("Pass 1");
    
  //   baseSpeed = 100;
  //   kp = 150;
  //   ki = 50;
  //   kd = 50;

  // }
   
    
  // else  {
  //   baseSpeed = 250;
  //   kp = 120;
  //   ki = 90;
  //   kd = 110;
  // }


  PID pid(kp, ki, kd);
  if (holdingCorrection) {
    correction = lastCorrection; 
  } else {
    error = line.readError();
    correction = pid.compute(error);
  }

 
  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  leftSpeed  = constrain(leftSpeed , 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  motor_move(leftSpeed, rightSpeed);

  
  Serial.print("Err: "); Serial.print(error);
  Serial.print(" Corr: "); Serial.print(correction);
  Serial.print(" Hold: "); Serial.print(holdingCorrection);
  Serial.print(" L: "); Serial.print(leftSpeed);
  Serial.print(" R: "); Serial.println(rightSpeed);
  



  lastError = error;
  lastCorrection = correction;

  // Enforce 50ms loop time budget
  unsigned long elapsed = millis() - frameStart;
  if (elapsed < LOOP_PERIOD_MS) {
    delay(LOOP_PERIOD_MS - elapsed);
  } else {
    
    Serial.println("Loop overrun");
  }
  Serial.print(millis()- frameStart);
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
