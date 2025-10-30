#include <Arduino.h>
#include "MotorControl.h"
#include "PID.h"
#include "LineSensor.h"

#define START_BUTTON_PIN 14   
#define STOP_BUTTON_PIN  15  

LineSensor line(2, 3, 4, 5, 6); 
PID pid(10,15,10);


bool isRunning = false; 
unsigned long start;

void checkStartButton();
void checkStopButton();

void setup() {
  Serial.begin(115200);

  motor_setup();
  line.setDebug(true);

  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(STOP_BUTTON_PIN, INPUT);

  Serial.println("Sẵn sàng - nhấn nút START để bắt đầu, STOP để dừng!");
  
}

void loop() {
  checkStartButton();
  checkStopButton();

  int baseSpeed;

  if (!isRunning) {
    motor_stop();
    return;
  }

 
 
  int error = line.readError();
 
  float correction = pid.compute(error);
  

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  motor_move(leftSpeed , rightSpeed);

  Serial.print("Error: "); Serial.print(error);
  Serial.print("  Correction: "); Serial.print(correction);
  Serial.print("  Left: "); Serial.print(leftSpeed);
  Serial.print("  Right: "); Serial.println(rightSpeed);
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