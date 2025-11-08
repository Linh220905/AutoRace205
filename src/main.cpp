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

bool holdingCorrection = false;
unsigned long holdStartTime = 0;


unsigned long lastLoopTime = 0;
unsigned long LOOP_INTERVAL = 20; 
// Giữ correction 0.2s khi error ở biên (±4) trong giai đoạn đầu (<9000ms)
const unsigned long HOLD_MAX_ERROR_DURATION = 150; // ms

// Trim động cơ để cân lại tốc độ hai bánh (có thể điều chỉnh sau quan sát).
int TRIM_LEFT = 0;    // giá trị dương làm bánh trái nhanh hơn
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

  // ⏱ chỉ chạy phần PID + điều khiển mỗi 50ms
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_INTERVAL) return;
  lastLoopTime = now;

  // Kiểm tra timeout cho trạng thái giữ correction
  if (holdingCorrection && (now - holdStartTime > HOLD_MAX_ERROR_DURATION)) {
    holdingCorrection = false;
    Serial.println("Het thoi gian giu correction.");
  }

  float kp, ki, kd;
  int baseSpeed;

  if (millis() - start < 3800) {
    baseSpeed = 255;
    kp = 0.53;
    ki = 0.05;
    kd = 0.3;
  }
  else if (millis() - start > 3800 && millis() - start < 9000)
  {

    TRIM_LEFT = 0;
    LOOP_INTERVAL = 19.8;
    
    baseSpeed = 200;
    kp = 0.87;
    ki = 0.003;
    kd = 0.482;

    int newError = line.readError();

    if (abs(lastError) >= 5) {
      holdingCorrection = true;
      holdStartTime = millis();
      Serial.println(">>> Giữ correction");
    }

    if (holdingCorrection && millis() - holdStartTime > 2000) {
      holdingCorrection = false;
      Serial.println("Het thoi gian giu correction.");
    }

    error = newError;

  }
  else if (millis() - start >= 9000 && millis() - start < 500000) {
    TRIM_LEFT = 0;
    LOOP_INTERVAL = 20;
    TRIM_RIGHT = 0;
    baseSpeed = 85;
    kp = 0.5;
    ki = 0;
    kd = 0.14;

    int newError = line.readError();

    if (abs(lastError) > 2 && newError == 0) {
      holdingCorrection = true;
      holdStartTime = millis();
      Serial.println(">>> Giữ correction");
    }

    if (holdingCorrection && millis() - holdStartTime > 1000) {
      holdingCorrection = false;
      Serial.println("Het thoi gian giu correction.");
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
    // Đang giữ correction cũ
    correction = lastCorrection;
  } else {
    int currentError = line.readError();
    // Chỉ áp dụng giữ correction khi error = ±4 trong 9 giây đầu
    if ((now - start) < 8500 && abs(currentError) == 4) {
      holdingCorrection = true;
      holdStartTime = now;
      Serial.println(">>> Giu correction vi error ±4 (<9000ms)");
      correction = lastCorrection; // dùng correction cũ ngay lập tức
      error = currentError;
    } else {
      error = currentError;
      correction = pid.compute(error);
    }
  }

  // Áp dụng correction cân đối + trim điều chỉnh nhẹ, bỏ các offset lớn cố định gây lệch lâu dài
  int leftSpeed  = baseSpeed + correction + TRIM_LEFT;
  int rightSpeed = baseSpeed - correction + TRIM_RIGHT;
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (millis() - start >= 8500 && millis() - start < 500000)
  {
    leftSpeed = max(-255,leftSpeed);
    rightSpeed = max(-255, rightSpeed);
  }

   

  motor_move(leftSpeed, rightSpeed);

  Serial.print("Err: "); Serial.print(error);
  Serial.print(" Corr: "); Serial.print(correction);
  Serial.print(" Hold: "); Serial.print(holdingCorrection);
  Serial.print(" L: "); Serial.print(leftSpeed);
  Serial.print(" R: "); Serial.println(rightSpeed);
   

  lastError = error;
  lastCorrection = correction;

  int adcValue = analogRead(ULTRA_PIN);
  float voltage = adcValue * (5.0 / 1023.0);
  float distance_cm = voltage * 10.0;
  Serial.println(distance_cm);

  if (distance_cm <0 ) {
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
  holdingCorrection = false;
  holdStartTime = 0;
  Serial.println("Da reset toan bo bien!");
}
