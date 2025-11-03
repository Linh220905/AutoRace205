#include "LineSensor.h"

LineSensor::LineSensor(int s1, int s2, int s3, int s4, int s5) {
  _pins[0] = s1;
  _pins[1] = s2;
  _pins[2] = s3;
  _pins[3] = s4;
  _pins[4] = s5;
  _debug = false;

  for (int i = 0; i < 5; i++) pinMode(_pins[i], INPUT);
}

void LineSensor::setDebug(bool on) {
  _debug = on;
}

int* LineSensor::getValues() {
    static int values[5];
    for (int i = 0; i < 5; i++) {
        values[i] = digitalRead(_pins[i]);
    }
    return values;
}


int LineSensor::readError() {
  int sensors[5];
  int pattern = 0;

  
  for (int i = 0; i < 5; i++) {
    sensors[i] = digitalRead(_pins[i]);
    pattern = (pattern << 1) | sensors[i];
  }

  int error = 0;

 
  switch (pattern) {

    case 0b11111: error = 0; break;  
    case 0b01111: error = -4; break;  
    case 0b00111: error = -3; break;
    case 0b00011: error = -2; break;
    case 0b10111: error = -2; break;
    case 0b10011: error = -1; break;
    case 0b00000: error = 0;  break; 
    case 0b11011: error = 0; break;
    case 0b11001: error = +1; break;
    case 0b11101: error = +2; break;
    case 0b11000: error = +2; break;
    case 0b11100: error = +3; break;
    case 0b11110: error = +4; break;

    default: error = 0; break;        
  }

  if (_debug) {
    Serial.print("Sensors: ");
    for (int i = 0; i < 5; i++) Serial.print(sensors[i]);
    Serial.print(" | Pattern: ");
    Serial.print(pattern, BIN);
    Serial.print(" | Error: ");
    Serial.println(error);
  }

  return error;
}


  // case 0b11111: error = 0; break;  
  //   case 0b10000: error = -4; break;  
  //   case 0b11000: error = -3; break;
  //   case 0b01000: error = -2; break;
  //   case 0b01100: error = -1; break;
  //   case 0b00000: error = 0;  break; 
  //   case 0b00100: error = 0; break;
  //   case 0b00110: error = +1; break;
  //   case 0b00010: error = +2; break;
  //   case 0b00011: error = +3; break;
  //   case 0b00001: error = +4; break;