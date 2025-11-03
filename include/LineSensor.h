#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>

class LineSensor {
public:
  LineSensor(int s1, int s2, int s3, int s4, int s5);
  int readError();       
  void setDebug(bool on); 
  int* getValues();

private:
  int _pins[5];
  bool _debug;
};

#endif
