#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>

class Driver {
 private:
  uint8_t ENA;
  uint8_t IN1;
  uint8_t IN2;
  uint8_t ENB;
  uint8_t IN3;
  uint8_t IN4;

 public:
  Driver(uint8_t ENA, uint8_t IN1, uint8_t IN2, uint8_t ENB, uint8_t IN3, uint8_t IN4);
  void init();
  void drive(uint8_t *throttle, int16_t *steering, bool *reverse);
  void stop();
};

#endif