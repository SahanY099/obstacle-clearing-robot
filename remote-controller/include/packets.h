#ifndef PACKETS_H
#define PACKETS_H

#include <Arduino.h>

#pragma pack(1)
struct UpStream {
  bool reverse;
  int16_t steering;
  uint8_t throttle;
  uint8_t brakeDistance;
};

#endif