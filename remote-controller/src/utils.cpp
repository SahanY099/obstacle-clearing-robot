#include <Arduino.h>

#include "packets.h"

void printDataPacket(UpStream packet) {
  Serial.print("Steering: " + String(packet.steering) + " ");
  Serial.print("Throttle: " + String(packet.throttle) + " ");
  Serial.println("Reverse: " + String(packet.reverse) + " ");
  // Serial.print("Mode: " + String(packet.mode) + " ");
  // Serial.print("Pot L: " + String(packet.potL) + " ");
  // Serial.print("Pot R: " + String(packet.potR));
  // Serial.println("Pot M: " + String(packet.potM));
}

int mapAndAdjustJoystickDeadBandValues(int value, bool reverse) {
  if (value >= 550) {
    value = map(value, 550, 1023, 0, 255);
  } else if (value <= 480) {
    value = map(value, 480, 0, -255, 0);
  } else {
    value = 0;
  }

  if (reverse) {
    value = -1 * value;
  }

  return value;
}