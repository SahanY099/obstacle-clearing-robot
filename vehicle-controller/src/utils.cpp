#include "utils.h"

void printRemoteDownStream(HardwareSerial &debugSerial, DownStream &downStream) {
  debugSerial.print("Steering: " + String(downStream.steering) + " ");
  debugSerial.print("Throttle: " + String(downStream.throttle) + " ");
  debugSerial.println("Reverse: " + String(downStream.reverse) + " ");
  // debugSerial.print("Mode: " + String(downStream.mode) + " ");
  // debugSerial.print("Pot L: " + String(downStream.potL) + " ");
  // debugSerial.print("Pot R: " + String(downStream.potR));
  // debugSerial.println("Pot M: " + String(downStream.potM));
}