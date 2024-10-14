#ifndef UTILS_H
#define UTILS_H

#include <HardwareSerial.h>

#include "packets.h"

void printRemoteDownStream(HardwareSerial &debugSerial, DownStream &downStream);

#endif