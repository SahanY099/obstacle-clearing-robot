#include <Arduino.h>
#include <SoftwareSerial.h>

#include "packets.h"
#include "utils.h"

#define JOY_X_PIN A2
#define JOY_Y_PIN A3

#define STEERING_RATIO_PIN A0
#define THROTTLE_RATIO_PIN A1

SoftwareSerial btSerial(12, 11);

UpStream upStream;

unsigned long previousMillis = 0;
const long interval = 100;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  btSerial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    int xAxis = analogRead(JOY_X_PIN);
    int yAxis = analogRead(JOY_Y_PIN);

    int steering = 0;
    int throttle = 0;

    int steeringRatio = analogRead(STEERING_RATIO_PIN);
    int throttleRatio = analogRead(THROTTLE_RATIO_PIN);

    if (xAxis >= 550) {
      steering = map(xAxis, 550, 1023, 0, 255);
    } else if (xAxis <= 480) {
      steering = map(xAxis, 480, 0, 0, -255);
    }

    if (yAxis >= 550) {
      throttle = map(yAxis, 550, 1023, 0, 255);
      upStream.reverse = true;
    } else if (yAxis <= 480) {
      upStream.reverse = false;
      throttle = map(yAxis, 480, 0, 0, 255);
    }

    upStream.steering = steering * (float)steeringRatio / 1023.0;
    upStream.throttle = throttle * (float)throttleRatio / 1023.0;
    upStream.brakeDistance = 50;

    printDataPacket(upStream);

    btSerial.write((uint8_t*)&upStream, sizeof(UpStream));
  }
}
