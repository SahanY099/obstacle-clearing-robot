#include "driver.h"

Driver::Driver(uint8_t ENA, uint8_t IN1, uint8_t IN2, uint8_t ENB, uint8_t IN3, uint8_t IN4) {
  this->ENA = ENA;
  this->IN1 = IN1;
  this->IN2 = IN2;
  this->ENB = ENB;
  this->IN3 = IN3;
  this->IN4 = IN4;
}

void Driver::init() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void Driver::drive(uint8_t *throttle, int16_t *steering, bool *reverse) {
  int leftMotorSpeed = *throttle - *steering;
  int rightMotorSpeed = *throttle + *steering;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  if (*reverse) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  if (*throttle > 0) {
    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  } else {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  }
}

void Driver::stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}