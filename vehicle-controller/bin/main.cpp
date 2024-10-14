#include <Arduino.h>
#include <HardwareSerial.h>
#include <Servo.h>

#include "driver.h"

#define ENA PA15
#define IN1 PB3
#define IN2 PB4

#define IN3 PB5
#define IN4 PB6
#define ENB PB7

#define BASE_SERVO_PIN PB1
#define CLAW_SERVO_PIN PB11
#define ELBOW_SERVO_PIN PB0
#define SHOULDER_SERVO_PIN PB10

HardwareSerial debugSerial(PA10, PA9);

Servo clawServo;
Servo baseServo;
Servo elbowServo;
Servo shoulderServo;

Driver driver(ENA, IN1, IN2, ENB, IN3, IN4);

void setup() {
  debugSerial.begin(9600);

  driver.init();
  driver.stop();

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  clawServo.attach(CLAW_SERVO_PIN);
  baseServo.attach(BASE_SERVO_PIN);
  elbowServo.attach(ELBOW_SERVO_PIN);
  shoulderServo.attach(SHOULDER_SERVO_PIN);

  // set base posisions at the beginning

  delay(1000);
  shoulderServo.write(130);
  delay(1000);
  shoulderServo.write(180);
  delay(1000);

  elbowServo.write(60);

  baseServo.write(30);
  delay(1000);
  for (int i = 30; i < 150; i += 3) {
    baseServo.write(i);
    delay(25);
  }
  delay(1000);
  baseServo.write(90);

  delay(1000);
  shoulderServo.write(130);
}

void loop() {
}
