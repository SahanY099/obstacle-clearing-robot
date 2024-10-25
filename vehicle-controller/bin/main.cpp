#include <Arduino.h>
#include <HardwareSerial.h>
#include <NewPing.h>
#include <Servo.h>
#include <TaskScheduler.h>

#include "driver.h"
#include "packets.h"
#include "utils.h"

// #define ENA PA15
#define ENA PA1
#define IN1 PB3
#define IN2 PB4

#define IN3 PB5
#define IN4 PB6
// #define ENB PB7
#define ENB PA0

#define TRIG_PIN PB12
#define ECHO_PIN PB13

#define BASE_SERVO_PIN PB0
#define ELBOW_SERVO_PIN PA7
#define SHOULDER_SERVO_PIN PB1

HardwareSerial debugSerial(PA10, PA9);
HardwareSerial btSerial(PA3, PA2);

DownStream downStream;

Scheduler runner;
Driver driver(ENA, IN1, IN2, ENB, IN3, IN4);

Servo clawServo;
Servo baseServo;
Servo elbowServo;
Servo shoulderServo;

NewPing sonar(TRIG_PIN, ECHO_PIN, 400);

void initArm();
void clearBtSerialBuffer();
void driveCallback();
void sonarTaskCallBack();
void communicationCallback();
void obstacleClearCallback();

bool obstacleDetected = false;

void setup() {
  driver.init();
  driver.stop();

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  baseServo.attach(BASE_SERVO_PIN);
  elbowServo.attach(ELBOW_SERVO_PIN);
  shoulderServo.attach(SHOULDER_SERVO_PIN);

  initArm();

  btSerial.begin(9600);
  debugSerial.begin(9600);
}

unsigned long mainPreviousMillis = 0;
const long mainInterval = 100;

unsigned long obstaclePreviousMillis = 0;
const long obstacleInterval = 50;

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - mainPreviousMillis >= mainInterval) {
    mainPreviousMillis = currentMillis;

    communicationCallback();
    sonarTaskCallBack();

    if (!obstacleDetected) {
      driveCallback();
    } else {
      obstacleClearCallback();
    }
  }
}

void communicationCallback() {
  if (btSerial.available() >= sizeof(DownStream)) {
    btSerial.readBytes((char*)&downStream, sizeof(DownStream));

    btSerial.flush();

    if (downStream.steering > 250) {
      clearBtSerialBuffer();
    }
    printRemoteDownStream(debugSerial, downStream);

  } else {
    if (btSerial.available()) {
      debugSerial.println("bt data");
    } else {
      debugSerial.println("No bt data");
    }
  }
}

void driveCallback() {
  driver.drive(&downStream.throttle, &downStream.steering, &downStream.reverse);
}

uint8_t obstacleClearDistace = 20;
uint8_t obstacleCount = 0;

void sonarTaskCallBack() {
  uint16_t distance = sonar.ping_cm(30);

  if (distance > 1 && distance < obstacleClearDistace) {
    obstacleCount++;

    if (obstacleCount > 2) {
      obstacleDetected = true;

      driver.stop();

      debugSerial.print("    detected: ");
      debugSerial.println(distance);
    }
    return;
  }

  debugSerial.print("not detected: ");
  debugSerial.println(distance);

  obstacleCount = 0;
  obstacleDetected = false;
}

void obstacleClearCallback() {
  baseServo.write(30);
  delay(1000);

  shoulderServo.write(180);
  delay(1000);

  elbowServo.write(60);

  delay(1000);
  for (int i = 30; i < 150; i += 3) {
    baseServo.write(i);
    delay(25);
  }

  shoulderServo.write(130);

  delay(1000);
  baseServo.write(90);

  obstacleDetected = false;
}

void initArm() {
  shoulderServo.write(130);
  delay(1000);
  elbowServo.write(60);
  delay(1000);
  baseServo.write(90);
  delay(1000);
}

void clearBtSerialBuffer() {
  while (btSerial.available() > 0) {  // While there is data in the buffer
    btSerial.read();                  // Read and discard each byte
  }
}