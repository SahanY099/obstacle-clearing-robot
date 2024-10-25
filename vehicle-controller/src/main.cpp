#include <Arduino.h>
#include <HardwareSerial.h>
#include <NewPing.h>
#include <Servo.h>
#include <TaskScheduler.h>

#include "driver.h"
#include "packets.h"
#include "utils.h"

#define ENA PA1
#define IN1 PB3
#define IN2 PB4

#define IN3 PB5
#define IN4 PB6
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
bool clearingStarted = false;
uint8_t breakDistance = 30;
uint8_t collissionDistance = 0;

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

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - mainPreviousMillis >= mainInterval) {
    mainPreviousMillis = currentMillis;

    communicationCallback();
    sonarTaskCallBack();

    if ((!obstacleDetected && (collissionDistance > breakDistance || collissionDistance == 0)) || (downStream.reverse && downStream.throttle > 1)) {
      driveCallback();
    } else {
      driver.stop();

      if (collissionDistance < 10 || clearingStarted) {
        clearingStarted = true;
        obstacleClearCallback();
      }
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

uint8_t obstacleCount = 0;

void sonarTaskCallBack() {
  uint8_t distance = sonar.ping_cm(40);

  if (distance > 1 && distance < breakDistance) {
    obstacleCount++;
    if (obstacleCount > 2) {
      obstacleDetected = true;
      collissionDistance = distance;

      debugSerial.print("    detected: ");
      debugSerial.println(distance);
    }
    return;
  }

  debugSerial.print("not detected: ");
  debugSerial.println(distance);

  obstacleCount = 0;
  collissionDistance = 0;
}

unsigned long armPreviousMillis = 0;  // Variable to store the last time an action was performed
int step = 0;                         // Step counter to track which part of the sequence to execute next
int i = 30;                           // Initial position for baseServo

void obstacleClearCallback() {
  unsigned long currentMillis = millis();

  switch (step) {
    case 0:
      if (currentMillis - armPreviousMillis >= 1000) {
        baseServo.write(30);
        armPreviousMillis = currentMillis;
        step++;
      }
      break;

    case 1:
      if (currentMillis - armPreviousMillis >= 1000) {
        shoulderServo.write(180);
        armPreviousMillis = currentMillis;
        step++;
      }
      break;

    case 2:
      if (currentMillis - armPreviousMillis >= 1000) {
        elbowServo.write(60);
        armPreviousMillis = currentMillis;
        step++;
      }
      break;

    case 3:
      if (i < 150) {
        if (currentMillis - armPreviousMillis >= 15) {
          baseServo.write(i);
          i += 2;
          armPreviousMillis = currentMillis;
        }
      } else {
        i = 30;
        step++;
      }
      break;

    case 4:
      if (currentMillis - armPreviousMillis >= 1000) {
        shoulderServo.write(130);
        armPreviousMillis = currentMillis;
        step++;
      }
      break;

    case 5:
      if (currentMillis - armPreviousMillis >= 1000) {
        baseServo.write(90);
        armPreviousMillis = currentMillis;
        step = 0;
        obstacleDetected = false;  // Reset flag
        clearingStarted = false;
      }
      break;
  }
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