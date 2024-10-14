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

void obstacleClearCallbackRefactored();

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

unsigned long lastStepTime = 0;
int step = 0;  // Step to track which part of the sequence we're in

void obstacleClearCallbackRefactored() {
  unsigned long currentMillis = millis();  // Get current time

  // Step 0: Move base servo to 30 degrees and wait 1 second
  if (step == 0 && currentMillis - lastStepTime >= 1000) {
    baseServo.write(30);
    Serial.println("Base servo set to 30");
    lastStepTime = currentMillis;
    step++;
  }

  // Step 1: Move shoulder servo to 180 degrees and wait 1 second
  else if (step == 1 && currentMillis - lastStepTime >= 1000) {
    shoulderServo.write(180);
    Serial.println("Shoulder servo set to 180");
    lastStepTime = currentMillis;
    step++;
  }

  // Step 2: Move elbow servo to 60 degrees and wait 1 second
  else if (step == 2 && currentMillis - lastStepTime >= 1000) {
    elbowServo.write(60);
    Serial.println("Elbow servo set to 60");
    lastStepTime = currentMillis;
    step++;
  }

  // Step 3: Sweep base servo from 30 to 150 degrees over 25ms increments
  else if (step == 3 && currentMillis - lastStepTime >= 25) {
    static int i = 30;  // Starting position
    if (i < 150) {
      baseServo.write(i);
      i += 3;
      Serial.print("Base servo sweeping to: ");
      Serial.println(i);
    } else {
      i = 30;  // Reset for next time
      lastStepTime = currentMillis;
      step++;  // Move to the next step
    }
  }

  // Step 4: Reset base servo to 90 degrees and wait 1 second
  else if (step == 4 && currentMillis - lastStepTime >= 1000) {
    baseServo.write(90);
    Serial.println("Base servo reset to 90");
    lastStepTime = currentMillis;
    step++;
  }

  // Step 5: Move shoulder servo back to 130 degrees and complete
  else if (step == 5 && currentMillis - lastStepTime >= 1000) {
    shoulderServo.write(130);
    Serial.println("Shoulder servo set to 130");
    lastStepTime = currentMillis;
    step = 0;  // Reset step for next run
  }
}

void clearBtSerialBuffer() {
  while (btSerial.available() > 0) {  // While there is data in the buffer
    btSerial.read();                  // Read and discard each byte
  }
}