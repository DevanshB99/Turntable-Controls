#include <Wire.h>
#include "MT6701.hpp"

#define STEP_PIN 27
#define DIR_PIN  26
#define EN_PIN   25
#define SDA_PIN  21
#define SCL_PIN  22

MT6701 encoder(MT6701::DEFAULT_ADDRESS, 10);  // 10ms update, faster than the 20ms sample loop below

float targetStepRate = 0;
unsigned long lastStepTime = 0;
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_US = 20000;  // 20ms = 50Hz

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  encoder.begin();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  lastStepTime = micros();
  lastSampleTime = micros();
}

void loop() {
  unsigned long now = micros();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("V")) {
      targetStepRate = cmd.substring(1).toFloat();
    } else if (cmd == "STOP") {
      targetStepRate = 0;
    }
  }

  if (targetStepRate != 0) {
    unsigned long stepInterval = 1000000.0 / abs(targetStepRate);
    if (now - lastStepTime >= stepInterval) {
      digitalWrite(DIR_PIN, targetStepRate >= 0 ? HIGH : LOW);
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(STEP_PIN, LOW);
      lastStepTime = now;
    }
  }

  if (now - lastSampleTime >= SAMPLE_INTERVAL_US) {
    Serial.println(encoder.getAngleDegrees());
    lastSampleTime = now;
  }
}
