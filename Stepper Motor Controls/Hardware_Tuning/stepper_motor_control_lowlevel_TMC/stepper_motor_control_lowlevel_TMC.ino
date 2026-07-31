#include <Wire.h>
#include "MT6701.hpp"

#define STEP_PIN 27
#define DIR_PIN  26
#define EN_PIN   25
#define SDA_PIN  21
#define SCL_PIN  22

const int MICROSTEPS = 16;
const int STEPS_PER_REV = 200 * MICROSTEPS;

MT6701 encoder(MT6701::DEFAULT_ADDRESS, 10);  // 10ms update, faster than the 20ms control loop below

float Kp = 3.0;
float Ki = 0.9;
float Kd = 0.3;
float target_angle = 0;
float integral = 0;
float prev_error = 0;
bool control_active = false;

unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL = 20000;
float targetStepRate = 0;
unsigned long lastStepTime = 0;

void homeToZero() {
  Serial.println("HOMING");

  float current_angle = encoder.getAngleDegrees();
  float error = 0 - current_angle;

  // Wrap to shortest path
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  // Calculate steps needed
  int steps = abs(error) * STEPS_PER_REV / 360.0;
  digitalWrite(DIR_PIN, error >= 0 ? HIGH : LOW);

  // Move to zero at constant speed
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(900);  // ~1kHz = slow, safe homing
  }

  delay(500);  // Settle

  // Check final position
  float final_angle = encoder.getAngleDegrees();
  Serial.print("HOMED:");
  Serial.println(final_angle);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  encoder.begin();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);

  Serial.println("READY");
}

void loop() {
  unsigned long now = micros();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "HOME") {
      homeToZero();
    }
    else if (cmd.startsWith("KP")) {
      Kp = cmd.substring(2).toFloat();
      Serial.println("OK");
    }
    else if (cmd.startsWith("KI")) {
      Ki = cmd.substring(2).toFloat();
      Serial.println("OK");
    }
    else if (cmd.startsWith("KD")) {
      Kd = cmd.substring(2).toFloat();
      Serial.println("OK");
    }
    else if (cmd.startsWith("TARGET")) {
      target_angle = cmd.substring(6).toFloat();
      Serial.println("OK");
    }
    else if (cmd == "START") {
      control_active = true;
      integral = 0;
      prev_error = 0;
      Serial.println("STARTED");
    }
    else if (cmd == "STOP") {
      control_active = false;
      targetStepRate = 0;
      Serial.println("STOPPED");
    }
    else if (cmd == "READ") {
      Serial.println(encoder.getAngleDegrees());
    }
  }

  if (control_active && (now - lastControlTime >= CONTROL_INTERVAL)) {
    float current_angle = encoder.getAngleDegrees();
    float error = target_angle - current_angle;

    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    integral += error * 0.02;
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;

    float derivative = (error - prev_error) / 0.02;
    float output = Kp * error + Ki * integral + Kd * derivative;

    if (output > 400) output = 400;
    if (output < -400) output = -400;

    targetStepRate = output;
    prev_error = error;

    Serial.print("A");
    Serial.print(current_angle, 2);
    Serial.print(",E");
    Serial.print(error, 2);
    Serial.print(",C");
    Serial.println(output, 2);

    lastControlTime = now;
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
}
