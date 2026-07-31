#include <Wire.h>
#include "MT6701.hpp"

#define STEP_PIN 27
#define DIR_PIN  26
#define EN_PIN   25
#define SDA_PIN  21
#define SCL_PIN  22

const int MICROSTEPS = 16;
const int STEPS_PER_REV = 200 * MICROSTEPS;  // motor microsteps per MOTOR revolution
const float GEAR_RATIO = 4.0;                // motor:turntable - motor turns 4x per turntable turn
const float MAX_STEP_RATE = 1000;            // steps/s output clamp - conservative starting point, not yet validated for this belt drive

MT6701 encoder(MT6701::DEFAULT_ADDRESS, 10);  // 10ms update, faster than the 20ms control loop below
                                               // magnet is at the turntable disc's own center, so this
                                               // reads true turntable angle directly, not motor angle

float Kp = 3.0;
float Ki = 0.9;
float Kd = 0.3;
float target_angle = 0;
float integral = 0;
float prev_error = 0;
bool control_active = false;
bool sysIdStreaming = false;

// Derivative low-pass filter (exponential moving average). Equivalent
// continuous-time filter time constant: Tf = Ts*(1-alpha)/alpha, with
// Ts=0.02s below this gives Tf=0.08s - use the same Tf in
// RootLocus_PI_Design_Turntable.m's pid(Kp,Ki,Kd,Tf) so the linear model
// matches what's actually running here. Revisit both alpha and Tf once
// the real plant is identified, in case the belt introduces a resonance
// at a frequency where this starting choice isn't a good fit.
const float D_FILTER_ALPHA = 0.2;
float filtered_derivative = 0;

unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL = 20000;
float targetStepRate = 0;
unsigned long lastStepTime = 0;

void homeToZero() {
  Serial.println("HOMING");

  float current_angle = encoder.getAngleDegrees();
  float error = 0 - current_angle;

  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  // error is in TURNTABLE degrees (encoder is on the disc); the motor
  // must turn GEAR_RATIO times further to move the turntable by "error".
  int steps = abs(error) * GEAR_RATIO * STEPS_PER_REV / 360.0;
  digitalWrite(DIR_PIN, error >= 0 ? LOW : HIGH);  // NOT yet verified for this belt drive - confirm with Direction_Test_Turntable.ino

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(900);  // ~1kHz = slow, safe homing
  }

  delay(500);  // Settle

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
      sysIdStreaming = false;
      integral = 0;
      prev_error = 0;
      filtered_derivative = 0;
      Serial.println("STARTED");
    }
    else if (cmd == "STOP") {
      control_active = false;
      sysIdStreaming = false;
      targetStepRate = 0;
      Serial.println("STOPPED");
    }
    else if (cmd == "READ") {
      Serial.println(encoder.getAngleDegrees());
    }
    else if (cmd.startsWith("V")) {
      targetStepRate = cmd.substring(1).toFloat();
      control_active = false;
      sysIdStreaming = true;
    }
  }

  if (now - lastControlTime >= CONTROL_INTERVAL) {
    lastControlTime = now;
    float current_angle = encoder.getAngleDegrees();

    if (control_active) {
      float error = target_angle - current_angle;
      if (error > 180) error -= 360;
      if (error < -180) error += 360;

      float raw_derivative = (error - prev_error) / 0.02;
      filtered_derivative = D_FILTER_ALPHA * raw_derivative + (1 - D_FILTER_ALPHA) * filtered_derivative;
      float trial_output = Kp * error + Ki * integral + Kd * filtered_derivative;

      // Anti-windup: only integrate if we're not already saturated in the
      // same direction the error is pushing (conditional integration)
      bool saturating = (trial_output >= MAX_STEP_RATE && error > 0) || (trial_output <= -MAX_STEP_RATE && error < 0);
      if (!saturating) {
        integral += error * 0.02;
        if (integral > 100) integral = 100;
        if (integral < -100) integral = -100;
      }

      float output = Kp * error + Ki * integral + Kd * filtered_derivative;
      if (output > MAX_STEP_RATE) output = MAX_STEP_RATE;
      if (output < -MAX_STEP_RATE) output = -MAX_STEP_RATE;

      targetStepRate = output;
      prev_error = error;

      Serial.print("A");
      Serial.print(current_angle, 2);
      Serial.print(",E");
      Serial.print(error, 2);
      Serial.print(",C");
      Serial.println(output, 2);
    }
    else if (sysIdStreaming) {
      Serial.println(current_angle);
    }
  }

  if (targetStepRate != 0) {
    unsigned long stepInterval = 1000000.0 / abs(targetStepRate);
    if (now - lastStepTime >= stepInterval) {
      digitalWrite(DIR_PIN, targetStepRate >= 0 ? LOW : HIGH);  // NOT yet verified for this belt drive - confirm with Direction_Test_Turntable.ino
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(STEP_PIN, LOW);
      lastStepTime = now;
    }
  }
}
