#include <Wire.h>
#include <AS5600.h>   // Tilaart library

AS5600 encoder;

// A4988 Pins
const int STEP_PIN = 25;
const int DIR_PIN  = 26;
const int EN_PIN   = 27;

const int MS1 = 14;
const int MS2 = 12;
const int MS3 = 13;

// Steps per revolution (full step)
const int FULL_STEPS_PER_REV = 200;
const int MICROSTEP = 8;                   // 1/8 microstepping
const int STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEP;

// microstep pulse delay
const int STEP_DELAY_US = 800;

// Read angle in degrees
float readAngle() {
  uint16_t raw = encoder.rawAngle();
  float deg = raw * 0.087890625;
  return fmod((360.0 - deg), 360.0);
}

void stepMotor(int steps, bool direction) {
  digitalWrite(DIR_PIN, direction);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void goToAngle(float targetAngle) {
  float current = readAngle();
  float diff = targetAngle - current;

  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  bool direction = diff > 0;
  float stepFraction = (abs(diff) / 360.0) * STEPS_PER_REV;
  int steps = (int)stepFraction;

  stepMotor(steps, direction);
}

void setup() {
  Wire.begin();
  encoder.begin();

  Serial.begin(115200);
  delay(800);
  
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);   // ENABLE the driver

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  // Enable driver
  digitalWrite(EN_PIN, LOW);

  // Set 1/8 microstepping: MS1=HIGH, MS2=HIGH, MS3=LOW
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, LOW);

  delay(300);
}

void loop() {
  // AS5600 magnet detection flags:
  // bit 5 = magnet detected
  // bit 4 = magnet too weak
  // bit 3 = magnet too strong
  uint8_t status = encoder.readStatus();

  bool magnetDetected = status & 0x20;
  bool magnetTooWeak  = status & 0x10;
  bool magnetTooStrong = status & 0x08;

  Serial.print("Magnet: ");
  if (!magnetDetected) Serial.print("NOT DETECTED");
  else Serial.print("OK");

  Serial.print(" | Weak:");
  Serial.print(magnetTooWeak);

  Serial.print(" | Strong:");
  Serial.print(magnetTooStrong);

  Serial.print(" | Angle: ");
  Serial.println(readAngle());

  if (Serial.available()) {
    float target = Serial.parseFloat();
    if (target >= 0 && target <= 360) {
      goToAngle(target);
      delay(200);

      Serial.print("Moved to: ");
      Serial.println(readAngle());
    }
  }

  delay(200);
}

