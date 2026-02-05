#include <Wire.h>

#define STEP_PIN 25
#define DIR_PIN 26
#define EN_PIN 27
#define MS1_PIN 14
#define MS2_PIN 12
#define MS3_PIN 13
#define AS5600_ADDR 0x36

const int MICROSTEPS = 8;
const int STEPS_PER_REV = 200 * MICROSTEPS;

float targetStepRate = 0;
unsigned long lastStepTime = 0;
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_US = 20000;  // 20ms = 50Hz

int readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);
  byte high = Wire.read();
  byte low = Wire.read();
  int rawAngle = ((high & 0x0F) << 8) | low;
  return (rawAngle * 360.0) / 4095.0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  
  // Set 1/8 microstepping
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, LOW);
  
  digitalWrite(EN_PIN, LOW);
  
  lastStepTime = micros();
  lastSampleTime = micros();
}

void loop() {
  unsigned long now = micros();
  
  // Handle velocity commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("V")) {
      targetStepRate = cmd.substring(1).toFloat();
    }
    else if (cmd == "STOP") {
      targetStepRate = 0;
    }
  }
  
  // Generate step pulses (non-blocking)
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
  
  // Send encoder POSITION every 20ms
  if (now - lastSampleTime >= SAMPLE_INTERVAL_US) {
    float angle = readAS5600();
    Serial.println(angle);  // Send position only
    lastSampleTime = now;
  }
}