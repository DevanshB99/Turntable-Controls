/*
 * TURNTABLE ENCODER TEST - A4988 + AS5600
 * Tests stepper motor control AND encoder angle reading
 * 
 * Hardware (from your pinout diagram):
 *   A4988 Stepper Driver:
 *     GPIO 25 → A4988 STEP
 *     GPIO 26 → A4988 DIR
 *     GPIO 27 → A4988 ENABLE
 *   
 *   AS5600 Magnetic Encoder:
 *     GPIO 21 → AS5600 SDA (I2C)
 *     GPIO 22 → AS5600 SCL (I2C)
 * 
 * Commands from MATLAB:
 *   "CW"     - Rotate clockwise
 *   "CCW"    - Rotate counter-clockwise  
 *   "STOP"   - Stop rotation
 *   "ANGLE"  - Read current angle (single reading)
 */

#include <Wire.h>

// A4988 Driver pins (from your pinout)
#define STEP_PIN 25
#define DIR_PIN 26
#define ENABLE_PIN 27

// AS5600 I2C Configuration
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_H 0x0E  // High byte of angle register
#define AS5600_ANGLE_L 0x0F  // Low byte of angle register

// Motor settings
int motorSpeed = 500;  // Steps per second
bool isRunning = false;
bool clockwise = true;

// Timing for encoder reading
unsigned long lastEncoderRead = 0;
unsigned long encoderReadInterval = 100;  // Read every 100ms

void setup() {
  // Initialize serial
  Serial.begin(115200);
  
  // Initialize I2C for AS5600 encoder
  Wire.begin(21, 22);  // SDA=21, SCL=22
  Wire.setClock(400000);  // 400kHz I2C speed
  
  // Initialize stepper pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Enable motor driver (LOW = enabled for A4988)
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  // Wait for AS5600 to initialize
  delay(100);
  
  // Check if AS5600 is connected
  if (checkAS5600()) {
    Serial.println("AS5600 READY");
  } else {
    Serial.println("AS5600 ERROR - Check I2C connections!");
  }
  
  Serial.println("SYSTEM READY");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check for commands from MATLAB
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "CW") {
      clockwise = true;
      isRunning = true;
      digitalWrite(DIR_PIN, HIGH);
      Serial.println("OK:CW");
    }
    else if (command == "CCW") {
      clockwise = false;
      isRunning = true;
      digitalWrite(DIR_PIN, LOW);
      Serial.println("OK:CCW");
    }
    else if (command == "STOP") {
      isRunning = false;
      Serial.println("OK:STOP");
    }
    else if (command == "ANGLE") {
      // Read and print angle once
      float angle = readAngle();
      Serial.print("ANGLE:");
      Serial.println(angle, 2);
    }
  }
  
  // Generate step pulses if motor should be running
  if (isRunning) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000000 / motorSpeed / 2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000000 / motorSpeed / 2);
  }
  
  // Continuously read and print encoder angle while motor is running
  if (currentTime - lastEncoderRead >= encoderReadInterval) {
    if (isRunning) {
      float angle = readAngle();
      Serial.print("ANGLE:");
      Serial.println(angle, 2);
    }
    lastEncoderRead = currentTime;
  }
}

bool checkAS5600() {
  // Try to communicate with AS5600
  Wire.beginTransmission(AS5600_ADDRESS);
  byte error = Wire.endTransmission();
  return (error == 0);
}

float readAngle() {
  // Read 12-bit angle from AS5600
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_H);
  Wire.endTransmission(false);
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    uint16_t rawAngle = (highByte << 8) | lowByte;
    
    // Convert to degrees (0-360)
    // AS5600 gives 0-4095 for 0-360 degrees
    float angle = (rawAngle * 360.0) / 4096.0;
    return angle;
  }
  
  return -1;  // Error reading
}
