/*
 * COMPLETE TURNTABLE SYSTEM
 * NEMA 17 + A4988 (1/16 microstepping) + AS5600 Encoder
 * 
 * Hardware (from your pinout):
 *   A4988 Stepper Driver:
 *     GPIO 25 → A4988 STEP
 *     GPIO 26 → A4988 DIR
 *     GPIO 27 → A4988 ENABLE
 *     
 *   Microstepping Configuration (for 1/16):
 *     GPIO 14 → A4988 MS1 (HIGH)
 *     GPIO 12 → A4988 MS2 (HIGH)
 *     GPIO 13 → A4988 MS3 (HIGH)
 *   
 *   AS5600 Magnetic Encoder:
 *     GPIO 21 → AS5600 SDA (I2C)
 *     GPIO 22 → AS5600 SCL (I2C)
 * 
 * Motor Specs:
 *   - NEMA 17 (200 steps/rev full step)
 *   - 1/16 microstepping = 3200 steps/rev
 *   - 4:1 gear ratio = 12,800 steps per turntable revolution
 * 
 * Commands from MATLAB:
 *   "CW"      - Rotate clockwise
 *   "CCW"     - Rotate counter-clockwise
 *   "STOP"    - Stop rotation
 *   "SPEED:n" - Set speed (steps per second, e.g., "SPEED:500")
 *   "DATA"    - Get current angle and motor position
 */

#include <Wire.h>

// A4988 Driver pins
#define STEP_PIN 25
#define DIR_PIN 26
#define ENABLE_PIN 27

// Microstepping configuration pins (for 1/16 step)
#define MS1_PIN 14
#define MS2_PIN 12
#define MS3_PIN 13

// AS5600 I2C Configuration
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_H 0x0E
#define AS5600_ANGLE_L 0x0F

// Motor configuration
#define MICROSTEPS_PER_REV 3200  // 200 * 16 = 3200 steps per motor revolution
#define GEAR_RATIO 4             // 4:1 reduction (motor:turntable)
#define ENCODER_RESOLUTION 4096  // AS5600 is 12-bit (0-4095)

// Motor control variables
int motorSpeed = 800;            // Steps per second
bool isRunning = false;
bool clockwise = true;
long stepCount = 0;              // Track total steps taken

// Timing for data output
unsigned long lastDataSend = 0;
unsigned long dataSendInterval = 100;  // Send data every 100ms when running

void setup() {
  // Initialize serial
  Serial.begin(115200);
  
  // Initialize I2C for AS5600 encoder
  Wire.begin(21, 22);  // SDA=21, SCL=22
  Wire.setClock(400000);  // 400kHz I2C
  
  // Initialize motor control pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Initialize microstepping pins (all HIGH for 1/16 step)
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  
  digitalWrite(MS1_PIN, HIGH);   // MS1 = HIGH
  digitalWrite(MS2_PIN, HIGH);   // MS2 = HIGH
  digitalWrite(MS3_PIN, HIGH);   // MS3 = HIGH
  
  // Enable motor driver (LOW = enabled for A4988)
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  // Wait for AS5600 to initialize
  delay(100);
  
  // Check encoder connection
  if (checkAS5600()) {
    Serial.println("AS5600_READY");
  } else {
    Serial.println("AS5600_ERROR");
  }
  
  Serial.println("MICROSTEP_1/16");
  Serial.println("SYSTEM_READY");
  
  // Print system info
  Serial.print("INFO:Microsteps/rev=");
  Serial.println(MICROSTEPS_PER_REV);
  Serial.print("INFO:Gear_ratio=");
  Serial.println(GEAR_RATIO);
  Serial.print("INFO:Steps/turntable_rev=");
  Serial.println(MICROSTEPS_PER_REV * GEAR_RATIO);
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
    else if (command.startsWith("SPEED:")) {
      String speedStr = command.substring(6);
      int newSpeed = speedStr.toInt();
      if (newSpeed > 0 && newSpeed <= 2000) {
        motorSpeed = newSpeed;
        Serial.print("OK:SPEED=");
        Serial.println(motorSpeed);
      } else {
        Serial.println("ERROR:Speed must be 1-2000");
      }
    }
    else if (command == "DATA") {
      sendData();
    }
    else if (command == "RESET") {
      stepCount = 0;
      Serial.println("OK:RESET");
    }
  }
  
  // Generate step pulses if motor should be running
  if (isRunning) {
    // Calculate delay for desired speed
    unsigned long stepDelay = 1000000UL / motorSpeed;
    
    // Generate step pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);  // Minimum pulse width for A4988
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 2);
    
    // Update step count
    if (clockwise) {
      stepCount++;
    } else {
      stepCount--;
    }
  }
  
  // Send data periodically while running
  if (isRunning && (currentTime - lastDataSend >= dataSendInterval)) {
    sendData();
    lastDataSend = currentTime;
  }
}

bool checkAS5600() {
  Wire.beginTransmission(AS5600_ADDRESS);
  byte error = Wire.endTransmission();
  return (error == 0);
}

float readEncoderAngle() {
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
    float angle = (rawAngle * 360.0) / 4096.0;
    return angle;
  }
  
  return -1.0;  // Error
}

float calculateMotorAngle() {
  // Calculate expected turntable angle from step count
  // Account for gear ratio
  float motorRevolutions = (float)stepCount / MICROSTEPS_PER_REV;
  float turntableRevolutions = motorRevolutions / GEAR_RATIO;
  
  // Convert to degrees and normalize to 0-360
  float angle = turntableRevolutions * 360.0;
  
  // Normalize to 0-360 range
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  
  return angle;
}

void sendData() {
  float encoderAngle = readEncoderAngle();
  float motorAngle = calculateMotorAngle();
  float error = encoderAngle - motorAngle;
  
  // Handle wraparound for error calculation
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;
  
  // Send data in CSV format: encoderAngle,motorAngle,stepCount,error
  Serial.print("DATA:");
  Serial.print(encoderAngle, 2);
  Serial.print(",");
  Serial.print(motorAngle, 2);
  Serial.print(",");
  Serial.print(stepCount);
  Serial.print(",");
  Serial.println(error, 2);
}
