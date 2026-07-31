/*
 * Hardware Test for AS5600 Encoder + NEMA17 + A4988 Driver
 * 
 * This code tests:
 * 1. AS5600 encoder I2C communication
 * 2. Magnet detection and field strength
 * 3. Motor rotation in both directions
 * 4. Encoder reading during motor movement
 * 
 * Hardware Connections:
 * - AS5600: SDA -> GPIO21, SCL -> GPIO22
 * - A4988: ENABLE -> GPIO25, DIR -> GPIO26, STEP -> GPIO27
 * - Power: 12V to motor, 5V logic via buck converter
 */

#include <Wire.h>
#include <AS5600.h>

// A4988 Driver Pins
#define ENABLE_PIN 25
#define DIR_PIN 26
#define STEP_PIN 27

// Motor Parameters
#define STEPS_PER_REV 200        // NEMA17 standard
#define MICROSTEPS 8             // 1/8 microstepping
#define GEAR_RATIO 4.0           // 4:1 reduction (100T/25T)

// AS5600 Encoder
AS5600 encoder;

// Test parameters
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500; // Print status every 500ms

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial or timeout after 3s
  
  Serial.println("\n=== AS5600 + NEMA17 + A4988 Hardware Test ===\n");
  
  // Initialize A4988 pins
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  
  digitalWrite(ENABLE_PIN, HIGH); // Disable motor initially
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  
  // Initialize I2C
  Wire.begin(21, 22); // SDA, SCL
  Wire.setClock(400000); // 400kHz I2C
  
  delay(100);
  
  // Test AS5600 connection
  Serial.println("1. Testing AS5600 Encoder Connection...");
  encoder.begin();
  
  if (encoder.isConnected()) {
    Serial.println("   ✓ AS5600 is connected!");
  } else {
    Serial.println("   ✗ AS5600 NOT FOUND! Check I2C connections.");
    Serial.println("   Expected: SDA->GPIO21, SCL->GPIO22");
    while(1) { delay(1000); } // Halt
  }
  
  delay(500);
  
  // Test magnet detection
  Serial.println("\n2. Testing Magnet Detection...");
  testMagnetDetection();
  
  delay(1000);
  
  // Test motor rotation
  Serial.println("\n3. Testing Motor Rotation...");
  Serial.println("   Motor will rotate in both directions...\n");
  
  digitalWrite(ENABLE_PIN, LOW); // Enable motor
  delay(100);
}

void loop() {
  static int testPhase = 0;
  static unsigned long phaseStartTime = 0;
  static unsigned long lastStepTime = 0;
  const unsigned long stepInterval = 1000; // 1ms per step (1000 steps/sec)
  
  unsigned long currentTime = millis();
  
  // Print encoder status periodically
  if (currentTime - lastPrintTime >= printInterval) {
    printEncoderStatus();
    lastPrintTime = currentTime;
  }
  
  // Execute test phases
  switch(testPhase) {
    case 0: // Stationary - check baseline
      if (phaseStartTime == 0) {
        Serial.println("\n--- Phase 1: Stationary Baseline (3s) ---");
        phaseStartTime = currentTime;
      }
      if (currentTime - phaseStartTime >= 3000) {
        testPhase++;
        phaseStartTime = 0;
      }
      break;
      
    case 1: // Rotate CW
      if (phaseStartTime == 0) {
        Serial.println("\n--- Phase 2: Rotating Clockwise (5s) ---");
        digitalWrite(DIR_PIN, LOW); // CW
        phaseStartTime = currentTime;
      }
      if (currentTime - phaseStartTime < 5000) {
        if (currentTime - lastStepTime >= stepInterval / 1000.0) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(5);
          digitalWrite(STEP_PIN, LOW);
          lastStepTime = currentTime;
        }
      } else {
        testPhase++;
        phaseStartTime = 0;
      }
      break;
      
    case 2: // Stop
      if (phaseStartTime == 0) {
        Serial.println("\n--- Phase 3: Stopped (3s) ---");
        phaseStartTime = currentTime;
      }
      if (currentTime - phaseStartTime >= 3000) {
        testPhase++;
        phaseStartTime = 0;
      }
      break;
      
    case 3: // Rotate CCW
      if (phaseStartTime == 0) {
        Serial.println("\n--- Phase 4: Rotating Counter-Clockwise (5s) ---");
        digitalWrite(DIR_PIN, HIGH); // CCW
        phaseStartTime = currentTime;
      }
      if (currentTime - phaseStartTime < 5000) {
        if (currentTime - lastStepTime >= stepInterval / 1000.0) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(5);
          digitalWrite(STEP_PIN, LOW);
          lastStepTime = currentTime;
        }
      } else {
        testPhase++;
        phaseStartTime = 0;
      }
      break;
      
    case 4: // Final stop
      if (phaseStartTime == 0) {
        Serial.println("\n--- Phase 5: Final Stop (3s) ---");
        phaseStartTime = currentTime;
      }
      if (currentTime - phaseStartTime >= 3000) {
        testPhase++;
        phaseStartTime = 0;
      }
      break;
      
    case 5: // Test complete
      Serial.println("\n=== Test Complete ===");
      Serial.println("Observations:");
      Serial.println("- Did the encoder angle change smoothly during rotation?");
      Serial.println("- Did the magnet stay detected throughout?");
      Serial.println("- Did the motor rotate in both directions?");
      Serial.println("\nTest will restart in 5 seconds...\n");
      delay(5000);
      testPhase = 0;
      break;
  }
}

void testMagnetDetection() {
  int magnetStatus = encoder.getMagnetStrength();
  int rawAngle = encoder.rawAngle();
  
  Serial.print("   Magnet Status: ");
  
  if (magnetStatus == 32) {
    Serial.println("✓ DETECTED (Good strength)");
  } else if (magnetStatus == 16) {
    Serial.println("⚠ TOO WEAK - Move magnet closer!");
  } else if (magnetStatus == 8) {
    Serial.println("⚠ TOO STRONG - Move magnet farther!");
  } else {
    Serial.println("✗ NOT DETECTED - Check magnet alignment!");
  }
  
  Serial.print("   Raw Angle: ");
  Serial.print(rawAngle);
  Serial.println(" (0-4095)");
  
  // Check if angle is changing (magnet present but may be too weak/strong)
  int angle1 = encoder.rawAngle();
  delay(10);
  int angle2 = encoder.rawAngle();
  
  if (abs(angle2 - angle1) > 100) {
    Serial.println("   ⚠ WARNING: Angle changing rapidly - magnet may be loose!");
  }
  
  if (magnetStatus != 32) {
    Serial.println("\n   ⚠ IMPORTANT: Adjust magnet position before proceeding!");
    Serial.println("   Magnet should be centered over AS5600 with ~1-2mm gap");
  }
}

void printEncoderStatus() {
  // Check if still connected
  if (!encoder.isConnected()) {
    Serial.println("✗ ERROR: Lost connection to AS5600!");
    return;
  }
  
  // Get magnet status
  int magnetStatus = encoder.getMagnetStrength();
  String magnetStr;
  
  if (magnetStatus == 32) {
    magnetStr = "OK";
  } else if (magnetStatus == 16) {
    magnetStr = "WEAK";
  } else if (magnetStatus == 8) {
    magnetStr = "STRONG";
  } else {
    magnetStr = "NONE";
  }
  
  // Get angle
  float angle = encoder.rawAngle() * 360.0 / 4096.0;
  
  // Print status line
  Serial.print("Encoder: ");
  Serial.print(angle, 2);
  Serial.print("° | Raw: ");
  Serial.print(encoder.rawAngle());
  Serial.print(" | Magnet: ");
  Serial.print(magnetStr);
  
  // Warning if magnet not optimal
  if (magnetStatus != 32) {
    Serial.print(" ⚠");
  }
  
  Serial.println();
}
