// ESP32 Stepper Motor Control with A4988 Driver
// Hardware: NEMA 17 (17HE115-1504S) + A4988 + ESP32

// Pin Definitions
#define STEP_PIN 25
#define DIR_PIN 26
#define ENABLE_PIN 27
#define MS1_PIN 14
#define MS2_PIN 12
#define MS3_PIN 13

// Motor Parameters
const int STEPS_PER_REV = 200;  // NEMA 17 standard (1.8° per step)
const int MICROSTEPS = 1;       // Will be set by MS pins
const int MOTOR_SPEED_RPM = 60; // Desired speed in RPM

// DIRECTION CONFIGURATION
// After testing, update these values based on your observations:
// Set to HIGH if DIR_PIN = HIGH gives you clockwise rotation
// Set to LOW if DIR_PIN = LOW gives you clockwise rotation
const int CLOCKWISE_DIRECTION = HIGH;     // ← UPDATE THIS after testing
const int COUNTERCLOCKWISE_DIRECTION = LOW; // ← This will be automatically opposite

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Stepper Motor Controller Starting...");
  
  // Configure pins as outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  
  // Initial pin states
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW);  // LOW = Motor enabled, HIGH = Motor disabled
  
  // Set microstepping mode (Full step initially)
  setMicrostepMode(1);  // 1 = full step, 2 = half step, 4 = quarter step, etc.
  
  Serial.println("Setup complete. Motor ready.");
  delay(1000);
}

void loop() {
  // Direction Testing Mode - 800 steps for clear observation
  directionTestingMode();
  
  // Wait for user input to continue
  Serial.println("\n=== Press any key and Enter to run test again ===");
  while (!Serial.available()) {
    delay(100);
  }
  while (Serial.available()) {
    Serial.read(); // Clear the buffer
  }
}

// Function to rotate motor
void rotateMotor(bool clockwise, float revolutions) {
  // Use the configured direction constants
  digitalWrite(DIR_PIN, clockwise ? CLOCKWISE_DIRECTION : COUNTERCLOCKWISE_DIRECTION);
  
  int totalSteps = STEPS_PER_REV * MICROSTEPS * revolutions;
  int stepDelay = calculateStepDelay(MOTOR_SPEED_RPM);
  
  Serial.print("Moving ");
  Serial.print(totalSteps);
  Serial.print(" steps ");
  Serial.print(clockwise ? "CLOCKWISE" : "COUNTERCLOCKWISE");
  Serial.print(" at ");
  Serial.print(MOTOR_SPEED_RPM);
  Serial.println(" RPM");
  
  for (int i = 0; i < totalSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);  // Minimum pulse width for A4988
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}

// Calculate step delay for desired RPM
int calculateStepDelay(int rpm) {
  // Convert RPM to delay in microseconds between steps
  float stepsPerSecond = (rpm * STEPS_PER_REV * MICROSTEPS) / 60.0;
  int delayMicros = (1000000 / stepsPerSecond) - 10; // Subtract pulse width
  return max(delayMicros, 100); // Minimum delay for stability
}

// Set microstepping mode
void setMicrostepMode(int microsteps) {
  switch(microsteps) {
    case 1:    // Full step
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, LOW);
      digitalWrite(MS3_PIN, LOW);
      Serial.println("Microstepping: Full step (1:1)");
      break;
    case 2:    // Half step
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, LOW);
      digitalWrite(MS3_PIN, LOW);
      Serial.println("Microstepping: Half step (1:2)");
      break;
    case 4:    // Quarter step
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      Serial.println("Microstepping: Quarter step (1:4)");
      break;
    case 8:    // Eighth step
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      Serial.println("Microstepping: Eighth step (1:8)");
      break;
    case 16:   // Sixteenth step
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, HIGH);
      Serial.println("Microstepping: Sixteenth step (1:16)");
      break;
    default:
      Serial.println("Invalid microstep setting, using full step");
      setMicrostepMode(1);
  }
  // Update global variable
  // You'll need to create a global variable for this if you want dynamic microstepping
}

// Direction testing function with clear visual feedback
void directionTestingMode() {
  Serial.println("\n" + String("=").substring(0,50));
  Serial.println("DIRECTION TESTING MODE");
  Serial.println("800 steps = 4 full rotations (200 steps per revolution)");
  Serial.println(String("=").substring(0,50));
  
  // Test DIR_PIN = HIGH
  Serial.println("\n Testing DIR_PIN = HIGH:");
  Serial.println("   Watch the motor shaft and note the direction!");
  Serial.println("   Starting in 3 seconds...");
  delay(1000);
  Serial.println("   3...");
  delay(1000);
  Serial.println("   2...");
  delay(1000);
  Serial.println("   1...");
  delay(1000);
  Serial.println("   ➤ ROTATING NOW - DIR_PIN = HIGH");
  
  testRotation(HIGH, 800, "DIR_PIN = HIGH");
  
  delay(3000); // Pause between tests
  
  // Test DIR_PIN = LOW  
  Serial.println("\n Testing DIR_PIN = LOW:");
  Serial.println("   Watch the motor shaft and note the direction!");
  Serial.println("   Starting in 3 seconds...");
  delay(1000);
  Serial.println("   3...");
  delay(1000);
  Serial.println("   2...");
  delay(1000);
  Serial.println("   1...");
  delay(1000);
  Serial.println("   ➤ ROTATING NOW - DIR_PIN = LOW");
  
  testRotation(LOW, 800, "DIR_PIN = LOW");
  
  // Summary
  Serial.println("\n" + String("=").substring(0,50));
  Serial.println("DIRECTION TEST SUMMARY:");
  Serial.println("• DIR_PIN = HIGH → Note if this was clockwise or counterclockwise");
  Serial.println("• DIR_PIN = LOW  → Note if this was clockwise or counterclockwise");
  Serial.println("\nTo configure your preferred directions:");
  Serial.println("1. Decide which direction you want as 'clockwise'");
  Serial.println("2. Update the rotateMotor() function accordingly");
  Serial.println(String("=").substring(0,50));
}

// Test rotation with specific DIR pin state
void testRotation(int dirPinState, int totalSteps, String description) {
  digitalWrite(DIR_PIN, dirPinState);
  
  Serial.print("Executing ");
  Serial.print(totalSteps);
  Serial.print(" steps with ");
  Serial.println(description);
  
  int stepDelay = calculateStepDelay(30); // Slower speed for better observation (30 RPM)
  
  for (int i = 0; i < totalSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
    
    // Progress indicator every 200 steps (1 revolution)
    if ((i + 1) % 200 == 0) {
      Serial.print("   Completed revolution ");
      Serial.print((i + 1) / 200);
      Serial.println("/4");
    }
  }
  
  Serial.print("✓ Completed ");
  Serial.print(description);
  Serial.println(" test");
}

// Function to enable/disable motor
void enableMotor(bool enable) {
  digitalWrite(ENABLE_PIN, enable ? LOW : HIGH);
  Serial.println(enable ? "Motor enabled" : "Motor disabled");
}

// Emergency stop function
void emergencyStop() {
  digitalWrite(ENABLE_PIN, HIGH); // Disable motor
  Serial.println("EMERGENCY STOP - Motor disabled");
}