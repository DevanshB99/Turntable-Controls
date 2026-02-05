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
  Serial.println("Rotating clockwise 1 full revolution...");
  rotateMotor(true, 1);  // Clockwise, 1 revolution
  delay(2000);
  
  Serial.println("Rotating counter-clockwise 1 full revolution...");
  rotateMotor(false, 1); // Counter-clockwise, 1 revolution
  delay(2000);
  
  Serial.println("Testing different speeds...");
  testDifferentSpeeds();
  delay(3000);
}

// Function to rotate motor
void rotateMotor(bool clockwise, float revolutions) {
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  
  int totalSteps = STEPS_PER_REV * MICROSTEPS * revolutions;
  int stepDelay = calculateStepDelay(MOTOR_SPEED_RPM);
  
  Serial.print("Moving ");
  Serial.print(totalSteps);
  Serial.print(" steps at ");
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

// Test different speeds and microstepping modes
void testDifferentSpeeds() {
  Serial.println("Testing different microstepping modes:");
  
  // Test different microstepping modes
  int microstepModes[] = {1, 2, 4, 8, 16};
  
  for (int i = 0; i < 5; i++) {
    Serial.print("Testing ");
    Serial.print(microstepModes[i]);
    Serial.println("x microstepping:");
    
    setMicrostepMode(microstepModes[i]);
    delay(500);
    
    // Rotate quarter turn with different microstepping
    digitalWrite(DIR_PIN, HIGH);
    int steps = (STEPS_PER_REV * microstepModes[i]) / 4; // Quarter turn
    
    for (int j = 0; j < steps; j++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(2000); // Slow speed for observation
    }
    
    delay(1000);
  }
  
  // Reset to full step
  setMicrostepMode(1);
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