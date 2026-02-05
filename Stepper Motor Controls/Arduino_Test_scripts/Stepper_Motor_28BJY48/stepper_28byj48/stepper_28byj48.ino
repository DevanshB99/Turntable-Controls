/*
 * SIMPLE ESP32 STEPPER MOTOR CONTROL - 28BYJ-48 Version
 * For 28BYJ-48 stepper motor with ULN2003 driver board
 * 
 * Hardware:
 *   - ESP32
 *   - ULN2003 Driver Board (has IN1, IN2, IN3, IN4 pins)
 *   - 28BYJ-48 Stepper Motor (5-wire unipolar)
 * 
 * Wiring:
 *   ESP32 GPIO 27 → Driver IN1
 *   ESP32 GPIO 26 → Driver IN2
 *   ESP32 GPIO 25 → Driver IN3
 *   ESP32 GPIO 33 → Driver IN4
 *   ESP32 GND     → Driver GND (-)
 *   5V Power      → Driver VCC (+)
 *   
 * Commands from MATLAB (send via serial):
 *   "CW"   - Rotate clockwise
 *   "CCW"  - Rotate counter-clockwise  
 *   "STOP" - Stop rotation
 */

// Pin definitions - ULN2003 requires 4 control pins
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33

// Motor settings for 28BYJ-48
// This motor has 2048 steps per revolution in full-step mode
int motorSpeed = 1200;  // Microseconds between steps (lower = faster)
bool isRunning = false;
bool clockwise = true;

// Step sequence for 28BYJ-48 (half-step mode for smoother operation)
// This gives 4096 steps per revolution
int stepSequence[8][4] = {
  {1, 0, 0, 0},  // Step 0
  {1, 1, 0, 0},  // Step 1
  {0, 1, 0, 0},  // Step 2
  {0, 1, 1, 0},  // Step 3
  {0, 0, 1, 0},  // Step 4
  {0, 0, 1, 1},  // Step 5
  {0, 0, 0, 1},  // Step 6
  {1, 0, 0, 1}   // Step 7
};

int currentStep = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Setup motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize all pins to LOW
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  Serial.println("READY");  // Tell MATLAB we're ready
}

void loop() {
  // Check for commands from MATLAB
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "CW") {
      clockwise = true;
      isRunning = true;
      Serial.println("OK:CW");
    }
    else if (command == "CCW") {
      clockwise = false;
      isRunning = true;
      Serial.println("OK:CCW");
    }
    else if (command == "STOP") {
      isRunning = false;
      // Turn off all coils when stopped (reduces heat and power)
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      Serial.println("OK:STOP");
    }
  }
  
  // Generate steps if motor should be running
  if (isRunning) {
    takeStep();
    delayMicroseconds(motorSpeed);  // Control speed
  }
}

void takeStep() {
  // Move to next step in sequence
  if (clockwise) {
    currentStep++;
    if (currentStep >= 8) currentStep = 0;
  } else {
    currentStep--;
    if (currentStep < 0) currentStep = 7;
  }
  
  // Set the pins according to step sequence
  digitalWrite(IN1, stepSequence[currentStep][0]);
  digitalWrite(IN2, stepSequence[currentStep][1]);
  digitalWrite(IN3, stepSequence[currentStep][2]);
  digitalWrite(IN4, stepSequence[currentStep][3]);
}
