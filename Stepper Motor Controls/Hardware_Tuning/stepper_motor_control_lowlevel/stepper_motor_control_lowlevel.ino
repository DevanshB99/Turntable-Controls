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

void homeToZero() {
  Serial.println("HOMING");
  
  float current_angle = readAS5600();
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
  float final_angle = readAS5600();
  Serial.print("HOMED:");
  Serial.println(final_angle);
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
  
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, LOW);
  
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
      Serial.println(readAS5600());
    }
  }
  
  if (control_active && (now - lastControlTime >= CONTROL_INTERVAL)) {
    float current_angle = readAS5600();
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