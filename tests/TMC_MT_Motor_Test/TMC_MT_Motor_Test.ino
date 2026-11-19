#include <Wire.h>
#include "MT6701.hpp"
#include <FastAccelStepper.h>

#define STEP_PIN 27
#define DIR_PIN  26
#define EN_PIN   25
#define SDA_PIN  21
#define SCL_PIN  22

const int   MOTOR_FULL_STEPS_PER_REV = 200;
const int   MICROSTEPS               = 16;
const long  MICROSTEPS_PER_REV       = (long)MOTOR_FULL_STEPS_PER_REV * MICROSTEPS;
const float MOVE_DEG_PER_SEC         = 90.0;
const float MOVE_SPEED_STEPS_PER_SEC = MOVE_DEG_PER_SEC / 360.0 * MICROSTEPS_PER_REV;
const float RAMP_ACCEL_STEPS_PER_S2  = 100000; 

MT6701 encoder;
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

enum TestState { IDLE, MOVING };
TestState state = IDLE;

float targetAngle = 0;

void setupDriver() {
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN, false);  // false: motor's positive direction runs opposite the encoder's increasing-angle direction
  stepper->setEnablePin(EN_PIN);  // TMC2209 EN is active-low, which is the library default
  stepper->setAutoEnable(true);   // enable driver just before stepping, disable once idle
  stepper->setSpeedInHz((uint32_t)MOVE_SPEED_STEPS_PER_SEC);
  stepper->setAcceleration((int32_t)RAMP_ACCEL_STEPS_PER_S2);
}

bool checkEncoderPresent() {
  Wire.beginTransmission(MT6701::DEFAULT_ADDRESS);
  return Wire.endTransmission() == 0;
}

void setupEncoder() {
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  encoder.begin();

  Serial.print("MT6701 I2C: ");
  Serial.println(checkEncoderPresent() ? "OK" : "NOT FOUND - check IIC mode pads/MODE pin on the module");

  delay(200);
}

float wrapDelta(float delta) {
  while (delta > 180.0)  delta -= 360.0;
  while (delta < -180.0) delta += 360.0;
  return delta;
}

void startMove(float target) {
  if (!checkEncoderPresent()) {
    Serial.println("Encoder not detected - refusing to move");
    return;
  }

  float currentAngle = encoder.getAngleDegrees();
  float delta = wrapDelta(target - currentAngle);
  targetAngle = target;

  Serial.print("Moving ");
  Serial.print(delta, 2);
  Serial.println(" deg");

  stepper->move(lround(delta / 360.0 * MICROSTEPS_PER_REV));
  state = MOVING;
}

void checkMoveComplete() {
  if (stepper->isRunning()) return;

  Serial.print("Reached target ");
  Serial.print(targetAngle, 2);
  Serial.print(" deg | actual ");
  Serial.print(encoder.getAngleDegrees(), 3);
  Serial.println(" deg");
  state = IDLE;
}

void checkSerialInput() {
  if (state != IDLE) return;

  static char    buf[32];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (idx == 0) continue; // ignore stray CR/LF or empty lines
      buf[idx] = '\0';
      idx = 0;

      char *endPtr;
      float target = strtod(buf, &endPtr);

      if (endPtr == buf || *endPtr != '\0') {
        Serial.print("Invalid input: \"");
        Serial.print(buf);
        Serial.println("\" - enter a numeric angle in degrees");
      } else {
        startMove(target);
      }
      return;
    }

    if (idx < sizeof(buf) - 1) buf[idx++] = c;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  setupDriver();
  setupEncoder();
  Serial.println("Enter target angle (deg):");
}

void loop() {
  checkSerialInput();
  if (state == MOVING) checkMoveComplete();
}
