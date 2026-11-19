/*
 * Turntable Direction Test (belt-driven, 4:1 motor:turntable ratio)
 * Determine which DIR pin state (HIGH/LOW) increases vs decreases the
 * encoder angle. The MT6701 magnet is mounted at the turntable disc's
 * own center, so this reads true turntable angle directly - belt routing
 * can flip rotation sense independently of the bare-motor wiring, so
 * this must be re-verified from scratch, not assumed from earlier tests.
 */

#include <Wire.h>
#include "MT6701.hpp"

#define STEP_PIN 27
#define DIR_PIN  26
#define EN_PIN   25
#define SDA_PIN  21
#define SCL_PIN  22

MT6701 encoder(MT6701::DEFAULT_ADDRESS, 10);

const float MAX_STEP_RATE = 1000;  // steps/s - conservative starting point, not yet validated for this belt drive
int step_delay_us = 1000000 / MAX_STEP_RATE;
bool motor_running = false;
float start_angle = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(EN_PIN, HIGH);  // disabled initially

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  encoder.begin();

  printHelp();
}

void loop() {
  if (Serial.available()) {
    processCommand(Serial.read());
  }

  if (motor_running) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(step_delay_us);
  }
}

void processCommand(char cmd) {
  switch (cmd) {
    case 'e':
      digitalWrite(EN_PIN, LOW);
      Serial.println("Motor ENABLED");
      break;

    case 'd':
      digitalWrite(EN_PIN, HIGH);
      motor_running = false;
      Serial.println("Motor DISABLED");
      break;

    case 'f':
      digitalWrite(DIR_PIN, HIGH);
      Serial.println("DIR pin set HIGH");
      break;

    case 'b':
      digitalWrite(DIR_PIN, LOW);
      Serial.println("DIR pin set LOW");
      break;

    case 'r':
      start_angle = encoder.getAngleDegrees();
      motor_running = true;
      Serial.print("Running... start angle = ");
      Serial.println(start_angle, 2);
      break;

    case 's': {
      motor_running = false;
      float end_angle = encoder.getAngleDegrees();
      float delta = end_angle - start_angle;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;
      Serial.print("Stopped. End angle = ");
      Serial.print(end_angle, 2);
      Serial.print(" | Delta = ");
      Serial.print(delta, 2);
      Serial.println(" deg");
      break;
    }

    case 'a':
      Serial.print("Encoder angle: ");
      Serial.println(encoder.getAngleDegrees(), 2);
      break;

    case '?':
      printHelp();
      break;

    case '\n':
    case '\r':
      break;

    default:
      Serial.print("Unknown: '");
      Serial.print(cmd);
      Serial.println("'");
      break;
  }
}

void printHelp() {
  Serial.println("\n--- Turntable Direction Test (belt-driven) ---");
  Serial.println("e - enable driver");
  Serial.println("d - disable driver");
  Serial.println("f - set DIR pin HIGH");
  Serial.println("b - set DIR pin LOW");
  Serial.println("r - run continuous at MAX_STEP_RATE (records start angle)");
  Serial.println("s - stop (prints end angle + delta)");
  Serial.println("a - read encoder angle");
  Serial.println("? - help");
  Serial.println("\nDelta is TURNTABLE degrees (encoder is on the disc, not the motor shaft).");
  Serial.println("Start: e, then f (or b), then r ... let it run a bit ... s");
  Serial.println("Watch the turntable while it runs and note CW/CCW yourself,");
  Serial.println("then match that to whichever DIR level gave a positive Delta.\n");
}
