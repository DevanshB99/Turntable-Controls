// Please always refer to the TMC2209 datasheet found here: https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2209_datasheet_rev1.09.pdf

// ## TMCStepper Library
// For documentation go here: https://teemuatlut.github.io/TMCStepper/class_t_m_c2209_stepper.html
#include <TMCStepper.h>

// ## FastAccelStepper Library
// Documentation can be found in the header file here: https://github.com/gin66/FastAccelStepper/blob/master/src/FastAccelStepper.h
#include <FastAccelStepper.h>

// ## STEP PIN SETUP:
// Change these pins on the ESP32
#define DIR_PIN 26
#define STEP_PIN 27
#define ENABLE_PIN 25
#define RX_PIN 19
#define TX_PIN 18

// NOTE: Connect CLK16 pin to ground

// ## Position
// Change these values to set the maximum step position.
// We will rotate the motor 5 revolutions
int32_t move_to_step = 3200 * 5;  //Change this value to set the position to move to (Negative will reverse)

// ## Speed
// Sets the speed in microsteps per second.
// If for example the the motor_microsteps is set to 16 and your stepper motor has 200 full steps per revolution (Most common type. The motor angle will be 1.8 degrees on the datasheet)
// It means 200 x 16 = 3200 steps per revolution. To set the speed to rotate one revolution per seconds, we would set the value below to 3200.
int32_t set_velocity = 3200;

// ## Acceleration
//  setAcceleration() expects as parameter the change of speed as step/s².
//  If for example the speed should ramp up from 0 to 10000 steps/s within
//  10s, then the acceleration is 10000 steps/s / 10s = 1000 steps/s²
//
// New value will be used after call to
// move/moveTo/runForward/runBackward/applySpeedAcceleration/moveByAcceleration
//
// Returns 0 on success, or -1 on invalid value (<=0)
int32_t set_accel = 3200 * 100;  // We're using a fast acceleration for this example.

// ## Current
// Always keep you current as low as possible for your application
int32_t set_current = 300;

// ## Microsteps
uint16_t motor_microsteps = 16;

// Stall variables. Do not change.
bool motor_moving = false;

// We communicate with the TMC2209 over UART
// But the Arduino UNO only have one Serial port which is connected to the Serial Monitor
// We can use software serial on the UNO, and hardware serial on the ESP32 or Mega 2560

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
TMC2209Stepper driver(&Serial1, 0.12f, 0);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 can use any pins to Serial

  pinMode(ENABLE_PIN, OUTPUT);

  driver.begin();                       // Start all the UART communications functions behind the scenes
  driver.toff(4);                       // For operation with StealthChop, this parameter is not used, but it is required to enable the motor. In case of operation with StealthChop only, any setting is OK
  driver.blank_time(24);                // Recommended blank time select value
  driver.I_scale_analog(false);         // Disbaled to use the extrenal current sense resistors
  driver.internal_Rsense(false);        // Use the external Current Sense Resistors. Do not use the internal resistor as it can't handle high current.
  driver.mstep_reg_select(true);        // Microstep resolution selected by MSTEP register and NOT from the legacy pins.
  driver.microsteps(motor_microsteps);  // Set the number of microsteps. Due to the "MicroPlyer" feature, all steps get converterd to 256 microsteps automatically. However, setting a higher step count allows you to more accurately more the motor exactly where you want.
  driver.TPWMTHRS(0);                   // DisableStealthChop PWM mode/ Page 25 of datasheet
  driver.semin(0);                      // Turn off smart current control, known as CoolStep. It's a neat feature but is more complex and messes with StallGuard.
  driver.shaft(true);                   // Set the shaft direction. Only use this command one time during setup to change the direction of your motor. Do not use this in your code to change directions.
  driver.en_spreadCycle(false);         // Disable SpreadCycle. We want StealthChop becuase it works with StallGuard.
  driver.pdn_disable(true);             // Enable UART control
  driver.VACTUAL(0);                    // Enable UART control
  driver.rms_current(set_current);      // Set RMS Current

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);  // This sets the enable pin and turns the TMC2209 on/off atomatically
  stepper->setSpeedInHz(set_velocity);
  stepper->setAcceleration(set_accel);

  // Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    MotorTask  //Motor Task
    ,
    "MotorTask"  // A name just for humans
    ,
    1024 * 4  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, 0);

  Serial.println("Setup Complete");
}

void loop() {
  // Add your Wi-Fi code here if required. The motor control will be done in the other task and core.

  while (stepper->isRunning() == true) {

    delay(50);
  }
}

void MotorTask(void *pvParameters) {

  stepper->setCurrentPosition(0);

  while (true) {

    stepper->moveTo(move_to_step);  // We tell the motor to move to a specific position. Use move() to move a certain distance rather than to a position.

    while (stepper->isRunning() == true) {
      delay(1);  // We need to kill some time while the motor moves
    }

    delay(3000);  // Delay 3 seconds for testing

    stepper->moveTo(0);  // Move to position 0

    while (stepper->isRunning() == true)  // isRunning() is true while the stepper is moving to position. We can use this time to wait for a stall
    {
      delay(1);  // We need to kill some time while the motor moves
    }

    delay(3000);  // Delay 3 seconds for testing
  }
}
