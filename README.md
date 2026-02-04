# Turntable Controls

A stepper motor-based turntable with closed-loop position control for robotics applications. Eventually this will sync with a UR5e arm at 125Hz.

## Hardware

- **Motor**: NEMA 17 stepper (17HE15-1504S, 1.5A, 1.8°/step)
- **Driver**: A4988 stepper driver (1/8 microstepping, 1600 steps/rev)
- **Encoder**: AS5600 magnetic encoder (12-bit absolute position)
- **Controller**: ESP32 microcontroller
- **Power**: 12V motor supply, 5V logic via buck converter
- **Mechanical**: 4:1 reduction using HTD-5M timing belt and pulleys

## Setup

The firmware runs on ESP32 using Arduino IDE. We're using the Rob Tillaart AS5600 library for the encoder and basic GPIO control for the stepper (pins 25/26/27 for STEP/DIR/ENABLE).

MATLAB handles the control side - system identification, PID tuning, and real-time communication with the ESP32 over serial at 115200 baud.

## Progress

**Electrical Dynamics**  
We isolated and controlled just the electrical side of the motor first. Did system ID on the current/voltage dynamics to understand the motor's electrical behavior independent of mechanical load. Tuned a PID controller for just the electrical dynamics of the system.

**Full Mechanical System**  
Ran system ID on the complete turntable system including the motor, timing belt reduction, encoder, and mechanical inertia. Got transfer functions that capture the real dynamics of the physical setup.

**Velocity Profile System ID**  
Since the end application uses trajectory messages (position + velocity commands), we identified the system using velocity as the input. This matches how it'll actually be used with the robot arm.

**Controller Tuning**  
Currently working on tuning PID controller. Using hardware-in-the-loop testing with MATLAB's System Identification Toolbox and the Arduino Hardware Toolbox to iterate on gains.
