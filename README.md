# ServoGimbal
2D Gimbal for Arduino based on 2 Servos and MPU6050
## Requirements
- Arduino Pro, Pro Mini, Nano, Uno or Atmega2560 (atmega323 processor minimum)
- MPU6050 (gyro + accelerometer) mounted on camera and connected via I2C to Arduino
- Roll servo connected to PIN 7
- Pitch servo connected to PIN 6
- (optional) RC Channel (PWM) connected to PIN 2 for manual roll axis set
- (optional) RC Channel (PWM) connected to PIN 3 for manual pitch axis set

## Features
- full PID support for Roll and Pitch
- 50/333 Hz for Servo refresh frequency
- manual set Roll and Pitch axis via RC channels
- mautomatic MPU6050 orientation detection
- setup via serial port menu or graphical interface (using processing.org)
- experimental mavlink integration: please uncomment #define USE_MAVLINK 1 in GlobalDefines.h first. Actualy mavlink uses second hardware serial port (if present) or software serial port on PIN 4 (TX) and PIN 5 (RX)

## Graphical interface setup
- install processing.org (http://processing.org)
- add ControlP5 library to processing.org (http://www.sojamo.de/libraries/controlP5/) 
- open pde project from repository localized in Processing.org subfolder
- connect to arduino using properly serial port

## First steps
- connect to Arduino
- using serial port menu or processing.org detect gyro orientation
- set correct servo refresh frequency: 50Hz is standard, 333Hz for quick servos
- set properly P, I, D and Direction values for roll and pitch PID settings (using standard PID tuning methods)

##Licence
GNU/GPL 3.0
