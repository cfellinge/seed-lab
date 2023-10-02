# MINI PROJECT HARDWARE SETUP

The Arduino code is object-oriented, and the code is split up into several libraries for ease of access and long-term scalability.

## Hardware Setup:

#### Digital Pins:
Pin 2 - Left Motor Encoder
Pin 3 - Right Motor Encoder

Pin 4 - Reserved for H Bridge
Pin 7, 8, 9, 10: Reserved for H Bridge

Pin 11: V+ for Task LED

Pin 12: Reserved for H Bridge

Pin 13: V+ for Clock LED

#### Analog Pins:
Analog 0: Reserved for H Bridge
Analog 1: Reserved for H Bridge
Analog 3: Raspberry Pi Data Link
Analog 4: Raspberry Pi Data Link

## Files

#### main-project.ino:

Main super-loop and timer interrupt, controls primary functions of Arduino.

#### MotorControl.h / MotorControl.cpp:

Controls motor encoder inputs and feedback control for the two motors.

#### PiCommunication.h / PiCommunication.cpp:

Recieves instructions from Raspberry Pi, and sends instructions to Raspberry Pi

#### PositionMath.h / PositionMath.cpp:

Math for dead reckoning position finding

#### StatusLEDControl.h / StatusLEDControl.cpp:

Basic class to control LEDs
