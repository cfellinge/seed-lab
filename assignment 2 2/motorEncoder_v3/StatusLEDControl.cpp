/*
    StatusLEDControl.cpp
    Code to control motors on SEED Lab robot
*/

#include "Arduino.h"
#include "StatusLEDControl.h"

int _LEDPin;
int _LEDState;

StatusLEDControl::StatusLEDControl(int pin) {
    this->_LEDPin = pin;
    _LEDState = LOW;
}

int StatusLEDControl::toggleLED() {
    digitalWrite(_LEDPin, _LEDState);
    _LEDState = !_LEDState;
    return _LEDState;
}