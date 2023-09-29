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
    pinMode(_LEDPin, OUTPUT);
    _LEDState = LOW;
}

int StatusLEDControl::toggleLED() {
    digitalWrite(_LEDPin, _LEDState);
    _LEDState = !_LEDState;
    return _LEDState;
}

void StatusLEDControl::onLED() {
    digitalWrite(_LEDPin, HIGH);
    _LEDState = HIGH;
}

void StatusLEDControl::offLED() {
    digitalWrite(_LEDPin, LOW);
    _LEDState = LOW;
}