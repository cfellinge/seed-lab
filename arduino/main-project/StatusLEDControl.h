/*
    StatusLEDControl.h
    Code to control an LED for seed lab robot
*/

#ifndef StatusLEDControl_h
#define StatusLEDControl_h

#include "Arduino.h"

class StatusLEDControl
{
public:
    StatusLEDControl(int pin);
    int toggleLED();
    void onLED();
    void offLED();

private:
    int _LEDState;
    int _LEDPin;
};

#endif