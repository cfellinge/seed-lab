/*
    StatusLEDControl.h
    Code to control motors on SEED Lab robot
*/

#include "Arduino.h"

class StatusLEDControl {
    public:
        StatusLEDControl(int pin);
        int toggleLED();
    private:
        int _LEDState;
        int _LEDPin;
};