/*
    PiCommunication.h
    Controls communication with Raspberry Pi
*/

#ifndef PiCommunication_h
#define PiCommunication_h

#include "Arduino.h"
#include <Wire.h>

class PiCommunication {
    public:
        PiCommunication(int pin);
        void begin();
    private:
        
};

#endif