/*
    PiCommunication.h
    Controls communication with Raspberry Pi
*/

#include "Arduino.h"
#include <Wire.h>

class PiCommunication {
    public:
        PiCommunication(int pin);
        void begin();
    private:
        
};