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
        PiCommunication();
    
        void begin();

        void updatePi(int numMillis);

        void printReceived();
        void receive(int);
        float getAngle();
        float getDistance();
        // void request();

    private:
};

#endif
