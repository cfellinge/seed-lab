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

        void printReceived();
        void getInstruction();
        void receive();
        void request();

        
    private:
        volatile uint8_t offset;
        volatile uint8_t instruction[32];
        volatile uint8_t msgLength;
        volatile uint8_t reply;
        uint8_t localReply;
};

#endif