/*
    MotorControl.h
    Code to control motors on SEED Lab robot
*/
#ifndef MotorControl_h
#define MotorControl_h

#include "Arduino.h"

class MotorControl {
    public:
        MotorControl(int pin);
        
        void begin();
        void setSpeed(double leftSpeed, double rightSpeed);
        
        double getLeftSpeed();
        double getRightSpeed();

        int leftPinInterrupt();
        int rightPinInterrupt();

        void printFive();
    private:
        int clockwise(int motorCount);
        int counterClockwise(int motorCount);

        // h bridge control pins
        int _togglePin;

        int _leftVoltagePin;
        int _rightVoltagePin;

        int _switchForwardsPin;
        int _switchBackwardsPin;

        int _leftEncoderAPin;
        int _leftEncoderBPin;

        int _rightEncoderAPin;
        int _rightEncoderBPin;

        // internal motor variables
        int leftCount;
        int rightCount;

        // int leftLastCount;
        // int rightLastCount;

        // int leftLastEncoder;
        // int rightLastEncoder;
};

#endif