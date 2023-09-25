/*
    MotorControl.h
    Code to control motors on SEED Lab robot
*/
#ifndef MotorControl_h
#define MotorControl_h

#include "Arduino.h"

class MotorControl
{
public:
    MotorControl();

    void begin();

    double getLeftVelocity();
    double getRightVelocity();

    int getLeftCount();
    int getRightCount();

    int getLeftEncoderPin();
    int getRightEncoderPin();

    void setVelocities(double targetLeftVelocity, double targetRightVelocity);

    int leftPinInterrupt();
    int rightPinInterrupt();

    void updateMotorValues(int millisecondInterval);
    double calculateMetersPerSecond(int countsRotated, int lastCountsRotated, int numMilliSeconds);

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
    int _leftCount;
    int _rightCount;
};

#endif