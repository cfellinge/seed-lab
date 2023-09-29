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
    MotorControl(int initialMode);

    void begin();

    // set velocity of each motor, in m/s
    void setVelocities(double targetLeftVelocity, double targetRightVelocity);

    // set position of each motor, in radians
    // assumes start position is 0 radians
    void setPositions(double leftPosition, double rightPosition);

    void setMotorMode(int mode);

    int leftPinInterrupt();
    int rightPinInterrupt();

    void updateMotorValues(int millisecondInterval);
    double calculateMetersPerSecond(int countsRotated, int lastCountsRotated, int numMilliSeconds);

    double calculatePosition(int countsRotated);

    // sets direction of motors
    // side: 0 = left, 1 = right
    // direction: 0 = forwards, 1 = backwards
    void setDirection(int side, int direction);
    
    double getLeftVelocity();
    double getRightVelocity();

    int getLeftCount();
    int getRightCount();

    int getLeftEncoderPin();
    int getRightEncoderPin();

    double getLeftPosition();
    double getRightPosition();

    double mod2Pi(double input);

private:
    int clockwise(int motorCount);
    int counterClockwise(int motorCount);

    // h bridge control pins
    int _togglePin;

    int _leftVoltagePin;
    int _rightVoltagePin;

    int _leftDirectionPin;
    int _rightDirectionPin;

    int _leftEncoderAPin;
    int _leftEncoderBPin;

    int _rightEncoderAPin;
    int _rightEncoderBPin;

    // internal motor variables
    int _leftCount;
    int _rightCount;
};

#endif