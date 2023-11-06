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
    void setVelocities(float targetLeftVelocity, float targetRightVelocity);

    // set position of each motor, in radians
    // assumes start position is 0 radians
    void setPositions(float leftPosition, float rightPosition);

    void setMotorMode(int mode);

    int leftPinInterrupt();
    int rightPinInterrupt();

    void updateMotorValues(int millisecondInterval);

    float calculateMetersPerSecond(int countsRotated, int lastCountsRotated, int numMilliSeconds);

    float calculatePosition(int countsRotated);

    void setWriteValues(float leftWrite, float rightWrite);

    // sets direction of motors
    // side: 0 = left, 1 = right
    // direction: 0 = forwards, 1 = backwards
    void setDirection(int side, int direction);
    
    float getLeftVelocity();
    float getRightVelocity();

    int getLeftCount();
    int getRightCount();

    int getLeftEncoderPin();
    int getRightEncoderPin();

    float getLeftPosition();
    float getRightPosition();

    float getLeftWriteValue();
    float getRightWriteValue();

    float mod2Pi(float input);

    // void setWheelPosition(float targetLeftWheelPosition, float targetRightWheelPosition);
    // void updateWheelPositionValues(int millisecondInterval);
    
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