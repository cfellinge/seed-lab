/*
    MotorControl.h
    Code to control motors on SEED Lab robot
*/

#include "Arduino.h"
#include "MotorControl.h"

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
// int leftCount;
// int rightCount;

// int leftLastCount;
// int rightLastCount;

int _leftEncoderState;
int _rightEncoderState;
int _leftLastEncoderState;
int _rightLastEncoderState;

MotorControl::MotorControl(int pin)
{
    this->_togglePin = 4;

    this->_leftVoltagePin = 9;
    this->_rightVoltagePin = 10;

    this->_switchForwardsPin = 7;
    this->_switchBackwardsPin = 8;

    this->_leftEncoderAPin = 2;
    this->_leftEncoderBPin = 6;

    this->_rightEncoderAPin = 3;
    this->_rightEncoderBPin = 6;
}

void MotorControl::printFive()
{
    Serial.println("Print 5");

    pinMode(_togglePin, OUTPUT);
    pinMode(_leftVoltagePin, OUTPUT);
    pinMode(_rightVoltagePin, OUTPUT);
    pinMode(_switchForwardsPin, OUTPUT);
    pinMode(_switchBackwardsPin, OUTPUT);

    pinMode(_leftEncoderAPin, INPUT);
    pinMode(_leftEncoderBPin, INPUT);
    pinMode(_rightEncoderAPin, INPUT);
    pinMode(_rightEncoderBPin, INPUT);
}

void MotorControl::begin()
{
    // attachInterrupt(digitalPinToInterrupt(_leftEncoderAPin), leftPinInterrupt, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(_rightEncoderAPin), rightPinInterrupt, CHANGE);
}

int MotorControl::leftPinInterrupt()
{
    _leftEncoderState = digitalRead(_leftEncoderAPin);

    if (_leftEncoderState == HIGH && _leftLastEncoderState == LOW)
    {
        // logic for CW and CCW rotations
        if (digitalRead(_leftEncoderBPin) == HIGH)
        {
            leftCount = counterClockwise(leftCount);
        }
        else
        {
            leftCount = clockwise(leftCount);
        }
    }

    // save current state of A
    _leftLastEncoderState = _leftEncoderState;
}

int MotorControl::rightPinInterrupt()
{
    _rightEncoderState = digitalRead(_rightEncoderAPin);

    if (_rightEncoderState == HIGH && _rightLastEncoderState == LOW)
    {
        // logic for CW and CCW rotations
        if (digitalRead(_rightEncoderBPin) == HIGH)
        {
            rightCount = counterClockwise(rightCount);
        }
        else
        {
            rightCount = clockwise(rightCount);
        }
    }

    // save current state of A
    _rightLastEncoderState = _rightEncoderState;
}

int MotorControl::clockwise(int motorCount)
{
    return motorCount + 1;
}

int MotorControl::counterClockwise(int motorCount)
{
    return motorCount - 1;
}

// double calculateMetersPerSecond(int countsRotated, int lastCountsRotated, int numMilliSeconds)
// {
//   double numRotations = (double)(countsRotated - lastCountsRotated) / encoderCountsPerRotation;
//   numRotations = numRotations * (numMilliSeconds / 1000.0) * 60.0;  //RETURN THIS FOR RPM VALUE
//   return numRotations * 0.00764; //THIS IS M/S USING A WHEEL DIAMETER OF 14.6 CM, CAN BE CHANGED
// }