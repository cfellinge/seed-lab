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
int _leftCount;
int _rightCount;

int _leftLastCount;
int _rightLastCount;

int _leftEncoderState;
int _rightEncoderState;
int _leftLastEncoderState;
int _rightLastEncoderState;

double _leftVelocity;
double _rightVelocity;

double _targetLeftVelocity;
double _targetRightVelocity;

int _leftWriteValue;
int _rightWriteValue;

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
    this->_rightEncoderBPin = 5;
}

void MotorControl::printFive()
{
    Serial.println("Print 5");
}

void MotorControl::begin()
{
    pinMode(_togglePin, OUTPUT);
    pinMode(_leftVoltagePin, OUTPUT);
    pinMode(_rightVoltagePin, OUTPUT);
    pinMode(_switchForwardsPin, OUTPUT);
    pinMode(_switchBackwardsPin, OUTPUT);

    pinMode(_leftEncoderAPin, INPUT);
    pinMode(_leftEncoderBPin, INPUT);
    pinMode(_rightEncoderAPin, INPUT);
    pinMode(_rightEncoderBPin, INPUT);

    // enable motors
    digitalWrite(_togglePin, HIGH);

    digitalWrite(_switchForwardsPin, HIGH);
    digitalWrite(_switchBackwardsPin, LOW);
}

void MotorControl::updateMotorValues(int millisecondInterval)
{
    _leftVelocity = calculateMetersPerSecond(_leftCount, _leftLastCount, millisecondInterval);
    _rightVelocity = calculateMetersPerSecond(_rightCount, _rightLastCount, millisecondInterval);
    _leftLastCount = _leftCount;
    _rightLastCount = _rightCount;

    // Serial.println((String)_leftVelocity + "\t" + (String)_rightVelocity);

    // feedback control code goes here
    // inputs: _leftVelocity, _targetLeftVelocity
    // output: _leftWriteValue (0 - 255)
    if (abs(_leftVelocity) < (_targetLeftVelocity - 0.05))
    {
        _leftWriteValue += 2;
    }
    if (abs(_leftVelocity) > (_targetLeftVelocity + 0.05))
    {
        _leftWriteValue -= 2;
    }
    if (abs(_rightVelocity) < (_targetRightVelocity - 0.05))
    {
        _rightWriteValue += 2;
    }
    if (abs(_rightVelocity) > (_targetRightVelocity + 0.05))
    {
        _rightWriteValue -= 2;
    }

    // check that write values are within hard bounds
    if (_leftWriteValue > 255) {
        _leftWriteValue = 255;
    }
    if (_leftWriteValue < 0) {
        _leftWriteValue = 0;
    }
    if (_rightWriteValue > 255) {
        _rightWriteValue = 255;
    }
    if (_rightWriteValue < 0) {
        _rightWriteValue = 0;
    }

    // write to motors
    analogWrite(_leftVoltagePin, _leftWriteValue);
    analogWrite(_rightVoltagePin, _rightWriteValue);
}

double MotorControl::calculateMetersPerSecond(int countsRotated, int lastCountsRotated, int numMilliSeconds)
{
    double numRotations = (double)(countsRotated - lastCountsRotated) / 800.0;
    double rotationsPerMinute = numRotations * (numMilliSeconds / 1000.0) * 60.0;

    // Serial.println((String)countsRotated + "\t" + (String)lastCountsRotated + "\t" + (String)numRotations + "\t" + (String)rotationsPerMinute + "\t" + (String)(rotationsPerMinute * 0.00764));

    return rotationsPerMinute;
    // return rotationsPerMinute * 0.00764; // THIS IS M/S USING A WHEEL DIAMETER OF 14.6 CM, CAN BE CHANGED
}

int MotorControl::leftPinInterrupt()
{
    _leftEncoderState = digitalRead(_leftEncoderAPin);

    if (_leftEncoderState == HIGH && _leftLastEncoderState == LOW)
    {
        // logic for CW and CCW rotations
        if (digitalRead(_leftEncoderBPin) == HIGH)
        {
            _leftCount = counterClockwise(_leftCount);
        }
        else
        {
            _leftCount = clockwise(_leftCount);
        }
        // Serial.println("Left Count: " + (String)_leftCount);
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
            _rightCount = counterClockwise(_rightCount);
        }
        else
        {
            _rightCount = clockwise(_rightCount);
        }
        // Serial.println("Right Count: " + (String)_rightCount);
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

double MotorControl::getLeftVelocity()
{
    return _leftVelocity;
}

double MotorControl::getRightVelocity()
{
    return _rightVelocity;
}

int MotorControl::getLeftCount()
{
return _leftCount;
}

int MotorControl::getRightCount()
{
return _rightCount;
}

int MotorControl::getLeftEncoderPin()
{
return _leftEncoderAPin;
}

int MotorControl::getRightEncoderPin()
{
return _rightEncoderAPin;
}

void MotorControl::setVelocities(double targetLeftVelocity, double targetRightVelocity)
{
    _targetLeftVelocity = targetLeftVelocity;
    _targetRightVelocity = targetRightVelocity;
}
