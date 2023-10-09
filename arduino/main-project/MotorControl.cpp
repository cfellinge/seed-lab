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

int _leftDirectionPin;
int _rightDirectionPin;

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

double _leftPosition;
double _rightPosition;

double _targetLeftPosition;
double _targetRightPosition;

int _motorMode;

int _leftWriteValue;
int _rightWriteValue;

// PID Tuning Variables
const double _KP = 9;
const double _KI = 0.15;

double _leftIntegralError;
double _rightIntegralError;

double _leftPosError;
double _rightPosError;

double _rawLeftWriteValue;
double _rawRightWriteValue;

// default constructor
MotorControl::MotorControl(int initalMode)
{
    this->_togglePin = 4;

    this->_leftVoltagePin = 9;
    this->_rightVoltagePin = 10;

    this->_leftDirectionPin = 7;
    this->_rightDirectionPin = 8;

    this->_leftEncoderAPin = 2;
    this->_leftEncoderBPin = 6;

    this->_rightEncoderAPin = 3;
    this->_rightEncoderBPin = 5;

    _leftIntegralError = 0;
    _rightIntegralError = 0;

    setMotorMode(initalMode);
}

// called in setup to initialize state
void MotorControl::begin()
{
    pinMode(_togglePin, OUTPUT);
    pinMode(_leftVoltagePin, OUTPUT);
    pinMode(_rightVoltagePin, OUTPUT);
    pinMode(_leftDirectionPin, OUTPUT);
    pinMode(_rightDirectionPin, OUTPUT);

    pinMode(_leftEncoderAPin, INPUT);
    pinMode(_leftEncoderBPin, INPUT);
    pinMode(_rightEncoderAPin, INPUT);
    pinMode(_rightEncoderBPin, INPUT);

    // enable motors
    digitalWrite(_togglePin, HIGH);

    // digitalWrite(_switchForwardsPin, HIGH);
    // digitalWrite(_switchBackwardsPin, LOW);

    setVelocities(0, 0);
    setPositions(0, 0);

    setDirection(0, 0);
    setDirection(1, 0);

    Serial.println("Motor Controller Configured.");
}

// update how fast motors are spinning and their current position
void MotorControl::updateMotorValues(int millisecondInterval)
{
    _leftVelocity = calculateMetersPerSecond(_leftCount, _leftLastCount, millisecondInterval);
    _rightVelocity = calculateMetersPerSecond(_rightCount, _rightLastCount, millisecondInterval);

    _leftPosition = calculatePosition(_leftCount);
    _rightPosition = calculatePosition(_rightCount);

    _leftLastCount = _leftCount;
    _rightLastCount = _rightCount;

    // Serial.println((String)_leftVelocity + "\t" + (String)_rightVelocity);

    // feedback control code goes here
    // inputs: _leftVelocity, _targetLeftVelocity
    // output: _leftWriteValue (0 - 255)

    if (_motorMode == 0)
    {
        _leftWriteValue = 0;
        _rightWriteValue = 0;
    }

    // velocity control, proportional (bad)
    else if (_motorMode == 1)
    {
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
    }

    // positional control, PID tuned (good)
    else if (_motorMode == 2)
    {
        if (_leftPosition > _targetLeftPosition)
        {
            setDirection(0, 0);
        }
        else
        {
            setDirection(0, 1);
        }

        if (_rightPosition > _targetRightPosition)
        {
            setDirection(1, 0);
        }
        else
        {
            setDirection(1, 1);
        }

        _leftPosError = abs(_targetLeftPosition - _leftPosition);
        _leftIntegralError = _leftIntegralError + _leftPosError * (double)millisecondInterval / 1000;
        double _leftDesiredSpeed = _KP * _leftPosError + _KI * _leftIntegralError;
        double _leftError = _leftDesiredSpeed - _leftVelocity;
        _leftWriteValue = _KP * _leftError;

        if (_leftWriteValue > 255)
        {
            _leftWriteValue = 255;

            if (_leftError > 255.0 / _KP)
            {
                _leftError = 255.0 / _KP;
            }

            _leftIntegralError = (_leftWriteValue - _KP * _leftError) / _KI;
        }

        _rightPosError = abs(_targetRightPosition - _rightPosition);
        _rightIntegralError = _rightIntegralError + _rightPosError * (double)millisecondInterval / 1000;
        double _rightDesiredSpeed = _KP * _rightPosError + _KI * _rightIntegralError;
        double _rightError = _rightDesiredSpeed - _rightVelocity;
        _rightWriteValue = _KP * _rightError;

        if (_rightWriteValue > 255)
        {
            _rightWriteValue = 255;

            if (_rightError > 255.0 / _KP)
            {
                _rightError = 255.0 / _KP;
            }

            _rightIntegralError = (_rightWriteValue - _KP * _rightError) / _KI;
        }
    }

    // accept a raw write value
    else if (_motorMode == 3) {
        _leftWriteValue = _rawLeftWriteValue;
        _rightWriteValue = _rawRightWriteValue;
    }

    // go foward a set distance
    else if (_motorMode == 4)
    {
        if (_leftPosition > _targetLeftPosition)
        {
            setDirection(0, 0);
        }
        else
        {
            setDirection(0, 1);
        }

        if (_rightPosition > _targetRightPosition)
        {
            setDirection(1, 1);
        }
        else
        {
            setDirection(1, 0);
        }

        _leftPosError = abs(_targetLeftPosition - _leftPosition);
        _leftIntegralError = _leftIntegralError + _leftPosError * (double)millisecondInterval / 1000;
        double _leftDesiredSpeed = _KP * _leftPosError + _KI * _leftIntegralError;
        double _leftError = _leftDesiredSpeed - _leftVelocity;
        _leftWriteValue = _KP * _leftError;

        if (_leftWriteValue > 255)
        {
            _leftWriteValue = 255;

            if (_leftError > 255.0 / _KP)
            {
                _leftError = 255.0 / _KP;
            }

            _leftIntegralError = (_leftWriteValue - _KP * _leftError) / _KI;
        }

        _rightPosError = abs(_targetRightPosition - _rightPosition);
        _rightIntegralError = _rightIntegralError + _rightPosError * (double)millisecondInterval / 1000;
        double _rightDesiredSpeed = _KP * _rightPosError + _KI * _rightIntegralError;
        double _rightError = _rightDesiredSpeed - _rightVelocity;
        _rightWriteValue = _KP * _rightError;

        if (_rightWriteValue > 255)
        {
            _rightWriteValue = 255;

            if (_rightError > 255.0 / _KP)
            {
                _rightError = 255.0 / _KP;
            }

            _rightIntegralError = (_rightWriteValue - _KP * _rightError) / _KI;
        }
    }

    // Serial.println("Left Goal: " + (String)_targetLeftPosition + ", Actual: " + (String)_leftPosition + ", Write Value: " +  (String)_leftWriteValue + ", Direction: " + (String)digitalRead(PIN7));

    // check that write values are within hard bounds


    // write to motors
    analogWrite(_leftVoltagePin, _leftWriteValue);
    analogWrite(_rightVoltagePin, _rightWriteValue);
}

void MotorControl::setWriteValues(double leftWrite, double rightWrite)
{
    if (leftWrite > 255)
    {
        leftWrite = 255;
    }
    if (leftWrite < 0)
    {
        leftWrite = 0;
    }
    if (rightWrite > 255)
    {
        rightWrite = 255;
    }
    if (rightWrite < 0)
    {
        rightWrite = 0;
    }

    _rawLeftWriteValue = leftWrite;
    _rawRightWriteValue = rightWrite;
}


double MotorControl::calculateMetersPerSecond(int countsRotated, int lastCountsRotated, int numMilliSeconds)
{
    double numRotations = (double)(countsRotated - lastCountsRotated) / 800.0;
    double rotationsPerMinute = numRotations * (numMilliSeconds / 1000.0) * 60.0;

    // Serial.println((String)countsRotated + "\t" + (String)lastCountsRotated + "\t" + (String)numRotations + "\t" + (String)rotationsPerMinute + "\t" + (String)(rotationsPerMinute * 0.00764));

    // return rotationsPerMinute;
    return rotationsPerMinute * 0.00764; // THIS IS M/S USING A WHEEL DIAMETER OF 14.6 CM, CAN BE CHANGED
}

// returns motor angle in radians
// assumes motor starts at 0 radians
double MotorControl::calculatePosition(int countsRotated)
{
    double numRotations = (double)(countsRotated) / 800.0;

    double numRadians = numRotations * (2 * PI);

    numRadians = mod2Pi(numRadians);

    return numRadians;
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
            _rightCount = clockwise(_rightCount);
        }
        else
        {
            _rightCount = counterClockwise(_rightCount);
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

void MotorControl::setDirection(int side, int direction)
{
    if (!(direction == 0 || direction == 1))
    {
        Serial.println("MotorControl::setDirection: Error: Tried to set invalid direction.");
        return;
    }
    if (side == 0)
    {
        digitalWrite(_leftDirectionPin, direction);
        // Serial.println("Set left direction to " + (String)direction);
    }
    if (side == 1)
    {
        digitalWrite(_rightDirectionPin, direction);
    }
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

double MotorControl::getLeftPosition()
{
    return _leftPosition;
}

double MotorControl::getRightPosition()
{
    return _rightPosition;
}

double MotorControl::getLeftWriteValue()
{
    return _leftWriteValue;
}

double MotorControl::getRightWriteValue()
{
    return _rightWriteValue;
}

double MotorControl::mod2Pi(double input)
{
    //     while (input > 2 * PI)
    //     {
    //         input -= 2 * PI;
    //     }
    //     while (input < 0)
    //     {
    //         input += 2 * PI;
    //     }
    return input;
}

void MotorControl::setVelocities(double targetLeftVelocity, double targetRightVelocity)
{
    _targetLeftVelocity = targetLeftVelocity;
    _targetRightVelocity = targetRightVelocity;
}

void MotorControl::setPositions(double leftPosition, double rightPosition)
{
    // leftPosition = mod2Pi(leftPosition);
    // rightPosition = mod2Pi(rightPosition);

    _targetLeftPosition = leftPosition;
    _targetRightPosition = rightPosition;
}

// 0 = off
// 1 = velocity control
// 2 = position control
void MotorControl::setMotorMode(int mode)
{
    if (!((mode == 0) || (mode == 1) || (mode == 2)))
    {
        Serial.println("Error: Attempted to set motor mode outside of range");
        return;
    }

    _motorMode = mode;
}