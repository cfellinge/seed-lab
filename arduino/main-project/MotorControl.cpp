/*
    MotorControl.h
    Code to control motors on SEED Lab robot
*/

#include "Arduino.h"
#include "MotorControl.h"

// wheel radius in meters
const double WHEEL_RADIUS = 0.0725;

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
}

void MotorControl::setWriteValues(double leftWrite, double rightWrite)
{
    // set direction based on +/- voltage inputs
    if (leftWrite < 0)
    {
        setDirection(0, 0);
        leftWrite = -leftWrite;
    }
    else
    {
        setDirection(0, 1);
    }

    if (rightWrite < 0)
    {
        setDirection(1, 1);
        rightWrite = -rightWrite;
    }
    else
    {
        setDirection(1, 0);
    }

    // check values bounded
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

    _leftWriteValue = leftWrite;
    _rightWriteValue = rightWrite;

    analogWrite(_leftVoltagePin, _leftWriteValue);
    analogWrite(_rightVoltagePin, _rightWriteValue);
}

double MotorControl::calculateMetersPerSecond(int countsRotated, int lastCountsRotated, int numMilliSeconds)
{
    double metersTravelled = (double)(countsRotated - lastCountsRotated) * 0.000561;
    double metersPerSecond = metersTravelled / ((double)numMilliSeconds / 1000.0);
    return metersPerSecond;
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
        if (digitalRead(_rightEncoderBPin) == LOW) // change this from HIGH to LOW to change which way the wheel turns
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
    if (!(mode >= 0 && mode <= 4))
    {
        Serial.println("Error: Attempted to set motor mode outside of range");
        return;
    }

    _motorMode = mode;
}