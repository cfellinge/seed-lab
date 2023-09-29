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

    setMotorMode(initalMode);
}

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

void MotorControl::updateMotorValues(int millisecondInterval)
{
    _leftVelocity = -1 * calculateMetersPerSecond(_leftCount, _leftLastCount, millisecondInterval);
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


    else if (_motorMode == 2)
    {
        double _leftTargetPlusPi = mod2Pi(_targetLeftPosition + PI);
        double _rightTargetPlusPi = mod2Pi(_targetRightPosition + PI);
        
        if (_targetLeftPosition > PI) {
            if (_leftPosition > _targetLeftPosition || _leftPosition < _leftTargetPlusPi) {
                setDirection(0, 0);
            }
            else {
                setDirection(0, 1);
            }
        }
        else if (_targetLeftPosition < PI) {
            
            if (_leftPosition < _targetLeftPosition || _leftPosition > _leftTargetPlusPi) {
                setDirection(0, 1);
            }
            else {
                setDirection(0, 0);
            }
        }

         if (_targetRightPosition > PI) {
            if (_rightPosition > _targetRightPosition || _rightPosition < _rightTargetPlusPi) {
                setDirection(0, 0);
            }
            else {
                setDirection(0, 1);
            }
        }
        else if (_targetRightPosition < PI) {
            
            if (_rightPosition < _targetRightPosition || _rightPosition > _rightTargetPlusPi) {
                setDirection(0, 1);
            }
            else {
                setDirection(0, 0);
            }
        }
        
        _leftWriteValue = abs(( _targetLeftPosition - _leftPosition) * 40);
        _rightWriteValue = abs(( _targetRightPosition - _rightPosition) * 40);

        // Serial.println("Left Goal: " + (String)_targetLeftPosition + ", Actual: " + (String)_leftPosition + ", Write Value: " +  (String)_leftWriteValue + ", Direction: " + (String)digitalRead(PIN7));
    }

    // check that write values are within hard bounds
    if (_leftWriteValue > 255)
    {
        _leftWriteValue = 255;
    }
    if (_leftWriteValue < 0)
    {
        _leftWriteValue = 0;
    }
    if (_rightWriteValue > 255)
    {
        _rightWriteValue = 255;
    }
    if (_rightWriteValue < 0)
    {
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

    // return rotationsPerMinute;
    return rotationsPerMinute * 0.00764; // THIS IS M/S USING A WHEEL DIAMETER OF 14.6 CM, CAN BE CHANGED
}

// returns motor angle in radians
// assumes motor starts at 0 radians
double MotorControl::calculatePosition(int countsRotated)
{
    double numRotations = (double)(countsRotated) / 800.0;
    double numRadians = numRotations * (2 * PI);
    while (numRadians > 2 * PI) {
        numRadians -= 2 * PI;
    }
    while (numRadians < 0) {
        numRadians += 2 * PI;
    }
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
    if (!(direction == 0 || direction == 1)) {
        Serial.println("MotorControl::setDirection: Error: Tried to set invalid direction.");
        return;
    }
    if (side == 0) {
        digitalWrite(_leftDirectionPin, direction);
        // Serial.println("Set left direction to " + (String)direction);
    }
    if (side == 1) {
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

double MotorControl::mod2Pi(double input)
{
    while (input > 2 * PI) {
        input -= 2 * PI;
    }
    while (input < 0) {
        input += 2 * PI;
    }
    return input;
}

void MotorControl::setVelocities(double targetLeftVelocity, double targetRightVelocity)
{
    _targetLeftVelocity = targetLeftVelocity;
    _targetRightVelocity = targetRightVelocity;
}

void MotorControl::setPositions(double leftPosition, double rightPosition)
{
    leftPosition = mod2Pi(leftPosition);
    rightPosition = mod2Pi(rightPosition);

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