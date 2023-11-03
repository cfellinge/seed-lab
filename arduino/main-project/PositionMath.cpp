/*
    StatusLEDControl.cpp
    Code to control motors on SEED Lab robot
*/

#include "Arduino.h"
#include "PositionMath.h"

float _x;
float _y;
float _phi;

// wheelbase width in meters
float _wheelBaseWidth;

PositionMath::PositionMath(float wheelBaseWidth)
{
    _wheelBaseWidth = wheelBaseWidth;
}

float PositionMath::getX()
{
    return _x;
}

float PositionMath::getY()
{
    return _y;
}

float PositionMath::getPhi()
{
    return _phi;
}

float PositionMath::getRho()
{
    return _rho;
}

void PositionMath::resetPosition()
{
    this->resetPosition(0, 0, 0);
}

void PositionMath::resetPosition(float x, float y, float phi)
{
    _x = x;
    _y = y;
    _phi = phi;
}

void PositionMath::updatePosition(float numMilliseconds, float velocityLeft, float velocityRight)
{
    float numSeconds = numMilliseconds / 1000.0;

    float newX = this->calculateX(_x, numSeconds, _phi, velocityLeft, velocityRight);
    float newY = this->calculateY(_y, numSeconds, _phi, velocityLeft, velocityRight);
    float newPhi = this->calculatePhi(_phi, numSeconds, _wheelBaseWidth, velocityLeft, velocityRight);
    
    _x = newX;
    _y = newY;
    _phi = newPhi;
    _rho = sqrt(pow(_x, 2) + pow(_y, 2));
}

float PositionMath::calculateX(float xOld, float deltaT, float phiOld, float velocityLeft, float velocityRight)
{
    return xOld + deltaT * cos(phiOld) * (velocityLeft + velocityRight) / 2;
}

float PositionMath::calculateY(float yOld, float deltaT, float phiOld, float velocityLeft, float velocityRight)
{
    return yOld + deltaT * sin(phiOld) * (velocityLeft + velocityRight) / 2;
}

float PositionMath::calculatePhi(float phiOld, float deltaT, float wheelBaseWidth, float velocityLeft, float velocityRight)
{
    float phiTemp = phiOld + deltaT * (velocityLeft - velocityRight) / wheelBaseWidth;

    return phiTemp;
}
