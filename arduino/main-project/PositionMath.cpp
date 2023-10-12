/*
    StatusLEDControl.cpp
    Code to control motors on SEED Lab robot
*/

#include "Arduino.h"
#include "PositionMath.h"

double _x;
double _y;
double _phi;

// wheelbase width in meters
double _wheelBaseWidth;

PositionMath::PositionMath(double wheelBaseWidth)
{
    _wheelBaseWidth = wheelBaseWidth;
}

double PositionMath::getX()
{
    return _x;
}

double PositionMath::getY()
{
    return _y;
}

double PositionMath::getPhi()
{
    return _phi;
}

double PositionMath::getRho()
{
    return _rho;
}

void PositionMath::resetPosition()
{
    this->resetPosition(0, 0, 0);
}

void PositionMath::resetPosition(double x, double y, double phi)
{
    _x = x;
    _y = y;
    _phi = phi;
}

void PositionMath::updatePosition(double numSeconds, double velocityLeft, double velocityRight)
{
    double newX = this->calculateX(_x, numSeconds, _phi, velocityLeft, velocityRight);
    double newY = this->calculateY(_y, numSeconds, _phi, velocityLeft, velocityRight);
    double newPhi = this->calculatePhi(_phi, numSeconds, _wheelBaseWidth, velocityLeft, velocityRight);
    _x = newX;
    _y = newY;
    _phi = newPhi;
    _rho = sqrt(pow(_x, 2) + pow(_y, 2));
}

double PositionMath::calculateX(double xOld, double deltaT, double phiOld, double velocityLeft, double velocityRight)
{
    return xOld + deltaT * cos(phiOld) * (velocityLeft + velocityRight) / 2;
}

double PositionMath::calculateY(double yOld, double deltaT, double phiOld, double velocityLeft, double velocityRight)
{
    return yOld + deltaT * sin(phiOld) * (velocityLeft + velocityRight) / 2;
}

double PositionMath::calculatePhi(double phiOld, double deltaT, double wheelBaseWidth, double velocityLeft, double velocityRight)
{
    double phiTemp = phiOld + deltaT * (velocityLeft - velocityRight) / wheelBaseWidth;
    if (phiTemp > 6.283)
        phiTemp = 0;
    if (phiTemp < 0)
        phiTemp = 6.283;
    return phiTemp;
}
