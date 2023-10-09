/*
    Movement.cpp
    Code to handle robot movement
*/

#include "Arduino.h"
#include "Movement.h"

MotorControl mc;

Movement::Movement(MotorControl motorController)
{
    mc = motorController;
}

void Movement::moveToCoordinates(double x, double y, double phi)
{
}

void Movement::moveAtSpeed(double leftSpeed, double rightSpeed)
{
}

void Movement::moveForwards(double distance)
{
    double numRadians = distance * 10;
    setRotations(mc.getLeftPosition() + numRadians, mc.getRightPosition() + numRadians);
}

void Movement::setRotations(double leftPosition, double rightPosition)
{
    
}

double integralControl()
{
}

void Movement::rotateLeft(double angle, int millisecondInterval)
{
    
}

