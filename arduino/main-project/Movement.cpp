/*
    Movement.cpp
    Code to handle robot movement
*/

#include "Arduino.h"
#include "Movement.h"
#include "MotorControl.h"

MotorControl mc(0);

double forwardVel;
double rotationalVel;

double WHEEL_RADIUS = 0.0725;

double leftWriteValue;
double rightWriteValue;

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
    // setRotations(mc.getLeftPosition() + numRadians, mc.getRightPosition() + numRadians);
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


// takes in Va and deltaV, both in volts
void Movement::setVAandDV(double va, double dv)
{
    va = va * 255.0 / 8.0;
    dv = dv * 255.0 / 8.0;

    double leftVel = mc.getLeftVelocity();
    double rightVel = mc.getRightVelocity();

    forwardVel = WHEEL_RADIUS * (leftVel + rightVel) / 2;
    rotationalVel = WHEEL_RADIUS * (leftVel - rightVel) / 2;

    leftWriteValue = (va + dv) / 2;
    rightWriteValue = (va - dv) / 2;

    mc.setWriteValues(leftWriteValue, rightWriteValue);
}

double Movement::getForwardVel()
{
    return forwardVel;
}

double Movement::getRotationalVel()
{
    return rotationalVel;
}