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

double forwardVelTarget;
double rotationalVelTarget;

double WHEEL_RADIUS = 0.0725;

double leftWriteValue;
double rightWriteValue;

double KP_VEL_INNER = 10;

Movement::Movement(MotorControl motorController)
{
    mc = motorController;
}

void Movement::moveToCoordinates(double x, double y, double phi)
{
}

void Movement::moveAtSpeed(double leftSpeed, double rightSpeed)
{
    forwardVelTarget = leftSpeed;
}

void Movement::updateMovement(double numMilliseconds) {
    double leftVel = mc.getLeftVelocity();
    double rightVel = mc.getRightVelocity();

    forwardVel = WHEEL_RADIUS * (leftVel + rightVel) / 2;
    rotationalVel = WHEEL_RADIUS * (leftVel - rightVel) / 2;

    double va = velInnerProportionalControl(forwardVel, forwardVelTarget);

    driveMotor(va, 0);
}

void Movement::moveForwards(double distance)
{
    
}

void Movement::setRotations(double leftPosition, double rightPosition)
{

}

double Movement::velOuterIntegralControl(double rho, double rho_desired)
{
}

double Movement::velInnerProportionalControl(double vel_actual, double vel_desired) {
    double error = vel_desired - vel_actual;
    
    double kError = error * KP_VEL_INNER;
    
    double kErrorBounded = kError;

    if (kErrorBounded > 8)
    {
        kErrorBounded = 8;
    }
    if (kErrorBounded < -8)
    {
        kErrorBounded = -8;
    }
    
    return kErrorBounded;
}   


void Movement::rotateLeft(double angle, int millisecondInterval)
{
}


// takes in Va and deltaV, both in volts
void Movement::driveMotor(double va, double dv)
{
    va = va * 255.0 / 8.0;
    dv = dv * 255.0 / 8.0;

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