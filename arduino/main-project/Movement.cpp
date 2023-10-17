/*
    Movement.cpp
    Code to handle robot movement
*/

#include "Arduino.h"
#include "Movement.h"

MotorControl mc(0);
PositionMath pos(0);

double forwardVel;
double rotationalVel;

double forwardVelTarget;
double rotationalVelTarget;
double rhoTarget;

double WHEEL_RADIUS = 0.0725;

double leftWriteValue;
double rightWriteValue;

double KP_VEL_INNER = 10;
double KP_VEL_OUTER = 10;
double KI_VEL_OUTER = 10;

double velIntegralError = 0;

// max speed of robot, m/s
double ROBOT_MAX_SPEED = 0.5;

Movement::Movement(MotorControl motorController, PositionMath positionMath)
{
    mc = motorController;
    pos = positionMath;
}

void Movement::moveToCoordinates(double x, double y, double phi)
{
}

void Movement::moveAtSpeed(double leftSpeed, double rightSpeed)
{
    forwardVelTarget = leftSpeed;
}

void Movement::updateMovement(double numMilliseconds)
{
    double leftVel = mc.getLeftVelocity();
    double rightVel = mc.getRightVelocity();

    forwardVel = WHEEL_RADIUS * (leftVel + rightVel) / 2;
    rotationalVel = WHEEL_RADIUS * (leftVel - rightVel) / 2;

    double va = velOuterIntegralControl(pos.getRho(), rhoTarget, forwardVel, numMilliseconds);

    driveMotor(va, 0);
}

void Movement::moveForwards(double distance)
{
    rhoTarget = distance;
}

void Movement::setRotations(double leftPosition, double rightPosition)
{
}

double Movement::velOuterIntegralControl(double rho, double rho_desired, double vel, int millisecondInterval)
{
    // PI block
    double velPosError = rho_desired - rho;
    velIntegralError = velIntegralError + velPosError * (double)millisecondInterval / 1000.0;
    double desiredSpeed = KP_VEL_OUTER * velPosError + KI_VEL_OUTER * velIntegralError;
    double velError = desiredSpeed - vel;
    double vel_desired = KP_VEL_OUTER * velError;

    // integral windup block
    if (vel_desired > ROBOT_MAX_SPEED)
    {
        vel_desired = 255;

        if (velError > 255.0 / KP_VEL_OUTER)
        {
            velError = 255.0 / KP_VEL_OUTER;
        }

        velIntegralError = (vel_desired - KP_VEL_OUTER * velError) / KI_VEL_OUTER;
    }

    // inner feedback loop
    double va = velInnerProportionalControl(vel, vel_desired);

    // voltage to apply to both motors
    return va;
}

double Movement::velInnerProportionalControl(double vel_actual, double vel_desired)
{
    // P block
    double error = vel_desired - vel_actual;
    double kError = error * KP_VEL_INNER;
    double kErrorBounded = kError;

    // bounding block
    if (kErrorBounded > 8)
    {
        kErrorBounded = 8;
    }
    if (kErrorBounded < -8)
    {
        kErrorBounded = -8;
    }

    double va = kErrorBounded;

    return va;
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