/*
    Movement.cpp
    Code to handle robot movement
*/

#include "Arduino.h"
#include "Movement.h"

float forwardVel;
float rotationalVel;

float forwardVelTarget;
float rotationalVelTarget;

float xTarget;
float yTarget;
float rhoTarget;
float phiTarget;
float radiusTarget;

float va;
float dv;

float WHEEL_RADIUS;

float leftWriteValue;
float rightWriteValue;

enum MODE
{
    GO_TO_COORDINATES,
    ROTATE_TO_ANGLE,
    STAY_STILL,
    NO_FEEDBACK_CONTROLS,
    CIRCLE_TIME,
    ROTATE_AT_SPEED
};

MODE mode;

float velIntegralError = 0;
float angularVelIntegralError = 0;

// VA - go zoom straight
float KP_VEL_INNER = 6;
float KP_VEL_OUTER = 4;
float KI_VEL_OUTER = 0.12;

// DV - spin in circle
float KP_SPIN_INNER = 6;
float KP_SPIN_OUTER = 3;
float KI_SPIN_OUTER = 0.2;

// max speed of robot, m/s
float ROBOT_MAX_SPEED = 1.5;

// max rotational velocity of robot, rad/s
float ROBOT_MAX_SPIN = 0.8;

Movement::Movement(MotorControl &motorController, PositionMath &positionMath, float wheelRadius) : mc(motorController), pos(positionMath)
{
    WHEEL_RADIUS = wheelRadius;
}

// move robot to coordinates (x, y) in meters
// phi currently does not do anything
void Movement::moveToCoordinates(float x, float y, float phi)
{
    mode = GO_TO_COORDINATES;
    xTarget = x;
    yTarget = y;
    phiTarget = phi;
}

float Movement::getXYError()
{
    float xErr = pos.getX() - xTarget;
    float yErr = pos.getY() - yTarget;
    return sqrt(pow(xErr, 2) + pow(yErr, 2));
}

float Movement::getPhiError()
{
    return calculatePhiError(pos.getPhi(), phiTarget);
}

// move robot in circle around coordinates (x, y) with radius r
void Movement::goInCircle(float x, float y, float r)
{
    forwardVelTarget = 0.5;
    mode = CIRCLE_TIME;
    xTarget = x;
    yTarget = y;
    radiusTarget = r;
}

// move robot straight forward a set distance (meters)
void Movement::moveForwards(float distance)
{
    mode = GO_TO_COORDINATES;
    xTarget = distance * cos(pos.getPhi());
    yTarget = distance * sin(pos.getPhi());
    phiTarget = pos.getPhi();
}

// rotate robot to face a set angle (radians)
void Movement::rotateLeft(float angle)
{
    mode = ROTATE_TO_ANGLE;
    phiTarget = angle;
    xTarget = pos.getX();
    yTarget = pos.getY();
}

// doesn't work
void Movement::moveAtSpeed(float leftSpeed, float rightSpeed)
{
}

void Movement::rotateAtSpeed(float rotationalSpeed) {
    mode = ROTATE_AT_SPEED;
    rotationalVelTarget = rotationalSpeed;
}

void Movement::stop()
{
    mode = STAY_STILL;
}

void Movement::updateMovement(float numMilliseconds)
{
    float leftVel = mc.getLeftVelocity();
    float rightVel = mc.getRightVelocity();

    forwardVel = WHEEL_RADIUS * (leftVel + rightVel) / 2;
    rotationalVel = WHEEL_RADIUS * (leftVel - rightVel) / 2;

    float rhoActual = pos.getRho();
    float phiActual = pos.getPhi();

    switch (mode)
    {
    case GO_TO_COORDINATES:
        rhoTarget = getXYError();
        // phiActual = fmod(phiActual, 2*PI);
        phiTarget = atan2(yTarget - pos.getY(), xTarget - pos.getX());

        va = velOuterIntegralControl(0, rhoTarget, forwardVel, numMilliseconds);
        dv = angularVelOuterIntegralControl(phiActual, phiTarget, rotationalVel, numMilliseconds);
        break;

    case ROTATE_TO_ANGLE:
        rhoTarget = pos.getRho();
        phiTarget = phiTarget;
        va = velOuterIntegralControl(rhoActual, rhoTarget, forwardVel, numMilliseconds);
        dv = angularVelOuterIntegralControl(phiActual, phiTarget, rotationalVel, numMilliseconds);
        break;

    case STAY_STILL:
        va = velOuterIntegralControl(rhoActual, rhoActual, forwardVel, numMilliseconds);
        dv = angularVelOuterIntegralControl(phiActual, phiActual, rotationalVel, numMilliseconds);
        break;

    case CIRCLE_TIME:
        Serial.println("Forward vel: " + (String)forwardVel + ", forwardVelTarget: " + (String)forwardVelTarget);
        va = velInnerProportionalControl(forwardVel, forwardVelTarget);
        dv = angularVelInnerProportionalControl(rotationalVel, forwardVelTarget / radiusTarget);
        break;

    case ROTATE_AT_SPEED:
        va = velInnerProportionalControl(forwardVel, 0);
        dv = angularVelInnerProportionalControl(rotationalVel, rotationalVelTarget);
        break;

    case NO_FEEDBACK_CONTROLS:
        break;
    }

    // dv = 0;
    // Serial.println("VA: " + (String)va + " DV: " + (String)dv);
    driveMotor(va, dv);
}

float Movement::velOuterIntegralControl(float rho, float rho_desired, float vel, int millisecondInterval)
{
    // PI block
    float velPosError = rho_desired - rho;
    velIntegralError = velIntegralError + velPosError * (float)millisecondInterval / 1000.0;
    float desiredSpeed = KP_VEL_OUTER * velPosError + KI_VEL_OUTER * velIntegralError;
    float velError = desiredSpeed - vel;
    float vel_desired = KP_VEL_OUTER * velError;

    // integral windup block
    if (vel_desired > ROBOT_MAX_SPEED)
    {
        vel_desired = ROBOT_MAX_SPEED;

        if (velError > ROBOT_MAX_SPEED / KP_VEL_OUTER)
        {
            velError = ROBOT_MAX_SPEED / KP_VEL_OUTER;
        }

        velIntegralError = (vel_desired - KP_VEL_OUTER * velError) / KI_VEL_OUTER;
    }

    if (vel_desired < -ROBOT_MAX_SPEED)
    {
        vel_desired = -ROBOT_MAX_SPEED;

        if (velError < -ROBOT_MAX_SPEED / KP_VEL_OUTER)
        {
            velError = -ROBOT_MAX_SPEED / KP_VEL_OUTER;
        }

        velIntegralError = (vel_desired - KP_VEL_OUTER * velError) / KI_VEL_OUTER;
    }

    // inner feedback loop
    float va = velInnerProportionalControl(vel, vel_desired);

    // voltage to apply to both motors
    return va;
}

float Movement::velInnerProportionalControl(float vel_actual, float vel_desired)
{
    // P block
    float error = vel_desired - vel_actual;
    float kError = error * KP_VEL_INNER;
    float kErrorBounded = kError;

    // bounding block
    if (kErrorBounded > 8)
    {
        kErrorBounded = 8;
    }
    if (kErrorBounded < -8)
    {
        kErrorBounded = -8;
    }

    float va = kErrorBounded;

    return va;
}

float Movement::angularVelOuterIntegralControl(float phi, float phi_desired, float angularVel, int millisecondInterval)
{
    // PI block
    float angularVelPosError = calculatePhiError(phi, phi_desired); // radians

    angularVelIntegralError = angularVelIntegralError + angularVelPosError * (float)millisecondInterval / 1000.0; // radians * seconds

    float desiredSpeed = KP_SPIN_OUTER * angularVelPosError + KI_SPIN_OUTER * angularVelIntegralError;

    float angularVelError = desiredSpeed - angularVel;

    float angularVel_desired = KP_SPIN_OUTER * angularVelError;

    // integral windup block
    if (angularVel_desired > ROBOT_MAX_SPIN)
    {
        angularVel_desired = ROBOT_MAX_SPIN;

        if (angularVelError > ROBOT_MAX_SPIN / KP_SPIN_OUTER)
        {
            angularVelError = ROBOT_MAX_SPIN / KP_SPIN_OUTER;
        }

        angularVelIntegralError = (angularVel_desired - KP_SPIN_OUTER * angularVelError) / KI_SPIN_OUTER;
    }

    if (angularVel_desired < -ROBOT_MAX_SPIN)
    {
        angularVel_desired = -ROBOT_MAX_SPIN;

        if (angularVelError < -ROBOT_MAX_SPIN / KP_SPIN_OUTER)
        {
            angularVelError = -ROBOT_MAX_SPIN / KP_SPIN_OUTER;
        }

        angularVelIntegralError = (angularVel_desired - KP_SPIN_OUTER * angularVelError) / KI_SPIN_OUTER;
    }

    // inner feedback loop
    float dv = angularVelInnerProportionalControl(angularVel, angularVel_desired);

    // voltage to apply to both motors
    return dv;
}

float Movement::angularVelInnerProportionalControl(float angularVel_actual, float angularVel_desired)
{
    // P block
    float error = angularVel_desired - angularVel_actual; // rad/s
    float kError = error * KP_SPIN_INNER;                 // volts
    float kErrorBounded = kError;                         // volts

    // bounding block
    if (kErrorBounded > 8)
    {
        kErrorBounded = 8;
    }
    if (kErrorBounded < -8)
    {
        kErrorBounded = -8;
    }

    float dv = kErrorBounded;

    return dv;
}

// takes in Va and deltaV, both in volts
void Movement::driveMotor(float va, float dv)
{
    // convert volts to write values
    va = va * 255.0 / 8.0;
    dv = dv * 255.0 / 8.0;

    leftWriteValue = (va + dv) / 2;
    rightWriteValue = (va - dv) / 2;

    mc.setWriteValues(leftWriteValue, rightWriteValue);
}

float Movement::getForwardVel()
{
    return forwardVel;
}

float Movement::getRotationalVel()
{
    return rotationalVel;
}

float Movement::getDV()
{
    return dv;
}

float Movement::getVA()
{
    return va;
}

float Movement::getRhoTarget()
{
    return rhoTarget;
}

float Movement::getXTarget()
{
    return xTarget;
}

float Movement::getYTarget()
{
    return yTarget;
}

float Movement::getPhiTarget()
{
    return phiTarget;
}

float Movement::calculatePhiError(float phi, float phiDes)
{

    // normalize to range (0, 2PI)
    phi = fmod(phi, 2 * PI);
    phiDes = fmod(phiDes, 2 * PI);

    if (phi < 0)
        phi += 2 * PI;
    if (phiDes < 0)
        phiDes += 2 * PI;

    // test 4 cases for which one is < PI

    // phi - phiDes
    if (abs(phiDes - phi) < PI)
    {
        return phiDes - phi;
    }

    // (phi - 360*) - phiDes
    if (abs((phiDes - 2 * PI) - phi) < PI)
    {
        return (phiDes - 2 * PI) - phi;
    }

    // phi - (phiDes - 360*)
    if (abs(phiDes - (phi - 2 * PI)) < PI)
    {
        return phiDes - (phi - 2 * PI);
    }

    return NAN;
}
