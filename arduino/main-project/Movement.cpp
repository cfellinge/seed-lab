/*
    Movement.cpp
    Code to handle robot movement
*/

#include "Arduino.h"
#include "Movement.h"

// MotorControl mc(0);
// PositionMath pos;

double forwardVel;
double rotationalVel;

double forwardVelTarget;
double rotationalVelTarget;

double xTarget;
double yTarget;
double rhoTarget;
double phiTarget;

double va;
double dv;

double WHEEL_RADIUS = 0.0725;

double leftWriteValue;
double rightWriteValue;

enum MODE
{
    GO_TO_COORDINATES,
    ROTATE_TO_ANGLE,
    STAY_STILL,
    NO_FEEDBACK_CONTROLS
};

MODE mode;

// VA
double KP_VEL_INNER = 6;
double KP_VEL_OUTER = 4;
double KI_VEL_OUTER = 0.04;


// DV - straight line
// double KP_SPIN_INNER = 0.5;
// double KP_SPIN_OUTER = 10;
// double KI_SPIN_OUTER = 0.2;

// DV - spin in circle
double KP_SPIN_INNER = 5;
double KP_SPIN_OUTER = 5;
double KI_SPIN_OUTER = 0.2;

double velIntegralError = 0;
double angularVelIntegralError = 0;

// max speed of robot, m/s
double ROBOT_MAX_SPEED = 0.7;

// max rotational velocity of robot, rad/s
double ROBOT_MAX_SPIN = 0.8;

Movement::Movement(MotorControl &motorController, PositionMath &positionMath) : mc(motorController), pos(positionMath)
{
}

// move robot to coordinates (x, y) in meters
// phi currently does not do anything
void Movement::moveToCoordinates(double x, double y, double phi)
{
    mode = GO_TO_COORDINATES;
    xTarget = x;
    yTarget = y;
    phiTarget = phi;
}

// move robot straight forward a set distance (meters)
void Movement::moveForwards(double distance)
{
    mode = GO_TO_COORDINATES;
    xTarget = distance * cos(pos.getPhi());
    yTarget = distance * sin(pos.getPhi());
    phiTarget = pos.getPhi();
}

// rotate robot to face a set angle (radians)
void Movement::rotateLeft(double angle)
{
    mode = ROTATE_TO_ANGLE;
    phiTarget = angle;
    Serial.println("PHI TARGET IS " + (String)phiTarget);
}

// doesn't work
void Movement::moveAtSpeed(double leftSpeed, double rightSpeed)
{
    mode = NO_FEEDBACK_CONTROLS;
    forwardVelTarget = leftSpeed;
}

void Movement::stop() {
    mode = STAY_STILL;
}

void Movement::updateMovement(double numMilliseconds)
{
    double leftVel = mc.getLeftVelocity();
    double rightVel = mc.getRightVelocity();

    forwardVel = WHEEL_RADIUS * (leftVel + rightVel) / 2;
    rotationalVel = WHEEL_RADIUS * (leftVel - rightVel) / 2;

    double rhoActual = pos.getRho();
    double phiActual = pos.getPhi();

    switch (mode)
    {
    case GO_TO_COORDINATES:
        rhoTarget = sqrt(pow(xTarget, 2) + pow(yTarget, 2));
        
        phiActual = fmod(phiActual, 2*PI);
        phiTarget = atan2(yTarget - pos.getY(), xTarget - pos.getX());

        // TODO: phiTarget is always (-pi/2, pi/2)
        // phi can be much greater
        // robot should be able to turn in circles

        va = velOuterIntegralControl(rhoActual, rhoTarget, forwardVel, numMilliseconds);
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

    case NO_FEEDBACK_CONTROLS:
        break;
    }

    // dv = 0;
    // Serial.println("VA: " + (String)va + " DV: " + (String)dv);
    driveMotor(va, dv);
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

double Movement::angularVelOuterIntegralControl(double phi, double phi_desired, double angularVel, int millisecondInterval)
{
    // PI block
    double angularVelPosError = phi_desired - phi; // radians

    angularVelIntegralError = angularVelIntegralError + angularVelPosError * (double)millisecondInterval / 1000.0; // radians * seconds

    double desiredSpeed = KP_SPIN_OUTER * angularVelPosError + KI_SPIN_OUTER * angularVelIntegralError;

    double angularVelError = desiredSpeed - angularVel;

    double angularVel_desired = KP_SPIN_OUTER * angularVelError;

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
    double dv = angularVelInnerProportionalControl(angularVel, angularVel_desired);

    // voltage to apply to both motors
    return dv;
}

double Movement::angularVelInnerProportionalControl(double angularVel_actual, double angularVel_desired)
{
    // P block
    double error = angularVel_desired - angularVel_actual; // rad/s
    double kError = error * KP_SPIN_INNER;                 // volts
    double kErrorBounded = kError;                         // volts

    // bounding block
    if (kErrorBounded > 8)
    {
        kErrorBounded = 8;
    }
    if (kErrorBounded < -8)
    {
        kErrorBounded = -8;
    }

    double dv = kErrorBounded;

    return dv;
}

// takes in Va and deltaV, both in volts
void Movement::driveMotor(double va, double dv)
{
    // convert volts to write values
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

double Movement::getDV()
{
    return dv;
}

double Movement::getVA()
{
    return va;
}

double Movement::getRhoTarget()
{
    return rhoTarget;
}

double Movement::getXTarget()
{
    return xTarget;
}

double Movement::getYTarget()
{
    return yTarget;
}


double Movement::getPhiTarget()
{
    return phiTarget;
}