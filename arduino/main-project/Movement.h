/*
    Movement.h
    Code to handle robot movement
*/

#ifndef Movement_h
#define Movement_h

#include "Arduino.h"
#include "MotorControl.h"
#include "PositionMath.h"

class Movement
{
    MotorControl& mc;
    PositionMath& pos;

public:
    Movement(MotorControl &motorController, PositionMath &positionMath);

    // rotates the robot, moves to coordinates (x, y) (meters), and rotates the robot to face phi (radians)
    void moveToCoordinates(double x, double y, double phi);

    // move motors forward at a constant speed (meters / second)
    void moveAtSpeed(double leftSpeed, double rightSpeed);

    void updateMovement(double numMilliseconds);

    // move the robot straight forward a set distance (meters)
    void moveForwards(double distance);

    // set each wheel to a certain number of radians
    void setRotations(double leftPosition, double rightPosition);

    // feedback control loops:
    double velOuterIntegralControl(double rho, double rho_desired, double vel_actual, int millisecondInterval);
    double velInnerProportionalControl(double vel_actual, double vel_desired);

    double angularVelOuterIntegralControl(double phi, double phi_desired, double angularVel, int millisecondInterval);
    double angularVelInnerProportionalControl(double angularVel_actual, double angularVel_desired);

    // rotate the robot a set angle (radians)
    void rotateLeft(double angle, int millisecondInterval);

    // step response test for demo 1
    void driveMotor(double va, double dv);

    double getForwardVel();
    double getRotationalVel();

    double getDV();

    double getVA();

private:


};

#endif