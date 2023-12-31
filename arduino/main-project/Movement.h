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
    Movement(MotorControl &motorController, PositionMath &positionMath, float WHEEL_RADIUS);

    float getXYError();

    float getPhiError();
    
    // rotates the robot, moves to coordinates(x, y)(meters), and rotates the robot to face phi(radians) 
    
    void moveToCoordinates(float x, float y, float phi);

    // move motors forward at a constant speed (meters / second)
    void moveAtSpeed(float leftSpeed, float rightSpeed);

    void rotateAtSpeed(float rotationalSpeed);

    void stop();

    void updateMovement(float numMilliseconds);

    void goInCircle(float x, float y, float r);

    // move the robot straight forward a set distance (meters)
    void moveForwards(float distance);

    // feedback control loops:
    float velOuterIntegralControl(float rho, float rho_desired, float vel_actual, int millisecondInterval);
    float velInnerProportionalControl(float vel_actual, float vel_desired);

    float angularVelOuterIntegralControl(float phi, float phi_desired, float angularVel, int millisecondInterval);
    float angularVelInnerProportionalControl(float angularVel_actual, float angularVel_desired);

    // rotate the robot a set angle (radians)
    void rotateLeft(float angle);

    // step response test for demo 1
    void driveMotor(float va, float dv);

    float getForwardVel();
    float getRotationalVel();

    float getDV();

    float getVA();

    float getRhoTarget();

    float getXTarget();

    float calculatePhiError(float phi, float phiDes);

    float getYTarget();

    float getPhiTarget();

private:


};

#endif