/*
    PositionMath.h
    Code to handle math for SEED Lab robot position
*/

#ifndef PositionMath_h
#define PositionMath_h

#include "Arduino.h"

class PositionMath {
    public:
        PositionMath(float wheelBaseWidth);
        float getX();
        float getY();
        float getPhi();
        float getRho();
        void resetPosition();
        void resetPosition(float x, float y, float phi);
        void updatePosition(float numMilliseconds, float velocityLeft, float velocityRight);
    private:
        float calculateX(float xOld, float deltaT, float phiOld, float velocityLeft, float velocityRight);
        float calculateY(float yOld, float deltaT, float phiOld, float velocityLeft, float velocityRight);
        float calculatePhi(float phiOld, float deltaT, float wheelBaseWidth, float velocityLeft, float velocityRight);
        float _x;
        float _y;
        float _phi;
        float _rho;
};

#endif