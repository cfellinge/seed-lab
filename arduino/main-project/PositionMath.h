/*
    PositionMath.h
    Code to handle math for SEED Lab robot position
*/

#ifndef PositionMath_h
#define PositionMath_h

#include "Arduino.h"

class PositionMath {
    public:
        PositionMath(double wheelBaseWidth);
        double getX();
        double getY();
        double getPhi();
        double getRho();
        void resetPosition();
        void resetPosition(double x, double y, double phi);
        void updatePosition(double numMilliseconds, double velocityLeft, double velocityRight);
    private:
        double calculateX(double xOld, double deltaT, double phiOld, double velocityLeft, double velocityRight);
        double calculateY(double yOld, double deltaT, double phiOld, double velocityLeft, double velocityRight);
        double calculatePhi(double phiOld, double deltaT, double wheelBaseWidth, double velocityLeft, double velocityRight);
        double _x;
        double _y;
        double _phi;
        double _rho;
};

#endif