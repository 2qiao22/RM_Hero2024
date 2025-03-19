#pragma once
#include <cmath>

class Mecanum2
{
public:
    Mecanum2(float wheelWidthDistance, float wheelVerticalDistance);
    void calculate(float velocityVector[3], float _out[4], float currentAngle = 0);

private:
    float inverseKinematicsMatrix[4][3]{{-1.0, 1.0,  1.0},
                                        { 1.0, 1.0, -1.0},
                                        {-1.0, 1.0, -1.0},
                                        { 1.0, 1.0,  1.0}};
};
