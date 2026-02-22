#pragma once
#include <vector>
#include "spline.hpp"
#include <bits/stdc++.h>
struct TrajPoint
{
    double x;
    double y;
    double theta;
    double t;

    double v;       // desired linear velocity
    double omega;   // desired angular velocity
};

class TrajectoryGenerator
{
public:
    std::vector<TrajPoint> generate(
        Spline2D& spline,
        double velocity,
        double ds);
};
