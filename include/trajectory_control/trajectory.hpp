#pragma once
#include <vector>
#include "spline.hpp"

struct TrajPoint
{
    double x;
    double y;
    double theta;
    double t;
};

class TrajectoryGenerator
{
public:
    std::vector<TrajPoint> generate(
        Spline2D& spline,
        double velocity,
        double ds);
};
