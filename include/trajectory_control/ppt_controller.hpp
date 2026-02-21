#pragma once

#include <vector>
#include "trajectory.hpp"

class PurePursuitController
{
public:
    PurePursuitController(
        double max_velocity,
        double omega_max,
        double k1,
        double k2,
        double k3);

    void setTrajectory(const std::vector<TrajPoint>& traj);

    void computeCommand(
        double x,
        double y,
        double theta,
        double current_time,
        double& v,
        double& omega);

private:
    double max_velocity_;
    double omega_max_;

    // Tracking gains
    double k1_;   // longitudinal
    double k2_;   // lateral
    double k3_;   // heading

    std::vector<TrajPoint> trajectory_;
};