#pragma once
#include <vector>
#include "trajectory.hpp"

class PurePursuitController
{
public:
PurePursuitController(
    double lookahead,
    double max_velocity,
    double curvature_gain,
    double k_theta,
    double omega_max);

    void setTrajectory(const std::vector<TrajPoint>& traj);

    void computeCommand(
        double x,
        double y,
        double theta,
        double& v,
        double& omega);

private:
    double lookahead_;
    double velocity_;
    double max_velocity_;
    double curvature_gain_;
    double k_theta_;
    double omega_max_;

    std::vector<TrajPoint> trajectory_;
};
