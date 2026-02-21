#include "trajectory_control/ppt_controller.hpp"
#include <cmath>

PurePursuitController::PurePursuitController(
    double max_velocity,
    double omega_max,
    double k1,
    double k2,
    double k3)
: max_velocity_(max_velocity),
  omega_max_(omega_max),
  k1_(k1),
  k2_(k2),
  k3_(k3)
{}

void PurePursuitController::setTrajectory(
    const std::vector<TrajPoint>& traj)
{
    trajectory_ = traj;
}

void PurePursuitController::computeCommand(
    double x,
    double y,
    double theta,
    double current_time,
    double& v,
    double& omega)
{
    if (trajectory_.empty())
    {
        v = 0.0;
        omega = 0.0;
        return;
    }

    // ---------------------------------------
    // Find desired trajectory point by time
    // ---------------------------------------
    size_t target_index = trajectory_.size() - 1;

    for (size_t i = 0; i < trajectory_.size(); ++i)
    {
        if (trajectory_[i].t >= current_time)
        {
            target_index = i;
            break;
        }
    }

    double x_d = trajectory_[target_index].x;
    double y_d = trajectory_[target_index].y;
    double theta_d = trajectory_[target_index].theta;

    // ---------------------------------------
    // Compute tracking errors in robot frame
    // ---------------------------------------
    double dx = x_d - x;
    double dy = y_d - y;

    double x_r = std::cos(theta) * dx +
                 std::sin(theta) * dy;

    double y_r = -std::sin(theta) * dx +
                  std::cos(theta) * dy;

    double theta_error =
        std::atan2(
            std::sin(theta_d - theta),
            std::cos(theta_d - theta));

    // ---------------------------------------
    // Control Law (Time-Parameterized Tracking)
    // ---------------------------------------
    double v_ref = max_velocity_;  // or compute from trajectory
    v = v_ref + k1_ * x_r;
    omega = k2_ * y_r + k3_ * theta_error;

    // ---------------------------------------
    // Velocity Saturation
    // ---------------------------------------
    if (v > max_velocity_)
        v = max_velocity_;
    if (v < -max_velocity_)
        v = -max_velocity_;

    if (omega > omega_max_)
        omega = omega_max_;
    if (omega < -omega_max_)
        omega = -omega_max_;
}