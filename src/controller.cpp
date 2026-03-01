#include "trajectory_control/ppt_controller.hpp"
#include <cmath>

PurePursuitController::PurePursuitController(
    double lookahead,
    double max_velocity,
    double curvature_gain,
    double k_theta,
    double omega_max)
: lookahead_(lookahead),
  max_velocity_(max_velocity),
  curvature_gain_(curvature_gain),
  k_theta_(k_theta),
  omega_max_(omega_max)
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
    double& v,
    double& omega)
{
    if (trajectory_.empty())
    {
        v = 0.0;
        omega = 0.0;
        return;
    }

    // Find closest point
    double min_dist = 1e9;
    size_t closest_index = 0;

    for (size_t i = 0; i < trajectory_.size(); ++i)
    {
        double dx = trajectory_[i].x - x;
        double dy = trajectory_[i].y - y;
        double dist = std::hypot(dx, dy);

        if (dist < min_dist)
        {
            min_dist = dist;
            closest_index = i;
        }
    }

    // Find lookahead point
    size_t target_index = closest_index;
    for (size_t i = closest_index; i < trajectory_.size(); ++i)
    {
        double dx = trajectory_[i].x - x;
        double dy = trajectory_[i].y - y;
        double dist = std::hypot(dx, dy);

        if (dist >= lookahead_)
        {
            target_index = i;
            break;
        }
    }

    double target_x = trajectory_[target_index].x;
    double target_y = trajectory_[target_index].y;

    // Transform to robot frame
    double dx = target_x - x;
    double dy = target_y - y;

    double x_r = std::cos(theta) * dx + std::sin(theta) * dy;
    double y_r = -std::sin(theta) * dx + std::cos(theta) * dy;

    double L = std::hypot(x_r, y_r);

    if (L < 1e-6)
    {
        v = 0.0;
        omega = 0.0;
        return;
    }

    double curvature = 2.0 * y_r / (L * L);

    // ----------------------------
    // Curvature-based velocity scaling
    // ----------------------------
    double v_cmd =
        max_velocity_ /
        (1.0 + curvature_gain_ * std::abs(curvature));

    // Optional minimum velocity
    double v_min = 0.05;
    if (v_cmd < v_min)
        v_cmd = v_min;

    // ----------------------------
    // Heading error damping
    // ----------------------------
    double path_heading =
        std::atan2(dy, dx);

    double heading_error =
        path_heading - theta;

    // Normalize angle to [-pi, pi]
    while (heading_error > M_PI)
        heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI)
        heading_error += 2.0 * M_PI;

    // ----------------------------
    // Angular command
    // ----------------------------
    double omega_cmd =
        v_cmd * curvature +
        k_theta_ * heading_error;

    // ----------------------------
    // Saturate angular velocity
    // ----------------------------
    if (omega_cmd > omega_max_)
        omega_cmd = omega_max_;
    if (omega_cmd < -omega_max_)
        omega_cmd = -omega_max_;

    v = v_cmd;
    omega = omega_cmd;

}
