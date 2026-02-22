#include "trajectory_control/ppt_controller.hpp"
#include <cmath>
#include <algorithm>

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
    // double v_ref = max_velocity_;  // or compute from trajectory
    // v = v_ref + k1_ * x_r;
    // omega = k2_ * y_r + k3_ * theta_error;
    // Desired feedforward
    double v_d = trajectory_[target_index].v;
    double omega_d = trajectory_[target_index].omega;

    // Optional curvature-based slowdown
    double curvature = (fabs(v_d) > 1e-6) ? omega_d / v_d : 0.0;
    double beta = 2.0;  // tune this
    v_d = v_d / (1.0 + beta * fabs(curvature));
    double kappa = curvature;

    double v_max = 1.0;          // straight speed
    double kappa_threshold = 0.5;

    double v_desired;

    if (std::abs(kappa) < kappa_threshold)
        v_desired = v_max;
    else
        v_desired = v_max / (1.0 + 3.0 * std::abs(kappa));

    omega_d = v_desired * kappa;
    // Nonlinear tracking law
    v = v_d * std::cos(theta_error) + k1_ * x_r;
    omega = omega_d + k2_ * v_d * y_r + k3_ * theta_error;
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
        double dt = 0.02;  // use real dt if possible

    v = std::clamp(v,
                v_prev_ - max_acc_ * dt,
                v_prev_ + max_acc_ * dt);

    omega = std::clamp(omega,
                    omega_prev_ - max_alpha_ * dt,
                    omega_prev_ + max_alpha_ * dt);

    v_prev_ = v;
    omega_prev_ = omega;
}