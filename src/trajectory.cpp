#include "trajectory_control/trajectory.hpp"

std::vector<TrajPoint> TrajectoryGenerator::generate(
    Spline2D& spline,
    double velocity,
    double ds)
{
    std::vector<TrajPoint> traj;

    double length = spline.get_length();

    for (double s = 0.0; s <= length; s += ds)
    {
        double x = spline.calc_x(s);
        double y = spline.calc_y(s);

        double dx = spline.calc_dx(s);
        double dy = spline.calc_dy(s);

        double ddx = spline.calc_ddx(s);
        double ddy = spline.calc_ddy(s);

        double theta = std::atan2(dy, dx);

        double denom = std::pow(dx*dx + dy*dy, 1.5);
        double curvature = 0.0;

        if (denom > 1e-6)
            curvature = (dx * ddy - dy * ddx) / denom;

        double v = velocity;
        double omega = v * curvature;

        double t = s / velocity;

        traj.push_back({x, y, theta, t, v, omega});
    }

    return traj;
}
