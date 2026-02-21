    #include "trajectory_control/trajectory.hpp"
    #include <cmath>

    std::vector<TrajPoint> TrajectoryGenerator::generate(
        Spline2D& spline,
        double velocity,
        double ds)
    {
        std::vector<TrajPoint> traj;

        double length = spline.get_length();

        for (double s = 0.0; s <= length; s += ds)
        {
            double x = spline.calc_x(s);        // gives x(s)
            double y = spline.calc_y(s);        // gives y(s)

            double dx = spline.calc_dx(s);      // detivative of x(s)
            double dy = spline.calc_dy(s);      //derivative of y(s)

            double theta = std::atan2(dy, dx);
            double t = s / velocity;

            traj.push_back({x, y, theta, t});  // trajectory profile
        }

        return traj;
    }
