#include "trajectory_control/spline.hpp"
#include <cmath>
#include <stdexcept>

void CubicSpline1D::set_points(const std::vector<double>& x,
                               const std::vector<double>& y)
{
    if (x.size() != y.size() || x.size() < 2)
        throw std::runtime_error("Invalid spline input");

    x_ = x;
    y_ = y;

    int n = x.size();

    a_ = y;
    b_.resize(n - 1);
    c_.resize(n);
    d_.resize(n - 1);

    std::vector<double> h(n - 1);
    // Defines each spline interval
    for (int i = 0; i < n - 1; ++i)
        h[i] = x[i + 1] - x[i];

    // Compute Alpha curvature constraint
    std::vector<double> alpha(n - 1);
    for (int i = 1; i < n - 1; ++i)
        alpha[i] = 3.0 * (a_[i + 1] - a_[i]) / h[i]
                 - 3.0 * (a_[i] - a_[i - 1]) / h[i - 1];

    std::vector<double> l(n), mu(n), z(n);

    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    // Decide intermediate values
    for (int i = 1; i < n - 1; ++i)
    {
        l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c_[n - 1] = 0.0;
    

    for (int j = n - 2; j >= 0; --j)
    {
        c_[j] = z[j] - mu[j] * c_[j + 1];
        b_[j] = (a_[j + 1] - a_[j]) / h[j]
              - h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
        d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
    }
}


// Returns the segment in the spline
int CubicSpline1D::find_segment(double t) const
{
    for (size_t i = 0; i < x_.size() - 1; ++i)
        if (t >= x_[i] && t <= x_[i + 1])
            return i;
    return x_.size() - 2;
}

// Gives the curvatre x(s) or y(s)
double CubicSpline1D::calc(double t) const
{
    int i = find_segment(t);
    double dx = t - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
}

// fices derivative that helps in computing tangent heading along the curve.
double CubicSpline1D::calc_derivative(double t) const
{
    int i = find_segment(t);
    double dx = t - x_[i];
    return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
}   

double CubicSpline1D::calc_second_derivative(double t) const
{
    int i = find_segment(t);
    double dx = t - x_[i];
    return 2.0 * c_[i] + 6.0 * d_[i] * dx;
}

// ------------------- Spline2D ---------------------

Spline2D::Spline2D(const std::vector<double>& x,
                   const std::vector<double>& y)
{
    if (x.size() != y.size() || x.size() < 2)
        throw std::runtime_error("Invalid input points");

    int n = x.size();
    s_.resize(n);
    s_[0] = 0.0;

    for (int i = 1; i < n; ++i)
    {
        double dx = x[i] - x[i - 1];
        double dy = y[i] - y[i - 1];
        s_[i] = s_[i - 1] + std::hypot(dx, dy);
    }

    total_length_ = s_.back();

    sx_.set_points(s_, x);             // generates 1 D splines in x 
    sy_.set_points(s_, y);             // generates 1 D splines in y
}
double Spline2D::calc_ddx(double s)
{
    return sx_.calc_second_derivative(s);
}

double Spline2D::calc_ddy(double s)
{
    return sy_.calc_second_derivative(s);
}

double Spline2D::calc_x(double s) { return sx_.calc(s); }        
double Spline2D::calc_y(double s) { return sy_.calc(s); }
double Spline2D::calc_dx(double s) { return sx_.calc_derivative(s); }
double Spline2D::calc_dy(double s) { return sy_.calc_derivative(s); }
double Spline2D::get_length() const { return total_length_; }
