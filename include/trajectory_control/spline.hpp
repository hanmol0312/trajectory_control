#pragma once
#include <vector>

class CubicSpline1D
{
public:
    void set_points(const std::vector<double>& x,
                    const std::vector<double>& y);

    double calc(double t) const;
    double calc_derivative(double t) const;
    double calc_second_derivative(double t) const;

private:
    int find_segment(double t) const;

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> a_, b_, c_, d_;
};

class Spline2D
{
public:
    Spline2D(const std::vector<double>& x,
             const std::vector<double>& y);

    double calc_x(double s);
    double calc_y(double s);

    double calc_dx(double s);
    double calc_dy(double s);

    double calc_ddx(double s);
    double calc_ddy(double s);

    double get_length() const;

private:
    std::vector<double> s_;
    double total_length_;

    CubicSpline1D sx_;
    CubicSpline1D sy_;
};
