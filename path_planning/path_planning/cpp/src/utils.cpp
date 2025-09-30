#include "utils.hpp"

namespace utils {

double M(double theta) {
    theta = fmod(theta, 2 * PI);
    if (theta < -PI) {
        theta += 2 * PI;
    } else if (theta >= PI) {
        theta -= 2 * PI;
    }
    return theta;
}

std::pair<double, double> R(double x, double y) {
    double r = std::sqrt(x * x + y * y);
    double theta = std::atan2(y, x);
    return {r, theta};
}

std::tuple<double, double, double> change_of_basis(const std::tuple<double, double, double>& p1,
                                                 const std::tuple<double, double, double>& p2) {
    double x1, y1, theta1_deg;
    std::tie(x1, y1, theta1_deg) = p1;

    double x2, y2, theta2_deg;
    std::tie(x2, y2, theta2_deg) = p2;

    double theta1_rad = deg2rad(theta1_deg);
    double dx = x2 - x1;
    double dy = y2 - y1;

    double new_x = dx * std::cos(theta1_rad) + dy * std::sin(theta1_rad);
    double new_y = -dx * std::sin(theta1_rad) + dy * std::cos(theta1_rad);
    double new_theta = theta2_deg - theta1_deg;

    return {new_x, new_y, new_theta};
}

} // namespace utils