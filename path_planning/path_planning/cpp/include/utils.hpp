#pragma once

// Cần thiết cho các hằng số và hàm toán học như M_PI, fmod, sqrt, atan2, sin, cos
#include <cmath>
// Cần thiết cho std::pair
#include <utility>
// Cần thiết cho std::tuple và std::tie
#include <tuple>

namespace utils {

// Hằng số PI, sử dụng M_PI từ cmath
constexpr double PI = M_PI;

/**
 * @brief Chuẩn hóa góc theta về khoảng [-pi, pi).
 */
double M(double theta);

/**
 * @brief Chuyển đổi tọa độ Descartes (x, y) sang tọa độ cực (r, theta).
 */
std::pair<double, double> R(double x, double y);

/**
 * @brief Chuyển đổi tọa độ của p2 sang hệ quy chiếu gốc p1.
 * @param p1 (x1, y1, góc_theo_độ)
 * @param p2 (x2, y2, góc_theo_độ)
 * @return (x_mới, y_mới, góc_mới_theo_độ)
 */
std::tuple<double, double, double> change_of_basis(const std::tuple<double, double, double>& p1,
                                                 const std::tuple<double, double, double>& p2);

/**
 * @brief Chuyển đổi radian sang độ.
 */
constexpr double rad2deg(double rad) {
    return 180.0 * rad / PI;
}

/**
 * @brief Chuyển đổi độ sang radian.
 */
constexpr double deg2rad(double deg) {
    return PI * deg / 180.0;
}

/**
 * @brief Trả về dấu của một số (1, -1, hoặc 0).
 */
template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

} // namespace utils