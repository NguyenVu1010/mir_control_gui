#pragma once

// Cần thiết cho std::vector
#include <vector>
// Cần thiết cho std::tuple
#include <tuple>
// Cần thiết cho std::ostream để overload toán tử <<
#include <iostream>

namespace reeds_shepp {

enum class Steering { LEFT = -1, STRAIGHT = 0, RIGHT = 1 };
enum class Gear { FORWARD = 1, BACKWARD = -1 };

struct PathElement {
    double param;
    Steering steering;
    Gear gear;

    PathElement(double p, Steering s, Gear g);

    PathElement reverse_steering() const;
    PathElement reverse_gear() const;
};

using Path = std::vector<PathElement>;

/**
 * @brief Tính tổng chiều dài của một đường đi.
 */
double path_length(const Path& path);

/**
 * @brief Tìm và trả về đường đi Reeds-Shepp ngắn nhất.
 */
Path get_optimal_path(const std::tuple<double, double, double>& start,
                      const std::tuple<double, double, double>& end);

/**
 * @brief Tạo ra tất cả các đường đi Reeds-Shepp hợp lệ.
 */
std::vector<Path> get_all_paths(const std::tuple<double, double, double>& start,
                                const std::tuple<double, double, double>& end);

/**
 * @brief Overload toán tử << để in thông tin của PathElement.
 */
std::ostream& operator<<(std::ostream& os, const PathElement& e);

} // namespace reeds_shepp