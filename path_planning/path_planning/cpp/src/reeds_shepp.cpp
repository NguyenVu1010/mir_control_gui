#include "reeds_shepp.hpp"
#include "utils.hpp"
#include <algorithm> //  std::min_element, std::copy_if
#include <functional> //  std::function
#include <vector>

namespace reeds_shepp {

// --- Định nghĩa các phương thức của PathElement ---
PathElement::PathElement(double p, Steering s, Gear g) {
    if (p >= 0) {
        param = p; steering = s; gear = g;
    } else {
        param = -p; steering = s; gear = static_cast<Gear>(-static_cast<int>(g));
    }
}

PathElement PathElement::reverse_steering() const {
    return PathElement(param, static_cast<Steering>(-static_cast<int>(steering)), gear);
}

PathElement PathElement::reverse_gear() const {
    return PathElement(param, steering, static_cast<Gear>(-static_cast<int>(gear)));
}

// --- Các hàm tiện ích ---
double path_length(const Path& path) {
    double length = 0.0;
    for (const auto& e : path) { length += e.param; }
    return length;
}

std::ostream& operator<<(std::ostream& os, const PathElement& e) {
    auto steering_str = e.steering == Steering::LEFT ? "LEFT" : (e.steering == Steering::RIGHT ? "RIGHT" : "STRAIGHT");
    auto gear_str = e.gear == Gear::FORWARD ? "FORWARD" : "BACKWARD";
    os << "{Steering: " << steering_str << ", Gear: " << gear_str << ", dist: " << e.param << "}";
    return os;
}

// --- Các hàm biến đổi đường đi ---
Path timeflip(Path path) {
    for (auto& e : path) { e = e.reverse_gear(); }
    return path;
}

Path reflect(Path path) {
    for (auto& e : path) { e = e.reverse_steering(); }
    return path;
}

// --- Định nghĩa đầy đủ 12 hàm tính toán đường đi cơ bản ---
namespace { // Sử dụng anonymous namespace để các hàm này là private cho file

Path path1(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [u, t] = utils::R(x - sin(phi), y - 1 + cos(phi));
    double v = utils::M(phi - t);
    return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::STRAIGHT, Gear::FORWARD}, {v, Steering::LEFT, Gear::FORWARD}};
}

Path path2(double x, double y, double phi_deg) {
    double phi = utils::M(utils::deg2rad(phi_deg));
    auto [rho, t1] = utils::R(x + sin(phi), y - 1 - cos(phi));
    if (rho * rho >= 4) {
        double u = sqrt(rho * rho - 4);
        double t = utils::M(t1 + atan2(2, u));
        double v = utils::M(t - phi);
        return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::STRAIGHT, Gear::FORWARD}, {v, Steering::RIGHT, Gear::FORWARD}};
    }
    return {};
}

Path path3(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x - sin(phi), y - 1 + cos(phi));
    if (rho <= 4) {
        double A = acos(rho / 4.0);
        double t = utils::M(theta + utils::PI / 2.0 + A);
        double u = utils::M(utils::PI - 2.0 * A);
        double v = utils::M(phi - t - u);
        return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::RIGHT, Gear::BACKWARD}, {v, Steering::LEFT, Gear::FORWARD}};
    }
    return {};
}

Path path4(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x - sin(phi), y - 1 + cos(phi));
    if (rho <= 4) {
        double A = acos(rho / 4.0);
        double t = utils::M(theta + utils::PI / 2.0 + A);
        double u = utils::M(utils::PI - 2.0 * A);
        double v = utils::M(t + u - phi);
        return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::RIGHT, Gear::BACKWARD}, {v, Steering::LEFT, Gear::BACKWARD}};
    }
    return {};
}

Path path5(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x - sin(phi), y - 1 + cos(phi));
    if (rho <= 4) {
        double u = acos(1 - rho * rho / 8.0);
        double A = asin(2 * sin(u) / rho);
        double t = utils::M(theta + utils::PI / 2.0 - A);
        double v = utils::M(t - u - phi);
        return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::RIGHT, Gear::FORWARD}, {v, Steering::LEFT, Gear::BACKWARD}};
    }
    return {};
}

Path path6(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x + sin(phi), y - 1 - cos(phi));
    double t, u, v;
    if (rho <= 4) {
        if (rho <= 2) {
            double A = acos((rho + 2) / 4);
            t = utils::M(theta + utils::PI/2 + A);
            u = A;
            v = utils::M(phi - t + 2*u);
        } else {
            double A = acos((rho - 2) / 4);
            t = utils::M(theta + utils::PI/2 - A);
            u = utils::PI - A;
            v = utils::M(phi - t + 2*u);
        }
        return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::RIGHT, Gear::FORWARD}, {u, Steering::LEFT, Gear::BACKWARD}, {v, Steering::RIGHT, Gear::BACKWARD}};
    }
    return {};
}

Path path7(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x + sin(phi), y - 1 - cos(phi));
    double u1 = (20 - rho*rho) / 16;
    if (rho <= 6 && 0 <= u1 && u1 <= 1) {
        double u = acos(u1);
        double A = asin(2 * sin(u) / rho);
        double t = utils::M(theta + utils::PI/2 + A);
        double v = utils::M(t - phi);
        return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::RIGHT, Gear::BACKWARD}, {u, Steering::LEFT, Gear::BACKWARD}, {v, Steering::RIGHT, Gear::FORWARD}};
    }
    return {};
}

Path path8(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x - sin(phi), y - 1 + cos(phi));
    if (rho >= 2) {
        double u = sqrt(rho*rho - 4) - 2;
        double A = atan2(2, u+2);
        double t = utils::M(theta + utils::PI/2 + A);
        double v = utils::M(t - phi + utils::PI/2);
        return {{t, Steering::LEFT, Gear::FORWARD}, {utils::PI/2, Steering::RIGHT, Gear::BACKWARD}, {u, Steering::STRAIGHT, Gear::BACKWARD}, {v, Steering::LEFT, Gear::BACKWARD}};
    }
    return {};
}

Path path9(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x - sin(phi), y - 1 + cos(phi));
    if (rho >= 2) {
        double u = sqrt(rho*rho - 4) - 2;
        double A = atan2(u+2, 2);
        double t = utils::M(theta + utils::PI/2 - A);
        double v = utils::M(t - phi - utils::PI/2);
        return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::STRAIGHT, Gear::FORWARD}, {utils::PI/2, Steering::RIGHT, Gear::FORWARD}, {v, Steering::LEFT, Gear::BACKWARD}};
    }
    return {};
}

Path path10(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x + sin(phi), y - 1 - cos(phi));
    if (rho >= 2) {
        double t = utils::M(theta + utils::PI/2);
        double u = rho - 2;
        double v = utils::M(phi - t - utils::PI/2);
        return {{t, Steering::LEFT, Gear::FORWARD}, {utils::PI/2, Steering::RIGHT, Gear::BACKWARD}, {u, Steering::STRAIGHT, Gear::BACKWARD}, {v, Steering::RIGHT, Gear::BACKWARD}};
    }
    return {};
}

Path path11(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x + sin(phi), y - 1 - cos(phi));
    if (rho >= 2) {
        double t = utils::M(theta);
        double u = rho - 2;
        double v = utils::M(phi - t - utils::PI/2);
        return {{t, Steering::LEFT, Gear::FORWARD}, {u, Steering::STRAIGHT, Gear::FORWARD}, {utils::PI/2, Steering::LEFT, Gear::FORWARD}, {v, Steering::RIGHT, Gear::BACKWARD}};
    }
    return {};
}

Path path12(double x, double y, double phi_deg) {
    double phi = utils::deg2rad(phi_deg);
    auto [rho, theta] = utils::R(x + sin(phi), y - 1 - cos(phi));
    if (rho >= 4) {
        double u = sqrt(rho*rho - 4) - 4;
        double A = atan2(2, u+4);
        double t = utils::M(theta + utils::PI/2 + A);
        double v = utils::M(t - phi);
        return {{t, Steering::LEFT, Gear::FORWARD}, {utils::PI/2, Steering::RIGHT, Gear::BACKWARD}, {u, Steering::STRAIGHT, Gear::BACKWARD}, {utils::PI/2, Steering::LEFT, Gear::BACKWARD}, {v, Steering::RIGHT, Gear::FORWARD}};
    }
    return {};
}

} // end anonymous namespace

// --- Các hàm API chính ---
std::vector<Path> get_all_paths(const std::tuple<double, double, double>& start,
                                const std::tuple<double, double, double>& end) {
    auto [x, y, theta] = utils::change_of_basis(start, end);

    using PathFunction = std::function<Path(double, double, double)>;
    const std::vector<PathFunction> path_fns = {
        path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12
    };

    std::vector<Path> all_paths;
    for (const auto& get_path : path_fns) {
        for (const auto& p : {
            get_path(x, y, theta),
            timeflip(get_path(-x, y, -theta)),
            reflect(get_path(x, -y, -theta)),
            reflect(timeflip(get_path(-x, -y, theta)))
        }) {
            if (!p.empty()) {
                Path filtered_path;
                std::copy_if(p.begin(), p.end(), std::back_inserter(filtered_path), 
                    [](const PathElement& e){ return e.param > 1e-6; }); // Bỏ qua các đoạn rất nhỏ
                if (!filtered_path.empty()) {
                    all_paths.push_back(filtered_path);
                }
            }
        }
    }
    return all_paths;
}

Path get_optimal_path(const std::tuple<double, double, double>& start,
                      const std::tuple<double, double, double>& end) {
    auto all_paths = get_all_paths(start, end);
    if (all_paths.empty()) {
        std::cerr << "Warning: No valid Reeds-Shepp path found." << std::endl;
        return {}; 
    }

    auto shortest_it = std::min_element(all_paths.begin(), all_paths.end(),
        [](const Path& a, const Path& b) {
            return path_length(a) < path_length(b);
        });

    return *shortest_it;
}

} // namespace reeds_shepp