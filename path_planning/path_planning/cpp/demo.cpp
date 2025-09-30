#include <iostream>
#include <vector>
#include <cmath>
#include "include/reeds_shepp.hpp"
#include "include/utils.hpp"

using namespace std;

struct Pose {
    double x, y, theta;
};
#include <sstream>

std::string path_to_string(const reeds_shepp::Path& path) {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < path.size(); i++) {
        ss << path[i];
        if (i != path.size() - 1) ss << ", ";
    }
    ss << "]";
    return ss.str();
}

int main() {
    // PATH định nghĩa sẵn
    vector<Pose> PATH = {
        {-5, 5, 90}, {-5, 5, -90}, {1, 4, 180}, {5, 4, 0},
        {6, -3, 90}, {4, -4, -40}, {-2, 0, 240}, {-6, -7, 160}, {-7, -1, 80}
    };

    double path_length = 0.0;

    for (size_t i = 0; i < PATH.size() - 1; i++) {
        auto path = reeds_shepp::get_optimal_path(
            std::make_tuple(PATH[i].x, PATH[i].y, PATH[i].theta),
            std::make_tuple(PATH[i+1].x, PATH[i+1].y, PATH[i+1].theta)
        );
        cout << path_to_string(path) << "\n";
        path_length += reeds_shepp::path_length(path);
    }

    cout << "Shortest path length: " << (int) path_length << " px." << endl;

    return 0;
}
