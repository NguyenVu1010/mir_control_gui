#include "theta_star_planner.hpp"

// ---- Các thư viện chuẩn C++ cần thiết cho thuật toán ----
#include <queue>          // Dùng cho std::priority_queue (open list)
#include <unordered_map>  // Dùng cho các map lưu trữ g_score, f_score, came_from
#include <cmath>          // Dùng cho sqrt, abs
#include <algorithm>      // Dùng cho std::reverse
#include <functional>     // Dùng cho std::greater

// Alias để code dễ đọc hơn
using Node = std::pair<int, int>;
using CostNodePair = std::pair<double, Node>;

// --- Cấu trúc Hash cho std::pair, cần thiết để dùng Node làm key trong unordered_map ---
struct PairHash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        // Một cách kết hợp hash đơn giản
        return h1 ^ (h2 << 1);
    }
};

// ---- Triển khai các phương thức của lớp ----

ThetaStarPathPlanner::ThetaStarPathPlanner(ros::NodeHandle& nh, double robot_width, double robot_height)
    : nh_(nh), map_width_(0), map_height_(0), map_resolution_(0.0) {
    
    // Lưu kích thước robot (sẽ được chuyển thành pixel sau khi có bản đồ)
    // Chia 2 vì ta sẽ mở rộng ra từ tâm vật cản
    double effective_width = robot_width / 2.0;
    double effective_height = robot_height / 2.0;
    
    // Đăng ký subscriber để nhận bản đồ
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
        "/map", 1, &ThetaStarPathPlanner::map_callback, this
    );

    ROS_INFO("ThetaStarPathPlanner initialized. Waiting for map data on /map topic...");
}

bool ThetaStarPathPlanner::is_map_ready() const {
    return map_ready_;
}

void ThetaStarPathPlanner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (map_ready_) return; // Chỉ xử lý bản đồ lần đầu tiên

    map_width_ = msg->info.width;
    map_height_ = msg->info.height;
    map_resolution_ = msg->info.resolution;
    map_origin_ = {msg->info.origin.position.x, msg->info.origin.position.y};
    map_data_ = msg->data;

    // Tính kích thước robot theo pixel
    // Sử dụng max để đảm bảo an toàn va chạm theo hướng tệ nhất
    double robot_size_m = std::max(nh_.param("robot_width", 0.3), nh_.param("robot_height", 0.5));
    robot_width_px_ = static_cast<int>(robot_size_m / map_resolution_);
    robot_height_px_ = static_cast<int>(robot_size_m / map_resolution_);

    generate_valid_map();

    map_ready_ = true;
    ROS_INFO("Map data received and processed. Planner is ready.");
}

void ThetaStarPathPlanner::generate_valid_map() {
    valid_map_.assign(map_width_ * map_height_, false); // false = hợp lệ

    for (unsigned int y = 0; y < map_height_; ++y) {
        for (unsigned int x = 0; x < map_width_; ++x) {
            // Nếu ô hiện tại là vật cản (giá trị > 50)
            if (map_data_[y * map_width_ + x] > 50) {
                // Đánh dấu một vùng xung quanh nó là không hợp lệ
                for (int dy = -robot_height_px_; dy <= robot_height_px_; ++dy) {
                    for (int dx = -robot_width_px_; dx <= robot_width_px_; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                            valid_map_[ny * map_width_ + nx] = true; // true = không hợp lệ
                        }
                    }
                }
            }
        }
    }
}

std::pair<int, int> ThetaStarPathPlanner::world_to_map(double world_x, double world_y) const {
    int map_x = static_cast<int>((world_x - map_origin_.first) / map_resolution_);
    int map_y = static_cast<int>((world_y - map_origin_.second) / map_resolution_);
    return {map_x, map_y};
}

std::pair<double, double> ThetaStarPathPlanner::map_to_world(int map_x, int map_y) const {
    double world_x = map_x * map_resolution_ + map_origin_.first + map_resolution_ / 2.0;
    double world_y = map_y * map_resolution_ + map_origin_.second + map_resolution_ / 2.0;
    return {world_x, world_y};
}

bool ThetaStarPathPlanner::is_valid(int map_x, int map_y) const {
    if (map_x < 0 || map_x >= map_width_ || map_y < 0 || map_y >= map_height_) {
        return false;
    }
    return !valid_map_[map_y * map_width_ + map_x];
}

bool ThetaStarPathPlanner::line_of_sight(const Node& p1, const Node& p2) const {
    int x0 = p1.first, y0 = p1.second;
    int x1 = p2.first, y1 = p2.second;
    int dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (!is_valid(x0, y0)) return false;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    return true;
}

std::vector<std::pair<double, double>> ThetaStarPathPlanner::find_path(
    const std::pair<double, double>& start, const std::pair<double, double>& goal) {
    
    if (!map_ready_) {
        ROS_ERROR("Cannot find path. Map is not ready yet.");
        return {};
    }

    Node start_node = world_to_map(start.first, start.second);
    Node goal_node = world_to_map(goal.first, goal.second);

    if (!is_valid(start_node.first, start_node.second) || !is_valid(goal_node.first, goal_node.second)) {
        ROS_ERROR("Start or goal is inside an obstacle or out of bounds.");
        return {};
    }

    auto heuristic = [](const Node& a, const Node& b) {
        return std::sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
    };

    std::priority_queue<CostNodePair, std::vector<CostNodePair>, std::greater<CostNodePair>> open_list;
    std::unordered_map<Node, Node, PairHash> came_from;
    std::unordered_map<Node, double, PairHash> g_score;

    g_score[start_node] = 0;
    open_list.push({heuristic(start_node, goal_node), start_node});
    came_from[start_node] = start_node; // Dùng chính nó làm cha ban đầu

    while (!open_list.empty()) {
        Node current = open_list.top().second;
        open_list.pop();

        if (current == goal_node) {
            // Tái tạo đường đi
            std::vector<std::pair<double, double>> path;
            while (current != start_node) {
                path.push_back(map_to_world(current.first, current.second));
                current = came_from[current];
            }
            path.push_back(map_to_world(start_node.first, start_node.second));
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Hàng xóm 8 hướng
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                Node neighbor = {current.first + dx, current.second + dy};

                if (!is_valid(neighbor.first, neighbor.second)) continue;

                Node parent = came_from[current];
                double new_g_score;

                // ---- Logic cốt lõi của Theta* ----
                if (line_of_sight(parent, neighbor)) {
                    // Nếu có đường đi thẳng từ cha của current đến neighbor
                    new_g_score = g_score[parent] + heuristic(parent, neighbor);
                    if (g_score.find(neighbor) == g_score.end() || new_g_score < g_score[neighbor]) {
                        g_score[neighbor] = new_g_score;
                        came_from[neighbor] = parent; // Bỏ qua current
                        open_list.push({new_g_score + heuristic(neighbor, goal_node), neighbor});
                    }
                } else {
                    // Nếu không, quay về logic A* bình thường
                    new_g_score = g_score[current] + heuristic(current, neighbor);
                    if (g_score.find(neighbor) == g_score.end() || new_g_score < g_score[neighbor]) {
                        g_score[neighbor] = new_g_score;
                        came_from[neighbor] = current;
                        open_list.push({new_g_score + heuristic(neighbor, goal_node), neighbor});
                    }
                }
            }
        }
    }
    
    ROS_WARN("No path found from start to goal.");
    return {}; // Không tìm thấy đường đi
}