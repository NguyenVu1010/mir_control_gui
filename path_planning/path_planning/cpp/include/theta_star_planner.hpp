#pragma once

// ---- Các thư viện ROS cần thiết ----
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

// ---- Các thư viện chuẩn C++ ----
#include <vector>
#include <utility> // Dùng cho std::pair

class ThetaStarPathPlanner {
public:
    /**
     * @brief Hàm tạo của lớp ThetaStarPathPlanner.
     * @param nh NodeHandle của ROS để đăng ký subscriber.
     * @param robot_width Chiều rộng của robot (mét).
     * @param robot_height Chiều cao của robot (mét).
     */
    ThetaStarPathPlanner(ros::NodeHandle& nh, double robot_width, double robot_height);

    /**
     * @brief Chạy thuật toán Theta* để tìm đường đi từ start đến goal.
     * @param start Tọa độ điểm bắt đầu (x, y) trong hệ tọa độ thế giới.
     * @param goal Tọa độ điểm kết thúc (x, y) trong hệ tọa độ thế giới.
     * @return Một vector chứa các điểm (x, y) của đường đi, hoặc vector rỗng nếu không tìm thấy.
     */
    std::vector<std::pair<double, double>> find_path(
        const std::pair<double, double>& start, 
        const std::pair<double, double>& goal
    );

    /**
     * @brief Kiểm tra xem bản đồ đã được nhận và xử lý hay chưa.
     */
    bool is_map_ready() const;

private:
    /**
     * @brief Callback function khi nhận được dữ liệu bản đồ từ topic /map.
     */
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    /**
     * @brief Tạo ra "bản đồ hợp lệ" (valid_map) bằng cách làm dày các vật cản
     *        dựa trên kích thước của robot để kiểm tra va chạm.
     */
    void generate_valid_map();

    // ---- Các hàm chuyển đổi tọa độ ----
    std::pair<int, int> world_to_map(double world_x, double world_y) const;
    std::pair<double, double> map_to_world(int map_x, int map_y) const;

    /**
     * @brief Kiểm tra một ô trên bản đồ có hợp lệ để di chuyển không (trong biên và không phải vật cản).
     */
    bool is_valid(int map_x, int map_y) const;

    /**
     * @brief Kiểm tra "tầm nhìn thẳng" (Line of Sight) giữa hai điểm trên bản đồ.
     *        Sử dụng thuật toán Bresenham.
     */
    bool line_of_sight(const std::pair<int, int>& p1, const std::pair<int, int>& p2) const;

    // ---- Các biến thành viên ----
    ros::NodeHandle& nh_;
    ros::Subscriber map_sub_;

    // Thông tin bản đồ
    std::vector<int8_t> map_data_;
    unsigned int map_width_;
    unsigned int map_height_;
    double map_resolution_;
    std::pair<double, double> map_origin_;
    
    // Bản đồ va chạm (true = có vật cản/không hợp lệ)
    std::vector<bool> valid_map_;

    // Kích thước robot (tính bằng số ô pixel trên bản đồ)
    int robot_width_px_;
    int robot_height_px_;

    // Cờ báo hiệu bản đồ đã sẵn sàng
    bool map_ready_ = false;
};