#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>

class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner() : Node("a_star_planner") {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::OccupancyGrid current_map_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received map: %u x %u", msg->info.width, msg->info.height);

        // TODO: Choose start and goal for now (test mode)
        auto start = std::make_pair(10, 10);
        auto goal = std::make_pair(50, 50);

        auto path = aStarSearch(start, goal);

        if (!path.empty()) {
            nav_msgs::msg::Path ros_path;
            ros_path.header.frame_id = "map";
            for (auto& point : path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point.first * msg->info.resolution + msg->info.origin.position.x;
                pose.pose.position.y = point.second * msg->info.resolution + msg->info.origin.position.y;
                ros_path.poses.push_back(pose);
            }
            path_pub_->publish(ros_path);
        }
    }

    std::vector<std::pair<int, int>> aStarSearch(const std::pair<int, int>& start, const std::pair<int, int>& goal) {
    struct Node {
        int x, y;
        float g, f;
        bool operator>(const Node& other) const { return f > other.f; }
    };

    auto index = [this](int x, int y) {
        return y * current_map_.info.width + x;
    };

    auto isValid = [this](int x, int y) {
        return x >= 0 && y >= 0 &&
               x < static_cast<int>(current_map_.info.width) &&
               y < static_cast<int>(current_map_.info.height) &&
               current_map_.data[index(x, y)] == 0;
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<int, std::pair<int, int>> came_from;
    std::unordered_map<int, float> g_score;

    auto h = [](int x1, int y1, int x2, int y2) {
        return std::hypot(x2 - x1, y2 - y1);
    };

    int start_idx = index(start.first, start.second);
    open.push({start.first, start.second, 0.0f, h(start.first, start.second, goal.first, goal.second)});
    g_score[start_idx] = 0.0f;

    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {-1, -1}, {1, -1}, {-1, 1}
    };

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        if (current.x == goal.first && current.y == goal.second) {
            std::vector<std::pair<int, int>> path;
            int current_idx = index(goal.first, goal.second);
            while (came_from.find(current_idx) != came_from.end()) {
                auto [x, y] = came_from[current_idx];
                path.emplace_back(x, y);
                current_idx = index(x, y);
            }
            path.emplace_back(start.first, start.second);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            if (!isValid(nx, ny)) continue;

            float tentative_g = g_score[index(current.x, current.y)] + h(current.x, current.y, nx, ny);
            int neighbor_idx = index(nx, ny);

            if (g_score.find(neighbor_idx) == g_score.end() || tentative_g < g_score[neighbor_idx]) {
                came_from[neighbor_idx] = {current.x, current.y};
                g_score[neighbor_idx] = tentative_g;
                float f_score = tentative_g + h(nx, ny, goal.first, goal.second);
                open.push({nx, ny, tentative_g, f_score});
            }
        }
    }

    RCLCPP_WARN(this->get_logger(), "No path found!");
    return {};
}

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}

