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

    std::vector<std::pair<int, int>> aStarSearch(
        const std::pair<int, int>& start,
        const std::pair<int, int>& goal) {
        
        // TODO: Implement A* algorithm here
        return {};
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}

