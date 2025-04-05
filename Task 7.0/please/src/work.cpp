#include <climits>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <vector>
#include <unordered_map>
#include <cmath>

struct AStarSearchNode {
    int coordinate_x, coordinate_y, cost_g, cost_h;
    
    AStarSearchNode(int x, int y, int g, int h) {
        coordinate_x = x;
        coordinate_y = y;
        cost_g = g;
        cost_h = h;
    }
    
    int getTotalCostF() const {
        int total_cost_f;
        total_cost_f = cost_g + cost_h;
        return total_cost_f;
    }
};

struct CompareAStarNodes {
    bool operator()(const AStarSearchNode& node_a, const AStarSearchNode& node_b) {
        return node_a.getTotalCostF() > node_b.getTotalCostF();
    }
};

class AStarPathfindingPlanner : public rclcpp::Node {

public:
    
    AStarPathfindingPlanner() : Node("a_star_pathfinding_planner") {
        
        RCLCPP_INFO(this->get_logger(), "A* Pathfinding Planner Node has started running!");

        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarPathfindingPlanner::mapDataCallback, this, std::placeholders::_1));
        
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/generated_navigation_path", 10);
    }

private:
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    nav_msgs::msg::OccupancyGrid::SharedPtr stored_map_data_;
    std::vector<bool> map_occupancy_grid_;
    int map_grid_width_, map_grid_height_;

    void mapDataCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr received_map) {
        
        stored_map_data_ = received_map;
        map_grid_width_ = stored_map_data_->info.width;
        map_grid_height_ = stored_map_data_->info.height;
        map_occupancy_grid_.assign(stored_map_data_->data.begin(), stored_map_data_->data.end());
        
        executeAStarPathSearch();
    }

    void executeAStarPathSearch() {
        
        std::priority_queue<AStarSearchNode, std::vector<AStarSearchNode>, CompareAStarNodes> open_node_queue;
        std::unordered_map<int, int> path_reconstruction_map;

        int start_position_x = 0, start_position_y = 0;
        int target_position_x = map_grid_width_ - 1, target_position_y = map_grid_height_ - 1;

        AStarSearchNode initial_node(start_position_x, start_position_y, 0, calculateHeuristic(start_position_x, start_position_y, target_position_x, target_position_y));
        open_node_queue.push(initial_node);

        std::vector<int> movement_cost_grid(map_grid_width_ * map_grid_height_, INT_MAX);
        movement_cost_grid[start_position_y * map_grid_width_ + start_position_x] = 0;

        const int movement_directions[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
        const int movement_costs[8] = {10, 10, 10, 10, 14, 14, 14, 14};

        while (!open_node_queue.empty()) {
            
            AStarSearchNode current_node = open_node_queue.top();
            open_node_queue.pop();

            if (current_node.coordinate_x == target_position_x && current_node.coordinate_y == target_position_y) {
                reconstructAndPublishPath(path_reconstruction_map, target_position_x, target_position_y);
                return;
            }

            for (int i = 0; i < 8; ++i) {
                int next_position_x = current_node.coordinate_x + movement_directions[i][0];
                int next_position_y = current_node.coordinate_y + movement_directions[i][1];
                
                if (!isCellValid(next_position_x, next_position_y)) continue;

                int next_index = next_position_y * map_grid_width_ + next_position_x;
                int new_cost_g = movement_cost_grid[current_node.coordinate_y * map_grid_width_ + current_node.coordinate_x] + movement_costs[i];

                if (new_cost_g < movement_cost_grid[next_index]) {
                    movement_cost_grid[next_index] = new_cost_g;
                    path_reconstruction_map[next_index] = current_node.coordinate_y * map_grid_width_ + current_node.coordinate_x;
                    open_node_queue.push(AStarSearchNode(next_position_x, next_position_y, new_cost_g, calculateHeuristic(next_position_x, next_position_y, target_position_x, target_position_y)));
                }
            }
        }
        RCLCPP_WARN(this->get_logger(), "Pathfinding failed: No valid path available!");
    }

    bool isCellValid(int x, int y) const {
        return x >= 0 && x < map_grid_width_ && y >= 0 && y < map_grid_height_ && map_occupancy_grid_[y * map_grid_width_ + x] == 0;
    }

    int calculateHeuristic(int x, int y, int goal_x, int goal_y) {
        return std::abs(x - goal_x) + std::abs(y - goal_y);
    }

    void reconstructAndPublishPath(const std::unordered_map<int, int>& path_reconstruction_map, int goal_x, int goal_y) {
        
        auto path_message = std::make_shared<nav_msgs::msg::Path>();
        path_message->header.stamp = this->now();
        path_message->header.frame_id = "map";

        int index = goal_y * map_grid_width_ + goal_x;
        while (path_reconstruction_map.find(index) != path_reconstruction_map.end()) {
            
            int x = index % map_grid_width_;
            int y = index / map_grid_width_;
            
            geometry_msgs::msg::PoseStamped waypoint;
            waypoint.header = path_message->header;
            waypoint.pose.position.x = x * stored_map_data_->info.resolution + stored_map_data_->info.origin.position.x;
            waypoint.pose.position.y = y * stored_map_data_->info.resolution + stored_map_data_->info.origin.position.y;
            
            path_message->poses.push_back(waypoint);
            index = path_reconstruction_map.at(index);
        }
        std::reverse(path_message->poses.begin(), path_message->poses.end());
        RCLCPP_INFO(this->get_logger(), "Successfully published path with %zu waypoints.", path_message->poses.size());
        while(rclcpp::ok())
        {
            path_publisher_->publish(*path_message);
        }
    }
};

int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    auto planner_node = std::make_shared<AStarPathfindingPlanner>();
    rclcpp::spin(planner_node);
    rclcpp::shutdown();
    
    return 0;
}







