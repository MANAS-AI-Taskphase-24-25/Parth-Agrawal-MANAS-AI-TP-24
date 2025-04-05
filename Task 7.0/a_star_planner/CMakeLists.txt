cmake_minimum_required(VERSION 3.8)
project(a_star_planner)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# C++ standard
add_definitions(-std=c++17)

# Planner node executable
add_executable(planner_node src/planner_node.cpp)

# Link against required dependencies
ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Package export
ament_package()

