cmake_minimum_required(VERSION 3.8)
project(planner)

# === C++ standard configuration ===
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# === ROS 2 dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# === Include directories ===
include_directories(
  include
)

# === Library: PlannerAlgorithm ===
add_library(planner
  src/planner.cpp
)
target_include_directories(planner PUBLIC include)
ament_target_dependencies(planner rclcpp)

# === Executable: PlannerNode (Lifecycle Node) ===
add_executable(planner_node
  src/planner_node.cpp
)
target_link_libraries(planner_node
  planner
)
ament_target_dependencies(planner_node
  rclcpp
  rclcpp_lifecycle
)

# === Installation rules ===
install(
  TARGETS
    planner_node
    planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# === Export package information ===
ament_export_include_directories(include)
ament_export_libraries(planner)
ament_export_dependencies(rclcpp rclcpp_lifecycle)
ament_package()
