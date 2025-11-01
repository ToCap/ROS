cmake_minimum_required(VERSION 3.8)
project(diagnostic_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(include)

add_executable(diagnostic_manager_node
  src/main.cpp
  src/diagnostic_manager_node.cpp
  src/diagnostic_manager.cpp
)

ament_target_dependencies(diagnostic_manager_node
  rclcpp
  diagnostic_msgs
)

install(TARGETS diagnostic_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
