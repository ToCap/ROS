cmake_minimum_required(VERSION 3.8)
project(recorder)

# Dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mcap REQUIRED)

include_directories(include)

add_executable(mcap_logger_node
  src/recorder_node.cpp
  src/main.cpp
)
ament_target_dependencies(mcap_logger_node
  rclcpp
  std_msgs
  mcap
)

install(TARGETS mcap_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
