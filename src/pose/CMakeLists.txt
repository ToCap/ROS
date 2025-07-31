cmake_minimum_required(VERSION 3.5)
project(pose)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(pose_estimator src/pose_estimator.cpp)
target_include_directories(pose_estimator PUBLIC include)

add_executable(pose_node src/pose_node.cpp)
ament_target_dependencies(pose_node 
rclcpp
nav_msgs
geometry_msgs
)

target_link_libraries(pose_node pose_estimator)

install(TARGETS pose_node pose_estimator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
