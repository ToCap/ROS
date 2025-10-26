cmake_minimum_required(VERSION 3.8)
project(state_manager)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# -------------------------------
# Library
# -------------------------------
add_library(state_manager_lib
  src/state_manager.cpp
)

ament_target_dependencies(state_manager_lib
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
)

# -------------------------------
# Executable
# -------------------------------
add_executable(state_manager_node
  src/state_manager_node.cpp
  src/main.cpp
)

# Link executable with the library
target_link_libraries(state_manager_node
  state_manager_lib
)

ament_target_dependencies(state_manager_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
)

# -------------------------------
# Install targets
# -------------------------------
install(TARGETS
  state_manager_node
  state_manager_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# -------------------------------
# Package
# -------------------------------
ament_package()
