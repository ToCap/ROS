cmake_minimum_required(VERSION 3.8)
project(motor_ev3)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# Include header files
include_directories(include)

add_library(motor_ev3 src/motor_ev3.cpp)
target_include_directories(motor_ev3 PUBLIC include)

# Create executable for the node
add_executable(motor_ev3_node
  src/motor_ev3_node.cpp
)

target_link_libraries(motor_ev3_node motor_ev3)


# Declare dependencies for the executable
ament_target_dependencies(motor_ev3_node
  rclcpp
  std_srvs
)

# Install the executable
install(TARGETS motor_ev3_node
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize ament package
ament_package()
