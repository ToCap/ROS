cmake_minimum_required(VERSION 3.8)
project(sensor_gyro)

# === C++ standard ===
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# === Find required packages ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp_components REQUIRED)

# === Include directories ===
include_directories(
  include
)

# === Build the library ===
add_library(${PROJECT_NAME} SHARED
  src/sensor_gyro.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# === Link dependencies ===
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  diagnostic_updater
)

# === Export as a component ===
rclcpp_components_register_nodes(${PROJECT_NAME} "sensor_gyro_abstraction::SensorGyro")

# === Installation ===
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
