cmake_minimum_required(VERSION 3.10)
project(map_grid)

# force required sub-packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# creation of library associated to node
add_library(map_grid_lib SHARED
  src/map_grid.cpp
)

target_include_directories(map_grid_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# creation of node
add_executable(map_grid_node
  src/main.cpp
  src/map_grid_node.cpp
)
install(TARGETS map_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

# configure dependencies of node
target_link_libraries(map_grid_node
  map_grid_lib
)

ament_target_dependencies(map_grid_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  nav_msgs
  sensor_msgs
)

# Instal binaries
install(TARGETS
  map_grid_lib
  map_grid_node
  EXPORT export_map_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(export_map_grid)
ament_export_include_directories(include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
