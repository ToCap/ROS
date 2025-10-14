cmake_minimum_required(VERSION 3.8)
project(collision_detection_system)

# Ensure modern C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Silence warning in GCC about ABI
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# ============================================================
#  Include directories
# ============================================================
include_directories(
  include
)

# ============================================================
#  Create library (component)
# ============================================================

# Algorithm library
add_library(${PROJECT_NAME}_core SHARED
  src/collision_detection_system.cpp
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
)

# Node library (component)
add_library(${PROJECT_NAME}_node SHARED
  src/collision_detection_system_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)

# Register node as a component for composition
rclcpp_components_register_nodes(${PROJECT_NAME}_node "CollisionDetectionNode")

# ============================================================
#  Optional executable (for standalone run)
# ============================================================
add_executable(collision_detection_system_main
  src/main.cpp
)
ament_target_dependencies(collision_detection_system_main
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
)
target_link_libraries(collision_detection_system_main
  ${PROJECT_NAME}_node
)

# ============================================================
#  Install rules
# ============================================================
install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_node
    collision_detection_system_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ============================================================
#  Export dependencies
# ============================================================
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_node)
ament_export_dependencies(rclcpp rclcpp_lifecycle rclcpp_components std_msgs)

ament_package()
