cmake_minimum_required(VERSION 3.8)
project(vda5050_210)

# Recommandé par ROS 2
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ===================================================================
# IDL interfaces
# ===================================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Header.idl"
  "msg/Order.idl"
  "msg/Node.idl"
  "msg/Edge.idl"
  "msg/Action.idl"
  "msg/State.idl"
  "msg/BatteryState.idl"
  "msg/Error.idl"
)

# ===================================================================
# Export
# ===================================================================

ament_export_dependencies(rosidl_default_runtime)

ament_package()
