cmake_minimum_required(VERSION 3.5)
project(sensor_ultrasonic_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pal_statistics REQUIRED)
find_package(pal_statistics_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib SHARED src/sensor_ultrasonic_interface.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC include)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include
  /opt/ros/jazzy/include/joint_limits
  /opt/ros/jazzy/include/hardware_interface
)

ament_target_dependencies(${PROJECT_NAME}_lib
  ros2_control
  pluginlib
  rclcpp
  pal_statistics
  pal_statistics_msgs
  realtime_tools
)

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# Installer les fichiers de description (URDF/Xacro)
install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

# Installer les fichiers de config (YAML)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()
