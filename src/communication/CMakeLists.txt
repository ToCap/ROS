cmake_minimum_required(VERSION 3.5)
project(communication)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/SetDeviceParameter.srv"
  "action/MoveTo.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_package()
