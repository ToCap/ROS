#include "diagnostic_manager/diagnostic_manager_node.hpp"

/**
 * @file main.cpp
 * @brief Entry point for the Diagnostic Manager ROS 2 node
 */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diagnostic_manager::DiagnosticManagerNode>());
  rclcpp::shutdown();
  return 0;
}
