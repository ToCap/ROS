// src/main.cpp
#include "collision_detection_system/collision_detection_system_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<collision_detection_system::CollisionDetectionNode>();
  
  // run as lifecycle node
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
