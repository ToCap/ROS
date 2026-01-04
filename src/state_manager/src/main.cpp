#include "state_manager/state_manager.hpp"
#include "state_manager/state_manager_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<state_manager::StateManagerNode>(options);

  // run as lifecycle node
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
