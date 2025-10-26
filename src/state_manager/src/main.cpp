#include "state_manager/state_manager.hpp"
#include "state_manager/state_manager_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // Ex: on peut charger param via remapping ou YAML launch
  auto node = std::make_shared<state_manager::StateManagerNode>(options);

  // run as lifecycle node: en normal on démarre en "unconfigured" puis configure via lifecycle manager
  rclcpp::spin(node->get_node_base_interface()); // spin minimal pour services/subscriptions
  rclcpp::shutdown();
  return 0;
}
