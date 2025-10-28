#include "recorder/recorder_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<McapLoggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
