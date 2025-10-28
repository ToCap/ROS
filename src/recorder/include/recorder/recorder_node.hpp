#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mcap/mcap.hpp>

namespace recorder
{
class RecorderNode : public rclcpp::Node
{
public:
  McapLoggerNode();
  ~McapLoggerNode();

private:
  void callback(const std_msgs::msg::String::SharedPtr msg);

  // ROS 2
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  
  // MCAP
  mcap::McapWriter writer_;
  mcap::SchemaId schema_id_;
  mcap::ChannelId channel_id_;
  uint64_t sequence_ = 0;
};
}
