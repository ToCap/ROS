// collision_detection_node.hpp
#pragma once

#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "collision_detection_system.hpp"

namespace collision_detection_system {

/**
 * @brief LifecycleNode that interfaces with bumper topics and publishes obstacle information.
 *
 * The node subscribes to left/right bumper states and measured values,
 * computes obstacle data using CollisionDetectionSystem, and publishes results
 * to /grid_occupancy/obstacle.
 */
class CollisionDetectionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CollisionDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CollisionDetectionNode() override = default;

protected:
  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Topics
  std::string left_state_topic_;
  std::string left_measured_topic_;
  std::string right_state_topic_;
  std::string right_measured_topic_;
  std::string output_topic_;

  // ROS interfaces
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_measured_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_measured_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Algorithm instance
  CollisionDetectionSystem detection_;

  // Parameters
  double publish_period_{0.05};

  // Timer callback
  void publish_obstacle_info();
};
}
