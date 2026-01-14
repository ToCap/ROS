// collision_detection_node.hpp
#pragma once

#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp" 
#include "collision_detection_system.hpp"

namespace collision_detection_system {

/**
 * @brief LifecycleNode that interfaces with abstracted touch sensors
 * and publishes obstacle information.
 *
 * The node subscribes to left/right touch sensor states and measured values
 * provided by the SensorTouch abstraction layer, computes obstacle data
 * using CollisionDetectionSystem, and publishes results to /grid_occupancy/obstacle.
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
  // Topics (SensorTouch abstraction)
  std::string left_touch_state_topic_;
  std::string left_touch_measured_topic_;
  std::string right_touch_state_topic_;
  std::string right_touch_measured_topic_;
  std::string output_topic_;

  // ROS interfaces
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_touch_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_touch_measured_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_touch_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_touch_measured_sub_;

  // Internal storage of touch states (necessary for periodic decision)
  double left_touch_state_{0.0};        // normalized: 0.0 = released, 1.0 = pressed
  int    left_touch_measured_{0};       // TouchMeasuredValue: 0 = Released, 1 = Pressed, 2 = Bumped

  double right_touch_state_{0.0};
  int    right_touch_measured_{0};

  // Algorithm instance
  CollisionDetectionSystem detection_;

  // Callbacks for subscriptions
  void on_left_touch_state(const std_msgs::msg::Float64::SharedPtr msg);
  void on_left_touch_measured(const std_msgs::msg::Float64::SharedPtr msg);
  void on_right_touch_state(const std_msgs::msg::Float64::SharedPtr msg);
  void on_right_touch_measured(const std_msgs::msg::Float64::SharedPtr msg);

  // Timer callback
  void publish_obstacle_info();
};

}  // namespace collision_detection_system
