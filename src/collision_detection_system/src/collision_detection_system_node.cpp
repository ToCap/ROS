// collision_detection_system_node.cpp
#include "collision_detection_system/collision_detection_system_node.hpp"
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;
using namespace collision_detection_system;

CollisionDetectionNode::CollisionDetectionNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("collision_detection_system", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("left_touch_state_topic", "/robot/left_touch/state");
  this->declare_parameter<std::string>("left_touch_measured_topic", "/robot/left_touch/measured");
  this->declare_parameter<std::string>("right_touch_state_topic", "/robot/right_touch/state");
  this->declare_parameter<std::string>("right_touch_measured_topic", "/robot/right_touch/measured");
  this->declare_parameter<std::string>("output_topic", "/collision_detection/output");

  this->declare_parameter<double>("left_touch_x", -0.2);
  this->declare_parameter<double>("left_touch_y", 0.1);
  this->declare_parameter<double>("right_touch_x", 0.2);
  this->declare_parameter<double>("right_touch_y", 0.1);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring collision_detection_node...");

  // Load parameters
  left_touch_state_topic_ = get_parameter("left_touch_state_topic").as_string();
  left_touch_measured_topic_ = get_parameter("left_touch_measured_topic").as_string();
  right_touch_state_topic_ = get_parameter("right_touch_state_topic").as_string();
  right_touch_measured_topic_ = get_parameter("right_touch_measured_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();

  // load geometry related parameters
  Param_Geometry_t cfg;
  cfg.left_bumper_x = get_parameter("left_touch_x").as_double();
  cfg.left_bumper_y = get_parameter("left_touch_y").as_double();
  cfg.right_bumper_x = get_parameter("right_touch_x").as_double();
  cfg.right_bumper_y = get_parameter("right_touch_y").as_double();

  // Create publisher
  publisher_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);

  // Create subscriptions to SensorTouch
  left_touch_state_sub_ = create_subscription<std_msgs::msg::Float64>(
    left_touch_state_topic_, 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      left_touch_state_ = msg->data;
      // Optionally trigger obstacle update
      publish_obstacle_info();
    });

  left_touch_measured_sub_ = create_subscription<std_msgs::msg::Float64>(
    left_touch_measured_topic_, 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      left_touch_measured_ = static_cast<int>(msg->data);
      publish_obstacle_info();
    });

  right_touch_state_sub_ = create_subscription<std_msgs::msg::Float64>(
    right_touch_state_topic_, 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      right_touch_state_ = msg->data;
      publish_obstacle_info();
    });

  right_touch_measured_sub_ = create_subscription<std_msgs::msg::Float64>(
    right_touch_measured_topic_, 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      right_touch_measured_ = static_cast<int>(msg->data);
      publish_obstacle_info();
    });

  // copy the input geometry related params
  this->detection_.load_configuration(cfg);

  RCLCPP_INFO(get_logger(), "Node configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating collision_detection_node...");
  publisher_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating collision_detection_node...");
  if (publisher_) publisher_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up collision_detection_node...");
  publisher_.reset();
  left_touch_state_sub_.reset();
  left_touch_measured_sub_.reset();
  right_touch_state_sub_.reset();
  right_touch_measured_sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down collision_detection_node...");
  return on_cleanup(state);
}

void CollisionDetectionNode::publish_obstacle_info()
{
  // Translate measured value to collision flags
  bool left_collision  = (left_touch_measured_ >= 1);  // Pressed or Bumped
  bool right_collision = (right_touch_measured_ >= 1);

  detection_.set_left_state(left_collision, now());
  detection_.set_right_state(right_collision, now());

  // Compute obstacle info
  auto info = detection_.compute();

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "{";
  ss << "\"x\":" << info.origin_x << ",";
  ss << "\"y\":" << info.origin_y << ",";
  ss << "\"certainty\":" << info.validity_score << ",";
  ss << "\"source\":\"" << info.source << "\",";
  ss << "\"stamp\":" << info.stamp.seconds();
  ss << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  publisher_->publish(msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CollisionDetectionNode)
