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
  this->declare_parameter<std::string>("left_bumper_state_topic", "/robot/left_bumper/state");
  this->declare_parameter<std::string>("left_measured_topic", "/robot/left_bumper/measured");
  this->declare_parameter<std::string>("right_state_topic", "/robot/right_bumper/state");
  this->declare_parameter<std::string>("right_measured_topic", "/robot/right_bumper/measured");
  this->declare_parameter<std::string>("output_topic", "/collision_detection/output");

  this->declare_parameter<double>("left_bumper_x", -0.2);
  this->declare_parameter<double>("left_bumper_y", 0.1);
  this->declare_parameter<double>("right_bumper_x", 0.2);
  this->declare_parameter<double>("right_bumper_y", 0.1);

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring collision_detection_node...");

  // Load parameters
  left_bumper_state_topic_ = get_parameter("left_bumper_state_topic").as_string();
  left_bumper_meas_topic_ = get_parameter("left_bumper_meas_topic").as_string();
  right_bumper_state_topic_ = get_parameter("right_bumper_state_topic").as_string();
  right_bumper_meas_topic_ = get_parameter("right_bumper_meas_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();

  // load geometry related parameters
  Param_Geometry_t cfg;
  cfg.left_bumper_x = get_parameter("left_bumper_x").as_double();
  cfg.left_bumper_y = get_parameter("left_bumper_y").as_double();
  cfg.right_bumper_x = get_parameter("right_bumper_x").as_double();
  cfg.right_bumper_y = get_parameter("right_bumper_y").as_double();

  // Create publisher
  publisher_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);

  // Create subscriptions
  left_state_sub_ = create_subscription<std_msgs::msg::Bool>(
    left_bumper_state_topic_, 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      detection_.set_left_state(msg->data, now());
      this->publish_obstacle_info();
    });

  left_measured_sub_ = create_subscription<std_msgs::msg::Int32>(
    left_bumper_meas_topic_, 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      detection_.set_left_measured(msg->data, now());
      this->publish_obstacle_info();
    });

  right_state_sub_ = create_subscription<std_msgs::msg::Bool>(
    right_bumper_state_topic_, 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      detection_.set_right_state(msg->data, now());
      this->publish_obstacle_info();
    });

  right_measured_sub_ = create_subscription<std_msgs::msg::Int32>(
    right_bumper_meas_topic_, 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      detection_.set_right_measured(msg->data, now());
      this->publish_obstacle_info();
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
  left_state_sub_.reset();
  left_measured_sub_.reset();
  right_state_sub_.reset();
  right_measured_sub_.reset();

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
  auto info = detection_.compute();

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "{";
  //ss << "\"present\":" << (info.present ? "true" : "false") << ",";
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
