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
  this->declare_parameter<std::string>("left_state_topic", "/left_bumper/state");
  this->declare_parameter<std::string>("left_measured_topic", "/left_bumper/measured_value");
  this->declare_parameter<std::string>("right_state_topic", "/right_bumper/state");
  this->declare_parameter<std::string>("right_measured_topic", "/right_bumper/measured_value");
  this->declare_parameter<std::string>("output_topic", "/grid_occupancy/obstacle");
  this->declare_parameter<double>("publish_period_s", 0.05);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring collision_detection_node...");

  // Load parameters
  left_state_topic_ = get_parameter("left_state_topic").as_string();
  left_measured_topic_ = get_parameter("left_measured_topic").as_string();
  right_state_topic_ = get_parameter("right_state_topic").as_string();
  right_measured_topic_ = get_parameter("right_measured_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  publish_period_ = get_parameter("publish_period_s").as_double();

  // Create publisher
  publisher_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);

  // Create subscriptions
  left_state_sub_ = create_subscription<std_msgs::msg::Bool>(
    left_state_topic_, 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      detection_.set_left_state(msg->data, now());
    });

  left_measured_sub_ = create_subscription<std_msgs::msg::Int32>(
    left_measured_topic_, 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      detection_.set_left_measured(msg->data, now());
    });

  right_state_sub_ = create_subscription<std_msgs::msg::Bool>(
    right_state_topic_, 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      detection_.set_right_state(msg->data, now());
    });

  right_measured_sub_ = create_subscription<std_msgs::msg::Int32>(
    right_measured_topic_, 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      detection_.set_right_measured(msg->data, now());
    });

  RCLCPP_INFO(get_logger(), "Node configured with publish_period_s=%.2f", publish_period_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating collision_detection_node...");
  publisher_->on_activate();

  // Start periodic timer
  timer_ = create_wall_timer(
    std::chrono::duration<double>(publish_period_),
    std::bind(&CollisionDetectionNode::publish_obstacle_info, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CollisionDetectionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating collision_detection_node...");
  if (publisher_) publisher_->on_deactivate();
  if (timer_) timer_->cancel();
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
  timer_.reset();
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
  ss << "\"present\":" << (info.present ? "true" : "false") << ",";
  ss << "\"x\":" << info.x << ",";
  ss << "\"y\":" << info.y << ",";
  ss << "\"certainty\":" << info.certainty << ",";
  ss << "\"source\":\"" << info.source << "\",";
  ss << "\"stamp\":" << info.stamp.seconds();
  ss << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  publisher_->publish(msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CollisionDetectionNode)
