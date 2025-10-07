#include "sensor_ultrasonic_interface/sensor_ultrasonic_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sensor_ultrasonic_interface
{

hardware_interface::CallbackReturn SensorUltrasonicInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Retrieve topic name from hardware parameters
  if (info_.hardware_parameters.find("topic_name") != info_.hardware_parameters.end())
  {
    topic_name_ = info_.hardware_parameters.at("topic_name");
  }
  else
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorUltrasonicInterface"),
      "Missing parameter 'topic_name' in hardware config");
    return hardware_interface::CallbackReturn::ERROR;
  }

  distance_ = 0.0;
  node_ = std::make_shared<rclcpp::Node>("ultrasonic_sim_node");

  // Subscribe to the simulated distance topic published by Gazebo
  sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    topic_name_, 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg){distance_ = msg->data;});

  // Run the node in a background thread to handle callbacks
  executor_thread_ = std::thread([this]() { rclcpp::spin(node_); });

  RCLCPP_INFO(rclcpp::get_logger("SensorUltrasonicInterface"), "SensorUltrasonicInterface initialized on topic: %s", topic_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SensorUltrasonicInterface::export_state_interfaces()
{
  // Expose a single state: "distance"
  return {
    hardware_interface::StateInterface(info_.joints[0].name, "distance", &distance_)
  };
}

std::vector<hardware_interface::CommandInterface> SensorUltrasonicInterface::export_command_interfaces()
{
  // No command interfaces for a passive sensor
  return {};
}

hardware_interface::return_type SensorUltrasonicInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Nothing special to do: distance_ is updated asynchronously by the subscriber
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorUltrasonicInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Passive sensor: no writing operation
  return hardware_interface::return_type::OK;
}

SensorUltrasonicInterface::~SensorUltrasonicInterface()
{
  rclcpp::shutdown();
  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }
}

}  // namespace ultrasonic_sim

// Export the plugin to ROS 2
PLUGINLIB_EXPORT_CLASS(sensor_ultrasonic_interface::SensorUltrasonicInterface, hardware_interface::SystemInterface)
