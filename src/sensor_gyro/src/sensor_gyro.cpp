#include "sensor_gyro/sensor_gyro.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace sensor_gyro_abstraction
{

SensorGyro::SensorGyro(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sensor_gyro", options),
  diag_updater_(this)
{
  RCLCPP_INFO(get_logger(), "SensorGyro Lifecycle node constructed");
}

CallbackReturn SensorGyro::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring SensorGyro...");

  // Subscribe to gyro hardware topics
  gyro_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/gyro/angle", 10,
      std::bind(&SensorGyro::gyro_angle_callback, this, std::placeholders::_1));

  gyro_rate_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/gyro/rate", 10,
      std::bind(&SensorGyro::gyro_rate_callback, this, std::placeholders::_1));

  // Initialize diagnostics
  diag_updater_.setHardwareID("ev3_gyro");
  double min_freq = 1.0;
  double max_freq = 10.0;
  diag_publisher_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    "gyro_diagnostics", diag_updater_,
    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10),
    10);

  RCLCPP_INFO(get_logger(), "SensorGyro configured successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorGyro::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating SensorGyro...");

  if (!check_system_conditions()) {
    RCLCPP_WARN(get_logger(), "System conditions not met: activation denied");
    return CallbackReturn::FAILURE;
  }

  diag_updater_.force_update();
  RCLCPP_INFO(get_logger(), "SensorGyro activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorGyro::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating SensorGyro...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorGyro::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up SensorGyro...");
  gyro_angle_sub_.reset();
  gyro_rate_sub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorGyro::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down SensorGyro...");
  return CallbackReturn::SUCCESS;
}

// ---------------------------
// Callbacks
// ---------------------------

void SensorGyro::gyro_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  gyro_angle_ = msg->data;
  publish_diagnostics();
}

void SensorGyro::gyro_rate_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  gyro_rate_ = msg->data;
  publish_diagnostics();
}

// ---------------------------
// Diagnostics and rules
// ---------------------------

void SensorGyro::publish_diagnostics()
{
  // Use force_update() instead of update() to trigger diagnostics evaluation
  diag_updater_.force_update();

  // Tick the topic diagnostic to update frequency statistics
  if (diag_publisher_)
  {
    diag_publisher_->tick();
  }
}

bool SensorGyro::check_system_conditions()
{
  // Example business rules:
  // If gyro rate is too high or angle invalid, consider WARN/ERROR
  if (std::isnan(gyro_angle_) || std::isnan(gyro_rate_)) 
  {
    RCLCPP_ERROR(get_logger(), "Gyro sensor returned invalid value");
    return false;
  }

  // Placeholder for other rules: battery, motor temp, safety interlocks...
  return true;
}

}  // namespace sensor_gyro_abstraction

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_gyro_abstraction::SensorGyro)
