#include "sensor_rangefinder/sensor_rangefinder.hpp"

namespace sensor_rangefinder
{

SensorRangefinder::SensorRangefinder(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sensor_rangefinder", options), diag_updater_(this)
{
  RCLCPP_INFO(get_logger(), "Creating SensorRangefinder lifecycle node");

  // Initialize distances with "no data"
  distances_[RangefinderType::ULTRASONIC] = -1.0;
  distances_[RangefinderType::INFRARED]   = -1.0;

  // Diagnostics
  diag_updater_.setHardwareID("rangefinder");
  diag_updater_.add(
    "Rangefinder status",
    std::bind(&SensorRangefinder::produceDiagnostics, this, std::placeholders::_1));
}

CallbackReturn SensorRangefinder::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring SensorRangefinder");

  // Optional tuning parameter (kept because it's not a presence flag)
  max_infrared_range_ = declare_parameter<double>("max_infrared_range", 0.7);

  // Always subscribe to both topics
  ultrasonic_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/ultrasonic/distance", 10,
    std::bind(&SensorRangefinder::on_ultrasonic_callback, this, std::placeholders::_1));

  infrared_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/infrared/distance", 10,
    std::bind(&SensorRangefinder::on_infrared_callback, this, std::placeholders::_1));

  range_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/rangefinder/distance", 10);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorRangefinder::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating SensorRangefinder");
  range_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorRangefinder::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating SensorRangefinder");
  range_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorRangefinder::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up SensorRangefinder");

  ultrasonic_sub_.reset();
  infrared_sub_.reset();
  range_pub_.reset();

  distances_[RangefinderType::ULTRASONIC] = -1.0;
  distances_[RangefinderType::INFRARED]   = -1.0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorRangefinder::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down SensorRangefinder");
  return CallbackReturn::SUCCESS;
}

// ===================== CALLBACKS =====================

void SensorRangefinder::on_ultrasonic_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  distances_[RangefinderType::ULTRASONIC] = msg->data;
  publish_output();
  diag_updater_.force_update();
}

void SensorRangefinder::on_infrared_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  distances_[RangefinderType::INFRARED] = msg->data;
  publish_output();
  diag_updater_.force_update();
}

// ===================== OUTPUT =====================

void SensorRangefinder::publish_output()
{
  if (!range_pub_ || !range_pub_->is_activated())
  {
    return;
  }

  const double ir = distances_[RangefinderType::INFRARED];
  const double us = distances_[RangefinderType::ULTRASONIC];

  std_msgs::msg::Float64 out;
  out.data = 0.0;  // Default value (policy step 3)

  // 1. Infrared if valid and in range
  if (ir > 0.0 && ir <= max_infrared_range_)
  {
    out.data = ir;
  }
  // 2. Ultrasonic otherwise if valid and in range
  else if (us > 0.0) 
  {
    out.data = us;
  }
  // 3. 0.0 otherwise (already set)

  range_pub_->publish(out);
}

// ===================== DIAGNOSTICS =====================

void SensorRangefinder::produceDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  stat.summary(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "Rangefinder operating normally");

  for (const auto & [type, distance] : distances_)
  {
    const std::string name =
      (type == RangefinderType::INFRARED) ? "Infrared" : "Ultrasonic";

    if (distance < 0.0) 
    {
      stat.add(name, "No data received");
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "One or more sensors missing data");
    }
    else
    {
      stat.add(name, std::to_string(distance) + " m");
    }
  }
}

}  // namespace sensor_rangefinder
