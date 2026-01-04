#include "sensor_accelerometer.hpp"

namespace sensor_accelerometer
{

SensorAccelerometer::SensorAccelerometer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sensor_accelerometer", options),
  diag_updater_(this)
{
  diag_updater_.setHardwareID("sensor_accelerometer");
  RCLCPP_INFO(get_logger(), "SensorAccelerometer node created");
}

CallbackReturn SensorAccelerometer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring SensorAccelerometer...");

  measurements_.clear();
  for (auto type : {AccelerometerType::MINDSTORM_EV3}) {
    measurements_[type] = {0.0, 0.0, 0.0};
  }

  accel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "accelerometer/raw",
    rclcpp::QoS(10),
    std::bind(&SensorAccelerometer::accelerometer_callback, this, std::placeholders::_1));

  accel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "accelerometer/filtered", 10);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorAccelerometer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating SensorAccelerometer...");
  accel_pub_->on_activate();
  diag_updater_.force_update();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorAccelerometer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating SensorAccelerometer...");
  accel_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorAccelerometer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up SensorAccelerometer...");
  accel_sub_.reset();
  accel_pub_.reset();
  measurements_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorAccelerometer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down SensorAccelerometer from state %s...",
              state.label().c_str());
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// CALLBACKS
// ---------------------------------------------------------------------------

void SensorAccelerometer::accelerometer_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 3) return;

  std::lock_guard<std::mutex> lock(data_mutex_);
  measurements_[AccelerometerType::MINDSTORM_EV3] = {msg->data[0], msg->data[1], msg->data[2]};
  publish_output();
}

// ---------------------------------------------------------------------------
// PUBLISH & DIAGNOSTICS
// ---------------------------------------------------------------------------

void SensorAccelerometer::publish_output()
{
  if (!accel_pub_ || !accel_pub_->is_activated()) return;

  std::lock_guard<std::mutex> lock(data_mutex_);
  std_msgs::msg::Float64MultiArray out;
  out.data = {measurements_[AccelerometerType::MINDSTORM_EV3][0],
              measurements_[AccelerometerType::MINDSTORM_EV3][1],
              measurements_[AccelerometerType::MINDSTORM_EV3][2]};
  accel_pub_->publish(out);
}

void SensorAccelerometer::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto & m = measurements_[AccelerometerType::MINDSTORM_EV3];
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Accelerometer nominal");
  stat.add("Accel X", m[0]);
  stat.add("Accel Y", m[1]);
  stat.add("Accel Z", m[2]);
}

}  // namespace sensor_accelerometer
