#include "sensor_touch/sensor_touch.hpp"

namespace sensor_touch_abstraction
{

SensorTouch::SensorTouch(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sensor_touch", options),
  diag_updater_(this)
{
  diag_updater_.setHardwareID("sensor_touch");

  RCLCPP_INFO(get_logger(), "SensorTouch abstraction node created");
}

CallbackReturn SensorTouch::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring sensor_touch abstraction...");

  // Initialize maps for current sensor types
  angles_.clear();
  rates_.clear();
  pressed_.clear();

  for (auto type : {TouchSensorType::MINDSTORM_EV3}) {
    angles_[type] = 0.0;
    rates_[type] = 0.0;
    pressed_[type] = false;
  }

  // Subscriptions
  touch_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "touch/angle",
    rclcpp::QoS(10),
    std::bind(&SensorTouch::on_touch_angle_callback, this, std::placeholders::_1));

  touch_rate_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "touch/rate",
    rclcpp::QoS(10),
    std::bind(&SensorTouch::on_touch_rate_callback, this, std::placeholders::_1));

  // Publisher for consolidated touch state
  touch_pub_ = this->create_publisher<std_msgs::msg::Float64>("touch/state", 10);

  RCLCPP_INFO(get_logger(), "sensor_touch abstraction configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating sensor_touch abstraction...");

  if (!check_system_conditions()) {
    RCLCPP_ERROR(get_logger(), "System conditions invalid — activation rejected.");
    return CallbackReturn::ERROR;
  }

  touch_pub_->on_activate();
  diag_updater_.force_update();

  RCLCPP_INFO(get_logger(), "sensor_touch abstraction activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating sensor_touch abstraction...");
  touch_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up sensor_touch abstraction...");

  touch_angle_sub_.reset();
  touch_rate_sub_.reset();
  touch_pub_.reset();

  angles_.clear();
  rates_.clear();
  pressed_.clear();

  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(),
              "Shutting down sensor_touch abstraction from state %s...",
              state.label().c_str());
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// CALLBACKS
// ---------------------------------------------------------------------------

void SensorTouch::on_touch_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  double angle = msg->data;

  // RULE: angle > 0.5 → pressed
  pressed_[TouchSensorType::MINDSTORM_EV3] = (angle > 0.5);
  angles_[TouchSensorType::MINDSTORM_EV3] = angle;

  publish_output();

  RCLCPP_DEBUG(get_logger(), "Touch angle received: %.3f → pressed=%s",
               angle, pressed_[TouchSensorType::MINDSTORM_EV3] ? "true" : "false");
}

void SensorTouch::on_touch_rate_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  double rate = msg->data;
  rates_[TouchSensorType::MINDSTORM_EV3] = rate;

  // Optional logic
  if (rate > 10.0) {
    RCLCPP_WARN(get_logger(), "Touch rate unusually high: %.2f", rate);
  }

  RCLCPP_DEBUG(get_logger(), "Touch rate: %.3f", rate);
}

// ---------------------------------------------------------------------------
// PUBLISH & DIAGNOSTICS
// ---------------------------------------------------------------------------

void SensorTouch::publish_output()
{
  if (!touch_pub_ || !touch_pub_->is_activated()) {
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = pressed_[TouchSensorType::MINDSTORM_EV3] ? 1.0 : 0.0;
  touch_pub_->publish(msg);
}

void SensorTouch::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  bool pressed = pressed_[TouchSensorType::MINDSTORM_EV3];
  stat.summary(pressed ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                       : diagnostic_msgs::msg::DiagnosticStatus::OK,
               pressed ? "Touch sensor pressed" : "Sensor idle");
  stat.add("Angle", angles_[TouchSensorType::MINDSTORM_EV3]);
  stat.add("Rate", rates_[TouchSensorType::MINDSTORM_EV3]);
}

bool SensorTouch::check_system_conditions()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (pressed_[TouchSensorType::MINDSTORM_EV3]) {
    RCLCPP_WARN(get_logger(), "Activation refused: touch sensor already pressed.");
    return false;
  }
  return true;
}

}  // namespace sensor_touch_abstraction
