#include "sensor_touch/sensor_touch_types.hpp"
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
  pressed_.clear();
  measured_value_.clear();
  for (auto type : {TouchSensorType::MINDSTORM_EV3})
  {
    pressed_[type] = false;
    measured_value_[type] = TouchMeasuredValue::RELEASED;
  }

  // Subscribe to the low-level touch sensor state
  touch_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/touch_sensor/state",
    rclcpp::QoS(10),
    std::bind(&SensorTouch::on_touch_state_callback, this, std::placeholders::_1));

  // Publishers for consolidated touch state
  touch_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "touch/state_abstraction", 10);

  measured_value_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "touch/measured_value", 10);

  // Register diagnostic task
  diag_updater_.add("Touch Sensor Status", this, &SensorTouch::produceDiagnostics);

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
  measured_value_pub_->on_activate();
  diag_updater_.force_update();

  RCLCPP_INFO(get_logger(), "sensor_touch abstraction activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating sensor_touch abstraction...");
  touch_pub_->on_deactivate();
  measured_value_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up sensor_touch abstraction...");

  touch_state_sub_.reset();
  touch_pub_.reset();
  measured_value_pub_.reset();

  pressed_.clear();
  measured_value_.clear();

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

void SensorTouch::on_touch_state_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  bool pressed = msg->data;

  pressed_[TouchSensorType::MINDSTORM_EV3] = pressed;

  // Determine measured value
  if (!pressed) {
    measured_value_[TouchSensorType::MINDSTORM_EV3] = TouchMeasuredValue::RELEASED;
  } else {
    // Example logic: if previously released, now pressed = PRESSED
    // if sensor was already pressed and another event occurs, mark as BUMPED
    if (measured_value_[TouchSensorType::MINDSTORM_EV3] == TouchMeasuredValue::PRESSED) {
      measured_value_[TouchSensorType::MINDSTORM_EV3] = TouchMeasuredValue::BUMPED;
    } else {
      measured_value_[TouchSensorType::MINDSTORM_EV3] = TouchMeasuredValue::PRESSED;
    }
  }

  publish_output();

  RCLCPP_DEBUG(get_logger(), "Touch state received: %s, measured_value=%u",
               pressed ? "pressed" : "released",
               static_cast<uint8_t>(measured_value_[TouchSensorType::MINDSTORM_EV3]));
}

// ---------------------------------------------------------------------------
// PUBLISH & DIAGNOSTICS
// ---------------------------------------------------------------------------

void SensorTouch::publish_output()
{
  if (!touch_pub_ || !touch_pub_->is_activated() ||
      !measured_value_pub_ || !measured_value_pub_->is_activated()) {
    return;
  }

  // Publish normalized Boolean state
  std_msgs::msg::Float64 msg;
  msg.data = pressed_[TouchSensorType::MINDSTORM_EV3] ? 1.0 : 0.0;
  touch_pub_->publish(msg);

  // Publish measured value
  std_msgs::msg::Float64 val_msg;
  val_msg.data = static_cast<double>(measured_value_[TouchSensorType::MINDSTORM_EV3]);
  measured_value_pub_->publish(val_msg);
}

void SensorTouch::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  bool pressed = pressed_[TouchSensorType::MINDSTORM_EV3];
  TouchMeasuredValue value = measured_value_[TouchSensorType::MINDSTORM_EV3];

  stat.summary(pressed ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                       : diagnostic_msgs::msg::DiagnosticStatus::OK,
               pressed ? "Touch sensor pressed" : "Sensor idle");
  stat.add("Pressed", pressed);
  stat.add("Measured Value", static_cast<uint8_t>(value));
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
