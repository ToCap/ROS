#include "sensor_touch.hpp"

namespace sensor_touch_abstraction
{

SensorTouch::SensorTouch(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sensor_touch_abstraction", options),
  diag_updater_(this)
{
  diag_updater_.setHardwareID("sensor_touch");

  RCLCPP_INFO(get_logger(), "SensorTouch abstraction node created");
}

CallbackReturn SensorTouch::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring sensor_touch abstraction...");

  // Subscriptions
  touch_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "touch/angle",
    rclcpp::QoS(10),
    std::bind(&SensorTouch::touch_angle_callback, this, std::placeholders::_1));

  touch_rate_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "touch/rate",
    rclcpp::QoS(10),
    std::bind(&SensorTouch::touch_rate_callback, this, std::placeholders::_1));

  // Diagnostic publisher for message frequency
  diag_publisher_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    "touch/angle",
    diag_updater_,
    diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10),
    diagnostic_updater::TimeStampStatusParam());

  RCLCPP_INFO(get_logger(), "sensor_touch abstraction configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating sensor_touch abstraction...");

  if (!check_system_conditions()) {
    RCLCPP_ERROR(get_logger(),
      "System conditions invalid — activation rejected.");
    return CallbackReturn::ERROR;
  }

  diag_updater_.force_update();

  RCLCPP_INFO(get_logger(), "sensor_touch abstraction activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating sensor_touch abstraction...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorTouch::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up sensor_touch abstraction...");

  touch_angle_sub_.reset();
  touch_rate_sub_.reset();
  diag_publisher_.reset();

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

void SensorTouch::touch_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  double angle = msg->data;
  state_ = (angle > 0.5);  // RULE: angle > 0.5 → pressed

  // Feed diagnostics
  diag_publisher_->tick();

  RCLCPP_DEBUG(get_logger(), "Touch angle received: %.3f → state=%s",
               angle, state_ ? "pressed" : "released");
}

void SensorTouch::touch_rate_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  double rate = msg->data;

  // Optionally enforce rate-based logic
  if (rate > 10.0) {
    RCLCPP_WARN(get_logger(), "Touch rate unusually high: %.2f", rate);
  }

  RCLCPP_DEBUG(get_logger(), "Touch rate: %.3f", rate);
}

// ---------------------------------------------------------------------------
// DIAGNOSTICS & BUSINESS RULES
// ---------------------------------------------------------------------------

void SensorTouch::publish_diagnostics()
{
  diag_updater_.update();
}

bool SensorTouch::check_system_conditions()
{
  // Business rule examples — à adapter selon vos contraintes :
  //
  // - Interdire l’activation si le capteur est déjà « pressed »
  //   lors du démarrage.
  // - Vérifier qu'un flux est bien reçu.
  // - Filtrer des états aberrants.

  if (state_) {
    RCLCPP_WARN(get_logger(),
      "Activation refused: touch sensor is already pressed.");
    return false;
  }

  return true;
}

}  // namespace sensor_touch_abstraction
