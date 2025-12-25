#pragma once

/**
 * @file sensor_touch.hpp
 * @brief LifecycleNode that abstracts one or more touch sensors.
 *
 * This node acts as a facade over one or more sensor hardware interfaces.
 * It aggregates sensor measurements, publishes diagnostics, and applies
 * application-level rules to provide a safe and consistent touch output.
 *
 * It aggregates raw sensor data, publishes diagnostics, and applies
 * application-level rules to expose a safe and consistent touch state.
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <map>
#include <mutex>

namespace sensor_touch_abstraction
{

/**
 * @brief Supported touch sensor types.
 */
enum class TouchSensorType
{
  MINDSTORM_EV3
};

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class SensorTouch
 * @brief LifecycleNode that abstracts one or more touch sensors.
 *
 * This node acts as a facade over the touch hardware interface.
 * It aggregates sensor measurements, publishes diagnostics, and enforces
 * application-level rules before exposing the touch state.
 */
class SensorTouch : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options ROS 2 node options
   */
  explicit SensorTouch(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  /// @brief Callback for touch angle updates
  void on_touch_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /// @brief Callback for touch rate updates
  void on_touch_rate_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Publish the consolidated touch state.
   *
   * This method applies application-level rules to determine whether
   * the sensor is considered "pressed" and publishes the result.
   */
  void publish_output();

  /**
   * @brief Generate diagnostic information for the touch sensor.
   *
   * This function is registered with the diagnostic_updater and is called
   * periodically to publish the current sensor status.
   *
   * @param stat DiagnosticStatusWrapper object used to populate the diagnostic message.
   */
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  /// Subscriptions to raw sensor topics
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr touch_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr touch_rate_sub_;

  /// Lifecycle publisher for the consolidated touch state
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr touch_pub_;

  /// Latest raw measurements
  std::map<TouchSensorType, double> angles_;
  std::map<TouchSensorType, double> rates_;

  /// Derived touch state
  std::map<TouchSensorType, bool> pressed_;

  /// Diagnostic updater
  diagnostic_updater::Updater diag_updater_;

  /// Mutex protecting shared sensor data
  std::mutex data_mutex_;
};

}  // namespace sensor_touch_abstraction
