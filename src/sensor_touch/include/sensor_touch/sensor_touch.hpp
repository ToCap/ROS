#pragma once

/**
 * @file sensor_touch.hpp
 * @brief Lifecycle abstraction node for touch sensors.
 *
 * This node provides a unified touch sensor abstraction over one or more
 * hardware touch sensor interfaces.
 *
 * The node automatically adapts to the available sensor streams at runtime,
 * publishes diagnostics, and applies deterministic application-level rules
 * to expose a safe and consistent touch state.
 *
 * The node now publishes both:
 *  - touch_pub_: normalized Boolean state (0.0 = released, 1.0 = pressed)
 *  - measured_value_pub_: discrete measured value based on TouchMeasuredValue enum
 *    (0 = Released, 1 = Pressed, 2 = Bumped)
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <map>
#include <mutex>

#include "sensor_touch/sensor_touch_types.hpp"

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
 * This node acts as a facade over one or more touch hardware interfaces.
 * It aggregates raw sensor states, publishes diagnostics, and enforces
 * application-level rules before exposing a consolidated touch output.
 *
 * The abstraction ensures a clean separation between hardware access
 * and higher-level robot logic.
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
  /**
   * @brief Callback for raw touch state updates.
   *
   * This callback receives the low-level hardware state published by the
   * sensor interface and updates the internal touch state representation.
   *
   * @param msg Boolean message indicating whether the sensor is pressed.
   */
  void on_touch_state_callback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Publish the consolidated touch state.
   *
   * This method applies application-level rules and publishes:
   *  - touch_pub_: normalized Boolean state (0.0 = released, 1.0 = pressed)
   *  - measured_value_pub_: discrete TouchMeasuredValue (0 = Released, 1 = Pressed, 2 = Bumped)
   */
  void publish_output();

  /**
   * @brief Generate diagnostic information for the touch sensor.
   *
   * This function is registered with the diagnostic_updater and is called
   * periodically to publish the current status of the touch sensor.
   *
   * @param stat DiagnosticStatusWrapper object used to populate the diagnostic message.
   */
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check whether system conditions allow node activation.
   *
   * This method enforces application-level business rules that may prevent
   * lifecycle transitions (e.g., activation while the sensor is pressed).
   *
   * @return true if activation is allowed, false otherwise.
   */
  bool check_system_conditions();

private:
  /// Subscription to the raw hardware touch state topic
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr touch_state_sub_;

  /// Lifecycle publisher for the normalized Boolean touch state
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr touch_pub_;

  /// Lifecycle publisher for the measured value (TouchMeasuredValue)
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr measured_value_pub_;

  /// Latest Boolean touch state for each sensor type
  std::map<TouchSensorType, bool> pressed_;

  /// Latest measured value for each sensor type
  std::map<TouchSensorType, TouchMeasuredValue> measured_value_;

  /// Diagnostic updater
  diagnostic_updater::Updater diag_updater_;

  /// Mutex protecting shared sensor data
  std::mutex data_mutex_;
};

}  // namespace sensor_touch_abstraction
