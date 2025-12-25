#pragma once

/**
 * @file sensor_rangefinder.hpp
 * @brief Lifecycle abstraction node for rangefinder sensors.
 *
 * This node provides a unified rangefinder abstraction over one or more
 * distance sensors (e.g. infrared and ultrasonic).
 *
 * The node automatically adapts to the available sensor streams at runtime
 * and applies a deterministic selection policy to publish a single output.
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <map>
#include <mutex>

namespace sensor_rangefinder
{

/**
 * @brief Supported rangefinder sensor types.
 */
enum class RangefinderType
{
  INFRARED,
  ULTRASONIC
};

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class SensorRangefinder
 * @brief LifecycleNode that abstracts one or more rangefinder sensors.
 *
 * This node acts as a facade over one or more sensor hardware interfaces.
 * It aggregates sensor measurements, publishes diagnostics, and applies
 * application-level rules to provide a safe and consistent rangefinder output.
 *
 * Selection policy:
 *  - Infrared sensor if valid and within range
 *  - Ultrasonic sensor otherwise if valid
 *  - 0.0 if no valid measurement is available
 */
class SensorRangefinder : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options ROS 2 node options
   */
  explicit SensorRangefinder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  /// @brief Callback for ultrasonic distance updates
  void on_ultrasonic_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /// @brief Callback for infrared distance updates
  void on_infrared_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Select and publish the output distance.
   *
   * This method applies the rangefinder selection policy and publishes
   * a single distance value. If no valid measurement is available,
   * a value of 0.0 is published.
   */
  void publish_output();

  /**
   * @brief Generate diagnostic information for the sensors.
   *
   * This function is registered with the diagnostic_updater and is called
   * periodically to publish the current status of all detected rangefinder sensors.
   *
   * @param stat DiagnosticStatusWrapper object used to populate the diagnostic message.
   */
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  /// Subscriptions to sensor topics (may or may not receive data)
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr infrared_sub_;

  /// Lifecycle publisher for the unified rangefinder output
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr range_pub_;

  /// Latest distance measurement for each sensor type
  std::map<RangefinderType, double> distances_;

  /// Diagnostic updater
  diagnostic_updater::Updater diag_updater_;

  /// Mutex protecting shared sensor data
  std::mutex data_mutex_;

  /// Maximum reliable range for the infrared sensor
  double max_infrared_range_{0.7};
};

}  // namespace sensor_rangefinder
