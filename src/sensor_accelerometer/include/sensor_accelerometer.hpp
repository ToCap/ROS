#pragma once

/**
 * @file sensor_accelerometer.hpp
 * @brief Lifecycle abstraction node for accelerometer sensors.
 *
 * This node provides a unified acceleromter abstraction over one or more
 * acceleromter sensors.
 * 
 * The node automatically adapts to the available sensor streams at runtime
 * and applies a deterministic selection policy to publish a single output.
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <map>
#include <mutex>

namespace sensor_accelerometer
{

/**
 * @brief Supported accelerometer sensor types.
 */
enum class AccelerometerType
{
  MINDSTORM_EV3
};

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class SensorAccelerometer
 * @brief LifecycleNode that abstracts one or more accelerometer sensors.
 *
 * This node acts as a facade over the accelerometer hardware interface.
 * It aggregates X, Y, Z measurements, publishes diagnostics, and applies
 * application-level rules to provide a consistent output.
 */
class SensorAccelerometer : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options ROS 2 node options
   */
  explicit SensorAccelerometer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  /// @brief Callback for accelerometer data updates
  void accelerometer_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  /**
   * @brief Publish the consolidated accelerometer measurements.
   *
   * This method can apply filtering or validation rules before publishing.
   */
  void publish_output();

  /**
   * @brief Generate diagnostic information for the accelerometer.
   *
   * This function is registered with the diagnostic_updater and called periodically.
   *
   * @param stat DiagnosticStatusWrapper object used to populate the diagnostic message.
   */
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  /// Subscription to raw accelerometer data
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr accel_sub_;

  /// Lifecycle publisher for consolidated acceleration output
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr accel_pub_;

  /// Latest sensor measurements (X, Y, Z) per sensor type
  std::map<AccelerometerType, std::array<double, 3>> measurements_;

  /// Mutex protecting shared data
  std::mutex data_mutex_;

  /// Diagnostic updater
  diagnostic_updater::Updater diag_updater_;
};

}  // namespace sensor_accelerometer
