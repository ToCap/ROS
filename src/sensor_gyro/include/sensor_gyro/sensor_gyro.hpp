#pragma once
/**
 * @file sensor_gyro.hpp
 * @brief Lifecycle abstraction node for configurable gyro sensors.
 *
 * This node provides a unified gyro abstraction over one or more
 * gyro sensor
 * The available sensors are selected at runtime via ROS parameters.
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

namespace sensor_gyro_abstraction
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class SensorGyro
 * @brief LifecycleNode that abstracts one or more gyro sensors.
 *
 * This node acts as a facade over one or more sensor hardware interfaces.
 * It aggregates sensor state information, publishes diagnostics, and applies
 * application-level rules to provide a safe and consistent sensor output.
 */
class SensorGyro : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorGyro)

  /**
   * @brief Constructor
   * @param options NodeOptions for LifecycleNode
   */
  explicit SensorGyro(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // Lifecycle overrides
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  /// @brief Callback for gyro angle updates
  void gyro_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /// @brief Callback for gyro rate updates
  void gyro_rate_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /// @brief Publish diagnostics
  void publish_diagnostics();

  /// @brief Evaluate business rules and overall robot health
  bool check_system_conditions();

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gyro_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gyro_rate_sub_;

  // Latest sensor values
  double gyro_angle_{0.0};
  double gyro_rate_{0.0};

  // Diagnostics updater
  diagnostic_updater::Updater diag_updater_;

  // Diagnostic publisher
  std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_publisher_;
};

}  // namespace sensor_gyro_abstraction
