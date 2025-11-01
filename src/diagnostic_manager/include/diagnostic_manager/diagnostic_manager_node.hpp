#pragma once

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "diagnostic_manager/diagnostic_manager.hpp"

namespace diagnostic_manager
{
/**
 * @brief ROS 2 node responsible for collecting diagnostics and publishing
 *        a system-wide health status.
 *
 * The DiagnosticManagerNode subscribes to the `/diagnostics` topic,
 * aggregates incoming statuses via a DiagnosticManager, and publishes
 * a consolidated `/system_health` status at a fixed rate.
 */
class DiagnosticManagerNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new DiagnosticManagerNode instance.
   */
  DiagnosticManagerNode();

private:
  /**
   * @brief Callback for incoming diagnostic messages.
   * @param msg Pointer to a received DiagnosticArray message.
   */
  void on_diagnostics_msg(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  /**
   * @brief Periodic timer callback to compute and publish system health.
   */
  void on_timer();

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;  ///< Diagnostics subscriber
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr system_pub_;  ///< System health publisher
  rclcpp::TimerBase::SharedPtr timer_;                                               ///< Timer for periodic updates

  std::unique_ptr<DiagnosticManager> manager_;  ///< Aggregator instance
};
}
