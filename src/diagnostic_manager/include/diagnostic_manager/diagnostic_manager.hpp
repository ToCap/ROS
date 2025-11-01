#pragma once

#include <string>
#include <unordered_map>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace diagnostic_manager
{

enum class EventStatus 
{
  PASSED,
  FAILED,
  PREPASSED,
  PREFAILED,
};


/**
 * @brief Aggregates diagnostic information from other  ROS 2 modules
 *
 * The DiagnosticManager collects individual module statuses and computes a
 * global system health summary. It is designed to be used by a higher-level
 * DiagnosticManagerNode.
 */
class DiagnosticManager
{
public:
  /**
   * @brief Structure representing a single module's diagnostic status
   */
  struct ModuleStatus 
  {
    int8_t level;              ///< Diagnostic level (0=OK, 1=WARN, 2=ERROR)
    std::string message;       ///< Description of the current state
    rclcpp::Time last_update;  ///< Timestamp of last received update
  };

  /**
   * @brief Construct a new DiagnosticManager instance.
   * @param clock Shared pointer to the ROS clock.
   */
  explicit DiagnosticManager(rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Update a module's diagnostic status.
   * @param status Diagnostic status message from the module.
   */
  void update_status(const diagnostic_msgs::msg::DiagnosticStatus &status);

  /**
   * @brief Compute the overall system diagnostic summary.
   * @return A DiagnosticStatus message representing the system health
   */
  diagnostic_msgs::msg::DiagnosticStatus compute_system_status() const;

  /**
   * @brief Retrieve all module statuses currently stored.
   * @return A map of module name and status
   */
  std::unordered_map<std::string, ModuleStatus> get_all_statuses() const;

private:
  rclcpp::Clock::SharedPtr clock_;
  std::unordered_map<std::string, ModuleStatus> module_status_;
};
}
