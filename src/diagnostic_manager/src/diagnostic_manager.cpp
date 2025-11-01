#include "diagnostic_manager/diagnostic_manager.hpp"

/**
 * @file diagnostic_aggregator.cpp
 * @brief Implementation of the DiagnosticManager class.
 */
namespace diagnostic_manager
{
DiagnosticManager::DiagnosticManager(rclcpp::Clock::SharedPtr clock)
: clock_(std::move(clock))
{}

void DiagnosticManager::update_status(const diagnostic_msgs::msg::DiagnosticStatus &status)
{


  
  ModuleStatus m;
  m.level = status.level;
  m.message = status.message;
  m.last_update = clock_->now();

  module_status_[status.name] = m;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticManager::compute_system_status() const
{
  diagnostic_msgs::msg::DiagnosticStatus system_status;
  system_status.name = "system_health";
  system_status.hardware_id = "system";
  system_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  system_status.message = "All systems nominal";

  for (const auto &[name, s] : module_status_)
  {
    if (s.level > system_status.level)
    {
      system_status.level = s.level;
      system_status.message = "Issue detected in " + name + ": " + s.message;
    }
  }

  return system_status;
}

std::unordered_map<std::string, DiagnosticManager::ModuleStatus>
DiagnosticManager::get_all_statuses() const
{
  return module_status_;
}

}