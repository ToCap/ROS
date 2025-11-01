#include "diagnostic_manager/diagnostic_manager_node.hpp"

/**
 * @file diagnostic_manager_node.cpp
 * @brief Implementation of the DiagnosticManagerNode class.
 */

namespace diagnostic_manager
{
DiagnosticManagerNode::DiagnosticManagerNode(): Node("diagnostic_manager_node")
{
  manager_ = std::make_unique<DiagnosticManager>(this->get_clock());

  // Subscribe to the /diagnostics topic
  diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    std::bind(&DiagnosticManagerNode::on_diagnostics_msg, this, std::placeholders::_1)
  );

  // Publisher for aggregated system status
  system_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/system_health", 10
  );

  // Timer for periodic publication
  timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&DiagnosticManagerNode::on_timer, this)
  );

  RCLCPP_INFO(this->get_logger(), "Diagnostic Manager Node started");
}

void DiagnosticManagerNode::on_diagnostics_msg(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{

  // Determine status based on topic data
  switch (status.level)
  {
    case DiagnosticStatus::OK:
    {
      RCLCPP_INFO(this->get_logger(), "[%s] OK: %s", status.name.c_str(), status.message.c_str());
    }
    break;

    case DiagnosticStatus::WARN:
    {
      RCLCPP_WARN(this->get_logger(),"[%s] WARNING : %s", status.name.c_str(), status.message.c_str());
    }
    break;

    case DiagnosticStatus::ERROR:
    {
      RCLCPP_ERROR(this->get_logger(), "[%s] ERROR: %s", status.name.c_str(), status.message.c_str());
    }
    break;

    case DiagnosticStatus::STALE:
    {
      RCLCPP_WARN(this->get_logger(), "[%s] TIMEOUT : %s", status.name.c_str(), status.message.c_str());
    }
    break;

    default:
    {
      RCLCPP_WARN(this->get_logger(), "[%s] UNKNOWN (%d): %s", status.name.c_str(), status.level, status.message.c_str());
    }
    break;
  }


  for (auto &status : msg->status)
  {
    this->manager_->update_status(status);
  }
}

void DiagnosticManagerNode::on_timer()
{
  auto system_status = manager_->compute_system_status();
  system_pub_->publish(system_status);

  RCLCPP_INFO(this->get_logger(), "System health: %s (level=%d)",
              system_status.message.c_str(),
              system_status.level);
}
}