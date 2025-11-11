#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp_lifecycle/state.hpp"

namespace sensor_touch_interface
{

/**
* @brief Minimal Lego Mindstorm EV3 touch sensor interface for ROS 2 control
*/
class SensorTouchInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorTouchInterface)

  // ───────────────────────────────
  // Méthodes de cycle de vie
  // ───────────────────────────────

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ───────────────────────────────
  // Interfaces d’état et de commande
  // ───────────────────────────────

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ───────────────────────────────
  // Boucle d’exécution principale
  // ───────────────────────────────

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ───────────────────────────────
  // Données internes
  // ───────────────────────────────

  // Valeur booléenne lue depuis la simulation
  double touch_state_ = 0.0;

  // Nom du topic ROS2 simulé (configuré via le .yaml ou hardware_info)
  std::string sim_topic_;

  // Node ROS2 utilisé pour le subscriber
  rclcpp::Node::SharedPtr node_;

  // Subscription au topic de la simulation
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  // Mutex pour protéger l’accès à touch_state_ depuis le callback
  std::mutex state_mutex_;

  // Callback de réception du message simulé
  void sim_callback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace sensor_touch_interface
