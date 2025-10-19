#pragma once

#include <string>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/logger.hpp>

namespace sensor_gyro_interface
{

class SensorGyroInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorGyroInterface)

  // Lifecycle
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  // State & command interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read/Write
  hardware_interface::return_type read(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  hardware_interface::return_type write(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;  // <-- ADD THIS

private:
  // Variables pour stocker les mesures du gyroscope
  double gyro_angle_{0.0};
  double gyro_rate_{0.0};

  // Simulation d’un capteur EV3 : pourrait être remplacé par un driver réel
  double simulate_angle_{0.0};
};

}  // namespace sensor_gyro_interface
