#include "sensor_gyro_interface/sensor_gyro_interface.hpp"
#include <chrono>
#include <cmath>

namespace sensor_gyro_interface
{

hardware_interface::CallbackReturn SensorGyroInterface::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // initialize sensor default value
  gyro_angle_ = 0.0;
  gyro_rate_ = 0.0;
  simulate_angle_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("SensorGyroInterface"), "SensorGyroInterface initialized successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SensorGyroInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface("gyro", "angle", &gyro_angle_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("gyro", "rate", &gyro_rate_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SensorGyroInterface::export_command_interfaces()
{
  return {};
}

hardware_interface::CallbackReturn SensorGyroInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorGyroInterface"), "SensorGyroInterface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorGyroInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorGyroInterface"), "SensorGyroInterface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SensorGyroInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
  // Simuler un mouvement de rotation pour test
  double dt = period.seconds();
  gyro_rate_ = 45.0 * std::sin(simulate_angle_ / 180.0 * M_PI);  //degree/second
  simulate_angle_ += gyro_rate_ * dt;
  gyro_angle_ = std::fmod(simulate_angle_, 360.0);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorGyroInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // Nothing to write for a sensor
    return hardware_interface::return_type::OK;
}

}  // namespace sensor_gyro_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sensor_gyro_interface::SensorGyroInterface, hardware_interface::SystemInterface)
