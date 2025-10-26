#include "sensor_acceleration_interface/sensor_acceleration_interface.hpp"
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

namespace sensor_acceleration_interface
{

hardware_interface::CallbackReturn SensorAccelerationInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize values
  accel_x_ = 0.0;
  accel_y_ = 0.0;
  accel_z_ = 0.0;

  // Load configuration parameters
  i2c_device_ = info.hardware_parameters.at("i2c_device");
  i2c_address_ = std::stoi(info.hardware_parameters.at("i2c_address"));

  RCLCPP_INFO(rclcpp::get_logger("SensorAccelerationInterface"), "Initialized with device: %s, address: 0x%X",
              i2c_device_.c_str(), i2c_address_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorAccelerationInterface::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorAccelerationInterface"), "Configuring NXT Accelerometer...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorAccelerationInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorAccelerationInterface"), "Activating NXT Accelerometer...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorAccelerationInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorAccelerationInterface"), "Deactivating NXT Accelerometer...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SensorAccelerationInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface("accel_x", hardware_interface::HW_IF_POSITION, &accel_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("accel_y", hardware_interface::HW_IF_POSITION, &accel_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("accel_z", hardware_interface::HW_IF_POSITION, &accel_z_));
  return state_interfaces;
}

hardware_interface::return_type SensorAccelerationInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  double x, y, z;
  if (read_sensor_data(x, y, z))
  {
    accel_x_ = x;
    accel_y_ = y;
    accel_z_ = z;
    return hardware_interface::return_type::OK;
  }
  return hardware_interface::return_type::ERROR;
}

bool SensorAccelerationInterface::read_sensor_data(double &x, double &y, double &z)
{
  // NOTE: This is a simplified example. Replace with actual I2C read logic.
  // Example: open /dev/i2c-1, set slave address, read bytes, convert to g-values.

  static double t = 0.0;
  t += 0.05;
  x = std::sin(t) * 0.5;
  y = std::cos(t) * 0.5;
  z = 1.0;  // simulate gravity along Z
  return true;
}

}  // namespace sensor_acceleration_interface
