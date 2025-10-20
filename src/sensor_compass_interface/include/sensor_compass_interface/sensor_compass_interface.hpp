// File: sensor_compass_interface.hpp
// Description: ROS2 Hardware SystemInterface for a LEGO-style compass sensor (I2C).
// Comments are in English as requested.

#ifndef SENSOR_COMPASS_INTERFACE_HPP_
#define SENSOR_COMPASS_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cstdint>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sensor_compass_interface
{

class SensorCompassInterface : public hardware_interface::SystemInterface
{
public:
  CompassSystemInterface() = default;
  ~CompassSystemInterface() override;

  // Initialize from URDF/hardware info. Called once during driver load.
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Called when bring hardware up for control (configure/start lifecycle)
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // read() should fetch sensor data and populate state interfaces
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // This sensor is read-only; write() is a no-op but must be implemented
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Helper low-level I2C access
  bool i2c_open_();
  void i2c_close_();
  bool i2c_read_registers_(uint8_t reg, uint8_t * buf, size_t len);
  bool init_sensor_();
  double compute_heading_from_raw_(int16_t mx, int16_t my);

  // Parameters read from hardware_info
  std::string i2c_device_ = "/dev/i2c-1"; // default device
  int i2c_address_ = 0x1E; // default for HMC5883L (may vary)
  double poll_hz_ = 50.0; // default polling

  int i2c_fd_ = -1; // file descriptor for /dev/i2c-X

  // Exported state: heading in radians
  double heading_rad_ = 0.0;

  rclcpp::Logger logger_ = rclcpp::get_logger("sensor_compass_interface");
};

} // namespace sensor_compass_interface

#endif 

