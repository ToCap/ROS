
#include "sensor_compass_interface/sensor_compass_interface.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cmath>

namespace sensor_compass_interface
{

SensorCompassInterface::~SensorCompassInterface()
{
  i2c_close_();
}

hardware_interface::CallbackReturn SensorCompassInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  // Read optional parameters from the hardware_info (ros2_control hw description)
  try {
    if (!info.hardware_parameters.empty()) {
      for (auto const & p : info.hardware_parameters) {
        if (p.first == "i2c_device") { i2c_device_ = p.second; }
        if (p.first == "i2c_address") { i2c_address_ = std::stoi(p.second, nullptr, 0); }
        if (p.first == "poll_hz") { poll_hz_ = std::stod(p.second); }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to parse hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Initialized SensorCompassInterface on %s addr=0x%02X poll_hz=%.1f",
              i2c_device_.c_str(), i2c_address_, poll_hz_);

  if (!i2c_open_()) {
    RCLCPP_ERROR(logger_, "Failed to open I2C device %s", i2c_device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!init_sensor_()) {
    RCLCPP_WARN(logger_, "Sensor initialization returned warning/failure; continuing but readings may be invalid");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorCompassInterface::on_configure(const rclcpp_lifecycle::State &)
{
  // Nothing special to do for this simple sensor
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorCompassInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  i2c_close_();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SensorCompassInterface::export_state_interfaces()
{
  // Export a single state interface named "compass/heading" (radians)
  std::vector<hardware_interface::StateInterface> states;
  states.emplace_back(hardware_interface::StateInterface("compass", "heading", &heading_rad_));
  return states;
}

std::vector<hardware_interface::CommandInterface> SensorCompassInterface::export_command_interfaces()
{
  // Read-only sensor: no command interfaces
  return {};
}

hardware_interface::return_type SensorCompassInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Read raw magnetometer values from the sensor registers.
  // Attempt reading 6 bytes (X MSB..LSB, Z MSB..LSB, Y MSB..LSB) which is common for HMC5883L.
  uint8_t buf[6] = {0};
  if (!i2c_read_registers_(0x03, buf, sizeof(buf))) {
    RCLCPP_WARN(logger_, "Failed to read sensor registers");
    return hardware_interface::return_type::ERROR;
  }

  // Convert to signed int16 values (big-endian from many compass sensors)
  int16_t mx = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t mz = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t my = (int16_t)((buf[4] << 8) | buf[5]);

  // Compute heading from X and Y (ignore Z for 2D heading)
  heading_rad_ = compute_heading_from_raw_(mx, my);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorCompassInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // No writable commands for a passive compass sensor
  return hardware_interface::return_type::OK;
}

bool SensorCompassInterface::i2c_open_()
{
  if (i2c_fd_ >= 0) return true; // already open

  i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(logger_, "Could not open I2C device %s: %s", i2c_device_.c_str(), strerror(errno));
    return false;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
    RCLCPP_ERROR(logger_, "Failed to set I2C slave address 0x%02X: %s", i2c_address_, strerror(errno));
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  return true;
}

void SensorCompassInterface::i2c_close_()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
}

bool SensorCompassInterface::i2c_read_registers_(uint8_t reg, uint8_t * buf, size_t len)
{
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(logger_, "I2C device not opened");
    return false;
  }

  // Write register address
  if (write(i2c_fd_, &reg, 1) != 1) {
    RCLCPP_DEBUG(logger_, "Failed to write register address 0x%02X: %s", reg, strerror(errno));
    return false;
  }

  ssize_t r = read(i2c_fd_, buf, len);
  if (r != (ssize_t)len) {
    RCLCPP_DEBUG(logger_, "I2C read returned %zd expected %zu: %s", r, len, strerror(errno));
    return false;
  }

  return true;
}

bool SensorCompassInterface::init_sensor_()
{
  // Try to put HMC5883L in continuous measurement mode (0x02 register = 0x00)
  // Many LEGO-compatible compass sensors use HMC5883L or equivalent; setting this
  // mode helps ensure continuous updates. Failure is non-fatal.
  uint8_t mode_reg[2] = {0x02, 0x00};
  if (i2c_fd_ < 0) return false;
  if (write(i2c_fd_, mode_reg, 2) != 2) {
    RCLCPP_DEBUG(logger_, "Failed to write mode register; sensor may use different chip or address");
    return false;
  }
  // Small delay for mode to apply
  usleep(1000);
  return true;
}

double SensorCompassInterface::compute_heading_from_raw_(int16_t mx, int16_t my)
{
  // Compute heading using atan2(y, x). Return radians in range [-pi, +pi].
  double xd = static_cast<double>(mx);
  double yd = static_cast<double>(my);
  double heading = std::atan2(yd, xd);
  // Normalize to [0, 2*pi)
  if (heading < 0) heading += 2.0 * M_PI;
  return heading;
}

} // namespace sensor_compass_interface

// Register as a plugin for ros2_control using pluginlib macro (KT: user must add pluginlib manifest)
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sensor_compass_interface::SensorCompassInterface, hardware_interface::SystemInterface)

