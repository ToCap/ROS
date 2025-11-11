#include "sensor_color_interface/sensor_color_interface.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdexcept>

namespace sensor_color_interface
{

using hardware_interface::CallbackReturn;

CallbackReturn SensorColorInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // Initialize default state values
  color_id_ = 0;
  reflected_intensity_ = 0.0;
  ambient_intensity_ = 0.0;
  red_component_ = 0.0;
  green_component_ = 0.0;
  blue_component_ = 0.0;
  device_fd_ = -1;

  // Load configuration parameters
  if (info.hardware_parameters.count("device_path") == 0 ||
      info.hardware_parameters.count("port_number") == 0 ||
      info.hardware_parameters.count("mode") == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SensorColorInterface"),
                 "Missing required hardware parameters: device_path, port_number, or mode.");
    return CallbackReturn::ERROR;
  }

  device_path_ = info.hardware_parameters.at("device_path");
  port_number_ = std::stoi(info.hardware_parameters.at("port_number"));
  mode_ = info.hardware_parameters.at("mode");

  RCLCPP_INFO(rclcpp::get_logger("SensorColorInterface"),
              "Initialized SensorColorInterface on %s (port %d) mode %s",
              device_path_.c_str(), port_number_, mode_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorColorInterface::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorColorInterface"), "Configuring EV3 Color Sensor...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorColorInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorColorInterface"), "Activating EV3 Color Sensor...");

  // Open the sensor device (example only)
  device_fd_ = open(device_path_.c_str(), O_RDONLY);
  if (device_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SensorColorInterface"),
                 "Failed to open sensor device at %s", device_path_.c_str());
    return CallbackReturn::ERROR;
  }

  // TODO: Implement sensor mode setup (via I2C or sysfs write)

  return CallbackReturn::SUCCESS;
}

CallbackReturn SensorColorInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorColorInterface"), "Deactivating EV3 Color Sensor...");
  if (device_fd_ >= 0)
  {
    close(device_fd_);
    device_fd_ = -1;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SensorColorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  interfaces.emplace_back(hardware_interface::StateInterface("color_id", hardware_interface::HW_IF_POSITION, &color_id_));
  interfaces.emplace_back(hardware_interface::StateInterface("reflected_intensity", hardware_interface::HW_IF_POSITION, &reflected_intensity_));
  interfaces.emplace_back(hardware_interface::StateInterface("ambient_intensity", hardware_interface::HW_IF_POSITION, &ambient_intensity_));
  interfaces.emplace_back(hardware_interface::StateInterface("red", hardware_interface::HW_IF_POSITION, &red_component_));
  interfaces.emplace_back(hardware_interface::StateInterface("green", hardware_interface::HW_IF_POSITION, &green_component_));
  interfaces.emplace_back(hardware_interface::StateInterface("blue", hardware_interface::HW_IF_POSITION, &blue_component_));

  return interfaces;
}

hardware_interface::return_type SensorColorInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!fetch_sensor_data())
  {
    RCLCPP_WARN(rclcpp::get_logger("SensorColorInterface"), "Failed to fetch color sensor data");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool SensorColorInterface::fetch_sensor_data()
{
  // Placeholder implementation for simulation.
  // Replace this with actual sensor reading logic (I2C or sysfs access).
  color_id_ = 5;  // Example: Red
  reflected_intensity_ = 65.0;
  ambient_intensity_ = 22.0;
  red_component_ = 180.0;
  green_component_ = 45.0;
  blue_component_ = 30.0;
  return true;
}

}  // namespace sensor_color_interface
