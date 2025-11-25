#include "sensor_touch_interface/sensor_touch_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace sensor_touch_interface
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;


CallbackReturn SensorTouchInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Read configuration from hardware parameters
  if (info_.hardware_parameters.find("sim_topic") != info_.hardware_parameters.end())
  {
    sim_topic_ = info_.hardware_parameters.at("sim_topic");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("SensorTouchInterface"), "Missing parameter 'sim_topic' in hardware config");
    return CallbackReturn::ERROR;
  }

  // initialize sensor default value
  touch_state_ = 0.0;

  return CallbackReturn::SUCCESS;
}


CallbackReturn SensorTouchInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("sensor_touch_interface");

  RCLCPP_INFO(node_->get_logger(), "Configuration of touch sensor interface. Subscribing to %s", sim_topic_.c_str());
  return CallbackReturn::SUCCESS;
}


CallbackReturn SensorTouchInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Activation of touch sensor interface");
  return CallbackReturn::SUCCESS;
}


CallbackReturn SensorTouchInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Deactivation of touch sensor interface");
  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> SensorTouchInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.sensors[0].name,
      hardware_interface::HW_IF_POSITION,  // on utilise "position" pour un capteur booléen simple
      &touch_state_));
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> SensorTouchInterface::export_command_interfaces()
{
  return {};
}


return_type SensorTouchInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return return_type::OK;
}


return_type SensorTouchInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return return_type::OK;
}

}  // namespace sensor_touch_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sensor_touch_interface::SensorTouchInterface, hardware_interface::SystemInterface)
