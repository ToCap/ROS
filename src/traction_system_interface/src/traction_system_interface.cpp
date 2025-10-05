#include "traction_system_interface/traction_system_interface.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace traction_system_interface
{

CallbackReturn TractionSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialisation of ROS-related commands
  std::memset(this->position_, 0, sizeof(this->position_));
  std::memset(this->velocity_, 0, sizeof(this->velocity_));


  // Initialisation of Minsdstorm-related commands
  power_cmd_ = 0.0;
  steering_cmd_ = 0.0;
  position_cmd_ = 0.0;
  brake_cmd_ = false;

  // Store configuration parameters (from URDF/YAML)
  wheel_radius_ = stod(info.hardware_parameters.at("wheel_radius"));
  max_velocity_ = stod(info.hardware_parameters.at("max_velocity"));

  RCLCPP_INFO(rclcpp::get_logger("TractionSystemInterface"), 
              "TractionSystemInterface initialized : wheel radius = %f m, max velocity = %f rad/s",
              wheel_radius_, max_velocity_);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TractionSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Declare states
  state_interfaces.emplace_back("left_wheel_joint", "position", &position_[0]);
  state_interfaces.emplace_back("left_wheel_joint", "velocity", &velocity_[0]);
  state_interfaces.emplace_back("right_wheel_joint", "position", &position_[1]);
  state_interfaces.emplace_back("right_wheel_joint", "velocity", &velocity_[1]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TractionSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Declare commands
  command_interfaces.emplace_back(get_name(), "effort", &power_cmd_);     // Power
  command_interfaces.emplace_back(get_name(), "steering", &steering_cmd_); // Steering (custom)
  command_interfaces.emplace_back(get_name(), "position", &position_cmd_); // Position (degrees/rotations)
  command_interfaces.emplace_back(get_name(), "brake", reinterpret_cast<double*>(&brake_cmd_)); // Brake (custom)

  return command_interfaces;
}

return_type TractionSystemInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
/*
     +-----------------------+             +-----------------------+
     |   ROS 2 Controller    |             |   Hardware Interface  |
     | (e.g. diff_drive,     |             |        Mindstorm      |
     |  position, velocity)  |             |                       |
     +----------+------------+             +-----------+-----------+
                |                                      |
                |<-------------------------------------| read()
                | Position, Velocity, Effort, Brake    |
*/

  // Update of ROS2 control for simulation only
  double dt = period.seconds();
  position_[0] += velocity_[0] * dt;
  position_[1] += velocity_[1] * dt;

  return return_type::OK;
}

return_type TractionSystemInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  if (!brake_cmd_)
  {
    // Compute base velocity without impact of steering
    double base_velocity = power_cmd_ * max_velocity_;

    // Compute command applied on each motor to reflect steering command
    double left_factor  = 1.0 - steering_cmd_;
    double right_factor = 1.0 + steering_cmd_;

    velocity_[0] = base_velocity * left_factor;
    velocity_[1] = base_velocity * right_factor;
  }
  else
  {
    // Bloquer les moteurs
    velocity_[0] = 0.0;
    velocity_[1] = 0.0;
  }

  return return_type::OK;
}

} // namespace traction_system_interface

// Export du plugin
PLUGINLIB_EXPORT_CLASS(traction_system_interface::TractionSystemInterface, hardware_interface::SystemInterface)
