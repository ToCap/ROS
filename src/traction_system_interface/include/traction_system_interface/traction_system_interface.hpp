#pragma once

#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace traction_system_interface
{

/**
 * @class TractionSystemInterface
 * @brief Hardware interface implementation for a traction system with up to two motors.
 *
 * This class provides a ROS 2 hardware interface for controlling and monitoring
 * a traction system composed of two standard Mindstorm motors (EV2 or EV3). It supports initialization,
 * state and command interface exports, and data exchange between ROS 2 Control
 * and the physical hardware - only for simulation.
 *
 * The interface manages both state feedback (position, velocity) and command
 * signals (power, steering, position, brake).
 */
class TractionSystemInterface : public hardware_interface::SystemInterface
{
public:
  /// Maximum number of motors supported by this interface.
  static constexpr unsigned int kMaximumMotorCount = 2u;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TractionSystemInterface)

  /**
   * @brief Initializes the hardware interface from the provided hardware information.
   *
   * This function reads configuration parameters from the `HardwareInfo` structure
   * provided by the ROS 2 Control system and sets up the internal state and command
   * variables accordingly.
   *
   * @param info Hardware information containing configuration and parameters.
   * @return CallbackReturn::SUCCESS if initialization succeeded,
   *         CallbackReturn::ERROR otherwise.
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Exports the state interfaces to ROS 2 Control.
   *
   * This function provides access to the hardware state variables (e.g., wheel
   * positions and velocities) that can be read by controllers.
   *
   * @return A vector of `hardware_interface::StateInterface` objects representing
   *         the hardware states.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Exports the command interfaces to ROS 2 Control.
   *
   * This function provides the command handles (e.g., power, steering, position, brake)
   * that can be written to by controllers.
   *
   * @return A vector of `hardware_interface::CommandInterface` objects representing
   *         the hardware commands.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Reads data from the physical hardware.
   *
   * This method updates the internal position and velocity estimates of the wheels.
   * In a real robot, this would read encoder data and possibly effort sensors.
   *
   * Mathematical model (simplified):
   *   wheel position(t+Δt) = wheel position(t) + wheel angular velocity * Δt
   *
   * @param time Current ROS time.
   * @param period Duration since the last read operation.
   * @return return_type::OK if the read was successful,
   *         return_type::ERROR otherwise.
   */
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Writes command data to the physical hardware.
   *
   * This function maps ROS 2 control commands (effort, steering, brake, position)
   * to motor-level actions (velocities for left/right wheels).
   *
   * Mathematical model (simplified):
   *   base_velocity = power_cmd_ * max_velocity_
   *   ω_L = base_velocity * (1 - steering_cmd_)
   *   ω_R = base_velocity * (1 + steering_cmd_)
   *
   * Brake logic:
   *   if brake_cmd_ == true → ω_L = ω_R = 0
   *
   * Position control (simplified):
   *   if |q| >= |q_target| → stop motors
   *
   * In a real setup, this would send commands to a motor driver (PWM, CAN, I2C, etc.).
   *
   * @param time Current ROS time.
   * @param period Duration since the last write operation.
   * @return return_type::OK if the write was successful,
   *         return_type::ERROR otherwise.
   */
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- Internal state variables for both motors ---
  double position_[kMaximumMotorCount];   ///< [rad] Wheel positions.
  double velocity_[kMaximumMotorCount];   ///< [rad/s] Wheel angular velocities.

  // --- ROS 2 Control command variables ---
  double power_cmd_;                      ///< [-1.0 .. 1.0] Normalized power command.
  double steering_cmd_;                   ///< [-1.0 .. 1.0] Steering command (left/right).
  double position_cmd_;                   ///< [rad] Angular position target.
  bool brake_cmd_;                        ///< Brake state (true = engaged).

  // --- Configuration parameters ---
  double wheel_radius_;                   ///< [m] Wheel radius.
  double max_velocity_;                   ///< [rad/s] Maximum wheel angular velocity.
};

} // namespace traction_system_interface
