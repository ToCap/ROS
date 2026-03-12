#pragma once

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace large_motor_interface
{

/**
 * @class LargeMotorInterface
 * @brief Hardware interface for a single LEGO Mindstorms EV3 Large Motor.
 *
 * Implements all five native EV3 Large Motor block modes, mapped to
 * ROS 2 Control command interfaces:
 *
 * | EV3 Mode          | Trigger condition                                  |
 * |-------------------|----------------------------------------------------|
 * | On                | power != 0, no seconds / degrees / rotations target|
 * | Off               | power == 0  (brake_cmd_ controls hold vs coast)    |
 * | On for Seconds    | seconds_cmd_ > 0                                   |
 * | On for Degrees    | |degrees_cmd_| > dead-band                         |
 * | On for Rotations  | |rotations_cmd_| > ε  (converted to degrees × 360) |
 *
 * Priority order when multiple commands are set simultaneously:
 *   On-for-Degrees  >  On-for-Rotations  >  On-for-Seconds  >  On  >  Off
 *
 * Hardware specifications (EV3 Large Motor):
 *   - Encoder resolution : 1 degree per tick
 *   - No-load speed      : ~160–170 RPM  (~16.8–17.8 rad/s)
 *   - Stall torque       : ~40 N·cm
 *   - Power range        : -100 to 100 (%)
 *
 * @note Simulation only — no real I/O is performed.
 *
 * @par State interfaces  (joint_name / type)
 *   - <joint> / position  [rad]    Integrated shaft angle
 *   - <joint> / velocity  [rad/s]  Current angular speed
 *   - <joint> / effort    [%]      Power level currently applied
 *
 * @par Command interfaces  (hardware_name / type)
 *   - <hw> / effort      [-100..100]   Power percentage
 *   - <hw> / seconds     [s ≥ 0]       Duration  (On-for-Seconds mode)
 *   - <hw> / degrees     [°, any]      Angular displacement (On-for-Degrees)
 *   - <hw> / rotations   [rev, any]    Angular displacement (On-for-Rotations)
 *   - <hw> / brake       [0|1]         Brake-at-end (1 = hold, 0 = coast)
 *
 * @par Hardware parameters  (URDF <hardware_parameters>)
 *   - "joint_name"  string   Name of the single motor joint   [required]
 *   - "max_rpm"     double   No-load speed in RPM             [default: 165.0]
 *   - "invert"      bool     Invert motor direction           [default: false]
 */
class LargeMotorInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LargeMotorInterface)

  /**
   * @brief Initializes the interface from URDF / ros2_control parameters.
   *
   * Validates joint count, reads hardware parameters, and zeroes all state
   * and command variables.
   *
   * @param info  Hardware description parsed from the URDF.
   * @return CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Exports state interfaces (position, velocity, effort).
   * @return Vector of StateInterface handles bound to internal variables.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Exports command interfaces (effort, seconds, degrees, rotations, brake).
   * @return Vector of CommandInterface handles bound to internal variables.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Updates internal state from simulated encoder feedback.
   *
   * Integration step (Euler forward):
   *   position(t+Δt) = position(t) + velocity(t) * Δt
   *
   * Also advances the elapsed-time counter used by On-for-Seconds mode:
   *   elapsed_time_ += Δt   (only while the motor is running)
   *
   * @param time    Current ROS time (unused in simulation).
   * @param period  Control loop period Δt.
   * @return return_type::OK always.
   */
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Applies commands to the simulated motor.
   *
   * Mode resolution (evaluated in priority order):
   *
   * 1. **On-for-Degrees** (|degrees_cmd_| > dead-band):
   *      velocity = (power / 100) * max_velocity * direction_sign
   *      Stop when |position_deg_| >= |degrees_cmd_|, then brake or coast.
   *
   * 2. **On-for-Rotations** (|rotations_cmd_| > ε, degrees_cmd_ == 0):
   *      Converted: effective_degrees = rotations_cmd_ * 360
   *      Same logic as On-for-Degrees afterward.
   *
   * 3. **On-for-Seconds** (seconds_cmd_ > 0):
   *      velocity = (power / 100) * max_velocity * direction_sign
   *      Stop when elapsed_time_ >= seconds_cmd_, then brake or coast.
   *
   * 4. **On** (power != 0, no timed or positional target active):
   *      velocity = (power / 100) * max_velocity * direction_sign
   *      Runs indefinitely.
   *
   * 5. **Off** (power == 0):
   *      brake_cmd_ == 1  →  velocity = 0, motor held (active brake)
   *      brake_cmd_ == 0  →  velocity = 0, motor coasts (sim: instant stop)
   *
   * @param time    Current ROS time (unused).
   * @param period  Control loop period (unused — time tracking is in read()).
   * @return return_type::OK always.
   */
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Joint configuration ──────────────────────────────────────────────────
  std::string joint_name_;            ///< Joint name from URDF.

  // ── State variables ──────────────────────────────────────────────────────
  double position_;                   ///< [rad]   Integrated shaft angle.
  double velocity_;                   ///< [rad/s] Current angular velocity.
  double effort_;                     ///< [%]     Power level currently applied.

  // ── Command variables (all doubles — safe for ros2_control interfaces) ───
  double power_cmd_;                  ///< [-100..100] Power percentage.
  double seconds_cmd_;                ///< [s ≥ 0]     Duration for On-for-Seconds.
  double degrees_cmd_;                ///< [°]          Target displacement for On-for-Degrees.
  double rotations_cmd_;              ///< [rev]        Target displacement for On-for-Rotations.
  double brake_cmd_;                  ///< [0|1]        Brake-at-end (1 = hold, 0 = coast).

  // ── Internal tracking ────────────────────────────────────────────────────
  double position_deg_;               ///< [°] Displacement since last positional target reset.
  double target_degrees_;             ///< [°] Resolved positional target (degrees or rotations).
  double elapsed_time_;               ///< [s] Time elapsed since On-for-Seconds started.
  bool   target_reached_;             ///< True once any active target (position/time) is met.

  // Cached previous commands — used to detect a fresh target in write().
  double prev_degrees_cmd_;           ///< Previous degrees_cmd_ value.
  double prev_rotations_cmd_;         ///< Previous rotations_cmd_ value.
  double prev_seconds_cmd_;           ///< Previous seconds_cmd_ value.

  // ── Configuration parameters ─────────────────────────────────────────────
  double max_velocity_;               ///< [rad/s] No-load angular speed.
  double direction_sign_;             ///< +1.0 normal, -1.0 inverted.
};

} // namespace large_motor_interface
