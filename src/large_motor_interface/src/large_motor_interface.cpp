#include "large_motor_interface/large_motor_interface.hpp"
#include <cmath>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>

namespace large_motor_interface
{

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

/// Default no-load speed for an EV3 Large Motor [RPM].
static constexpr double kDefaultMaxRpm = 165.0;

/// RPM → rad/s
static constexpr double kRpmToRadS = 2.0 * M_PI / 60.0;

/// degrees → radians
static constexpr double kDegToRad = M_PI / 180.0;

/// Power dead-band: values within ±kPowerDeadBand are treated as zero [%].
static constexpr double kPowerDeadBand = 0.5;

/// Degrees dead-band: |degrees_cmd_| below this is treated as "no target" [°].
static constexpr double kDegreeDeadBand = 0.5;

/// Rotations epsilon: |rotations_cmd_| below this is treated as zero [rev].
static constexpr double kRotationsEps = 1e-6;

/// Seconds epsilon: seconds_cmd_ below this is treated as zero [s].
static constexpr double kSecondsEps = 1e-6;

// ─────────────────────────────────────────────────────────────────────────────
// on_init
// ─────────────────────────────────────────────────────────────────────────────

CallbackReturn LargeMotorInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // ── Validate joint count ──────────────────────────────────────────────────
  if (info.joints.size() != 1u)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("LargeMotorInterface"),
      "Expected exactly 1 joint, got %zu. Check your URDF.",
      info.joints.size());
    return CallbackReturn::ERROR;
  }

  joint_name_ = info.joints[0].name;

  // ── Hardware parameters ───────────────────────────────────────────────────
  double max_rpm = kDefaultMaxRpm;
  if (info.hardware_parameters.count("max_rpm"))
  {
    max_rpm = std::stod(info.hardware_parameters.at("max_rpm"));
    if (max_rpm <= 0.0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("LargeMotorInterface"),
                   "Parameter 'max_rpm' must be > 0 (got %.2f).", max_rpm);
      return CallbackReturn::ERROR;
    }
  }
  max_velocity_ = max_rpm * kRpmToRadS;

  direction_sign_ = 1.0;
  if (info.hardware_parameters.count("invert"))
  {
    const std::string & inv = info.hardware_parameters.at("invert");
    if (inv == "true" || inv == "1")
    {
      direction_sign_ = -1.0;
    }
  }

  // ── State variables ───────────────────────────────────────────────────────
  position_  = 0.0;
  velocity_  = 0.0;
  effort_    = 0.0;

  // ── Command variables ─────────────────────────────────────────────────────
  power_cmd_     = 0.0;
  seconds_cmd_   = 0.0;
  degrees_cmd_   = 0.0;
  rotations_cmd_ = 0.0;
  brake_cmd_     = 0.0;

  // ── Internal tracking ─────────────────────────────────────────────────────
  position_deg_      = 0.0;
  target_degrees_    = 0.0;
  elapsed_time_      = 0.0;
  target_reached_    = false;

  prev_degrees_cmd_   = 0.0;
  prev_rotations_cmd_ = 0.0;
  prev_seconds_cmd_   = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LargeMotorInterface"),
    "LargeMotorInterface initialized — joint: '%s', max_velocity: %.2f rad/s (%.1f RPM), "
    "direction: %s",
    joint_name_.c_str(), max_velocity_, max_rpm,
    (direction_sign_ > 0.0) ? "normal" : "inverted");

  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
// ─────────────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
LargeMotorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;

  si.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &position_);
  si.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY, &velocity_);
  si.emplace_back(joint_name_, hardware_interface::HW_IF_EFFORT,   &effort_);

  return si;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
LargeMotorInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;

  // Power [-100..100] — named "effort" to align with ros2_control conventions
  ci.emplace_back(get_name(), hardware_interface::HW_IF_EFFORT, &power_cmd_);

  // On-for-Seconds: duration [s]
  ci.emplace_back(get_name(), "seconds",   &seconds_cmd_);

  // On-for-Degrees: angular displacement [°]
  ci.emplace_back(get_name(), "degrees",   &degrees_cmd_);

  // On-for-Rotations: displacement [rev] (1 rev = 360 °)
  ci.emplace_back(get_name(), "rotations", &rotations_cmd_);

  // Brake-at-end: 0 = coast, 1 = active brake
  ci.emplace_back(get_name(), "brake",     &brake_cmd_);

  return ci;
}

// ─────────────────────────────────────────────────────────────────────────────
// read  — simulated encoder + elapsed-time update
// ─────────────────────────────────────────────────────────────────────────────

return_type LargeMotorInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  /*
   *  +-----------------------+        +-----------------------+
   *  |   ROS 2 Controller    |        |  LargeMotorInterface  |
   *  |                       |<-------| read()                |
   *  |                       | pos    |  position [rad]       |
   *  |                       | vel    |  velocity [rad/s]     |
   *  |                       | eff    |  effort   [%]         |
   *  +-----------------------+        +-----------------------+
   */
  const double dt = period.seconds();

  // Integrate shaft angle (rad and degrees kept in sync)
  position_     += velocity_ * dt;
  position_deg_ += (velocity_ / kDegToRad) * dt;

  // Advance time counter for On-for-Seconds (only while running)
  if (std::abs(velocity_) > 0.0)
  {
    elapsed_time_ += dt;
  }

  // Effort state mirrors the last commanded power
  effort_ = power_cmd_;

  return return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write  — EV3 Large Motor block logic
// ─────────────────────────────────────────────────────────────────────────────

return_type LargeMotorInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  /*
   *  +-----------------------+        +-----------------------+
   *  |   ROS 2 Controller    |------->| write()               |
   *  |                       | power  |  power_cmd_   [%]     |
   *  |                       | secs   |  seconds_cmd_ [s]     |
   *  |                       | deg    |  degrees_cmd_ [°]     |
   *  |                       | rot    |  rotations_cmd_ [rev] |
   *  |                       | brake  |  brake_cmd_   [0|1]   |
   *  +-----------------------+        +-----------------------+
   */

  const double clamped_power = std::clamp(power_cmd_, -100.0, 100.0);
  const bool   power_is_zero = (std::abs(clamped_power) < kPowerDeadBand);

  // Base velocity magnitude (direction applied later per mode)
  const double base_velocity = (clamped_power / 100.0) * max_velocity_ * direction_sign_;

  // ── Helper: stop motor (brake or coast) ───────────────────────────────────
  auto stop_motor = [&]() {
    velocity_       = 0.0;
    target_reached_ = true;
  };

  // ═══════════════════════════════════════════════════════════════════════════
  // Mode 1 — On-for-Degrees  (highest priority)
  // ═══════════════════════════════════════════════════════════════════════════
  if (std::abs(degrees_cmd_) > kDegreeDeadBand)
  {
    // Detect a new target
    if (std::abs(degrees_cmd_ - prev_degrees_cmd_) > kDegreeDeadBand)
    {
      position_deg_   = 0.0;
      target_reached_ = false;
      target_degrees_ = degrees_cmd_;
      prev_degrees_cmd_ = degrees_cmd_;
    }

    if (power_is_zero)
    {
      stop_motor();
      return return_type::OK;
    }

    if (!target_reached_)
    {
      if (std::abs(position_deg_) < std::abs(target_degrees_))
      {
        // Still moving toward target — direction follows sign of degrees_cmd_
        velocity_ = std::copysign(std::abs(base_velocity), target_degrees_);
      }
      else
      {
        // Target reached
        stop_motor();
        RCLCPP_DEBUG(rclcpp::get_logger("LargeMotorInterface"),
                     "[On-for-Degrees] Target reached: %.1f° / %.1f°",
                     position_deg_, target_degrees_);
      }
    }
    else
    {
      velocity_ = 0.0;  // hold or coast — both map to 0 in simulation
    }
    return return_type::OK;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Mode 2 — On-for-Rotations  (converted to degrees internally)
  // ═══════════════════════════════════════════════════════════════════════════
  if (std::abs(rotations_cmd_) > kRotationsEps)
  {
    // Detect a new target
    if (std::abs(rotations_cmd_ - prev_rotations_cmd_) > kRotationsEps)
    {
      position_deg_     = 0.0;
      target_reached_   = false;
      target_degrees_   = rotations_cmd_ * 360.0;  // 1 rotation = 360 degrees
      prev_rotations_cmd_ = rotations_cmd_;
    }

    if (power_is_zero)
    {
      stop_motor();
      return return_type::OK;
    }

    if (!target_reached_)
    {
      if (std::abs(position_deg_) < std::abs(target_degrees_))
      {
        velocity_ = std::copysign(std::abs(base_velocity), target_degrees_);
      }
      else
      {
        stop_motor();
        RCLCPP_DEBUG(rclcpp::get_logger("LargeMotorInterface"),
                     "[On-for-Rotations] Target reached: %.1f° / %.1f° (%.2f rev)",
                     position_deg_, target_degrees_, rotations_cmd_);
      }
    }
    else
    {
      velocity_ = 0.0;
    }
    return return_type::OK;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Mode 3 — On-for-Seconds
  // ═══════════════════════════════════════════════════════════════════════════
  if (seconds_cmd_ > kSecondsEps)
  {
    // Detect a new duration target
    if (std::abs(seconds_cmd_ - prev_seconds_cmd_) > kSecondsEps)
    {
      elapsed_time_    = 0.0;
      target_reached_  = false;
      prev_seconds_cmd_ = seconds_cmd_;
    }

    if (power_is_zero)
    {
      stop_motor();
      return return_type::OK;
    }

    if (!target_reached_)
    {
      if (elapsed_time_ < seconds_cmd_)
      {
        velocity_ = base_velocity;
      }
      else
      {
        stop_motor();
        RCLCPP_DEBUG(rclcpp::get_logger("LargeMotorInterface"),
                     "[On-for-Seconds] Duration reached: %.3f s / %.3f s",
                     elapsed_time_, seconds_cmd_);
      }
    }
    else
    {
      velocity_ = 0.0;
    }
    return return_type::OK;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Mode 4 — On  (continuous run)
  // ═══════════════════════════════════════════════════════════════════════════
  if (!power_is_zero)
  {
    velocity_       = base_velocity;
    target_reached_ = false;
    return return_type::OK;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Mode 5 — Off  (power == 0)
  //   brake_cmd_ == 1  →  active brake: hold shaft in position
  //   brake_cmd_ == 0  →  coast: release motor (simulation: instant stop)
  // ═══════════════════════════════════════════════════════════════════════════
  velocity_ = 0.0;  // both brake and coast result in velocity = 0 in simulation

  return return_type::OK;
}

} // namespace large_motor_interface

// ── Plugin export ─────────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  large_motor_interface::LargeMotorInterface,
  hardware_interface::SystemInterface)
