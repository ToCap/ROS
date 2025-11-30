#pragma once

#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace sensor_infrared_interface
{

/**
 * @class SensorInfraredInterface
 * @brief Hardware interface implementation for a Lego Mindstorm Infrared sensor (EV3)
 *
 * This class provides a ROS 2 hardware interface for controlling and monitoring
 * a Lego Mindstorm infrared sensor(EV2 or EV3). It supports initialization,
 * state and command interface exports, and data exchange between ROS 2 Control
 * and the physical hardware - only for simulation.
 *
 */
class SensorInfraredInterface : public hardware_interface::SystemInterface
{

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorInfraredInterface)


  ~SensorInfraredInterface();

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
   * @brief Export the available state interfaces for this hardware.
   *
   * For this sensor, a single state interface is provided:
   * - "distance": the current measured distance (in meters).
   *
   * @return A vector containing the state interface(s).
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;


  /**
   * @brief Export the command interfaces for this hardware.
   *
   * The EV3 infrared sensor is considered as passive and does not accept commands,
   * therefore this method returns an empty vector.
   *
   * @return An empty vector of CommandInterface.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Read the latest sensor data from the simulation.
   *
   * In this implementation, the distance value is updated asynchronously
   * through the ROS 2 subscription callback. Therefore, this function simply
   * returns success without performing any blocking operations.
   *
   * @param time The current ROS time.
   * @param period The duration since the last read.
   * @return return_type::OK if the operation succeeded.
   */
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Write commands to the simulated hardware
   *
   * Since this is a passive sensor, no commands are sent to the hardware.
   *
   * @param time The current ROS time.
   * @param period The duration since the last write.
   * @return return_type::OK.
   */
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// ROS 2 node that handles the subscription to the simulated topic
  std::shared_ptr<rclcpp::Node> node_;

  /// Subscription to the simulated distance topic
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

  /// Background thread for spinning the node.
  std::thread executor_thread_;

  /// Name of the topic used to receive simulated distance data
  std::string topic_name_;

private:
  double distance_{0.0};/// Latest measured distance (in meters)
};

} // namespace sensor_infrared
