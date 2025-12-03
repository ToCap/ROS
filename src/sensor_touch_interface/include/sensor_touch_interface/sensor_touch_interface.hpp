#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp_lifecycle/state.hpp"

namespace sensor_touch_interface
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
class SensorTouchInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorTouchInterface)

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
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Lifecycle transition: Configure the node.
   * Allocates and initializes the map and publisher.
   * 
   * @param state Current lifecycle state.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle transition: Activate the node.
   * Activates the publisher and publishes initial map.
   * 
   * @param state Current lifecycle state.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle transition: Deactivate the node.
   * Deactivates the publisher
   * 
   * @param state Current lifecycle state.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export the available state interfaces for this hardware.
   *
   * For this sensor, a single state interface is provided:
   * - "touch state": the current status (true or false)
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
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Write commands to the simulated hardware
   *
   * Since this is a passive sensor, no commands are sent to the hardware.
   *
   * @param time The current ROS time.
   * @param period The duration since the last write.
   * @return return_type::OK.
   */
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double touch_state_ = 0.0;/// Latest measured status 

  /// Name of the topic used to receive simulated distance data
  std::string sim_topic_;

  /// ROS 2 node that handles the subscription to the simulated topic
  rclcpp::Node::SharedPtr node_;

  /// Subscription to the simulated distance topic
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  std::mutex state_mutex_;
};

}  // namespace sensor_touch_interface
