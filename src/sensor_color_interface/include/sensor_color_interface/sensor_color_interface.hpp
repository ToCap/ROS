#ifndef SENSOR_COLOR_INTERFACE_HPP_
#define SENSOR_COLOR_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/handle.hpp>

namespace sensor_color_interface
{

/**
 * @class SensorColorInterface
 * @brief Minimal Lego Mindstorm EV3 color sensor interface for ROS 2 control
 *
 * This class implements a SystemInterface to integrate the Lego Mindstorm EV3 Color Sensor
 * with ros2_control. The interface exposes read-only state data for:
 * - Color ID
 * - Reflected light intensity
 * - Ambient light intensity
 * - RGB components (Red, Green, Blue)
 *
 * The implementation can be connected to the sensor via I2C or system file interface.
 * You can configure it through parameters in the URDF or YAML configuration.
 */
class SensorColorInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorColorInterface);

  /**
   * @brief Default constructor.
   */
  SensorColorInterface() = default;

  /**
   * @brief Default destructor.
   */
  ~SensorColorInterface() override = default;

  /**
   * @brief Initialize hardware with configuration parameters.
   * 
   * @param info Hardware information loaded from the URDF or YAML file.
   * @return hardware_interface::CallbackReturn SUCCESS if initialization succeeded, ERROR otherwise.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Configure the sensor before activation.
   * 
   * @param previous_state Lifecycle state before entering "configure".
   * @return hardware_interface::CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate the hardware interface (open device or start communication).
   * 
   * @param previous_state Lifecycle state before entering "activate".
   * @return hardware_interface::CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware interface (close connections).
   * 
   * @param previous_state Lifecycle state before entering "deactivate".
   * @return hardware_interface::CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export all read-only state interfaces provided by this sensor.
   * 
   * @return Vector of hardware_interface::StateInterface containing sensor data.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Periodically read sensor data and update state variables.
   * 
   * @param time Current time.
   * @param period Duration since last read.
   * @return hardware_interface::return_type OK if read succeeded, ERROR otherwise.
   */
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /**
   * @brief Reads the raw data from the EV3 color sensor device.
   * 
   * This function should contain the actual device communication logic
   * (e.g., I2C reads or sysfs file access).
   * 
   * @return true if data was successfully read, false otherwise.
   */
  bool fetch_sensor_data();

  // ---- Internal sensor data ----

  /** @brief Detected color ID (integer value representing color classification). */
  int color_id_;

  /** @brief Reflected light intensity (0–100%). */
  double reflected_intensity_;

  /** @brief Ambient light intensity (0–100%). */
  double ambient_intensity_;

  /** @brief Red color component value. */
  double red_component_;

  /** @brief Green color component value. */
  double green_component_;

  /** @brief Blue color component value. */
  double blue_component_;

  // ---- Configuration parameters ----

  /** @brief Path to the device (e.g., "/dev/i2c-1" or sysfs node). */
  std::string device_path_;

  /** @brief Port number where the sensor is connected (e.g., input port 1). */
  int port_number_;

  /** @brief Operating mode of the sensor ("COL-COLOR", "COL-REFLECT", "COL-AMBIENT", "COL-RGB"). */
  std::string mode_;

  /** @brief File descriptor or communication handle. */
  int device_fd_;
};

}  // namespace sensor_color_interface

#endif  // SENSOR_COLOR_INTERFACE_HPP_
