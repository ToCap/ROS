#ifndef SENSOR_ACCELERATION_INTERFACE_HPP_
#define SENSOR_ACCELERATION_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/hardware_info.hpp>

namespace sensor_acceleration_interface
{

/**
 * @brief A ROS 2 SystemInterface implementation for the LEGO NXT Accelerometer/Tilt Sensor.
 * 
 * This interface reads acceleration data (X, Y, Z axes) from the sensor via I2C or analog input
 * depending on the hardware configuration, and exposes it to ROS 2 Control as read-only states.
 */
class SensorAccelerationInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorAccelerationInterface);

  /**
   * @brief Initialize hardware info and configuration parameters.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Configure hardware before starting data acquisition.
   */
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Start the hardware interface (e.g., open I2C connection).
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Stop the hardware interface (e.g., close connection).
   */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export the state interfaces for the accelerometer (x, y, z).
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Periodically read sensor data (e.g., over I2C).
   */
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Sensor values (X, Y, Z acceleration)
  double accel_x_;
  double accel_y_;
  double accel_z_;

  // Example parameters
  std::string i2c_device_;
  int i2c_address_;

  // Internal function to read raw data from sensor
  bool read_sensor_data(double &x, double &y, double &z);
};

}  // namespace sensor_acceleration_interface

#endif  // SENSOR_ACCELERATION_INTERFACE_HPP_
