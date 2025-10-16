// collision_detection_system.hpp
#pragma once

#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "param_geometry.hpp"

namespace collision_detection_system {

/**
 * @brief Algorithm class for detecting obstacle presence based on bumper sensors.
 *
 * This class performs data fusion between two bumpers (left/right),
 * computing obstacle presence, position, and certainty score.
 */
class CollisionDetectionSystem
{
public:

  enum State
  {
    RELEASED = 0 ,  // input touch sensor is released e.g. not pressed
    PRESSED = 1,    // input touch sensor is pressed
    BUMPED = 2,     // input touch sensor has been pressed and released in the past. The next Bumped occurrence will then require a new press and release

    NUM_STATE_MAX
  };

  static constexpr float_t kValidityScore[NUM_STATE_MAX] = {1.0, 1.0, 0.5};

  struct SensorState
  {
    bool state_bool = false;      // True if pressed, false otherwise
    int measured_value = 0;       // 0=Released, 1=Pressed, 2=Bumped
    rclcpp::Time last_update;
  };

  /**
   * @struct ObstacleInfo
   * @brief Represents an obstacle area with position, size, and metadata.
   */
  struct ObstacleInfo
  {
    double origin_x;        ///< X-coordinate of the rectangle's origin (bottom-left corner) in robot coordinates
    double origin_y;        ///< Y-coordinate of the rectangle's origin (bottom-left corner) in robot coordinates
    double length;          ///< Length of the rectangle along the X-axis.
    double width;           ///< Width of the rectangle along the Y-axis.
    double validity_score;  ///< score indicating confidence of obstacle presence
    std::string source;     ///< Source identifier of the data (e.g., sensor name or module)
    rclcpp::Time stamp;     ///< Timestamp associated with the rectangle data
  };

  CollisionDetectionSystem();
  ~CollisionDetectionSystem() = default;

  // Thread-safe input setters
  void set_left_state(bool value, const rclcpp::Time & time);
  void set_left_measured(int value, const rclcpp::Time & time);
  void set_right_state(bool value, const rclcpp::Time & time);
  void set_right_measured(int value, const rclcpp::Time & time);

  // Compute obstacle detection result
  ObstacleInfo compute();

  /**
   * @brief Configure the collision detection algorithm with the provided parameters
   *
   * This method sets up the algorithm using the values defined in the
   * CollisionConfig structure. The algorithm does not access ROS parameters
   * directly; it relies entirely on the configuration provided by the caller.
   *
   * @param cfg A reference to a CollisionConfig structure containing all necessary
   *            configuration parameters such as topic names and thresholds.
   */
  void load_configuration(const Param_Geometry_t &cfg);

private:
  std::mutex mutex_;
  SensorState left_;
  SensorState right_;
  Param_Geometry_t prm_geometry_;

  static double evaluate_validity(int measured_value);
};
}