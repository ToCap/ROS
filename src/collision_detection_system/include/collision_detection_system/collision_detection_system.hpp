// collision_detection_system.hpp
#pragma once

#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"

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
    bool valid = false;
    bool state_bool = false;      // True if pressed, false otherwise
    int measured_value = 0;       // 0=Released, 1=Pressed, 2=Bumped
    rclcpp::Time last_update;
  };

  struct ObstacleInfo
  {
    bool present = false;
    double x = 0.0;
    double y = 0.0;
    double certainty = 0.0;
    std::string source; // "left", "right", "both", or "none"
    rclcpp::Time stamp;
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

private:
  std::mutex mutex_;
  SensorState left_;
  SensorState right_;

  static double measured_to_certainty(int measured_value);
};
}