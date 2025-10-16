// collision_detection_system.cpp
#include "collision_detection_system/collision_detection_system.hpp"
#include <algorithm>

using namespace collision_detection_system;

CollisionDetectionSystem::CollisionDetectionSystem() = default;

void CollisionDetectionSystem::set_left_state(bool value, const rclcpp::Time & time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  left_.state_bool = value;
  left_.last_update = time;
}

void CollisionDetectionSystem::set_left_measured(int value, const rclcpp::Time & time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  left_.measured_value = value;
  left_.last_update = time;
}

void CollisionDetectionSystem::set_right_state(bool value, const rclcpp::Time & time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  right_.state_bool = value;
  right_.last_update = time;
}

void CollisionDetectionSystem::set_right_measured(int value, const rclcpp::Time & time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  right_.measured_value = value;
  right_.last_update = time;
}

double CollisionDetectionSystem::evaluate_validity(int measured_value)
{
    // declaration of local variables
    double score = 0.0;

    if (measured_value < NUM_STATE_MAX)
    {
        score = kValidityScore[measured_value];
    }

    return score;
}

CollisionDetectionSystem::ObstacleInfo CollisionDetectionSystem::compute()
{
  std::lock_guard<std::mutex> lock(mutex_);

  ObstacleInfo info;
  info.stamp = rclcpp::Clock().now();

  const double left_x = -0.2;
  const double right_x = 0.2;
  const double y = 0.0;

  double left_cert = evaluate_validity(left_.measured_value);
  double right_cert =  evaluate_validity(right_.measured_value);

  // Slightly increase validity_score if boolean state is true
  if (left_.state_bool) left_cert = std::max(left_cert, 0.5);
  if (right_.state_bool) right_cert = std::max(right_cert, 0.5);

  const double threshold = 0.25;
  bool left_detected = left_cert > threshold;
  bool right_detected = right_cert > threshold;

  if (left_detected && right_detected)
  {
    info.origin_x = (left_x + right_x) / 2.0;
    info.origin_y = y;
    info.validity_score = std::min(1.0, (left_cert + right_cert) * 0.6);
    info.source = "both";
  }
  else if (left_detected)
  {
    info.origin_x = left_x;
    info.origin_y = y;
    info.validity_score = left_cert;
    info.source = "left";
  }
  else if (right_detected)
  {
    info.origin_x = right_x;
    info.origin_y = y;
    info.validity_score = right_cert;
    info.source = "right";
  }
  else
  {
    info.origin_x = 0.0;
    info.origin_y = 0.0;
    info.validity_score = 0.0;
    info.source = "none";
  }

  return info;
}
