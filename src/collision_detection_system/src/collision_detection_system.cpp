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

    // Compute if left or right sensor is active
    bool is_left_active = (left_.state_bool || (left_.measured_value > 0) );
    bool is_right_active = (right_.state_bool || (right_.measured_value > 0) );

    // Compute validity score
    double score = 0.0;
    if (is_left_active)
    {
        score = left_.state_bool ? 1.0 : kValidityScore[left_.measured_value];
    }
    else if (is_right_active)
    {
         score = right_.state_bool ? 1.0 : kValidityScore[right_.measured_value];
    }
    else
    {
        score = 0.0;
    }

    double length = this->prm_geometry_.right_bumper_x - this->prm_geometry_.left_bumper_x;

    // Build boundex box of detected obstacle
    if (is_left_active && is_right_active)
    {
        info.origin_x = this->prm_geometry_.left_bumper_x;
        info.origin_y = this->prm_geometry_.left_bumper_y;
        info.length = length;
        info.width = 0.1;
        info.validity_score = score;
    }
    else if (is_left_active)
    {
        info.origin_x = this->prm_geometry_.left_bumper_x;
        info.origin_y = this->prm_geometry_.left_bumper_y;
        info.length = length / 2;
        info.width = 0.1;
        info.validity_score = score;
    }
    else if (is_right_active)
    {
        info.origin_x = this->prm_geometry_.right_bumper_x;
        info.origin_y = this->prm_geometry_.right_bumper_y;
        info.length = -length/2;
        info.width = 0.1;
        info.validity_score = score;
    }
    else
    {
        info.origin_x = 0.0;
        info.origin_y = 0.0;
        info.length = 0;
        info.width = 0;
        info.validity_score = 0.0;
        info.source = "none";
    }

  return info;
}

void CollisionDetectionSystem::load_configuration(const Param_Geometry_t &prm)
{
    this->prm_geometry_ = prm;  // copy the input geometry related params
}


