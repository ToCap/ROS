#include "pose/pose_estimator.hpp"

void PoseEstimator::init(double x0, double y0, double theta_deg)
{
  robot_.x_cm = x0;
  robot_.y_cm = y0;
  robot_.theta_rad = theta_deg * M_PI / 180.0;
  last_left_cnt_ = last_right_cnt_ = 0.0;
}

void PoseEstimator::update()
{
  double left_cnt = getMotorPositionLeft();
  double right_cnt = getMotorPositionRight();

  double d_left_cm = 2 * M_PI * WHEEL_RADIUS_CM * (left_cnt - last_left_cnt_) / COUNTS_PER_ROTATION;
  double d_right_cm = 2 * M_PI * WHEEL_RADIUS_CM * (right_cnt - last_right_cnt_) / COUNTS_PER_ROTATION;

  last_left_cnt_ = left_cnt;
  last_right_cnt_ = right_cnt;

  double d_s = (d_left_cm + d_right_cm) / 2.0;
  int gyro_val = getGyroValue();
  robot_.theta_rad = gyro_val * M_PI / 180.0;

  robot_.x_cm += d_s * cos(robot_.theta_rad);
  robot_.y_cm += d_s * sin(robot_.theta_rad);
}

Pose PoseEstimator::get() const
{
  return robot_;
}

// MOCK - À remplacer avec vraies fonctions capteur
double PoseEstimator::getMotorPositionLeft() { return 0.0; }
double PoseEstimator::getMotorPositionRight() { return 0.0; }
int PoseEstimator::getGyroValue() { return 0; }
