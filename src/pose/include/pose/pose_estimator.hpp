#ifndef ROBOT_POSE_ESTIMATOR__POSE_ESTIMATOR_HPP_
#define ROBOT_POSE_ESTIMATOR__POSE_ESTIMATOR_HPP_

#include <cmath>

struct Pose
{
  double x_cm;
  double y_cm;
  double theta_rad;
};

class PoseEstimator
{
public:
  void init(double x0, double y0, double theta_deg);
  void update();
  Pose get() const;

private:
  Pose robot_;
  double last_left_cnt_ = 0.0;
  double last_right_cnt_ = 0.0;

  double getMotorPositionLeft();
  double getMotorPositionRight();
  int getGyroValue();

  // Constantes physiques à adapter selon ton robot
  static constexpr double WHEEL_RADIUS_CM = 2.8; // exemple
  static constexpr double COUNTS_PER_ROTATION = 360.0;
};

#endif  // ROBOT_POSE_ESTIMATOR__POSE_ESTIMATOR_HPP_
