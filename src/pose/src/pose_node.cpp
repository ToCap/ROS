#include "pose/pose_node.hpp"

PoseNode::PoseNode(): Node("pose_node")
{
  pose_estimator_.init(0.0, 0.0, 0.0);

  pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10);


  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PoseNode::timer_callback, this));
}

/**
 * @brief Callback triggered upon receiving odometry data.
 * 
 * This function processes the odometry message to update the internal
 * state or map accordingly based on the robot’s movement.
 * 
 * @param msg Shared pointer to the received Odometry message containing
 *            position and velocity information of the robot.
 */
void PoseNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Example: log the received position
    RCLCPP_INFO(get_logger(), "Received odometry: position=(%.2f, %.2f, %.2f)",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);

    // TODO: update map or internal state based on odometry
}

void PoseNode::timer_callback()
{
  pose_estimator_.update();
  Pose pose = pose_estimator_.get();

  // // Log info
  // RCLCPP_INFO(this->get_logger(), "Pose: x=%.2f, y=%.2f, theta=%.2f°",
  //             pose.x_cm, pose.y_cm, pose.theta_rad * 180.0 / M_PI);

  // publish pose message
  geometry_msgs::msg::Pose2D msg;
  msg.x = pose.x_cm / 100.0;
  msg.y = pose.y_cm / 100.0;
  msg.theta = pose.theta_rad;

  pose_publisher_->publish(msg);
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseNode>());
  rclcpp::shutdown();
  return 0;
}
