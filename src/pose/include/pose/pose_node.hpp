#ifndef ROBOT_POSE_ESTIMATOR__POSE_NODE_HPP_
#define ROBOT_POSE_ESTIMATOR__POSE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pose/pose_estimator.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

class PoseNode : public rclcpp::Node
{
public:
    PoseNode();

private:
    void timer_callback();

    /**
     * @brief Callback for processing incoming odometry messages.
     * 
     * This function is called whenever a new odometry message is received.
     * It should handle updating the map or internal robot state based on
     * the robot’s pose and velocity.
     * 
     * @param msg Shared pointer to the nav_msgs::msg::Odometry message.
     */
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
    
    PoseEstimator pose_estimator_;

};

#endif  // ROBOT_POSE_ESTIMATOR__POSE_NODE_HPP_
