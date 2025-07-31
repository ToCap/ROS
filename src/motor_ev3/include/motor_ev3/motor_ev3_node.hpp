#ifndef MOTOR_EV3_NODE_HPP
#define MOTOR_EV3_NODE_HPP

#include "motor_ev3/motor_ev3.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"


class MotorEv3Node : public rclcpp::Node
{
public:
    MotorEv3Node();

private:
 

private:
  void handle_reset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  int counter_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    
    MotorEv3 motor_;

};

#endif  // MOTOR_EV3_NODE_HPP
