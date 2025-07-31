#include "motor_ev3/motor_ev3_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

MotorEv3Node::MotorEv3Node(): Node("motor_ev3_node")
{
    motor_.init();

  service_ = this->create_service<std_srvs::srv::Empty>(
    "reset", std::bind(&MotorEv3Node::handle_reset, this,
                       std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Reset service ready.");
}





void MotorEv3Node::handle_reset(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  counter_ = 0;
  RCLCPP_INFO(this->get_logger(), "State reset. Counter is now: %d", counter_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorEv3Node>());
  rclcpp::shutdown();
  return 0;
}
