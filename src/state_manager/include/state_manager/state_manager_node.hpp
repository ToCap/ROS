#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include "state_manager/state_manager.hpp"

using rclcpp_lifecycle::LifecycleNode;
using lifecycle_msgs::msg::TransitionEvent;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;

namespace state_manager
{

class StateManagerNode : public LifecycleNode {
public:
  explicit StateManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StateManagerNode() override = default;

  // lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  void init_monitors();
  void on_transition_event(const std::string & node_name, const TransitionEvent::SharedPtr msg);
  void evaluate_and_act(); // logique périodique
  bool call_change_state(const std::string & node_name, uint8_t transition_id);
  uint8_t call_get_state(const std::string & node_name);

  // configuration
  std::vector<std::string> monitored_nodes_;
  double evaluation_period_s_;

  // subscriptions (one per monitored node)
  std::map<std::string, rclcpp::Subscription<TransitionEvent>::SharedPtr> subs_;

  // clients cache for change_state & get_state
  std::map<std::string, rclcpp::Client<ChangeState>::SharedPtr> change_state_clients_;
  std::map<std::string, rclcpp::Client<GetState>::SharedPtr> get_state_clients_;

  // timer
  rclcpp::TimerBase::SharedPtr eval_timer_;

  // logic
  StateManager manager_;
};
}
