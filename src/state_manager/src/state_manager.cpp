#include "state_manager/state_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace state_manager
{


StateManager::StateManager() {}

void StateManager::set_monitored_nodes(const std::vector<std::string> & nodes) {
  std::lock_guard<std::mutex> lk(mtx_);
  monitored_nodes_ = nodes;
  node_states_.clear();
  for (const auto & n : monitored_nodes_) {
    node_states_[n] = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

void StateManager::update_node_state(const std::string & node_name, uint8_t state_id) {
  std::lock_guard<std::mutex> lk(mtx_);
  node_states_[node_name] = state_id;
}

std::map<std::string, uint8_t> StateManager::snapshot_node_states() {
  std::lock_guard<std::mutex> lk(mtx_);
  return node_states_;
}

SystemMode StateManager::decide_system_mode() {
  std::lock_guard<std::mutex> lk(mtx_);
  return heuristic_from_states_locked();
}

SystemMode StateManager::heuristic_from_states_locked()
{
  
}