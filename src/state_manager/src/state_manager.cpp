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

SystemMode StateManager::heuristic_from_states_locked() {
  // Rappels des constantes : (voir lifecycle_msgs/msg/state.hpp)
  // PRIMARY_STATE_UNKNOWN = 0
  // PRIMARY_STATE_UNCONFIGURED = 1
  // PRIMARY_STATE_INACTIVE = 2
  // PRIMARY_STATE_ACTIVE = 3
  // PRIMARY_STATE_FINALIZED = 4

  bool any_active = false;
  bool any_inactive = false;
  bool any_unconfigured = false;
  bool any_finalized = false;
  bool any_unknown = false;

  for (const auto & kv : node_states_) {
    uint8_t s = kv.second;
    if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) any_active = true;
    else if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) any_inactive = true;
    else if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) any_unconfigured = true;
    else if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) any_finalized = true;
    else any_unknown = true;
  }

  // Heuristiques simples (à adapter selon ton besoin / règles AUTOSAR)
  if (any_finalized) {
    // si n'importe quel node est finalisé, on considère le system OFF (ou erreur si non attendu)
    return SystemMode::OFF;
  }

  if (any_unknown) {
    // un ou plusieurs nodes encore inconnus -> PRERUN (phase d'initialisation), sauf si tu veux ERROR
    return SystemMode::PRERUN;
  }

  // Si tous inactifs ou un mélange d'inactifs et non-configurés -> PRERUN
  if (any_inactive && !any_active) {
    return SystemMode::PRERUN;
  }

  // Si au moins un node actif, on est en RUN
  if (any_active) {
    return SystemMode::RUN;
  }

  // Par défaut : diagnostic
  return SystemMode::DIAGNOSTIC;
}
}