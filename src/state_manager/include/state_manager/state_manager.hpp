#pragma once

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include "lifecycle_msgs/msg/state.hpp"

namespace state_manager
{
enum class SystemMode {
  OFF,
  PRERUN,
  RUN,
  POSTRUN,
  ERROR,
  DIAGNOSTIC
};

class StateManager {
public:
  StateManager();

  // Mettre à jour l'état observé d'un node (value = lifecycle_msgs::msg::State::id)
  void update_node_state(const std::string & node_name, uint8_t state_id);

  // Retourne le mode système actuellement déduit
  SystemMode decide_system_mode();

  // Retourne la liste de nodes et l'état observé (copie sûre)
  std::map<std::string, uint8_t> snapshot_node_states();

  // configuration: nodes monitorés
  void set_monitored_nodes(const std::vector<std::string> & nodes);

private:
  std::map<std::string, uint8_t> node_states_;
  std::vector<std::string> monitored_nodes_;
  std::mutex mtx_;

  // Heuristique interne : mapping des états nodaux -> choix de mode global
  SystemMode heuristic_from_states_locked();
};
}