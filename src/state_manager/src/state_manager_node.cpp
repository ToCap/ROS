#include "state_manager/state_manager_node.hpp"
//#include "rclcpp_components/register_node_macro.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace state_manager
{

StateManagerNode::StateManagerNode(const rclcpp::NodeOptions & options) : LifecycleNode("state_manager", options)
{
  // declare parameters
  this->declare_parameter<std::vector<std::string>>("monitored_nodes", std::vector<std::string>{});
  this->declare_parameter<double>("evaluation_period_s", 1.0);

  // load parameters
  monitored_nodes_ = this->get_parameter("monitored_nodes").as_string_array();
  evaluation_period_s_ = this->get_parameter("evaluation_period_s").as_double();

  this->manager_.set_monitored_nodes(monitored_nodes_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "StateManager: on_configure");
  // initialise les subscriptions/clients
  init_monitors();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "StateManager: on_activate");
  // démarre timer d'évaluation
  eval_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(evaluation_period_s_),
    std::bind(&StateManagerNode::evaluate_and_act, this)
  );
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "StateManager: on_deactivate");
  eval_timer_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "StateManager: on_cleanup");
  subs_.clear();
  change_state_clients_.clear();
  get_state_clients_.clear();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
StateManagerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "StateManager: on_shutdown");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void StateManagerNode::init_monitors() {
  // Créer une subscription pour chaque node sur /<node>/transition_event
  for (const auto & node : monitored_nodes_) {
    std::string topic = "/" + node + "/transition_event";
    auto cb = [this, node](const TransitionEvent::SharedPtr msg){
      this->on_transition_event(node, msg);
    };
    auto sub = this->create_subscription<TransitionEvent>(
      topic,
      10,
      cb
    );
    subs_[node] = sub;

    // create change_state client
    std::string change_service = "/" + node + "/change_state";
    auto client_change = this->create_client<ChangeState>(change_service);
    change_state_clients_[node] = client_change;

    // create get_state client
    std::string get_service = "/" + node + "/get_state";
    auto client_get = this->create_client<GetState>(get_service);
    get_state_clients_[node] = client_get;

    RCLCPP_INFO(this->get_logger(), "Monitoring node '%s' (topic: %s)", node.c_str(), topic.c_str());
  }
}

void StateManagerNode::on_transition_event(const std::string & node_name, const TransitionEvent::SharedPtr msg) {
  // msg->goal_state.id contient le nouvel état
  uint8_t new_state = msg->goal_state.id;
  RCLCPP_INFO(this->get_logger(), "Transition event from %s: start=%u -> goal=%u",
              node_name.c_str(), msg->start_state.id, msg->goal_state.id);
  this->manager_.update_node_state(node_name, new_state);
}

uint8_t StateManagerNode::call_get_state(const std::string & node_name) {
  auto it = get_state_clients_.find(node_name);
  if (it == get_state_clients_.end()) return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  auto client = it->second;
  if (!client->wait_for_service(100ms)) {
    RCLCPP_WARN(get_logger(), "get_state service not available for %s", node_name.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  auto req = std::make_shared<GetState::Request>();
  // (GetState request uses empty "request" fields in some versions) 
  auto fut = client->async_send_request(req);
  // attente bloquante courte (pour simplifier)
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 200ms)
      == rclcpp::FutureReturnCode::SUCCESS) {
    auto res = fut.get();
    if (res && res->current_state.id) {
      return res->current_state.id;
    }
  }
  return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
}

bool StateManagerNode::call_change_state(const std::string & node_name, uint8_t transition_id) {
  auto it = change_state_clients_.find(node_name);
  if (it == change_state_clients_.end()) {
    RCLCPP_WARN(this->get_logger(), "No change_state client for %s", node_name.c_str());
    return false;
  }
  auto client = it->second;
  if (!client->wait_for_service(200ms)) {
    RCLCPP_WARN(this->get_logger(), "change_state service not available for %s", node_name.c_str());
    return false;
  }
  auto req = std::make_shared<ChangeState::Request>();
  req->transition.id = transition_id;
  auto fut = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 500ms)
      == rclcpp::FutureReturnCode::SUCCESS) {
    auto res = fut.get();
    if (res) {
      RCLCPP_INFO(this->get_logger(), "change_state(%s, %u) -> success=%s", node_name.c_str(), transition_id, res->success ? "true":"false");
      return res->success;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "change_state call timed out for %s", node_name.c_str());
  }
  return false;
}

void StateManagerNode::evaluate_and_act() 
{
  
  // get snapshot of state of monitored modules
  auto node_states = this->manager_.snapshot_node_states();
  
  // check if one or more module does not yet respond
  for (auto & kv : node_states)
  {
    if (kv.second == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN)
    {
      // request state of current monitored module
      uint8_t s = call_get_state(kv.first);
      this->manager_.update_node_state(kv.first, s);
    }
  }

  SystemMode target_mode = this->manager_.decide_system_mode();
  RCLCPP_INFO(get_logger(), "Decided system mode = %d", static_cast<int>(target_mode));

  // Exemple d'actions pour chaque mode (heuristique) :
  if (target_mode == SystemMode::PRERUN) 
  {
    // s'assurer que tous les nodes sont configurés (transition configure = 1)
    for (const auto & node : monitored_nodes_) {
      uint8_t s = this->manager_.snapshot_node_states()[node];
      if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED ||
          s == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
        // demander configure (TRANSITION_CONFIGURE = 1)
        call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      }
    }
  } 
  else if (target_mode == SystemMode::RUN)
  {
    // demandons configure -> activate si besoin
    for (const auto & node : monitored_nodes_) {
      uint8_t s = this->manager_.snapshot_node_states()[node];
      if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED ||
          s == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
        call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        // mise à jour locale immédiate optimiste
        this->manager_.update_node_state(node, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        s = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      }
      if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        // TRANSITION_ACTIVATE = 3
        call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        this->manager_.update_node_state(node, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      }
    }
  }
  else if (target_mode == SystemMode::POSTRUN)
  {
    // Start deactivation of nodes 
    for (const auto & node : monitored_nodes_) {
      uint8_t s = this->manager_.snapshot_node_states()[node];
      if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        // deactivate
        call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        this->manager_.update_node_state(node, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
      }
      if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        // cleanup (TRANSITION_CLEANUP = 6)
        call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        this->manager_.update_node_state(node, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
      }
    }
  }
  else if (target_mode == SystemMode::OFF)
  {
    // Start clean-up of nodes 
    for (const auto & node : monitored_nodes_) {
      uint8_t s = this->manager_.snapshot_node_states()[node];

      if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        // cleanup (TRANSITION_CLEANUP = 6)
        call_change_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        this->manager_.update_node_state(node, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
      }
    }
  }
  else if (target_mode == SystemMode::DIAGNOSTIC) 
  {
    // en mode diagnostic on laisse les nodes en l'état et on pourrait publier un topic / diagnostic
    RCLCPP_INFO(get_logger(), "Diagnostic mode: skipping automatic transitions.");
  }
  else if (target_mode == SystemMode::ERROR) 
  {
    RCLCPP_ERROR(get_logger(), "System in ERROR mode: manual intervention required.");
    // On pourrait implémenter politique de redémarrage, ou fallback sécurisé.
  }
}
}