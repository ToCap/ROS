#include "sensor_touch_interface/sensor_touch_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace sensor_touch_interface
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// ───────────────────────────────────────────────
// 1. INITIALISATION
// ───────────────────────────────────────────────
CallbackReturn SensorTouchInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Lecture du paramètre "sim_topic" défini dans le YAML de ros2_control
  if (info_.hardware_parameters.find("sim_topic") != info_.hardware_parameters.end()) {
    sim_topic_ = info_.hardware_parameters.at("sim_topic");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("SensorTouchInterface"),
                 "Le paramètre 'sim_topic' est manquant dans la configuration hardware_info.");
    return CallbackReturn::ERROR;
  }

  // Initialisation de la valeur du capteur
  touch_state_ = 0.0;

  return CallbackReturn::SUCCESS;
}

// ───────────────────────────────────────────────
// 2. CONFIGURATION
// ───────────────────────────────────────────────
CallbackReturn SensorTouchInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Création du node ROS2 utilisé pour la subscription
  node_ = std::make_shared<rclcpp::Node>("sensor_touch_interface");

  // Création du subscriber au topic simulé
  sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    sim_topic_,
    rclcpp::QoS(10),
    std::bind(&SensorTouchInterface::sim_callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(node_->get_logger(), "SensorTouchInterface configuré. Abonnement à %s", sim_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

// ───────────────────────────────────────────────
// 3. ACTIVATION
// ───────────────────────────────────────────────
CallbackReturn SensorTouchInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Activation du capteur SensorTouchInterface.");
  return CallbackReturn::SUCCESS;
}

// ───────────────────────────────────────────────
// 4. DÉSACTIVATION
// ───────────────────────────────────────────────
CallbackReturn SensorTouchInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Désactivation du capteur SensorTouchInterface.");
  return CallbackReturn::SUCCESS;
}

// ───────────────────────────────────────────────
// 5. EXPORTATION DES INTERFACES D'ÉTAT
// ───────────────────────────────────────────────
std::vector<hardware_interface::StateInterface> SensorTouchInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.sensors[0].name,
      hardware_interface::HW_IF_POSITION,  // on utilise "position" pour un capteur booléen simple
      &touch_state_));
  return state_interfaces;
}

// ───────────────────────────────────────────────
// 6. EXPORTATION DES INTERFACES DE COMMANDE
// ───────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface> SensorTouchInterface::export_command_interfaces()
{
  // Aucun handle de commande pour un capteur passif
  return {};
}

// ───────────────────────────────────────────────
// 7. LECTURE (read)
// ───────────────────────────────────────────────
return_type SensorTouchInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // La variable touch_state_ est déjà mise à jour par le callback.
  // Ici on ne fait qu’assurer sa visibilité dans les StateInterfaces.
  return return_type::OK;
}

// ───────────────────────────────────────────────
// 8. ÉCRITURE (write)
// ───────────────────────────────────────────────
return_type SensorTouchInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Pas d’action à écrire (capteur passif)
  return return_type::OK;
}

// ───────────────────────────────────────────────
// 9. CALLBACK DE LA SIMULATION
// ───────────────────────────────────────────────
void SensorTouchInterface::sim_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  touch_state_ = msg->data ? 1.0 : 0.0;
}

}  // namespace sensor_touch_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sensor_touch_interface::SensorTouchInterface, hardware_interface::SystemInterface)
