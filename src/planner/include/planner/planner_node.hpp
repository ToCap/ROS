#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "planner_package/planner_algorithm.hpp"
#include <unordered_set>
#include <utility>
#include <string>

namespace planner {


/**
 * @brief Lifecycle PlannerNode
 * Implémente un nœud ROS 2 avec gestion du cycle de vie (configure/activate/etc.)
 * et exécute un plan A* avec orientation.
 */
class PlannerNode : public LifecycleNode {
public:
    PlannerNode();

protected:
    // Lifecycle methods
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
    std::shared_ptr<PlannerAlgorithm> planner_;
    std::unordered_set<std::pair<int,int>, std::hash<long long>> obstacles_;
    std::pair<int,int> start_, goal_;
    std::string initial_orientation_;
    int map_width_, map_height_;
};

}
