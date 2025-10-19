#include "planner/planner_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

using namespace planner;

PlannerNode::PlannerNode()
: LifecycleNode("planner_node_lifecycle") {
    RCLCPP_INFO(get_logger(), "Lifecycle PlannerNode created.");
}

CallbackReturn PlannerNode::on_configure(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Configuring planner...");
    planner_ = std::make_shared<PlannerAlgorithm>();

    // Configuration d'exemple
    obstacles_ = {
        {4,4}, {4,5}, {5,5}
    };
    map_width_ = 10;
    map_height_ = 10;
    start_ = {0, 0};
    goal_ = {9, 9};
    initial_orientation_ = "E";

    RCLCPP_INFO(get_logger(), "Planner configured successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlannerNode::on_activate(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Activating planner...");

    auto path = planner_->a_star(start_, goal_, initial_orientation_, obstacles_, map_width_, map_height_);
    RCLCPP_INFO(get_logger(), "Path computed: %zu steps", path.size());

    for (auto& c : path) {
        std::cout << "(" << c.x << "," << c.y << "," << c.orientation << ")\n";
    }

    RCLCPP_INFO(get_logger(), "Planner active.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlannerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Deactivating planner...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlannerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up planner resources...");
    planner_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlannerNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Shutting down planner node.");
    return CallbackReturn::SUCCESS;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
