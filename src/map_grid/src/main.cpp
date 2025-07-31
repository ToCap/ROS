#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "map_grid/map_grid_node.hpp"

int main(int argc, char ** argv)
{
    std::cout << "Node is wekeup..." << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "Node is starting..." << std::endl;

    RCLCPP_INFO(rclcpp::get_logger("startup"), "Node started");
    
    auto node = std::make_shared<map_grid::MapGridNode>(rclcpp::NodeOptions{});

    // Création d’un exécuteur compatible LifecycleNode
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());

    exec.spin();

    rclcpp::shutdown();
    return 0;
}

