#pragma once

#include "map_grid/map_grid.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

namespace map_grid {

/**
 * @class MapGridNode
 * @brief LifecycleNode responsible for managing and publishing a grid-based occupancy map.
 *
 * This node wraps around the MapGrid data structure, exposing it via topics
 * and managing its lifecycle using the rclcpp_lifecycle API.
 */
class MapGridNode : public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * @brief Construct a new MapGridNode object.
     * 
     * @param options NodeOptions passed to the LifecycleNode.
     */
    explicit MapGridNode(const rclcpp::NodeOptions & options);

    /**
     * @brief Lifecycle transition: Configure the node.
     * Allocates and initializes the map and publisher.
     * 
     * @param state Current lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

    /**
     * @brief Lifecycle transition: Activate the node.
     * Activates the publisher and publishes initial map.
     * 
     * @param state Current lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);

    /**
     * @brief Lifecycle transition: Deactivate the node.
     * Deactivates the publisher.
     * 
     * @param state Current lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

    /**
     * @brief Lifecycle transition: Cleanup the node.
     * Releases resources such as map and publisher.
     * 
     * @param state Current lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

    /**
     * @brief Lifecycle transition: Shutdown the node.
     * No-op in this implementation, but allows for graceful shutdown.
     * 
     * @param state Current lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
    /// Internal map representation used for storing occupancy data.
    std::unique_ptr<MapGrid> grid_;

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;

    void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);

    void tactileCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /// Lifecycle publisher for the OccupancyGrid message.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    /**
     * @brief Converts internal map representation into a OccupancyGrid message.
     * 
     * @param grid The internal MapGrid to convert.
     * @param frame_id TF frame to associate with the message.
     * @return nav_msgs::msg::OccupancyGrid The formatted ROS message.
     */
    nav_msgs::msg::OccupancyGrid toOccupancyGridMsg(const MapGrid& grid, const std::string &frame_id);
};

}  // namespace map_grid
