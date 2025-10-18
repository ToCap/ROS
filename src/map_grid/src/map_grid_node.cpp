#include "map_grid/map_grid_node.hpp"
#include <chrono>

using namespace map_grid;
using namespace std::chrono_literals;

/// @brief Constructor.
/// @param options NodeOptions passed to the LifecycleNode base class.
MapGridNode::MapGridNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("map_grid_node", options) {}

/// @brief Lifecycle callback when configuring the node.
/// Initializes the map grid and publisher.
/// @param[in] state The current lifecycle state (not used).
/// @return SUCCESS on successful configuration.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MapGridNode::on_configure(const rclcpp_lifecycle::State &)
{
    grid_ = std::make_unique<MapGrid>();
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", rclcpp::QoS(10));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "/pose", rclcpp::QoS(10),
        std::bind(&MapGridNode::poseCallback, this, std::placeholders::_1));

    obstacle_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/obstacle", rclcpp::QoS(10),
        std::bind(&MapGridNode::obstacleCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "MapGrid now configured");


    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// @brief Lifecycle callback when activating the node.
/// Activates the publisher and publishes an example occupancy grid.
/// @param[in] state The current lifecycle state (not used).
/// @return SUCCESS on successful activation.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MapGridNode::on_activate(const rclcpp_lifecycle::State &) 
{
    grid_pub_->on_activate();

    // Example test data: mark a cell occupied and a free ray
    //grid_->setOccupied(80, 80);
    //grid_->markFreeRay(0, 0, 80, 80);

    auto msg = toOccupancyGridMsg(*grid_, "map");
    grid_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "MapGrid now activated");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// @brief Lifecycle callback when deactivating the node.
/// Deactivates the publisher.
/// @param[in] state The current lifecycle state (not used).
/// @return SUCCESS on successful deactivation.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MapGridNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    grid_pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// @brief Lifecycle callback when cleaning up the node.
/// Resets the map grid and publisher.
/// @param[in] state The current lifecycle state (not used).
/// @return SUCCESS on successful cleanup.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MapGridNode::on_cleanup(const rclcpp_lifecycle::State &) 
{
    grid_.reset();
    grid_pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// @brief Lifecycle callback when shutting down the node.
/// @param[in] state The current lifecycle state (not used).
/// @return SUCCESS on successful shutdown.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MapGridNode::on_shutdown(const rclcpp_lifecycle::State &) 
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/**
 * @brief Callback called when a tactile sensor message is received.
 * 
 * If the tactile sensor is triggered (msg->data == true), marks
 * a specific cell in the grid as occupied and republishes the updated occupancy grid.
 * 
 * @param msg Shared pointer to the received Bool message from the tactile sensor.
 */
void MapGridNode::tactileCallback(const std_msgs::msg::Bool::SharedPtr msg) 
{
    if (msg->data) 
    {
        RCLCPP_INFO(get_logger(), "Tactile sensor triggered");
        // Exemple simple : on marque la cellule (50, 50) occupée
        grid_->setOccupied(50, 50);

        // Republier la map mise à jour
        auto msg = toOccupancyGridMsg(*grid_, "map");
        grid_pub_->publish(msg);
    }
}

void MapGridNode::obstacleCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received obstacle info");
}

void MapGridNode::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{

    RCLCPP_INFO(get_logger(), "Received Pose2D: x=%.2f y=%.2f theta=%.2f",
                msg->x, msg->y, msg->theta);


    int gx = static_cast<int>(msg->x / 0.05);  // 0.05 = résolution
    int gy = static_cast<int>(msg->y / 0.05);

    if (grid_ && grid_->inBounds(gx, gy))
    {
        grid_->setOccupied(gx, gy);

        auto grid_msg = toOccupancyGridMsg(*grid_, "map");
        grid_pub_->publish(grid_msg);
    }
}

/// @brief Converts the internal MapGrid to a ROS occupancy grid message.
/// @param[in] grid The internal occupancy grid representation.
/// @param[in] frame_id The TF frame ID to assign to the message header.
/// @return A nav_msgs::msg::OccupancyGrid message reflecting the current map state.
nav_msgs::msg::OccupancyGrid MapGridNode::toOccupancyGridMsg(const MapGrid& grid, const std::string &frame_id) {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = now();
    msg.info.resolution = 0.05f;
    msg.info.width = GRID_W;
    msg.info.height = GRID_H;
    msg.info.origin.position.x = 0.0;
    msg.info.origin.position.y = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(GRID_W * GRID_H, -1);
    for (int y = 0; y < GRID_H; ++y) {
        for (int x = 0; x < GRID_W; ++x) {
            int idx = y * GRID_W + x;
            auto cell = grid.getCell(x, y);
            if (cell == CellState::OCCUPIED) {
                msg.data[idx] = 100;
            } else if (cell == CellState::FREE) {
                msg.data[idx] = std::min(20 + int(grid.getFreeConfidence(x, y) * 0.5), 99);
            } else {
                msg.data[idx] = -1;
            }
        }
    }
    return msg;
}
