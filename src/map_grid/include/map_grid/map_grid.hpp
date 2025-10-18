#pragma once

#include <cstdint>
#include <string>
#include <array>

namespace map_grid {

/**
 * @brief Width of the occupancy grid in cells.
 */
constexpr int GRID_W = 160;

/**
 * @brief Height of the occupancy grid in cells.
 */
constexpr int GRID_H = 160;

struct Pose2D 
{
    double x;
    double y;
    double theta;
};

/**
 * @enum CellState
 * @brief Represents the possible states of a grid cell.
 */
enum class CellState : int8_t {
    UNKNOWN = -1,  ///< The state of the cell is unknown.
    FREE    = 0,   ///< The cell is free.
    OCCUPIED = 100 ///< The cell is occupied.
};

/**
 * @class MapGrid
 * @brief A 2D occupancy grid map with confidence tracking for free space.
 * 
 * This class provides a lightweight in-memory representation of a 2D grid map,
 * supporting operations like marking occupancy, ray-based free space clearing,
 * and inflation of obstacles.
 */
class MapGrid {
public:
    /**
     * @brief Construct a new empty MapGrid.
     */
    MapGrid();

    /**
     * @brief Clears the grid and resets all cells to UNKNOWN.
     */
    void clear();


    /**
     * @brief Marks the robot's current position on the occupancy grid
     *
     * This function updates the internal grid representation to indicate
     * the cell corresponding to the robot's position as occupied. It also
     * stores the robot's current pose for future reference.
     *
     * The world coordinates (in meters) are converted into grid indices
     * according to the map resolution before updating the grid cell.
     *
     * @param pose The current pose of the robot in world coordinates.
     *             - pose.x : X position in meters
     *             - pose.y : Y position in meters
     *             - pose.theta : Orientation in radians
     *
     * @note If the computed grid coordinates are outside the map bounds,
     *       no update is performed.
     */
    void markRobotPosition(const Pose2D& pose);


    /**
     * @brief Marks the cell at (gx, gy) as occupied with inflation.
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     */
    void setOccupied(int gx, int gy);

    /**
     * @brief Updates the internal map representation with a rectangular obstacle.
     *
     * This function adds or updates a rectangular obstacle in the map using its
     * origin coordinates, size, and validity factor. The obstacle is defined by
     * its bottom-left corner (origin) in robot coordinates, and its dimensions
     * along the X and Y axes.
     *
     * @param x X-coordinate of the rectangle's origin (bottom-left corner) in robot coordinates (meters).
     * @param y Y-coordinate of the rectangle's origin (bottom-left corner) in robot coordinates (meters).
     * @param length Length of the rectangle along the X-axis (meters).
     * @param width Width of the rectangle along the Y-axis (meters).
     * @param validity Confidence or validity factor of the obstacle detection (typically a value between 0.0 and 1.0).
     *
     * @return void
     */
    void updateMapWithObstacle(double x, double y, double length, double width, double validity);

    /**
     * @brief Marks cells along a ray as free space between two points.
     *
     * This function traverses the grid cells along the line segment defined by 
     * the start point (x0, y0) and the end point (x1, y1), marking each cell 
     * as free space. The traversal uses Bresenham's line algorithm for efficient 
     * and accurate discretization. For each cell visited along the ray, the 
     * confidence level for being free is increased.
     *
     * @param x0 Starting X-coordinate of the ray in grid coordinates.
     * @param y0 Starting Y-coordinate of the ray in grid coordinates.
     * @param x1 Ending X-coordinate of the ray in grid coordinates.
     * @param y1 Ending Y-coordinate of the ray in grid coordinates.
     *
     * @return void
     */
    void markFreeRay(int x0, int y0, int x1, int y1);

    /**
     * @brief Checks if the given cell is inside the bounds of the grid.
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     * @return true if the cell is within bounds.
     */
    bool inBounds(int gx, int gy) const;

    /**
     * @brief Returns the CellState of a given cell.
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     * @return CellState of the cell.
     */
    CellState getCell(int gx, int gy) const;

    /**
     * @brief Returns the confidence value for a free cell.
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     * @return uint8_t Confidence level (0–255).
     */
    uint8_t getFreeConfidence(int gx, int gy) const;

private:

    Pose2D robot_pose_;

    /// The 2D array representing cell states (occupied/free/unknown).
    CellState grid_[GRID_H][GRID_W];

    /// Confidence values associated with cells marked as free.
    uint8_t validity_[GRID_H][GRID_W];

    /**
     * @brief Internally applies inflation when updating cell status (occupied, free, ...)
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     */
    void setCellStatusWithInflation(int gx, int gy, CellState state);

    /**
     * @brief Converts world coordinates (in meters) into grid cell indexes
     *
     * This function transforms a position given in continuous world
     * coordinates (x, y) into discrete grid coordinates (gx, gy) based on
     * the map resolution. The returned indices correspond to the cell in
     * the occupancy grid that contains the given world position.
     *
     * @param x The x-coordinate in world space (meters).
     * @param y The y-coordinate in world space (meters).
     * @return A pair of integers (gx, gy) representing the grid cell indices.
     *
     * @note This function does not perform bounds checking. Use @ref inBounds()
     *       to verify that the resulting indices are valid within the grid.
     */
    std::pair<int, int> worldToGrid(double x, double y) const;

};

}  // namespace map_grid
