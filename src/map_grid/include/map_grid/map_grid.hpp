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
     * @brief Marks the cell at (gx, gy) as occupied with inflation.
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     */
    void setOccupied(int gx, int gy);

    /**
     * @brief Marks a ray from (x0, y0) to (x1, y1) as free.
     * 
     * @param x0 Start X coordinate
     * @param y0 Start Y coordinate
     * @param x1 End X coordinate
     * @param y1 End Y coordinate
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
    /// The 2D array representing cell states (occupied/free/unknown).
    CellState grid_[GRID_H][GRID_W];

    /// Confidence values associated with cells marked as free.
    uint8_t free_confidence_[GRID_H][GRID_W];

    /**
     * @brief Internally applies inflation when marking a cell as occupied.
     * 
     * @param gx Grid X coordinate
     * @param gy Grid Y coordinate
     */
    void setOccupiedWithInflation(int gx, int gy);
};

}  // namespace map_grid
