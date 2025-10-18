/// @file map_grid.cpp
/// @brief Implementation of the MapGrid class managing a 2D occupancy grid map.

#include "map_grid/map_grid.hpp"
#include <algorithm>
#include <chrono>

using namespace map_grid;

/// @brief Default constructor.
/// Initializes the grid by clearing all cells to UNKNOWN state.
MapGrid::MapGrid() {
    clear();
}

/// @brief Clears the grid by setting all cells to UNKNOWN and resetting free confidence.
void MapGrid::clear() 
{
    for (int y = 0; y < GRID_H; ++y) 
    {
        for (int x = 0; x < GRID_W; ++x) 
        {
            grid_[y][x] = CellState::UNKNOWN;
            validity_[y][x] = 0.0;
        }
    }
}


bool MapGrid::inBounds(int gx, int gy) const 
{
    return gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H;
}


void MapGrid::setOccupied(int gx, int gy) 
{
    if (!inBounds(gx, gy)) return;
    setCellStatusWithInflation(gx, gy, CellState::OCCUPIED);
}


void MapGrid::setCellStatusWithInflation(int gx, int gy, CellState state)
{
    for (int dx = -1; dx <= 1; ++dx) 
    {
        for (int dy = -1; dy <= 1; ++dy) 
        {
            int nx = gx + dx;
            int ny = gy + dy;
            if (inBounds(nx, ny)) 
            {
                grid_[ny][nx] = state;
            }
        }
    }  
}


void MapGrid::markFreeRay(int x0, int y0, int x1, int y1) 
{
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (!(x0 == x1 && y0 == y1)) 
    {
        if (inBounds(x0, y0)) 
        {
            if (grid_[y0][x0] == CellState::UNKNOWN) 
            {
                grid_[y0][x0] = CellState::FREE;
                validity_[y0][x0] = 1;
            } 
            else if (grid_[y0][x0] == CellState::FREE) 
            {
                if (validity_[y0][x0] < 100)
                {
                    ++validity_[y0][x0];
                }
            }
        }
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/// @brief Returns the cell state at the specified coordinates.
/// @param gx X coordinate.
/// @param gy Y coordinate.
/// @return CellState enum indicating the occupancy status of the cell.
/// Returns UNKNOWN if coordinates are out of bounds.
CellState MapGrid::getCell(int gx, int gy) const {
    if (!inBounds(gx, gy)) return CellState::UNKNOWN;
    return grid_[gy][gx];
}

/// @brief Returns the confidence level of a free cell at the specified coordinates.
/// @param gx X coordinate.
/// @param gy Y coordinate.
/// @return Confidence value [0-255]. Returns 0 if out of bounds.
uint8_t MapGrid::getFreeConfidence(int gx, int gy) const 
{
    if (!inBounds(gx, gy)) return 0;
    return validity_[gy][gx];
}


void MapGrid::markRobotPosition(const Pose2D& pose)
{
    // Store previous robot coordinates
    Pose2D previous_robot_pose = this->robot_pose_;

    // Update robot coordinates
    this->robot_pose_ = pose;

    // Convert new robot coordinates into grid cell indexes
    auto [grid_x, grid_y] = this->worldToGrid(pose.x, pose.y);

    // Update robot position on grid
    if (this->inBounds(grid_x, grid_y))
    {
        this->setCellStatusWithInflation(grid_x, grid_y, CellState::OCCUPIED);
    }

    if ((previous_robot_pose.x != pose.x) || (previous_robot_pose.y != pose.y))
    {
        // Convert previous robot coordinates into grid cell indexes
        auto [grid_x, grid_y] = this->worldToGrid(previous_robot_pose.x, previous_robot_pose.y);

        // Remove previous robot position from grid
        if (this->inBounds(grid_x, grid_y))
        {
            this->setCellStatusWithInflation(grid_x, grid_y, CellState::FREE);
        }
    }
}


void MapGrid::updateMapWithObstacle(double  x, double y, double length, double width, double validity)
{
    // Convert origin coordinates and sizes into grid cell indexes
    auto [grid_x, grid_y] = this->worldToGrid(this->robot_pose_.x + x, this->robot_pose_.y + y);
    auto [grid_length, grid_width ] = this->worldToGrid(length, width);
    int start_x = std::min(grid_x, grid_x + grid_length);
    int stop_x = std::max(grid_x, grid_x + grid_length);
    // HINTS : length might be negative

    // Convert validity information
    validity_internal = validity * 100;

    for (int dx = start_x; dx <= stop_x; ++dx) 
    {
        for (int dy = grid_y; dy <= grid_y + grid_width; ++dy)
        {
            if (inBounds(dx, dy)) 
            {
                grid_[dy][dx] = CellState::OCCUPIED;
                validity_[dy][dx] = validity_internal;
            }
        }
    } 
}


std::pair<int, int> MapGrid::worldToGrid(double x, double y) const
{
    int gx = static_cast<int>(x / 0.05);
    int gy = static_cast<int>(y / 0.05);
    
    return {gx, gy};
}