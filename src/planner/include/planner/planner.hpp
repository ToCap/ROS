#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <tuple>
#include <cmath>
#include <functional>

namespace planner
{

struct Cell {
    int x;
    int y;
    std::string orientation;
    int steps_straight;

    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y && orientation == other.orientation && steps_straight == other.steps_straight;
    }
};

struct CellHash {
    std::size_t operator()(const Cell& c) const noexcept {
        return std::hash<int>()(c.x) ^ std::hash<int>()(c.y << 1) ^ std::hash<std::string>()(c.orientation);
    }
};

struct Neighbor {
    int nx;
    int ny;
    std::string new_orientation;
    double turn_penalty;
    std::string direction;
    int step_length;
    double move_cost;
};

class PlannerAlgorithm {
public:
    PlannerAlgorithm();

    std::vector<Cell> a_star(const std::pair<int,int>& start,
                             const std::pair<int,int>& goal,
                             const std::string& initial_orientation,
                             const std::unordered_set<std::pair<int,int>, 
                             std::hash<long long>>& obstacles,
                             int width,
                             int height);

private:
    double heuristic(const std::pair<int,int>& a, const std::pair<int,int>& b);
    std::vector<Neighbor> neighbors_with_orientation(int x, int y, const std::string& orientation,
                                                     const std::unordered_set<std::pair<int,int>, std::hash<long long>>& obstacles,
                                                     int width, int height);
};

}