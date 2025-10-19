#include "planner/planner.hpp"
#include <iostream>
#include <set>


using namespace planner;

static std::unordered_map<std::string, std::pair<int,int>> DIRECTIONS = {
    {"N", {0, -1}},
    {"E", {1, 0}},
    {"S", {0, 1}},
    {"W", {-1, 0}}
};

PlannerAlgorithm::PlannerAlgorithm() {}

double PlannerAlgorithm::heuristic(const std::pair<int,int>& a, const std::pair<int,int>& b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

std::vector<Neighbor> PlannerAlgorithm::neighbors_with_orientation(
    int x, int y, const std::string& orientation,
    const std::unordered_set<std::pair<int,int>, std::hash<long long>>& obstacles,
    int width, int height)
{
    std::vector<Neighbor> result;
    for (auto& [new_orientation, delta] : DIRECTIONS) {
        auto [dx, dy] = delta;
        double turn_penalty = (orientation == new_orientation) ? 0.0 : 0.1;

        for (int step = 1; step <= 3; ++step) {
            int nx = x + dx * step;
            int ny = y + dy * step;

            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                break;

            if (obstacles.count({nx, ny}) > 0)
                break;

            double move_cost = (3.0 - step) / 3.0;
            result.push_back({nx, ny, new_orientation, turn_penalty, new_orientation, step, move_cost});
        }
    }
    return result;
}

std::vector<Cell> PlannerAlgorithm::a_star(const std::pair<int,int>& start,
                                           const std::pair<int,int>& goal,
                                           const std::string& initial_orientation,
                                           const std::unordered_set<std::pair<int,int>, std::hash<long long>>& obstacles,
                                           int width,
                                           int height)
{
    using Node = std::tuple<double,double,Cell>;
    auto cmp = [](const Node& a, const Node& b) { return std::get<0>(a) > std::get<0>(b); };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open_set(cmp);

    Cell start_state{start.first, start.second, initial_orientation, 0};
    open_set.push({0, 0, start_state});

    std::unordered_map<Cell, Cell, CellHash> came_from;
    std::unordered_map<Cell, double, CellHash> g_score;
    g_score[start_state] = 0.0;

    while (!open_set.empty()) {
        auto [f, g, current] = open_set.top();
        open_set.pop();

        if (current.x == goal.first && current.y == goal.second) {
            std::vector<Cell> path;
            Cell state = current;
            while (came_from.find(state) != came_from.end()) {
                path.push_back(state);
                state = came_from[state];
            }
            path.push_back(start_state);
            std::reverse(path.begin(), path.end());
            return path;
        }

        auto neighbors = neighbors_with_orientation(current.x, current.y, current.orientation, obstacles, width, height);
        for (auto& n : neighbors) {
            Cell neighbor{n.nx, n.ny, n.new_orientation, n.step_length};
            double total_cost = g + 100.0 * n.move_cost + 1000.0 * n.turn_penalty;

            if (!g_score.count(neighbor) || total_cost < g_score[neighbor]) {
                g_score[neighbor] = total_cost;
                came_from[neighbor] = current;
                double f_score = total_cost + heuristic({n.nx, n.ny}, goal);
                open_set.push({f_score, total_cost, neighbor});
            }
        }
    }
    return {};
}
