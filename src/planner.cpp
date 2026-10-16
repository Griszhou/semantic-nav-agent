#include "semantic_nav/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace semantic_nav {
namespace {
struct Node { GridIndex cell; double f; };
struct Compare { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };
struct Hash { std::size_t operator()(const GridIndex& c) const { return (static_cast<std::size_t>(c.x) << 32) ^ static_cast<std::size_t>(c.y); } };
double heuristic(GridIndex a, GridIndex b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }
std::vector<GridIndex> neighbours(GridIndex c) { return {{c.x + 1, c.y}, {c.x - 1, c.y}, {c.x, c.y + 1}, {c.x, c.y - 1}}; }
}  // namespace

NavigationResult AStarPlanner::plan(GridIndex start, GridIndex goal) const {
  NavigationResult result;
  if (!map_.in_bounds(start) || !map_.in_bounds(goal) || map_.cell(goal) == CellState::Occupied) {
    result.reason = "goal outside map or occupied";
    return result;
  }
  std::priority_queue<Node, std::vector<Node>, Compare> open;
  std::unordered_map<GridIndex, double, Hash> cost;
  std::unordered_map<GridIndex, GridIndex, Hash> parent;
  open.push({start, heuristic(start, goal)});
  cost[start] = 0.0;
  while (!open.empty()) {
    const auto current = open.top().cell;
    open.pop();
    if (current == goal) {
      result.success = true;
      auto node = goal;
      while (!(node == start)) { result.path.push_back(node); node = parent[node]; }
      result.path.push_back(start);
      std::reverse(result.path.begin(), result.path.end());
      result.cost = cost[goal];
      return result;
    }
    for (const auto next : neighbours(current)) {
      if (!map_.in_bounds(next) || map_.cell(next) == CellState::Occupied) continue;
      const double candidate = cost[current] + 1.0;
      if (cost.find(next) == cost.end() || candidate < cost[next]) {
        cost[next] = candidate;
        parent[next] = current;
        open.push({next, candidate + heuristic(next, goal)});
      }
    }
  }
  result.reason = "no traversable path";
  return result;
}

}  // namespace semantic_nav
