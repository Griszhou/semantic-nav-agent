#include "semantic_nav/exploration.hpp"

#include <algorithm>
#include <cmath>

namespace semantic_nav {

double FrontierExplorer::semantic_score(const GridIndex& cell, const std::string& query) const {
  const auto world = map_.grid_to_world(cell);
  const auto object = map_.find_object(query);
  if (!object) return 0.25;
  const auto distance = std::hypot(world.x - object->pose.x, world.y - object->pose.y);
  return std::max(0.0, 1.0 - distance / 5.0);
}

std::vector<Frontier> FrontierExplorer::rank(const Pose2D& robot, const std::string& query,
                                             const AStarPlanner& planner) const {
  const auto start = map_.world_to_grid(robot.x, robot.y);
  std::vector<Frontier> candidates;
  for (int y = 1; y < map_.height() - 1; ++y) {
    for (int x = 1; x < map_.width() - 1; ++x) {
      const GridIndex cell{x, y};
      if (map_.cell(cell) != CellState::Free) continue;
      bool adjacent_unknown = false;
      for (const auto n : {GridIndex{x + 1, y}, GridIndex{x - 1, y}, GridIndex{x, y + 1}, GridIndex{x, y - 1}})
        adjacent_unknown = adjacent_unknown || map_.cell(n) == CellState::Unknown;
      if (!adjacent_unknown) continue;
      const auto route = planner.plan(start, cell);
      if (!route.success) continue;
      Frontier frontier;
      frontier.cell = cell;
      frontier.information_gain = 1.0;
      frontier.path_cost = route.cost;
      frontier.risk = map_.cell(cell) == CellState::Occupied ? 1.0 : 0.05;
      frontier.semantic_relevance = semantic_score(cell, query);
      frontier.utility = 1.8 * frontier.information_gain + 2.4 * frontier.semantic_relevance
                         - 0.25 * frontier.path_cost - 1.5 * frontier.risk;
      candidates.push_back(frontier);
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) { return a.utility > b.utility; });
  if (candidates.size() > 5) candidates.resize(5);
  return candidates;
}

}  // namespace semantic_nav
