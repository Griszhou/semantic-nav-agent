#include "semantic_nav/metrics.hpp"

#include <cassert>
#include <iostream>

using namespace semantic_nav;

int main() {
  SemanticMap map(12, 12, 1.0);
  for (int y = 2; y < 10; ++y) for (int x = 2; x < 10; ++x) map.set_cell({x, y}, CellState::Free);
  map.integrate_detection({0.0, 0.0, 0.0}, {"red chair", 0.9, 2.0, 0.0});
  assert(map.find_object("red chair").has_value());
  AStarPlanner planner(map);
  const auto route = planner.plan(map.world_to_grid(0.0, 0.0), map.world_to_grid(2.0, 0.0));
  assert(route.success && !route.path.empty());
  FrontierExplorer explorer(map);
  TaskAgent agent(map, planner, explorer);
  const auto report = agent.execute("Find the red chair", {0.0, 0.0, 0.0});
  assert(report.completed);
  assert(evaluate(report, report.navigation.cost).success);
  std::cout << "core tests passed\n";
}
