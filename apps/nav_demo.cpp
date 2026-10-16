#include "semantic_nav/metrics.hpp"

#include <iostream>

using namespace semantic_nav;

namespace {
void build_demo_map(SemanticMap& map) {
  // Known free corridor with two rooms and an unobserved frontier boundary.
  for (int y = 4; y < 16; ++y) {
    for (int x = 3; x < 21; ++x) map.set_cell({x, y}, CellState::Free);
  }
  for (int y = 4; y < 16; ++y) map.set_cell({12, y}, CellState::Occupied);
  map.set_cell({12, 10}, CellState::Free);
  map.set_cell({12, 11}, CellState::Free);
  map.integrate_detection({-3.0, -1.0, 0.0}, {"red chair", 0.94, 4.0, 0.0});
  map.integrate_detection({3.0, 1.0, 0.0}, {"blue box", 0.91, 2.2, 1.57});
}
}  // namespace

int main(int argc, char** argv) {
  const std::string instruction = argc > 1 ? argv[1] : "Find the red chair and then the blue box";
  SemanticMap map(24, 20, 0.5);
  build_demo_map(map);
  AStarPlanner planner(map);
  FrontierExplorer explorer(map);
  TaskAgent agent(map, planner, explorer);
  const Pose2D start{-4.0, -1.0, 0.0};
  const auto report = agent.execute(instruction, start);
  const auto metrics = evaluate(report, report.navigation.cost);

  std::cout << "\n  semantic-nav-agent / deterministic replay\n"
            << "  --------------------------------------\n"
            << "  instruction: " << instruction << "\n\n";
  for (const auto& step : report.trace)
    std::cout << "  [" << step.status << "] " << step.action << " -> " << step.target
              << " (" << step.detail << ")\n";
  std::cout << "\n  metrics: " << metrics_json(metrics) << "\n";
  return report.completed ? 0 : 2;
}
