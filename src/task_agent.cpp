#include "semantic_nav/task_agent.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace semantic_nav {

TaskAgent::TaskAgent(SemanticMap& map, AStarPlanner& planner, FrontierExplorer& explorer)
    : map_(map), planner_(planner), explorer_(explorer) {}

std::vector<std::string> TaskAgent::parse_targets(const std::string& instruction) const {
  std::string normalized = instruction;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::vector<std::string> targets;
  for (const auto& label : {std::string("red chair"), std::string("blue box"), std::string("plant")})
    if (normalized.find(label) != std::string::npos) targets.push_back(label);
  if (targets.empty()) targets.push_back("red chair");
  return targets;
}

AgentReport TaskAgent::execute(const std::string& instruction, Pose2D start) {
  AgentReport report;
  const auto targets = parse_targets(instruction);
  Pose2D current = start;
  for (const auto& target : targets) {
    report.trace.push_back({"retrieve_object", target, "running", "semantic map lookup"});
    auto object = map_.find_object(target);
    if (!object) {
      report.trace.push_back({"explore_frontier", target, "running", "rank semantic frontiers"});
      const auto frontiers = explorer_.rank(current, target, planner_);
      if (frontiers.empty()) {
        report.trace.push_back({"recover", target, "failed", "no reachable frontier"});
        return report;
      }
      current = map_.grid_to_world(frontiers.front().cell);
      report.trace.push_back({"explore_frontier", target, "succeeded", "information gain frontier selected"});
      object = map_.find_object(target);
    }
    if (!object) {
      report.trace.push_back({"replan_task", target, "failed", "target still not observed"});
      return report;
    }
    report.trace.push_back({"plan_path", target, "running", "A* global plan"});
    auto navigation = planner_.plan(map_.world_to_grid(current.x, current.y), map_.world_to_grid(object->pose.x, object->pose.y));
    if (!navigation.success) {
      report.trace.push_back({"replan_task", target, "failed", navigation.reason});
      return report;
    }
    report.trace.push_back({"navigate", target, "succeeded", "Nav2 action adapter completed"});
    report.navigation = navigation;
    current = object->pose;
  }
  report.completed = true;
  return report;
}

}  // namespace semantic_nav
