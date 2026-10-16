#pragma once

#include "semantic_nav/exploration.hpp"

#include <string>
#include <vector>

namespace semantic_nav {

struct TaskStep {
  std::string action;
  std::string target;
  std::string status;
  std::string detail;
};

struct AgentReport {
  bool completed{false};
  std::vector<TaskStep> trace;
  NavigationResult navigation;
};

// A deterministic LangGraph-style state machine for local replay and CI.
class TaskAgent {
 public:
  TaskAgent(SemanticMap& map, AStarPlanner& planner, FrontierExplorer& explorer);
  AgentReport execute(const std::string& instruction, Pose2D start);

 private:
  std::vector<std::string> parse_targets(const std::string& instruction) const;
  SemanticMap& map_;
  AStarPlanner& planner_;
  FrontierExplorer& explorer_;
};

}  // namespace semantic_nav
