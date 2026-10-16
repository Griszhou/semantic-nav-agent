#pragma once

#include "semantic_nav/planner.hpp"

#include <string>
#include <vector>

namespace semantic_nav {

class FrontierExplorer {
 public:
  explicit FrontierExplorer(const SemanticMap& map) : map_(map) {}
  std::vector<Frontier> rank(const Pose2D& robot, const std::string& query,
                             const AStarPlanner& planner) const;

 private:
  double semantic_score(const GridIndex& cell, const std::string& query) const;
  const SemanticMap& map_;
};

}  // namespace semantic_nav
