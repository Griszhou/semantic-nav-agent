#pragma once

#include "semantic_nav/semantic_map.hpp"

#include <vector>

namespace semantic_nav {

class AStarPlanner {
 public:
  explicit AStarPlanner(const SemanticMap& map) : map_(map) {}
  NavigationResult plan(GridIndex start, GridIndex goal) const;

 private:
  const SemanticMap& map_;
};

}  // namespace semantic_nav
