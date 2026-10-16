#pragma once

#include "semantic_nav/task_agent.hpp"

#include <string>

namespace semantic_nav {

struct Metrics {
  bool success{false};
  double path_length{0.0};
  double spl{0.0};
  double replanning_ms{0.0};
  int explored_frontiers{0};
};

Metrics evaluate(const AgentReport& report, double optimal_length);
std::string metrics_json(const Metrics& metrics);

}  // namespace semantic_nav
