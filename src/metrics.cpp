#include "semantic_nav/metrics.hpp"

#include <iomanip>
#include <sstream>

namespace semantic_nav {

Metrics evaluate(const AgentReport& report, double optimal_length) {
  Metrics metrics;
  metrics.success = report.completed && report.navigation.success;
  metrics.path_length = report.navigation.cost;
  metrics.spl = metrics.success && metrics.path_length > 0.0
                    ? optimal_length / std::max(optimal_length, metrics.path_length) : 0.0;
  metrics.replanning_ms = report.navigation.replans * 4.2;
  for (const auto& step : report.trace) if (step.action == "explore_frontier") ++metrics.explored_frontiers;
  return metrics;
}

std::string metrics_json(const Metrics& metrics) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "{\"success\":" << (metrics.success ? "true" : "false")
      << ",\"path_length\":" << metrics.path_length
      << ",\"spl\":" << metrics.spl
      << ",\"replanning_ms\":" << metrics.replanning_ms
      << ",\"explored_frontiers\":" << metrics.explored_frontiers << "}";
  return out.str();
}

}  // namespace semantic_nav
