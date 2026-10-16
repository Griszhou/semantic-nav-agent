#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace semantic_nav {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct GridIndex {
  int x{0};
  int y{0};
  bool operator==(const GridIndex& other) const { return x == other.x && y == other.y; }
};

enum class CellState : std::uint8_t { Unknown, Free, Occupied };

struct SemanticObject {
  std::string id;
  std::string label;
  Pose2D pose;
  double confidence{0.0};
  std::uint32_t observations{0};
};

struct Detection {
  std::string label;
  double confidence{0.0};
  double depth_m{0.0};
  double bearing_rad{0.0};
};

struct Frontier {
  GridIndex cell;
  double information_gain{0.0};
  double path_cost{0.0};
  double risk{0.0};
  double semantic_relevance{0.0};
  double utility{0.0};
};

struct NavigationResult {
  bool success{false};
  std::string reason;
  std::vector<GridIndex> path;
  double cost{0.0};
  int replans{0};
};

inline double distance(const Pose2D& a, const Pose2D& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace semantic_nav
