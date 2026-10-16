#pragma once

#include "semantic_nav/types.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace semantic_nav {

class SemanticMap {
 public:
  SemanticMap(int width, int height, double resolution);

  void set_cell(GridIndex cell, CellState state);
  CellState cell(GridIndex cell) const;
  bool in_bounds(GridIndex cell) const;
  GridIndex world_to_grid(double x, double y) const;
  Pose2D grid_to_world(GridIndex cell) const;

  // Represents YOLO + RGB-D + TF2 fusion in the standalone simulator.
  void integrate_detection(const Pose2D& robot_pose, const Detection& detection);
  std::optional<SemanticObject> find_object(const std::string& label) const;
  std::vector<SemanticObject> objects() const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }

 private:
  std::string make_id(const std::string& label, GridIndex cell) const;
  int width_;
  int height_;
  double resolution_;
  std::vector<CellState> cells_;
  std::unordered_map<std::string, SemanticObject> objects_;
};

}  // namespace semantic_nav
