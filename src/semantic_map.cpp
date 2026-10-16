#include "semantic_nav/semantic_map.hpp"

#include <algorithm>
#include <cmath>

namespace semantic_nav {

SemanticMap::SemanticMap(int width, int height, double resolution)
    : width_(width), height_(height), resolution_(resolution),
      cells_(static_cast<std::size_t>(width * height), CellState::Unknown) {}

bool SemanticMap::in_bounds(GridIndex cell) const {
  return cell.x >= 0 && cell.y >= 0 && cell.x < width_ && cell.y < height_;
}

void SemanticMap::set_cell(GridIndex cell, CellState state) {
  if (in_bounds(cell)) cells_[static_cast<std::size_t>(cell.y * width_ + cell.x)] = state;
}

CellState SemanticMap::cell(GridIndex cell) const {
  if (!in_bounds(cell)) return CellState::Occupied;
  return cells_[static_cast<std::size_t>(cell.y * width_ + cell.x)];
}

GridIndex SemanticMap::world_to_grid(double x, double y) const {
  return {static_cast<int>(std::floor(x / resolution_ + width_ / 2.0)),
          static_cast<int>(std::floor(y / resolution_ + height_ / 2.0))};
}

Pose2D SemanticMap::grid_to_world(GridIndex cell) const {
  return {(cell.x - width_ / 2.0 + 0.5) * resolution_,
          (cell.y - height_ / 2.0 + 0.5) * resolution_, 0.0};
}

std::string SemanticMap::make_id(const std::string& label, GridIndex cell) const {
  return label + "@" + std::to_string(cell.x) + ":" + std::to_string(cell.y);
}

void SemanticMap::integrate_detection(const Pose2D& robot_pose, const Detection& detection) {
  const double angle = robot_pose.yaw + detection.bearing_rad;
  const Pose2D world{robot_pose.x + detection.depth_m * std::cos(angle),
                    robot_pose.y + detection.depth_m * std::sin(angle), 0.0};
  const auto grid = world_to_grid(world.x, world.y);
  if (!in_bounds(grid)) return;
  const auto id = make_id(detection.label, grid);
  auto& object = objects_[id];
  if (object.id.empty()) {
    object.id = id;
    object.label = detection.label;
    object.pose = world;
  } else {
    const double n = static_cast<double>(object.observations);
    object.pose.x = (object.pose.x * n + world.x) / (n + 1.0);
    object.pose.y = (object.pose.y * n + world.y) / (n + 1.0);
  }
  object.confidence = std::max(object.confidence, detection.confidence);
  ++object.observations;
  set_cell(grid, CellState::Free);
}

std::optional<SemanticObject> SemanticMap::find_object(const std::string& label) const {
  std::optional<SemanticObject> best;
  for (const auto& [_, object] : objects_) {
    if (object.label == label && (!best || object.confidence > best->confidence)) best = object;
  }
  return best;
}

std::vector<SemanticObject> SemanticMap::objects() const {
  std::vector<SemanticObject> result;
  for (const auto& [_, object] : objects_) result.push_back(object);
  return result;
}

}  // namespace semantic_nav
