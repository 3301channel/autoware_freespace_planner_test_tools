// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_trajectory_evaluator/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace parking_trajectory_evaluator
{
namespace
{

struct Cell
{
  int ix{-1};
  int iy{-1};
  bool inside{false};
};

Cell worldToCell(double x, double y, const GridData & grid)
{
  Cell cell;
  cell.ix = static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution));
  cell.iy = static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution));
  cell.inside = cell.ix >= 0 && cell.ix < grid.width && cell.iy >= 0 && cell.iy < grid.height;
  return cell;
}

bool isBlocked(int8_t cost, int threshold) { return cost < 0 || cost >= threshold; }

int8_t costAt(const GridData & grid, int ix, int iy)
{
  return grid.data[static_cast<size_t>(iy) * grid.width + ix];
}

/// Distance from (x, y) to the nearest blocked cell within `radius` metres.
double clearanceAt(double x, double y, const GridData & grid, int threshold, double radius)
{
  const int r = static_cast<int>(std::ceil(radius / grid.resolution));
  const Cell centre = worldToCell(x, y, grid);
  if (!centre.inside) {
    return 0.0;
  }

  double best = radius;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const int ix = centre.ix + dx;
      const int iy = centre.iy + dy;
      if (ix < 0 || iy < 0 || ix >= grid.width || iy >= grid.height) {
        continue;
      }
      if (!isBlocked(costAt(grid, ix, iy), threshold)) {
        continue;
      }
      const double cx = grid.origin_x + (ix + 0.5) * grid.resolution;
      const double cy = grid.origin_y + (iy + 0.5) * grid.resolution;
      best = std::min(best, std::hypot(x - cx, y - cy));
    }
  }
  return best;
}

}  // namespace

CollisionResult checkCollision(
  const TrajectoryData & trajectory, const GridData & grid, const VehicleParams & vehicle,
  int cost_threshold, int samples_long, int samples_lat)
{
  CollisionResult result;
  if (!grid.valid() || trajectory.points.empty()) {
    return result;
  }

  result.evaluated = true;
  result.min_clearance = std::numeric_limits<double>::max();

  const int n_long = std::max(2, samples_long);
  const int n_lat = std::max(2, samples_lat);
  const double half_length = vehicle.length / 2.0;
  const double half_width = vehicle.width / 2.0;

  for (const auto & point : trajectory.points) {
    const Cell centre = worldToCell(point.x, point.y, grid);
    if (centre.inside && isBlocked(costAt(grid, centre.ix, centre.iy), cost_threshold)) {
      ++result.center_collisions;
    }

    const double c = std::cos(point.yaw);
    const double s = std::sin(point.yaw);
    bool footprint_hit = false;
    for (int i = 0; i < n_long && !footprint_hit; ++i) {
      const double a = -half_length + vehicle.length * static_cast<double>(i) / (n_long - 1);
      for (int j = 0; j < n_lat; ++j) {
        const double b = -half_width + vehicle.width * static_cast<double>(j) / (n_lat - 1);
        const double x = point.x + a * c - b * s;
        const double y = point.y + a * s + b * c;
        const Cell cell = worldToCell(x, y, grid);
        if (!cell.inside) {
          // Treat "outside the grid" as occupied: the planner has no evidence
          // that the cell is free.
          footprint_hit = true;
          break;
        }
        if (isBlocked(costAt(grid, cell.ix, cell.iy), cost_threshold)) {
          footprint_hit = true;
          break;
        }
      }
    }
    if (footprint_hit) {
      ++result.footprint_collisions;
    }

    if (centre.inside) {
      result.min_clearance =
        std::min(result.min_clearance, clearanceAt(point.x, point.y, grid, cost_threshold, 2.0));
    }
  }

  if (result.min_clearance == std::numeric_limits<double>::max()) {
    result.min_clearance = 0.0;
  }
  return result;
}

}  // namespace parking_trajectory_evaluator
