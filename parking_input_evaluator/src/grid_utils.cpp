// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_input_evaluator/grid_utils.hpp"

#include <algorithm>
#include <cmath>

namespace parking_input_evaluator
{

CellIndex worldToCell(double x, double y, const GridInfo & info)
{
  CellIndex idx;
  if (info.resolution <= 0.0) {
    return idx;
  }
  idx.ix = static_cast<int>(std::floor((x - info.origin_x) / info.resolution));
  idx.iy = static_cast<int>(std::floor((y - info.origin_y) / info.resolution));
  idx.inside = idx.ix >= 0 && idx.ix < info.width && idx.iy >= 0 && idx.iy < info.height;
  return idx;
}

CellIndex worldToCell(double x, double y, const GridData & grid) { return worldToCell(x, y, grid.info); }

double cellCenterX(int ix, const GridInfo & info)
{
  return info.origin_x + (static_cast<double>(ix) + 0.5) * info.resolution;
}

double cellCenterY(int iy, const GridInfo & info)
{
  return info.origin_y + (static_cast<double>(iy) + 0.5) * info.resolution;
}

int8_t costAt(const GridData & grid, int ix, int iy)
{
  if (ix < 0 || iy < 0 || ix >= grid.info.width || iy >= grid.info.height) {
    return static_cast<int8_t>(-1);
  }
  return grid.data[static_cast<size_t>(iy) * grid.info.width + ix];
}

RectCheck sampleRect(
  const GridData & grid, double cx, double cy, double yaw, double length, double width,
  int samples_long, int samples_lat, int obstacle_threshold)
{
  RectCheck result;
  const int n_long = std::max(2, samples_long);
  const int n_lat = std::max(2, samples_lat);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  bool has_first = false;

  for (int i = 0; i < n_long; ++i) {
    const double a = -length / 2.0 + length * static_cast<double>(i) / (n_long - 1);
    for (int j = 0; j < n_lat; ++j) {
      const double b = -width / 2.0 + width * static_cast<double>(j) / (n_lat - 1);
      const double x = cx + a * c - b * s;
      const double y = cy + a * s + b * c;
      const CellIndex idx = worldToCell(x, y, grid.info);
      if (!idx.inside) {
        result.ok = false;
        result.out_of_grid = true;
        result.first_ix = idx.ix;
        result.first_iy = idx.iy;
        return result;
      }
      if (isObstacle(costAt(grid, idx.ix, idx.iy), obstacle_threshold)) {
        if (!has_first) {
          result.first_ix = idx.ix;
          result.first_iy = idx.iy;
          has_first = true;
        }
        ++result.conflicts;
      }
    }
  }
  result.ok = (result.conflicts == 0);
  return result;
}

CostStats computeCostStats(const GridData & grid, int obstacle_threshold)
{
  CostStats stats;
  stats.total = grid.data.size();
  for (const auto cost : grid.data) {
    if (cost < 0) {
      ++stats.unknown_cells;
    } else if (cost >= obstacle_threshold) {
      ++stats.occupied_cells;
    } else {
      ++stats.free_cells;
    }
  }
  return stats;
}

}  // namespace parking_input_evaluator
