// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace parking_input_evaluator
{

struct GridInfo
{
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::string frame_id;
};

/// Mirrors nav_msgs/OccupancyGrid: row-major, int8, -1 = unknown.
struct GridData
{
  GridInfo info;
  std::vector<int8_t> data;

  bool valid() const
  {
    return info.width > 0 && info.height > 0 && info.resolution > 0.0 &&
           data.size() == static_cast<size_t>(info.width) * info.height;
  }
};

struct CellIndex
{
  int ix{-1};
  int iy{-1};
  bool inside{false};
};

CellIndex worldToCell(double x, double y, const GridInfo & info);
CellIndex worldToCell(double x, double y, const GridData & grid);
double cellCenterX(int ix, const GridInfo & info);
double cellCenterY(int iy, const GridInfo & info);

int8_t costAt(const GridData & grid, int ix, int iy);

/// Matches abstract_algorithm.cpp:119 -> cost < 0 || threshold <= cost.
inline bool isObstacle(int8_t cost, int threshold) { return cost < 0 || cost >= threshold; }

struct RectCheck
{
  bool ok{true};
  bool out_of_grid{false};
  int conflicts{0};
  int first_ix{-1};
  int first_iy{-1};
};

/// Samples the vehicle footprint (length x width) at pose (cx, cy, yaw).
RectCheck sampleRect(
  const GridData & grid, double cx, double cy, double yaw, double length, double width,
  int samples_long, int samples_lat, int obstacle_threshold);

/// Counts of each occupancy class inside the grid.
struct CostStats
{
  size_t total{0};
  size_t free_cells{0};
  size_t occupied_cells{0};
  size_t unknown_cells{0};

  double freeRatio() const { return total ? static_cast<double>(free_cells) / total : 0.0; }
  double occupiedRatio() const
  {
    return total ? static_cast<double>(occupied_cells) / total : 0.0;
  }
  double unknownRatio() const
  {
    return total ? static_cast<double>(unknown_cells) / total : 0.0;
  }
};

CostStats computeCostStats(const GridData & grid, int obstacle_threshold);

}  // namespace parking_input_evaluator
