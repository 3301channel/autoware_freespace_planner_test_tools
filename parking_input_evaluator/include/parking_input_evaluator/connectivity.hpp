// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <cstdint>
#include <vector>

#include "parking_input_evaluator/grid_utils.hpp"

namespace parking_input_evaluator
{

struct GridPoint
{
  int ix{-1};
  int iy{-1};
  bool inside{false};
};

struct ConnectivityResult
{
  /// Euclidean distance to the nearest obstacle, in metres, per cell.
  std::vector<float> edt_m;
  /// 1 = free and distance >= half vehicle width (same rule as astar_search.cpp:262).
  std::vector<uint8_t> passable;
  int passable_count{0};
  double half_width{0.0};

  bool start_inside{false};
  bool goal_inside{false};

  /// F1: reachable on the raw free space.
  bool free_connected{false};
  /// F2: reachable on the A*-equivalent passable set.
  bool passable_connected{false};

  bool start_passable{false};
  bool goal_passable{false};

  /// Component label of every cell on the passable set, row-major.
  /// 0 means not passable. Used to classify individual parking bays.
  std::vector<int32_t> passable_labels;
  /// Component label at the start cell (0 when the start is not passable).
  int32_t start_label{0};
};

/// Computes EDT, the A*-equivalent passable mask and connectivity between two cells.
ConnectivityResult analyzeConnectivity(
  const GridData & grid, int obstacle_threshold, double half_width, const GridPoint & start,
  const GridPoint & goal, int neighbors);

/// Convenience: distance (metres) from an arbitrary cell to the nearest obstacle.
double edtAt(const ConnectivityResult & result, const GridData & grid, int ix, int iy);

}  // namespace parking_input_evaluator
