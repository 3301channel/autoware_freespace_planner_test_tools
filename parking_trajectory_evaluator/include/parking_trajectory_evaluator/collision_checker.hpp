// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "parking_trajectory_evaluator/param.hpp"
#include "parking_trajectory_evaluator/result.hpp"

namespace parking_trajectory_evaluator
{

/// Minimal occupancy grid view, mirroring nav_msgs/OccupancyGrid semantics.
struct GridData
{
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::string frame_id;
  std::vector<int8_t> data;

  bool valid() const
  {
    return width > 0 && height > 0 && resolution > 0.0 &&
           data.size() == static_cast<size_t>(width) * height;
  }
};

/// Checks every trajectory point against the occupancy grid.
/// A point is a centre collision when its own cell is blocked; a footprint
/// collision when any sample of the rotated vehicle rectangle is blocked.
CollisionResult checkCollision(
  const TrajectoryData & trajectory, const GridData & grid, const VehicleParams & vehicle,
  int cost_threshold, int samples_long = 11, int samples_lat = 7);

}  // namespace parking_trajectory_evaluator
