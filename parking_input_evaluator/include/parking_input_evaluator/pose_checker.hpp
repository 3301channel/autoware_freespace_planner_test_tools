// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <string>

#include "parking_input_evaluator/grid_utils.hpp"
#include "parking_input_evaluator/map_parser.hpp"

namespace parking_input_evaluator
{

struct PoseCheckResult
{
  bool inside{false};
  int cost{-1};
  bool free{false};

  RectCheck rect;

  int nearest_space_index{-1};
  double nearest_space_distance{1e9};
  double heading_diff_deg{180.0};
  bool in_space_band{false};

  /// True when the pose is inside the grid, on a free cell, fits the vehicle
  /// footprint and (optionally) lies inside a parking space band.
  bool acceptable{false};
};

PoseCheckResult checkPose(
  const GridData & grid, double x, double y, double yaw, double collision_length,
  double collision_width, int obstacle_threshold, int samples_long, int samples_lat,
  const MapGeometry * map, bool require_in_space);

double yawFromQuaternion(double x, double y, double z, double w);
double wrapToPi(double angle);
/// Smallest angle between two headings, in degrees, folded into [0, 180].
double headingDiffDeg(double a, double b);

}  // namespace parking_input_evaluator
