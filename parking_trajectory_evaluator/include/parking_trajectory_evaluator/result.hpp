// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace parking_trajectory_evaluator
{

inline double quietNaN() { return std::numeric_limits<double>::quiet_NaN(); }

struct PointData
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
};

struct TrajectoryData
{
  std::string frame_id;
  double stamp{0.0};
  std::vector<PointData> points;
};

struct Metrics
{
  bool valid{false};
  bool has_nan{false};
  int point_count{0};

  double min_interval{0.0};
  double max_interval{0.0};
  double max_curvature{0.0};
  double max_steer{0.0};
  double max_steer_rate{0.0};
  double max_abs_velocity{0.0};
  double terminal_velocity{0.0};
  int switch_count{0};
  double max_lateral_acc{0.0};
  double max_lateral_jerk{0.0};
  double max_longitudinal_acc{0.0};
  double max_longitudinal_jerk{0.0};
  double max_delta_angle{0.0};
  double curvature_variance{0.0};
};

struct ScoreBreakdown
{
  double terminal_lat{0.0};
  double terminal_lon{0.0};
  double terminal_yaw{0.0};
  double switch_count{0.0};
  double lateral_acc{0.0};
  double lateral_jerk{0.0};
  double clearance{0.0};
  double smoothness{0.0};
  double total{0.0};
};

struct MetricItem
{
  std::string name;
  std::string unit;
  double value{0.0};
  double threshold{quietNaN()};
  bool passed{false};
};

struct CollisionResult
{
  bool evaluated{false};
  int center_collisions{0};
  int footprint_collisions{0};
  /// Smallest distance from a trajectory point to a blocked cell, in metres.
  double min_clearance{-1.0};
};

struct EvaluationResult
{
  Metrics metrics;
  ScoreBreakdown score;

  double lat_error{0.0};
  double lon_error{0.0};
  double yaw_error{0.0};
  bool goal_configured{false};

  bool is_control_ready{false};
  std::vector<std::string> failure_reasons;
  std::vector<MetricItem> items;

  bool occupancy_evaluated{false};
  CollisionResult collision;

  PointData terminal;
};

}  // namespace parking_trajectory_evaluator
