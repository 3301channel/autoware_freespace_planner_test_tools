// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <cmath>
#include <string>

namespace parking_trajectory_evaluator
{

struct VehicleParams
{
  double wheel_base{2.831};
  double max_steer{0.70};      // [rad]
  double length{4.5136};
  double width{2.0068};
  double rear_overhang{1.1};
};

struct ConstraintParams
{
  double min_point_interval{0.01};      // [m]
  double max_point_interval{2.0};       // [m]
  double max_curvature{0.398};          // [1/m]
  double max_steer_rate{0.3};           // [rad/s]
  double max_velocity{1.4};             // [m/s]
  double terminal_velocity{0.05};       // [m/s]
  int min_points{5};
  double max_delta_angle{0.5};          // [rad]
  int occupancy_cost_threshold{50};
  double min_velocity_for_dt{0.1};      // [m/s] avoid division by zero
};

struct ComfortParams
{
  double max_lateral_acc{2.0};          // [m/s^2]
  double max_lateral_jerk{1.5};         // [m/s^3]
  double max_longitudinal_acc{3.0};     // [m/s^2]
  double max_longitudinal_jerk{2.0};    // [m/s^3]
  int max_switch_count{4};
};

struct GoalParams
{
  bool enabled{true};
  double x{273.215};
  double y{236.173};
  double yaw{3.14159};
  double tolerance_lat{0.3};            // [m]
  double tolerance_lon{0.5};            // [m]
  double tolerance_yaw{0.1745};         // [rad]
};

struct ScoringParams
{
  double min_score{70.0};
};

struct Params
{
  VehicleParams vehicle;
  ConstraintParams constraints;
  ComfortParams comfort;
  GoalParams goal;
  ScoringParams scoring;

  std::string evaluation_mode{"single"};  // single | cumulative
  bool enable_occupancy_check{true};
  bool enable_visualization{true};
  double publish_rate{10.0};
  bool require_parking_scenario{false};
};

}  // namespace parking_trajectory_evaluator
