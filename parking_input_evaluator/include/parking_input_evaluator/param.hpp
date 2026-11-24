// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <string>

namespace parking_input_evaluator
{

/// Vehicle geometry, kept consistent with autoware_freespace_planner.
/// freespace_planner_node.cpp:442-445 adds the margin exactly once.
struct VehicleParams
{
  double length{4.5136};      // bumper to bumper
  double width{2.0068};       // including mirrors
  double wheel_base{2.831};
  double max_steer{0.70};     // [rad]
  double margin{0.05};

  double collisionLength() const { return length + margin; }
  double collisionWidth() const { return width + margin; }
};

struct GridParams
{
  std::string expected_frame{"map"};
  double expected_resolution{0.3};
  double resolution_tolerance{0.001};
  double expected_rate_hz{10.0};
  double rate_tolerance{0.2};
  int obstacle_threshold{100};
  double coverage_fail{0.75};
  double coverage_pass{0.95};
  double unknown_fail{0.20};
  double unknown_pass{0.05};
};

struct GoalParams
{
  double angle_tolerance_pass_deg{6.0};
  double angle_tolerance_warn_deg{18.0};
  bool require_in_parking_space{true};
  int rect_samples_long{11};
  int rect_samples_lat{7};
};

struct StartParams
{
  double route_gap_fail{3.0};
  double route_gap_pass{1.0};
  double odom_rate_min{5.0};
  double odom_rate_max{20.0};
};

struct ScenarioParams
{
  std::string required{"Parking"};
  double ratio_fail{0.30};
  double ratio_pass{0.90};
  bool enabled{true};
};

struct ConnectivityParams
{
  double edt_margin_pass{0.10};
  double edt_margin_warn{-0.05};
  bool strict_rect_connectivity{false};
  int neighbors{8};
};

struct ParkingSpaceParams
{
  bool enabled{true};
  std::string check_mode{"center"};  // center | multi_pose
  int multi_pose_count{3};
  double width_margin_fail{0.10};
  double width_margin_pass{0.20};
  double length_margin_fail{0.05};
  double length_margin_pass{0.20};
  double parkable_ratio_fail{0.50};
  double parkable_ratio_pass{0.90};
};

struct TfParams
{
  std::string global_frame{"map"};
  std::string base_frame{"base_link"};
  double timeout_sec{0.1};
  bool missing_is_fail{false};
};

struct Params
{
  VehicleParams vehicle;
  GridParams grid;
  GoalParams goal;
  StartParams start;
  ScenarioParams scenario;
  ConnectivityParams connectivity;
  ParkingSpaceParams spaces;
  TfParams tf;

  std::string lanelet2_map_path{};
  std::string vector_map_topic{"/map/vector_map"};

  double update_rate{5.0};
  int debounce_frames{3};
  bool publish_marker{true};
  bool enable_service{true};
};

}  // namespace parking_input_evaluator
