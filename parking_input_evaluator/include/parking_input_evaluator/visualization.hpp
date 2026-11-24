// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "parking_input_evaluator/grid_utils.hpp"
#include "parking_input_evaluator/map_parser.hpp"
#include "parking_input_evaluator/result.hpp"

namespace parking_input_evaluator
{

/// Builds the RViz overlay: grid extent, blind regions, bay colouring and goal arrow.
visualization_msgs::msg::MarkerArray buildMarkers(
  const Report & report, const GridData & grid, const MapGeometry & map,
  const geometry_msgs::msg::Pose & goal, bool has_goal, const std::string & frame_id,
  const rclcpp::Time & stamp, const std::string & ns_prefix = "parking_input");

}  // namespace parking_input_evaluator
