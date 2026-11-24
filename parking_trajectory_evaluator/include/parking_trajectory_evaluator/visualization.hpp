// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <string>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "parking_trajectory_evaluator/result.hpp"

namespace parking_trajectory_evaluator
{

/// Trajectory coloured by verdict, reverse segments highlighted, terminal arrow.
visualization_msgs::msg::MarkerArray buildMarkers(
  const EvaluationResult & result, const TrajectoryData & trajectory, const std::string & frame_id,
  const rclcpp::Time & stamp, const std::string & ns_prefix = "parking_trajectory");

}  // namespace parking_trajectory_evaluator
