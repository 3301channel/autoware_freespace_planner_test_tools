// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include "parking_trajectory_evaluator/param.hpp"
#include "parking_trajectory_evaluator/result.hpp"

namespace parking_trajectory_evaluator
{

/// Computes all kinematic metrics of a trajectory.
/// Pure function: no ROS types involved.
Metrics computeMetrics(const TrajectoryData & trajectory, const Params & params);

double wrapToPi(double angle);

}  // namespace parking_trajectory_evaluator
