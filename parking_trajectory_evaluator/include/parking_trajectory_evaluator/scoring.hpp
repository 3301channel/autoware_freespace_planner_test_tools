// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include "parking_trajectory_evaluator/collision_checker.hpp"
#include "parking_trajectory_evaluator/param.hpp"
#include "parking_trajectory_evaluator/result.hpp"

namespace parking_trajectory_evaluator
{

struct GoalError
{
  double lat{0.0};
  double lon{0.0};
  double yaw{0.0};
  bool configured{false};
};

GoalError computeGoalError(const PointData & terminal, const GoalParams & goal);

/// Full evaluation: hard constraints + soft scoring + verdict.
/// `grid` may be nullptr when occupancy checking is disabled or unavailable.
EvaluationResult evaluateTrajectory(
  const TrajectoryData & trajectory, const GridData * grid, const Params & params);

}  // namespace parking_trajectory_evaluator
