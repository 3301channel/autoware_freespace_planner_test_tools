// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "parking_trajectory_evaluator/scoring.hpp"
#include "parking_trajectory_evaluator/trajectory_metrics.hpp"

using namespace parking_trajectory_evaluator;

namespace
{

Params baseParams()
{
  Params params;
  params.goal.enabled = false;  // terminal accuracy skipped by default in these tests
  return params;
}

TrajectoryData makeStraight(int count, double spacing, double velocity)
{
  TrajectoryData trajectory;
  for (int i = 0; i < count; ++i) {
    PointData p;
    p.x = i * spacing;
    p.y = 0.0;
    p.yaw = 0.0;
    p.v = velocity;
    trajectory.points.push_back(p);
  }
  return trajectory;
}

TrajectoryData makeArc(int count, double spacing, double radius, double velocity)
{
  TrajectoryData trajectory;
  for (int i = 0; i < count; ++i) {
    const double theta = i * spacing / radius;
    PointData p;
    p.x = radius * std::sin(theta);
    p.y = radius * (1.0 - std::cos(theta));
    p.yaw = theta;
    p.v = velocity;
    trajectory.points.push_back(p);
  }
  return trajectory;
}

bool mentions(const std::vector<std::string> & reasons, const std::string & needle)
{
  for (const auto & r : reasons) {
    if (r.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(Scoring, GoodTrajectoryIsControlReady)
{
  auto params = baseParams();
  auto trajectory = makeStraight(20, 0.5, 1.0);
  trajectory.points.back().v = 0.0;

  const auto result = evaluateTrajectory(trajectory, nullptr, params);
  EXPECT_TRUE(result.is_control_ready)
    << (result.failure_reasons.empty() ? "" : result.failure_reasons.front());
  EXPECT_GE(result.score.total, params.scoring.min_score);
  EXPECT_TRUE(result.occupancy_evaluated == false);
}

TEST(Scoring, OverspeedBreaksHardConstraint)
{
  auto params = baseParams();
  auto trajectory = makeStraight(20, 0.5, 5.0);
  trajectory.points.back().v = 0.0;

  const auto result = evaluateTrajectory(trajectory, nullptr, params);
  EXPECT_FALSE(result.is_control_ready);
  EXPECT_TRUE(mentions(result.failure_reasons, "velocity"));
}

TEST(Scoring, NonZeroTerminalVelocityFails)
{
  auto params = baseParams();
  const auto trajectory = makeStraight(20, 0.5, 1.0);

  const auto result = evaluateTrajectory(trajectory, nullptr, params);
  EXPECT_FALSE(result.is_control_ready);
  EXPECT_TRUE(mentions(result.failure_reasons, "terminal velocity"));
}

TEST(Scoring, SharpCurvatureFails)
{
  auto params = baseParams();
  // radius 2 m -> curvature 0.5 > 0.398
  auto trajectory = makeArc(40, 0.2, 2.0, 0.5);
  trajectory.points.back().v = 0.0;

  const auto result = evaluateTrajectory(trajectory, nullptr, params);
  EXPECT_FALSE(result.is_control_ready);
  EXPECT_TRUE(mentions(result.failure_reasons, "curvature"));
  EXPECT_GT(result.metrics.max_curvature, params.constraints.max_curvature);
}

TEST(Scoring, TooFewPointsFails)
{
  auto params = baseParams();
  const auto trajectory = makeStraight(3, 0.5, 0.5);

  const auto result = evaluateTrajectory(trajectory, nullptr, params);
  EXPECT_FALSE(result.is_control_ready);
  EXPECT_TRUE(mentions(result.failure_reasons, "too few"));
}

TEST(Scoring, GoalErrorFails)
{
  auto params = baseParams();
  params.goal.enabled = true;
  params.goal.x = 100.0;  // far from the trajectory end
  params.goal.y = 100.0;

  auto trajectory = makeStraight(20, 0.5, 1.0);
  trajectory.points.back().v = 0.0;

  const auto result = evaluateTrajectory(trajectory, nullptr, params);
  EXPECT_FALSE(result.is_control_ready);
  EXPECT_TRUE(result.goal_configured);
  EXPECT_GT(result.lat_error + result.lon_error, 1.0);
}

TEST(Scoring, GoalErrorIsZeroAtExactGoal)
{
  auto params = baseParams();
  params.goal.enabled = true;
  auto trajectory = makeStraight(20, 0.5, 1.0);
  trajectory.points.back().v = 0.0;

  const auto & terminal = trajectory.points.back();
  params.goal.x = terminal.x;
  params.goal.y = terminal.y;
  params.goal.yaw = terminal.yaw;

  const auto result = evaluateTrajectory(trajectory, nullptr, params);
  EXPECT_NEAR(result.lat_error, 0.0, 1e-9);
  EXPECT_NEAR(result.lon_error, 0.0, 1e-9);
  EXPECT_NEAR(result.yaw_error, 0.0, 1e-9);
  EXPECT_TRUE(result.is_control_ready);
}

TEST(Scoring, ScoreDecreasesWithCurvatureVariance)
{
  auto params = baseParams();

  auto smooth = makeArc(60, 0.2, 20.0, 0.5);
  smooth.points.back().v = 0.0;

  // Alternating curvature increases the variance.
  TrajectoryData wiggly;
  double yaw = 0.0;
  double x = 0.0;
  double y = 0.0;
  for (int i = 0; i < 60; ++i) {
    PointData p;
    p.x = x;
    p.y = y;
    p.yaw = yaw;
    p.v = 0.5;
    wiggly.points.push_back(p);
    const double delta = (i % 2 == 0) ? 0.08 : -0.08;
    yaw = wrapToPi(yaw + delta);
    x += 0.2 * std::cos(yaw);
    y += 0.2 * std::sin(yaw);
  }
  wiggly.points.back().v = 0.0;

  const auto smooth_result = evaluateTrajectory(smooth, nullptr, params);
  const auto wiggly_result = evaluateTrajectory(wiggly, nullptr, params);
  EXPECT_GT(smooth_result.score.smoothness, wiggly_result.score.smoothness);
}

TEST(Scoring, ScoreNeverExceedsOneHundred)
{
  auto params = baseParams();
  auto trajectory = makeStraight(20, 0.5, 1.0);
  trajectory.points.back().v = 0.0;

  const auto result = evaluateTrajectory(trajectory, nullptr, params);
  EXPECT_LE(result.score.total, 100.0 + 1e-6);
  EXPECT_GE(result.score.total, 0.0);
}
