// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include <gtest/gtest.h>

#include <cmath>

#include "parking_trajectory_evaluator/trajectory_metrics.hpp"

using namespace parking_trajectory_evaluator;

namespace
{

TrajectoryData makeLine(int count, double spacing, double velocity)
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
    const double s = i * spacing;
    const double theta = s / radius;
    PointData p;
    p.x = radius * std::sin(theta);
    p.y = radius * (1.0 - std::cos(theta));
    p.yaw = theta;
    p.v = velocity;
    trajectory.points.push_back(p);
  }
  return trajectory;
}

}  // namespace

TEST(TrajectoryMetrics, StraightLineHasZeroCurvature)
{
  Params params;
  const auto metrics = computeMetrics(makeLine(20, 0.5, 1.0), params);
  ASSERT_TRUE(metrics.valid);
  EXPECT_EQ(metrics.point_count, 20);
  EXPECT_NEAR(metrics.max_curvature, 0.0, 1e-9);
  EXPECT_NEAR(metrics.max_steer, 0.0, 1e-9);
  EXPECT_NEAR(metrics.max_steer_rate, 0.0, 1e-9);
}

TEST(TrajectoryMetrics, ArcCurvatureMatchesRadius)
{
  Params params;
  const auto metrics = computeMetrics(makeArc(60, 0.2, 10.0, 1.0), params);
  ASSERT_TRUE(metrics.valid);
  EXPECT_NEAR(metrics.max_curvature, 0.1, 0.005);
}

TEST(TrajectoryMetrics, SteeringFollowsBicycleModel)
{
  Params params;
  const auto metrics = computeMetrics(makeArc(60, 0.2, 10.0, 1.0), params);
  const double expected = std::atan(params.vehicle.wheel_base * 0.1);
  EXPECT_NEAR(metrics.max_steer, expected, 0.01);
}

TEST(TrajectoryMetrics, NanIsDetected)
{
  auto trajectory = makeLine(10, 0.5, 1.0);
  trajectory.points[3].x = std::nan("");
  Params params;
  const auto metrics = computeMetrics(trajectory, params);
  EXPECT_TRUE(metrics.has_nan);
  EXPECT_FALSE(metrics.valid);
}

TEST(TrajectoryMetrics, EmptyTrajectoryIsInvalid)
{
  TrajectoryData trajectory;
  Params params;
  const auto metrics = computeMetrics(trajectory, params);
  EXPECT_FALSE(metrics.valid);
  EXPECT_EQ(metrics.point_count, 0);
}

TEST(TrajectoryMetrics, SwitchCount)
{
  TrajectoryData trajectory;
  for (int i = 0; i < 8; ++i) {
    PointData p;
    p.x = i * 0.5;
    p.v = 1.0;
    trajectory.points.push_back(p);
  }
  for (int i = 0; i < 8; ++i) {
    PointData p;
    p.x = 4.0 + i * 0.5;
    p.v = -1.0;
    trajectory.points.push_back(p);
  }
  Params params;
  const auto metrics = computeMetrics(trajectory, params);
  EXPECT_EQ(metrics.switch_count, 1);
}

TEST(TrajectoryMetrics, TerminalVelocityIsLastPoint)
{
  auto trajectory = makeLine(10, 0.5, 1.0);
  trajectory.points.back().v = 0.0;
  Params params;
  const auto metrics = computeMetrics(trajectory, params);
  EXPECT_NEAR(metrics.terminal_velocity, 0.0, 1e-9);
  EXPECT_NEAR(metrics.max_abs_velocity, 1.0, 1e-9);
}

TEST(TrajectoryMetrics, LargeGapIsDetected)
{
  auto trajectory = makeLine(10, 0.5, 1.0);
  for (size_t i = 5; i < trajectory.points.size(); ++i) {
    trajectory.points[i].x += 3.0;
  }
  Params params;
  const auto metrics = computeMetrics(trajectory, params);
  EXPECT_GT(metrics.max_interval, 2.0);
}

TEST(TrajectoryMetrics, LateralAccelerationGrowsWithSpeed)
{
  Params params;
  const auto slow = computeMetrics(makeArc(60, 0.2, 10.0, 0.5), params);
  const auto fast = computeMetrics(makeArc(60, 0.2, 10.0, 1.4), params);
  EXPECT_GT(fast.max_lateral_acc, slow.max_lateral_acc);
}
