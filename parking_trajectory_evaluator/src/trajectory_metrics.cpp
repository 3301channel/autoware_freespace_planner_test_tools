// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_trajectory_evaluator/trajectory_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace parking_trajectory_evaluator
{

double wrapToPi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Metrics computeMetrics(const TrajectoryData & trajectory, const Params & params)
{
  Metrics metrics;
  const auto & points = trajectory.points;
  metrics.point_count = static_cast<int>(points.size());

  if (points.empty()) {
    return metrics;
  }

  for (const auto & p : points) {
    if (
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
      !std::isfinite(p.v)) {
      metrics.has_nan = true;
      return metrics;
    }
  }
  metrics.valid = true;

  if (points.size() < 2) {
    metrics.min_interval = 0.0;
    metrics.terminal_velocity = std::fabs(points.front().v);
    metrics.max_abs_velocity = metrics.terminal_velocity;
    return metrics;
  }

  const double min_v = std::max(params.constraints.min_velocity_for_dt, 1e-3);

  std::vector<double> intervals;
  std::vector<double> curvatures;
  std::vector<double> steer_angles;
  std::vector<double> lateral_acc;
  std::vector<double> longitudinal_acc;
  intervals.reserve(points.size() - 1);
  curvatures.reserve(points.size() - 1);
  steer_angles.reserve(points.size() - 1);
  lateral_acc.reserve(points.size() - 1);
  longitudinal_acc.reserve(points.size() - 1);

  double min_interval = std::numeric_limits<double>::max();
  double max_interval = 0.0;

  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const double dx = points[i + 1].x - points[i].x;
    const double dy = points[i + 1].y - points[i].y;
    const double ds = std::hypot(dx, dy);
    intervals.push_back(ds);
    min_interval = std::min(min_interval, ds);
    max_interval = std::max(max_interval, ds);

    const double dtheta = wrapToPi(points[i + 1].yaw - points[i].yaw);
    metrics.max_delta_angle = std::max(metrics.max_delta_angle, std::fabs(dtheta));

    const double curvature = ds > 1e-6 ? dtheta / ds : 0.0;
    curvatures.push_back(curvature);
    metrics.max_curvature = std::max(metrics.max_curvature, std::fabs(curvature));

    const double steer = std::atan(params.vehicle.wheel_base * curvature);
    steer_angles.push_back(steer);
    metrics.max_steer = std::max(metrics.max_steer, std::fabs(steer));

    // dt from arc length and the speed at the segment start.
    const double dt = std::max(ds / std::max(std::fabs(points[i].v), min_v), 1e-3);
    const double a_lat = points[i].v * points[i].v * curvature;
    lateral_acc.push_back(a_lat);
    metrics.max_lateral_acc = std::max(metrics.max_lateral_acc, std::fabs(a_lat));

    const double a_lon = (points[i + 1].v - points[i].v) / dt;
    longitudinal_acc.push_back(a_lon);
    metrics.max_longitudinal_acc = std::max(metrics.max_longitudinal_acc, std::fabs(a_lon));
  }

  // Steering rate needs two consecutive steering angles.
  for (size_t i = 0; i + 1 < steer_angles.size(); ++i) {
    const double dt = std::max(intervals[i] / std::max(std::fabs(points[i].v), min_v), 1e-3);
    metrics.max_steer_rate = std::max(
      metrics.max_steer_rate, std::fabs(steer_angles[i + 1] - steer_angles[i]) / dt);
  }

  metrics.min_interval = min_interval == std::numeric_limits<double>::max() ? 0.0 : min_interval;
  metrics.max_interval = max_interval;

  // Jerk from finite differences of the accelerations.
  for (size_t i = 0; i + 1 < lateral_acc.size(); ++i) {
    const double dt = std::max(intervals[i] / std::max(std::fabs(points[i].v), min_v), 1e-3);
    metrics.max_lateral_jerk =
      std::max(metrics.max_lateral_jerk, std::fabs(lateral_acc[i + 1] - lateral_acc[i]) / dt);
    metrics.max_longitudinal_jerk = std::max(
      metrics.max_longitudinal_jerk, std::fabs(longitudinal_acc[i + 1] - longitudinal_acc[i]) / dt);
  }

  for (const auto & p : points) {
    metrics.max_abs_velocity = std::max(metrics.max_abs_velocity, std::fabs(p.v));
  }
  metrics.terminal_velocity = std::fabs(points.back().v);

  // Direction switches, ignoring near-zero speeds.
  const double eps = std::max(params.constraints.terminal_velocity, 1e-3);
  for (size_t i = 1; i < points.size(); ++i) {
    if (
      std::fabs(points[i].v) > eps && std::fabs(points[i - 1].v) > eps &&
      (points[i].v > 0.0) != (points[i - 1].v > 0.0)) {
      ++metrics.switch_count;
    }
  }

  // Curvature variance as a smoothness proxy.
  if (!curvatures.empty()) {
    double mean = 0.0;
    for (const auto c : curvatures) {
      mean += c;
    }
    mean /= static_cast<double>(curvatures.size());
    double var = 0.0;
    for (const auto c : curvatures) {
      var += (c - mean) * (c - mean);
    }
    metrics.curvature_variance = var / static_cast<double>(curvatures.size());
  }

  metrics.valid = true;
  return metrics;
}

}  // namespace parking_trajectory_evaluator
