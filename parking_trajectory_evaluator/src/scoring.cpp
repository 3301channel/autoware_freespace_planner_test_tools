// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_trajectory_evaluator/scoring.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "parking_trajectory_evaluator/trajectory_metrics.hpp"

namespace parking_trajectory_evaluator
{
namespace
{

/// Maps a value to [0, 1]. Works for both "smaller is better" (good < bad)
/// and "larger is better" (good > bad).
double linearScore(double value, double good, double bad)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  if (std::fabs(good - bad) < 1e-12) {
    return value <= good ? 1.0 : 0.0;
  }
  if (good < bad) {
    if (value <= good) {
      return 1.0;
    }
    if (value >= bad) {
      return 0.0;
    }
    return 1.0 - (value - good) / (bad - good);
  }
  // good > bad, larger values are better
  if (value >= good) {
    return 1.0;
  }
  if (value <= bad) {
    return 0.0;
  }
  return (value - bad) / (good - bad);
}

}  // namespace

GoalError computeGoalError(const PointData & terminal, const GoalParams & goal)
{
  GoalError error;
  if (!goal.enabled) {
    return error;
  }
  error.configured = true;

  const double dx = terminal.x - goal.x;
  const double dy = terminal.y - goal.y;
  const double c = std::cos(goal.yaw);
  const double s = std::sin(goal.yaw);

  // Goal frame: t = (c, s) longitudinal, n = (-s, c) lateral.
  error.lon = std::fabs(dx * c + dy * s);
  error.lat = std::fabs(-dx * s + dy * c);
  error.yaw = std::fabs(wrapToPi(terminal.yaw - goal.yaw));
  return error;
}

EvaluationResult evaluateTrajectory(
  const TrajectoryData & trajectory, const GridData * grid, const Params & params)
{
  EvaluationResult result;
  result.metrics = computeMetrics(trajectory, params);
  const Metrics & m = result.metrics;

  bool passed = true;
  auto fail = [&result, &passed](const std::string & reason) {
    passed = false;
    result.failure_reasons.push_back(reason);
  };
  auto addItem = [&result](
                   const std::string & name, double value, double threshold, bool ok,
                   const std::string & unit) {
    MetricItem item;
    item.name = name;
    item.value = value;
    item.threshold = threshold;
    item.passed = ok;
    item.unit = unit;
    result.items.push_back(std::move(item));
  };

  // ---------------- hard: validity ----------------
  addItem("point_count", static_cast<double>(m.point_count), params.constraints.min_points,
          m.point_count >= params.constraints.min_points, "");
  if (m.has_nan) {
    fail("invalid trajectory: NaN/Inf detected");
  }
  if (m.point_count < params.constraints.min_points) {
    fail("too few trajectory points");
  }
  if (m.has_nan || m.point_count < params.constraints.min_points || !m.valid) {
    result.is_control_ready = false;
    result.terminal = trajectory.points.empty() ? PointData{} : trajectory.points.back();
    return result;
  }

  // ---------------- hard: continuity / jump ----------------
  const bool interval_ok =
    m.min_interval >= params.constraints.min_point_interval &&
    m.max_interval <= params.constraints.max_point_interval;
  addItem("min_point_interval", m.min_interval, params.constraints.min_point_interval,
          m.min_interval >= params.constraints.min_point_interval, "m");
  addItem("max_point_interval", m.max_interval, params.constraints.max_point_interval,
          m.max_interval <= params.constraints.max_point_interval, "m");
  if (m.min_interval < params.constraints.min_point_interval) {
    fail("continuity: point spacing too small");
  }
  if (m.max_interval > params.constraints.max_point_interval) {
    fail("continuity: point spacing too large");
  }

  const bool jump_ok = m.max_delta_angle <= params.constraints.max_delta_angle;
  addItem("max_delta_angle", m.max_delta_angle, params.constraints.max_delta_angle, jump_ok, "rad");
  if (!jump_ok) {
    fail("jump: heading changes abruptly between points");
  }

  // ---------------- hard: curvature / steering ----------------
  const bool curvature_ok = m.max_curvature <= params.constraints.max_curvature;
  addItem("max_curvature", m.max_curvature, params.constraints.max_curvature, curvature_ok, "1/m");
  if (!curvature_ok) {
    fail("curvature exceeds the minimum turning radius");
  }

  const bool steer_ok = m.max_steer <= params.vehicle.max_steer;
  addItem("max_steer_angle", m.max_steer, params.vehicle.max_steer, steer_ok, "rad");
  if (!steer_ok) {
    fail("steering angle exceeds the vehicle limit");
  }

  const bool steer_rate_ok = m.max_steer_rate <= params.constraints.max_steer_rate;
  addItem("max_steer_rate", m.max_steer_rate, params.constraints.max_steer_rate, steer_rate_ok,
          "rad/s");
  if (!steer_rate_ok) {
    fail("steering rate exceeds the limit");
  }

  // ---------------- hard: velocity ----------------
  const bool speed_ok = m.max_abs_velocity <= params.constraints.max_velocity;
  addItem("max_abs_velocity", m.max_abs_velocity, params.constraints.max_velocity, speed_ok, "m/s");
  if (!speed_ok) {
    fail("velocity exceeds the parking limit");
  }

  const bool stop_ok = m.terminal_velocity <= params.constraints.terminal_velocity;
  addItem("terminal_velocity", m.terminal_velocity, params.constraints.terminal_velocity, stop_ok,
          "m/s");
  if (!stop_ok) {
    fail("terminal velocity is not zero");
  }

  // ---------------- hard: safety ----------------
  if (grid != nullptr && params.enable_occupancy_check) {
    result.collision = checkCollision(
      trajectory, *grid, params.vehicle, params.constraints.occupancy_cost_threshold);
    result.occupancy_evaluated = result.collision.evaluated;

    const bool safety_ok = result.collision.evaluated &&
                           result.collision.center_collisions == 0 &&
                           result.collision.footprint_collisions == 0;
    addItem("center_collisions", static_cast<double>(result.collision.center_collisions), 0.0,
            result.collision.center_collisions == 0, "");
    addItem("footprint_collisions", static_cast<double>(result.collision.footprint_collisions),
            0.0, result.collision.footprint_collisions == 0, "");
    if (!safety_ok && result.collision.evaluated) {
      fail("safety: trajectory intersects occupied cells");
    }
  } else {
    result.occupancy_evaluated = false;
  }

  // ---------------- goal accuracy ----------------
  result.terminal = trajectory.points.back();
  const GoalError goal_error = computeGoalError(result.terminal, params.goal);
  result.goal_configured = goal_error.configured;
  result.lat_error = goal_error.lat;
  result.lon_error = goal_error.lon;
  result.yaw_error = goal_error.yaw;

  if (goal_error.configured) {
    const bool lat_ok = goal_error.lat <= params.goal.tolerance_lat;
    const bool lon_ok = goal_error.lon <= params.goal.tolerance_lon;
    const bool yaw_ok = goal_error.yaw <= params.goal.tolerance_yaw;

    addItem("terminal_lat_error", goal_error.lat, params.goal.tolerance_lat, lat_ok, "m");
    addItem("terminal_lon_error", goal_error.lon, params.goal.tolerance_lon, lon_ok, "m");
    addItem("terminal_yaw_error", goal_error.yaw, params.goal.tolerance_yaw, yaw_ok, "rad");

    if (!lat_ok) {
      fail("terminal lateral error exceeds tolerance");
    }
    if (!lon_ok) {
      fail("terminal longitudinal error exceeds tolerance");
    }
    if (!yaw_ok) {
      fail("terminal heading error exceeds tolerance");
    }
  }

  // ---------------- soft scoring ----------------
  ScoreBreakdown & s = result.score;

  if (goal_error.configured) {
    s.terminal_lat = 20.0 * linearScore(goal_error.lat, 0.0, params.goal.tolerance_lat);
    s.terminal_lon = 15.0 * linearScore(goal_error.lon, 0.0, params.goal.tolerance_lon);
    s.terminal_yaw = 15.0 * linearScore(goal_error.yaw, 0.0, params.goal.tolerance_yaw);
  } else {
    // Not configured: do not punish the trajectory for missing information.
    s.terminal_lat = 20.0;
    s.terminal_lon = 15.0;
    s.terminal_yaw = 15.0;
  }

  if (m.switch_count <= 2) {
    s.switch_count = 15.0;
  } else {
    s.switch_count = std::max(0.0, 15.0 - 5.0 * static_cast<double>(m.switch_count - 2));
  }

  s.lateral_acc = 10.0 * linearScore(m.max_lateral_acc, params.comfort.max_lateral_acc, 9.8);
  s.lateral_jerk = 10.0 * linearScore(m.max_lateral_jerk, params.comfort.max_lateral_jerk, 7.0);

  if (result.occupancy_evaluated) {
    s.clearance = 10.0 * linearScore(result.collision.min_clearance, 0.5, 0.0);
  } else {
    s.clearance = 10.0;
  }

  s.smoothness = 5.0 * linearScore(m.curvature_variance, 0.001, 0.05);

  s.total = s.terminal_lat + s.terminal_lon + s.terminal_yaw + s.switch_count + s.lateral_acc +
            s.lateral_jerk + s.clearance + s.smoothness;

  if (s.total < params.scoring.min_score) {
    fail("score below the control-ready threshold");
  }

  addItem("total_score", s.total, params.scoring.min_score, s.total >= params.scoring.min_score,
          "");
  addItem("switch_count", static_cast<double>(m.switch_count),
          static_cast<double>(params.comfort.max_switch_count),
          m.switch_count <= params.comfort.max_switch_count, "");

  result.is_control_ready = passed && interval_ok;
  return result;
}

}  // namespace parking_trajectory_evaluator
