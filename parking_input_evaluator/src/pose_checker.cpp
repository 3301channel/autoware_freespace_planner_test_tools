// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_input_evaluator/pose_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace parking_input_evaluator
{
namespace
{

double pointSegmentDistance(double px, double py, const Point2D & a, const Point2D & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double len2 = vx * vx + vy * vy;
  if (len2 <= 1e-12) {
    return std::hypot(px - a.x, py - a.y);
  }
  double t = ((px - a.x) * vx + (py - a.y) * vy) / len2;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(px - (a.x + t * vx), py - (a.y + t * vy));
}

double polylineDistance(double px, double py, const std::vector<Point2D> & pts)
{
  if (pts.empty()) {
    return std::numeric_limits<double>::max();
  }
  if (pts.size() == 1) {
    return std::hypot(px - pts.front().x, py - pts.front().y);
  }
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    best = std::min(best, pointSegmentDistance(px, py, pts[i], pts[i + 1]));
  }
  return best;
}

}  // namespace

double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

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

double headingDiffDeg(double a, double b)
{
  const double diff = std::fabs(wrapToPi(a - b)) * 180.0 / M_PI;
  return std::min(diff, 180.0 - diff);
}

PoseCheckResult checkPose(
  const GridData & grid, double x, double y, double yaw, double collision_length,
  double collision_width, int obstacle_threshold, int samples_long, int samples_lat,
  const MapGeometry * map, bool require_in_space)
{
  PoseCheckResult result;

  const CellIndex idx = worldToCell(x, y, grid.info);
  result.inside = idx.inside;
  if (idx.inside) {
    result.cost = static_cast<int>(costAt(grid, idx.ix, idx.iy));
    result.free = !isObstacle(static_cast<int8_t>(result.cost), obstacle_threshold);
  }

  result.rect = sampleRect(
    grid, x, y, yaw, collision_length, collision_width, samples_long, samples_lat,
    obstacle_threshold);

  if (map != nullptr && !map->spaces.empty()) {
    double best = std::numeric_limits<double>::max();
    int best_index = -1;
    for (size_t i = 0; i < map->spaces.size(); ++i) {
      const double d = polylineDistance(x, y, map->spaces[i].pts);
      if (d < best) {
        best = d;
        best_index = static_cast<int>(i);
      }
    }
    result.nearest_space_index = best_index;
    if (best_index >= 0) {
      result.nearest_space_distance = best;
      result.heading_diff_deg = headingDiffDeg(yaw, map->spaces[best_index].heading());
      result.in_space_band = pointInSpaceBand(x, y, map->spaces[best_index]);
    }
  }

  const bool space_ok = (!require_in_space) || result.in_space_band;
  result.acceptable = result.inside && result.free && result.rect.ok && space_ok;
  return result;
}

}  // namespace parking_input_evaluator
