// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_input_evaluator/visualization.hpp"

#include <algorithm>

namespace parking_input_evaluator
{
namespace
{

visualization_msgs::msg::Marker makeLineMarker(
  const std::string & ns, int id, const std::string & frame_id, const rclcpp::Time & stamp,
  float r, float g, float b, float a, double line_width)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = line_width;
  marker.scale.y = line_width;
  marker.scale.z = line_width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(0.0);
  return marker;
}

void addPoint(visualization_msgs::msg::Marker & marker, double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  marker.points.push_back(p);
}

}  // namespace

visualization_msgs::msg::MarkerArray buildMarkers(
  const Report & report, const GridData & grid, const MapGeometry & map,
  const geometry_msgs::msg::Pose & goal, bool has_goal, const std::string & frame_id,
  const rclcpp::Time & stamp, const std::string & ns_prefix)
{
  visualization_msgs::msg::MarkerArray array;

  // 1. Grid extent
  if (grid.valid()) {
    auto marker = makeLineMarker(ns_prefix + "/grid_extent", 0, frame_id, stamp, 0.1F, 0.6F, 1.0F, 0.9F, 0.15);
    const double x0 = grid.info.origin_x;
    const double y0 = grid.info.origin_y;
    const double x1 = x0 + grid.info.width * grid.info.resolution;
    const double y1 = y0 + grid.info.height * grid.info.resolution;
    addPoint(marker, x0, y0, 0.0);
    addPoint(marker, x1, y0, 0.0);
    addPoint(marker, x1, y1, 0.0);
    addPoint(marker, x0, y1, 0.0);
    addPoint(marker, x0, y0, 0.0);
    array.markers.push_back(std::move(marker));
  }

  // 2. Parking lot outline
  if (map.has_lot && map.lot_polygon.size() >= 2) {
    auto marker = makeLineMarker(ns_prefix + "/parking_lot", 1, frame_id, stamp, 1.0F, 0.9F, 0.1F, 0.9F, 0.12);
    for (const auto & p : map.lot_polygon) {
      addPoint(marker, p.x, p.y, 0.05);
    }
    addPoint(marker, map.lot_polygon.front().x, map.lot_polygon.front().y, 0.05);
    array.markers.push_back(std::move(marker));
  }

  // 3. Parking bays coloured by status
  int marker_id = 10;
  for (const auto & result : report.spaces) {
    // Rebuild the bay geometry from the map so we can draw the real footprint.
    const ParkingSpaceGeom * geom = nullptr;
    for (const auto & candidate : map.spaces) {
      if (candidate.id == result.id) {
        geom = &candidate;
        break;
      }
    }
    if (geom == nullptr) {
      continue;
    }
    const auto polygon = spacePolygon(*geom);
    if (polygon.size() < 3) {
      continue;
    }

    float r = 0.1F;
    float g = 0.8F;
    if (result.status == SpaceStatus::OUT_OF_GRID) {
      r = 0.5F;
      g = 0.5F;
    } else if (result.status == SpaceStatus::BLOCKED) {
      r = 1.0F;
      g = 0.1F;
    } else if (result.status == SpaceStatus::UNREACHABLE) {
      r = 1.0F;
      g = 0.6F;
    }

    auto marker = makeLineMarker(ns_prefix + "/bays", marker_id++, frame_id, stamp, r, g, 0.1F, 0.95F, 0.08);
    for (const auto & p : polygon) {
      addPoint(marker, p.x, p.y, 0.1);
    }
    addPoint(marker, polygon.front().x, polygon.front().y, 0.1);
    array.markers.push_back(std::move(marker));
  }

  // 4. Goal arrow
  if (has_goal) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = stamp;
    arrow.ns = ns_prefix + "/goal";
    arrow.id = 1000;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = goal;
    arrow.scale.x = 2.0;
    arrow.scale.y = 0.3;
    arrow.scale.z = 0.3;
    const bool ok = report.can_plan_current_goal;
    arrow.color.r = ok ? 0.1F : 1.0F;
    arrow.color.g = ok ? 0.9F : 0.1F;
    arrow.color.b = 0.2F;
    arrow.color.a = 1.0F;
    array.markers.push_back(std::move(arrow));
  }

  // 5. Clean up stale markers from previous cycles
  visualization_msgs::msg::Marker cleanup;
  cleanup.header.frame_id = frame_id;
  cleanup.header.stamp = stamp;
  cleanup.ns = ns_prefix + "/bays";
  cleanup.id = marker_id + 10000;
  cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(std::move(cleanup));

  return array;
}

}  // namespace parking_input_evaluator
