// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_trajectory_evaluator/visualization.hpp"

#include <cmath>

namespace parking_trajectory_evaluator
{
namespace
{

visualization_msgs::msg::Marker makeLine(
  const std::string & ns, int id, const std::string & frame_id, const rclcpp::Time & stamp,
  float r, float g, float b, double width)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0F;
  marker.pose.orientation.w = 1.0;
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
  const EvaluationResult & result, const TrajectoryData & trajectory, const std::string & frame_id,
  const rclcpp::Time & stamp, const std::string & ns_prefix)
{
  visualization_msgs::msg::MarkerArray array;
  const auto & points = trajectory.points;
  if (points.size() < 2) {
    return array;
  }

  // Whole trajectory: green when control-ready, red otherwise.
  const bool ready = result.is_control_ready;
  auto main_line = makeLine(
    ns_prefix + "/path", 0, frame_id, stamp, ready ? 0.1F : 1.0F, ready ? 0.85F : 0.15F,
    ready ? 0.2F : 0.15F, 0.12);
  for (const auto & p : points) {
    addPoint(main_line, p.x, p.y, 0.05);
  }
  array.markers.push_back(std::move(main_line));

  // Reverse segments drawn on top in blue.
  int reverse_id = 1;
  visualization_msgs::msg::Marker reverse;
  bool open = false;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const bool is_reverse = points[i].v < 0.0;
    if (is_reverse && !open) {
      reverse = makeLine(ns_prefix + "/reverse", reverse_id++, frame_id, stamp, 0.1F, 0.4F, 1.0F, 0.08);
      addPoint(reverse, points[i].x, points[i].y, 0.08);
      open = true;
    }
    if (is_reverse) {
      addPoint(reverse, points[i + 1].x, points[i + 1].y, 0.08);
    } else if (open) {
      array.markers.push_back(reverse);
      open = false;
    }
  }
  if (open) {
    array.markers.push_back(reverse);
  }

  // Terminal arrow.
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = frame_id;
  arrow.header.stamp = stamp;
  arrow.ns = ns_prefix + "/terminal";
  arrow.id = 100;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.pose.position.x = points.back().x;
  arrow.pose.position.y = points.back().y;
  arrow.pose.orientation.z = std::sin(points.back().yaw / 2.0);
  arrow.pose.orientation.w = std::cos(points.back().yaw / 2.0);
  arrow.scale.x = 1.8;
  arrow.scale.y = 0.25;
  arrow.scale.z = 0.25;
  arrow.color.r = ready ? 0.1F : 1.0F;
  arrow.color.g = ready ? 0.9F : 0.3F;
  arrow.color.b = 0.3F;
  arrow.color.a = 1.0F;
  array.markers.push_back(std::move(arrow));

  // Collision highlight: mark offending points.
  if (result.collision.evaluated && result.collision.footprint_collisions > 0) {
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = frame_id;
    points_marker.header.stamp = stamp;
    points_marker.ns = ns_prefix + "/collision_points";
    points_marker.id = 200;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.scale.x = 0.25;
    points_marker.scale.y = 0.25;
    points_marker.scale.z = 0.25;
    points_marker.color.r = 1.0F;
    points_marker.color.g = 0.0F;
    points_marker.color.b = 0.0F;
    points_marker.color.a = 1.0F;
    points_marker.pose.orientation.w = 1.0;

    for (const auto & p : points) {
      addPoint(points_marker, p.x, p.y, 0.2);
    }
    array.markers.push_back(std::move(points_marker));
  }

  return array;
}

}  // namespace parking_trajectory_evaluator
