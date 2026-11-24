// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_input_evaluator/map_parser.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <unordered_map>

namespace parking_input_evaluator
{

Point2D ParkingSpaceGeom::center() const
{
  Point2D c;
  if (pts.empty()) {
    return c;
  }
  for (const auto & p : pts) {
    c.x += p.x;
    c.y += p.y;
  }
  c.x /= static_cast<double>(pts.size());
  c.y /= static_cast<double>(pts.size());
  return c;
}

double ParkingSpaceGeom::heading() const
{
  double best_length = -1.0;
  double best_heading = 0.0;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const double dx = pts[i + 1].x - pts[i].x;
    const double dy = pts[i + 1].y - pts[i].y;
    const double length = std::hypot(dx, dy);
    if (length > best_length) {
      best_length = length;
      best_heading = std::atan2(dy, dx);
    }
  }
  return best_heading;
}

double ParkingSpaceGeom::length() const
{
  double total = 0.0;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    total += std::hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
  }
  return total;
}

double MapGeometry::lotDiagonal() const { return std::hypot(lot_max_x - lot_min_x, lot_max_y - lot_min_y); }

namespace
{

/// Reads local_x / local_y tags. This project's OSM files carry empty lat/lon
/// attributes and store the projected position in tags instead, which the
/// stock lanelet2 OSM parser silently turns into (0, 0).
bool nodeCoordinates(const tinyxml2::XMLElement * node, Point2D * out)
{
  bool found = false;
  double x = 0.0;
  double y = 0.0;
  for (const auto * tag = node->FirstChildElement("tag"); tag != nullptr;
       tag = tag->NextSiblingElement("tag")) {
    const char * key = tag->Attribute("k");
    const char * value = tag->Attribute("v");
    if (key == nullptr || value == nullptr) {
      continue;
    }
    if (std::strcmp(key, "local_x") == 0) {
      x = std::atof(value);
      found = true;
    } else if (std::strcmp(key, "local_y") == 0) {
      y = std::atof(value);
      found = true;
    }
  }
  if (found) {
    out->x = x;
    out->y = y;
  }
  return found;
}

void updateLotExtent(MapGeometry * geometry)
{
  if (geometry->lot_polygon.empty()) {
    return;
  }
  double min_x = geometry->lot_polygon.front().x;
  double max_x = min_x;
  double min_y = geometry->lot_polygon.front().y;
  double max_y = min_y;
  for (const auto & p : geometry->lot_polygon) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
  geometry->lot_min_x = min_x;
  geometry->lot_min_y = min_y;
  geometry->lot_max_x = max_x;
  geometry->lot_max_y = max_y;
}

}  // namespace

MapGeometry loadMapGeometry(const std::string & osm_path)
{
  MapGeometry geometry;
  if (osm_path.empty()) {
    geometry.error = "lanelet2_map_path is empty";
    return geometry;
  }

  tinyxml2::XMLDocument document;
  if (document.LoadFile(osm_path.c_str()) != tinyxml2::XML_SUCCESS) {
    geometry.error = std::string("failed to parse OSM: ") + document.ErrorStr();
    return geometry;
  }

  const auto * osm = document.FirstChildElement("osm");
  if (osm == nullptr) {
    geometry.error = "no <osm> root element";
    return geometry;
  }

  std::unordered_map<uint64_t, Point2D> nodes;
  nodes.reserve(40000);

  for (const auto * node = osm->FirstChildElement("node"); node != nullptr;
       node = node->NextSiblingElement("node")) {
    const char * id_text = node->Attribute("id");
    if (id_text == nullptr) {
      continue;
    }
    Point2D position;
    if (nodeCoordinates(node, &position)) {
      nodes[std::strtoull(id_text, nullptr, 10)] = position;
    }
  }

  for (const auto * way = osm->FirstChildElement("way"); way != nullptr;
       way = way->NextSiblingElement("way")) {
    std::string type;
    double width = 2.5;

    for (const auto * tag = way->FirstChildElement("tag"); tag != nullptr;
         tag = tag->NextSiblingElement("tag")) {
      const char * key = tag->Attribute("k");
      const char * value = tag->Attribute("v");
      if (key == nullptr || value == nullptr) {
        continue;
      }
      if (std::strcmp(key, "type") == 0) {
        type = value;
      } else if (std::strcmp(key, "width") == 0) {
        width = std::atof(value);
      }
    }

    if (type != "parking_space" && type != "parking_lot") {
      continue;
    }

    std::vector<Point2D> points;
    for (const auto * nd = way->FirstChildElement("nd"); nd != nullptr;
         nd = nd->NextSiblingElement("nd")) {
      const char * ref = nd->Attribute("ref");
      if (ref == nullptr) {
        continue;
      }
      const auto it = nodes.find(std::strtoull(ref, nullptr, 10));
      if (it != nodes.end()) {
        points.push_back(it->second);
      }
    }
    if (points.size() < 2) {
      continue;
    }

    if (type == "parking_space") {
      ParkingSpaceGeom space;
      const char * id_text = way->Attribute("id");
      space.id = id_text != nullptr ? std::strtoull(id_text, nullptr, 10) : 0ULL;
      space.pts = std::move(points);
      space.width = width > 0.0 ? width : 2.5;
      geometry.spaces.push_back(std::move(space));
    } else if (!geometry.has_lot) {
      geometry.lot_polygon = std::move(points);
      geometry.has_lot = true;
    }
  }

  if (geometry.has_lot) {
    updateLotExtent(&geometry);
  } else if (!geometry.spaces.empty()) {
    // Derive a footprint from the bays when no parking_lot element exists.
    double min_x = geometry.spaces.front().center().x;
    double max_x = min_x;
    double min_y = geometry.spaces.front().center().y;
    double max_y = min_y;
    for (const auto & space : geometry.spaces) {
      const Point2D c = space.center();
      const double half = space.width / 2.0;
      min_x = std::min(min_x, c.x - half);
      max_x = std::max(max_x, c.x + half);
      min_y = std::min(min_y, c.y - half);
      max_y = std::max(max_y, c.y + half);
    }
    geometry.lot_min_x = min_x;
    geometry.lot_min_y = min_y;
    geometry.lot_max_x = max_x;
    geometry.lot_max_y = max_y;
    geometry.has_lot = true;
  }

  geometry.loaded = true;
  return geometry;
}

bool pointInSpaceBand(double px, double py, const ParkingSpaceGeom & space)
{
  const double half = space.width / 2.0;
  for (size_t i = 0; i + 1 < space.pts.size(); ++i) {
    const Point2D & a = space.pts[i];
    const Point2D & b = space.pts[i + 1];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    if (len2 <= 1e-12) {
      continue;
    }
    double t = ((px - a.x) * vx + (py - a.y) * vy) / len2;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = a.x + t * vx;
    const double cy = a.y + t * vy;
    if (std::hypot(px - cx, py - cy) <= half) {
      return true;
    }
  }
  return false;
}

std::vector<Point2D> spacePolygon(const ParkingSpaceGeom & space)
{
  std::vector<Point2D> polygon;
  if (space.pts.size() < 2) {
    return polygon;
  }
  const double half = space.width / 2.0;
  std::vector<Point2D> left;
  std::vector<Point2D> right;

  for (size_t i = 0; i + 1 < space.pts.size(); ++i) {
    const Point2D & a = space.pts[i];
    const Point2D & b = space.pts[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len <= 1e-9) {
      continue;
    }
    const double nx = -dy / len * half;
    const double ny = dx / len * half;
    if (left.empty()) {
      left.push_back(Point2D{a.x + nx, a.y + ny});
      right.push_back(Point2D{a.x - nx, a.y - ny});
    }
    left.push_back(Point2D{b.x + nx, b.y + ny});
    right.push_back(Point2D{b.x - nx, b.y - ny});
  }

  if (left.size() < 2) {
    return polygon;
  }
  polygon = left;
  for (auto it = right.rbegin(); it != right.rend(); ++it) {
    polygon.push_back(*it);
  }
  return polygon;
}

}  // namespace parking_input_evaluator
