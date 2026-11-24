// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace parking_input_evaluator
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct ParkingSpaceGeom
{
  uint64_t id{0};
  std::vector<Point2D> pts;  // centre-line of the bay
  double width{2.5};

  Point2D center() const;
  /// Direction of the longest segment, in radians.
  double heading() const;
  /// Total polyline length.
  double length() const;
};

struct MapGeometry
{
  bool loaded{false};
  std::string error;

  bool has_lot{false};
  std::vector<Point2D> lot_polygon;
  double lot_min_x{0.0};
  double lot_min_y{0.0};
  double lot_max_x{0.0};
  double lot_max_y{0.0};

  std::vector<ParkingSpaceGeom> spaces;

  double lotDiagonal() const;
};

/// Loads parking_lot / parking_space elements from an OSM lanelet2 map.
/// The map is projected with a LocalCartesian projector at origin (0,0,0),
/// matching the local_x / local_y tags used by this project's maps.
MapGeometry loadMapGeometry(const std::string & osm_path);

/// True when (px, py) falls inside the bay band (centre-line +/- width/2).
bool pointInSpaceBand(double px, double py, const ParkingSpaceGeom & space);

/// Builds a closed polygon from a centre-line and a width, used for visualisation.
std::vector<Point2D> spacePolygon(const ParkingSpaceGeom & space);

}  // namespace parking_input_evaluator
