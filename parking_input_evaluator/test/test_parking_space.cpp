// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include <gtest/gtest.h>

#include <cmath>

#include "parking_input_evaluator/map_parser.hpp"

using parking_input_evaluator::ParkingSpaceGeom;
using parking_input_evaluator::Point2D;

namespace
{

ParkingSpaceGeom makeSpace()
{
  ParkingSpaceGeom space;
  space.id = 42;
  space.width = 2.6;
  space.pts = {Point2D{270.0, 230.0}, Point2D{275.4, 230.0}};
  return space;
}

}  // namespace

TEST(ParkingSpace, CenterIsMidpoint)
{
  const auto space = makeSpace();
  const auto center = space.center();
  EXPECT_NEAR(center.x, 272.7, 1e-6);
  EXPECT_NEAR(center.y, 230.0, 1e-6);
}

TEST(ParkingSpace, HeadingFollowsPolyline)
{
  const auto space = makeSpace();
  // The centre line runs from x=270.0 to x=275.4, i.e. along +x.
  EXPECT_NEAR(space.heading(), 0.0, 1e-6);
}

TEST(ParkingSpace, Length)
{
  const auto space = makeSpace();
  EXPECT_NEAR(space.length(), 5.4, 1e-6);
}

TEST(ParkingSpace, PointInBandInside)
{
  const auto space = makeSpace();
  EXPECT_TRUE(parking_input_evaluator::pointInSpaceBand(272.7, 230.5, space));
  EXPECT_TRUE(parking_input_evaluator::pointInSpaceBand(272.7, 229.5, space));
}

TEST(ParkingSpace, PointInBandOutside)
{
  const auto space = makeSpace();
  // 2.0 m away from the centre line, band half width is 1.3 m.
  EXPECT_FALSE(parking_input_evaluator::pointInSpaceBand(272.7, 232.0, space));
  // Beyond the end of the segment by more than the tolerance.
  EXPECT_FALSE(parking_input_evaluator::pointInSpaceBand(277.0, 230.0, space));
}

TEST(ParkingSpace, PolygonHasExpectedExtent)
{
  const auto space = makeSpace();
  const auto polygon = parking_input_evaluator::spacePolygon(space);
  ASSERT_GE(polygon.size(), 4u);

  double min_y = polygon.front().y;
  double max_y = min_y;
  for (const auto & p : polygon) {
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
  EXPECT_NEAR(max_y - min_y, space.width, 1e-6);
}
