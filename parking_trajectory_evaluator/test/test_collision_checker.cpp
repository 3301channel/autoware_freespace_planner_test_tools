// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include <gtest/gtest.h>

#include "parking_trajectory_evaluator/collision_checker.hpp"

using namespace parking_trajectory_evaluator;

namespace
{

GridData makeGrid(int width, int height, double resolution, int fill)
{
  GridData grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.frame_id = "map";
  grid.data.assign(static_cast<size_t>(width) * height, static_cast<int8_t>(fill));
  return grid;
}

TrajectoryData makeStraight(double length, double spacing, double y)
{
  TrajectoryData trajectory;
  const int count = static_cast<int>(length / spacing) + 1;
  for (int i = 0; i < count; ++i) {
    PointData p;
    p.x = i * spacing;
    p.y = y;
    p.yaw = 0.0;
    p.v = 1.0;
    trajectory.points.push_back(p);
  }
  return trajectory;
}

}  // namespace

TEST(CollisionChecker, FreeGridHasNoCollision)
{
  const auto grid = makeGrid(60, 40, 0.3, 0);  // 18 m x 12 m
  auto trajectory = makeStraight(6.0, 0.5, 5.0);
  // Keep the whole vehicle footprint (4.5 m long) inside the grid.
  for (auto & p : trajectory.points) {
    p.x += 4.0;
  }

  VehicleParams vehicle;
  const auto result = checkCollision(trajectory, grid, vehicle, 50);

  ASSERT_TRUE(result.evaluated);
  EXPECT_EQ(result.center_collisions, 0);
  EXPECT_EQ(result.footprint_collisions, 0);
}

TEST(CollisionChecker, CenterCollisionDetected)
{
  auto grid = makeGrid(60, 40, 0.3, 0);
  const auto trajectory = makeStraight(10.0, 0.5, 5.0);

  // Block the cell at x = 3.0, y = 5.0.
  const int ix = static_cast<int>(3.0 / 0.3);
  const int iy = static_cast<int>(5.0 / 0.3);
  grid.data[static_cast<size_t>(iy) * grid.width + ix] = 100;

  VehicleParams vehicle;
  const auto result = checkCollision(trajectory, grid, vehicle, 50);
  EXPECT_GT(result.center_collisions, 0);
}

TEST(CollisionChecker, FootprintWiderThanCenterPoint)
{
  auto grid = makeGrid(60, 40, 0.3, 0);
  const auto trajectory = makeStraight(10.0, 0.5, 5.0);

  // Obstacle 1.0 m beside the path: the centre misses it, the 2 m wide
  // footprint does not.
  const int ix = static_cast<int>(3.0 / 0.3);
  const int iy = static_cast<int>(6.0 / 0.3);
  grid.data[static_cast<size_t>(iy) * grid.width + ix] = 100;

  VehicleParams vehicle;
  const auto result = checkCollision(trajectory, grid, vehicle, 50);
  EXPECT_EQ(result.center_collisions, 0);
  EXPECT_GT(result.footprint_collisions, 0);
}

TEST(CollisionChecker, OutsideGridCountsAsCollision)
{
  const auto grid = makeGrid(10, 10, 0.3, 0);  // 3 m x 3 m
  const auto trajectory = makeStraight(10.0, 0.5, 1.0);  // leaves the grid
  VehicleParams vehicle;
  const auto result = checkCollision(trajectory, grid, vehicle, 50);
  EXPECT_GT(result.footprint_collisions, 0);
}

TEST(CollisionChecker, InvalidGridIsNotEvaluated)
{
  GridData grid;  // empty
  const auto trajectory = makeStraight(5.0, 0.5, 1.0);
  VehicleParams vehicle;
  const auto result = checkCollision(trajectory, grid, vehicle, 50);
  EXPECT_FALSE(result.evaluated);
}
