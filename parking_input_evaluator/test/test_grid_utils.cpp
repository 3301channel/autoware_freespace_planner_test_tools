// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include <gtest/gtest.h>

#include <cmath>

#include "parking_input_evaluator/grid_utils.hpp"

using parking_input_evaluator::GridData;
using parking_input_evaluator::GridInfo;

namespace
{

GridData makeGrid(int width, int height, double resolution, int fill)
{
  GridData grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin_x = 0.0;
  grid.info.origin_y = 0.0;
  grid.info.frame_id = "map";
  grid.data.assign(static_cast<size_t>(width) * height, static_cast<int8_t>(fill));
  return grid;
}

}  // namespace

TEST(GridUtils, WorldToCellBasic)
{
  const auto grid = makeGrid(10, 10, 1.0, 0);
  const auto cell = parking_input_evaluator::worldToCell(2.5, 3.5, grid.info);
  EXPECT_TRUE(cell.inside);
  EXPECT_EQ(cell.ix, 2);
  EXPECT_EQ(cell.iy, 3);
}

TEST(GridUtils, WorldToCellOutside)
{
  const auto grid = makeGrid(10, 10, 1.0, 0);
  EXPECT_FALSE(parking_input_evaluator::worldToCell(-0.1, 0.5, grid.info).inside);
  EXPECT_FALSE(parking_input_evaluator::worldToCell(10.1, 0.5, grid.info).inside);
}

TEST(GridUtils, ObstacleRuleMatchesPlanner)
{
  // abstract_algorithm.cpp:119 -> cost < 0 || threshold <= cost
  EXPECT_TRUE(parking_input_evaluator::isObstacle(-1, 100));
  EXPECT_TRUE(parking_input_evaluator::isObstacle(100, 100));
  EXPECT_FALSE(parking_input_evaluator::isObstacle(0, 100));
  EXPECT_FALSE(parking_input_evaluator::isObstacle(99, 100));
}

TEST(GridUtils, RectDetectsCollision)
{
  auto grid = makeGrid(20, 20, 0.5, 0);
  // Put a single obstacle cell right at the centre.
  grid.data[10 * 20 + 10] = 100;

  const auto hit = parking_input_evaluator::sampleRect(
    grid, 5.25, 5.25, 0.0, 4.0, 2.0, 11, 7, 100);
  EXPECT_FALSE(hit.ok);
  EXPECT_GT(hit.conflicts, 0);

  const auto clear = parking_input_evaluator::sampleRect(
    grid, 2.0, 2.0, 0.0, 1.0, 1.0, 11, 7, 100);
  EXPECT_TRUE(clear.ok);
  EXPECT_EQ(clear.conflicts, 0);
}

TEST(GridUtils, RectDetectsOutOfGrid)
{
  auto grid = makeGrid(10, 10, 1.0, 0);
  const auto result = parking_input_evaluator::sampleRect(
    grid, 0.2, 0.2, 0.0, 4.0, 2.0, 11, 7, 100);
  EXPECT_TRUE(result.out_of_grid);
  EXPECT_FALSE(result.ok);
}

TEST(GridUtils, CostStats)
{
  auto grid = makeGrid(10, 10, 1.0, 0);
  grid.data[0] = -1;
  grid.data[1] = 100;
  const auto stats = parking_input_evaluator::computeCostStats(grid, 100);
  EXPECT_EQ(stats.total, 100u);
  EXPECT_EQ(stats.unknown_cells, 1u);
  EXPECT_EQ(stats.occupied_cells, 1u);
  EXPECT_EQ(stats.free_cells, 98u);
}
