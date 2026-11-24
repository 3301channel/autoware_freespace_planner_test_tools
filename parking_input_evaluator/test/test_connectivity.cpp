// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include <gtest/gtest.h>

#include "parking_input_evaluator/connectivity.hpp"
#include "parking_input_evaluator/grid_utils.hpp"

using parking_input_evaluator::GridData;
using parking_input_evaluator::GridPoint;

namespace
{

GridData makeGrid(int width, int height, double resolution)
{
  GridData grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin_x = 0.0;
  grid.info.origin_y = 0.0;
  grid.info.frame_id = "map";
  grid.data.assign(static_cast<size_t>(width) * height, 0);
  return grid;
}

}  // namespace

TEST(Connectivity, SplitByWallIsDisconnected)
{
  auto grid = makeGrid(20, 10, 0.5);
  // Vertical wall at ix = 10 covering every row.
  for (int iy = 0; iy < 10; ++iy) {
    grid.data[static_cast<size_t>(iy) * 20 + 10] = 100;
  }

  const GridPoint start{2, 5, true};
  const GridPoint goal{17, 5, true};
  const auto result = parking_input_evaluator::analyzeConnectivity(grid, 100, 1.0, start, goal, 8);

  EXPECT_FALSE(result.free_connected);
  EXPECT_FALSE(result.passable_connected);
}

TEST(Connectivity, OpenSpaceIsConnected)
{
  auto grid = makeGrid(30, 20, 0.3);
  const GridPoint start{2, 2, true};
  const GridPoint goal{27, 17, true};
  const auto result = parking_input_evaluator::analyzeConnectivity(grid, 100, 1.0, start, goal, 8);

  EXPECT_TRUE(result.free_connected);
  EXPECT_TRUE(result.passable_connected);
  EXPECT_GT(result.passable_count, 0);
}

TEST(Connectivity, NarrowGapFailsEdtConstraint)
{
  auto grid = makeGrid(20, 20, 0.3);
  // Two walls with a 2-cell (0.6 m) corridor between them at iy = 9 and iy = 12.
  for (int iy = 0; iy < 20; ++iy) {
    if (iy != 10 && iy != 11) {
      for (int ix = 0; ix < 20; ++ix) {
        grid.data[static_cast<size_t>(iy) * 20 + ix] = 100;
      }
    }
  }
  // Corridor is only 2 cells wide -> 0.6 m, half width 1.0 m cannot pass.
  const GridPoint start{1, 10, true};
  const GridPoint goal{18, 11, true};
  const auto result = parking_input_evaluator::analyzeConnectivity(grid, 100, 1.0, start, goal, 8);

  EXPECT_TRUE(result.free_connected);
  EXPECT_FALSE(result.passable_connected);
}

TEST(Connectivity, UnknownTreatedAsObstacle)
{
  auto grid = makeGrid(10, 10, 0.3);
  grid.data[5 * 10 + 5] = -1;
  const auto result = parking_input_evaluator::analyzeConnectivity(
    grid, 100, 0.5, GridPoint{0, 0, true}, GridPoint{9, 9, true}, 8);
  EXPECT_LT(result.edt_m[static_cast<size_t>(5) * 10 + 5], 1e-6F);
}
