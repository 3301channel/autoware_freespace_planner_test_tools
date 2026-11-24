// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include "parking_input_evaluator/connectivity.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace parking_input_evaluator
{

ConnectivityResult analyzeConnectivity(
  const GridData & grid, int obstacle_threshold, double half_width, const GridPoint & start,
  const GridPoint & goal, int neighbors)
{
  ConnectivityResult result;
  result.half_width = half_width;

  if (!grid.valid()) {
    return result;
  }

  const int width = grid.info.width;
  const int height = grid.info.height;
  const size_t cells = static_cast<size_t>(width) * height;

  result.edt_m.assign(cells, 0.0F);
  result.passable.assign(cells, 0);

  // OpenCV distanceTransform expects 8-bit input: non-zero = foreground.
  // We want the distance from free cells to the nearest obstacle, so obstacles
  // must be zero (background) and free cells 255.
  cv::Mat free_mask = cv::Mat::zeros(height, width, CV_8U);
  for (int iy = 0; iy < height; ++iy) {
    for (int ix = 0; ix < width; ++ix) {
      const auto cost = grid.data[static_cast<size_t>(iy) * width + ix];
      free_mask.at<uint8_t>(iy, ix) = isObstacle(cost, obstacle_threshold) ? 0 : 255;
    }
  }

  cv::Mat distance;
  cv::distanceTransform(free_mask, distance, cv::DIST_L2, 5);

  for (int iy = 0; iy < height; ++iy) {
    for (int ix = 0; ix < width; ++ix) {
      const size_t index = static_cast<size_t>(iy) * width + ix;
      const float metres = distance.at<float>(iy, ix) * static_cast<float>(grid.info.resolution);
      result.edt_m[index] = metres;
      if (free_mask.at<uint8_t>(iy, ix) != 0 && metres >= static_cast<float>(half_width)) {
        result.passable[index] = 1;
        ++result.passable_count;
      }
    }
  }

  result.start_inside = start.inside;
  result.goal_inside = goal.inside;

  // F1: connectivity on the raw free space.
  cv::Mat free_labels;
  cv::connectedComponents(free_mask, free_labels, neighbors, CV_32S);

  // F2: connectivity on the A*-equivalent passable set.
  cv::Mat passable_mask = cv::Mat::zeros(height, width, CV_8U);
  for (size_t i = 0; i < cells; ++i) {
    if (result.passable[i] != 0) {
      passable_mask.at<uint8_t>(static_cast<int>(i / width), static_cast<int>(i % width)) = 255;
    }
  }
  cv::Mat passable_labels;
  cv::connectedComponents(passable_mask, passable_labels, neighbors, CV_32S);

  result.passable_labels.resize(cells, 0);
  for (int iy = 0; iy < height; ++iy) {
    for (int ix = 0; ix < width; ++ix) {
      result.passable_labels[static_cast<size_t>(iy) * width + ix] =
        passable_labels.at<int>(iy, ix);
    }
  }

  if (start.inside && goal.inside) {
    const int free_start = free_labels.at<int>(start.iy, start.ix);
    const int free_goal = free_labels.at<int>(goal.iy, goal.ix);
    result.free_connected = (free_start != 0 && free_start == free_goal);

    const int pass_start = passable_labels.at<int>(start.iy, start.ix);
    const int pass_goal = passable_labels.at<int>(goal.iy, goal.ix);
    result.start_passable = (pass_start != 0);
    result.goal_passable = (pass_goal != 0);
    result.start_label = pass_start;
    result.passable_connected = (pass_start != 0 && pass_start == pass_goal);
  }

  return result;
}

double edtAt(const ConnectivityResult & result, const GridData & grid, int ix, int iy)
{
  if (ix < 0 || iy < 0 || ix >= grid.info.width || iy >= grid.info.height) {
    return 0.0;
  }
  const size_t index = static_cast<size_t>(iy) * grid.info.width + ix;
  if (index >= result.edt_m.size()) {
    return 0.0;
  }
  return static_cast<double>(result.edt_m[index]);
}

}  // namespace parking_input_evaluator
